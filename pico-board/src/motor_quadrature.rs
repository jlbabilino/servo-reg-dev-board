use core::cell::Cell;
use core::f32::consts;
use embassy_rp::adc;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use fixed::traits::ToFixed;
use fixed::types::I32F32;

#[embassy_executor::task]
pub async fn motor_quadrature_task(
    mut adc: adc::Adc<'static, adc::Blocking>,
    mut hall_a_pin: adc::Channel<'static>,
    mut hall_b_pin: adc::Channel<'static>,
    mut hall_c_pin: adc::Channel<'static>,
    motor_cum_angle_mutex: &'static Mutex<CriticalSectionRawMutex, Cell<I32F32>>,
    led_command_ch: &'static Channel<CriticalSectionRawMutex, crate::rgb_led::Command, 16>,
) {
    let mut tracker = HallAngleTracker::new();

    let ticker_duration = embassy_time::Duration::from_hz(3000);
    let ticker_initial_time = embassy_time::Instant::now();
    let mut ticker = embassy_time::Ticker::every(ticker_duration);
    ticker.reset_at(ticker_initial_time);
    let mut iter_idx: u32 = 0;

    loop {
        use crate::constants::{HA_AMP, HA_AVG, HB_AMP, HB_AVG, HC_AMP, HC_AVG};

        let ha_raw = adc.blocking_read(&mut hall_a_pin).unwrap();
        let hb_raw = adc.blocking_read(&mut hall_b_pin).unwrap();
        let hc_raw = adc.blocking_read(&mut hall_c_pin).unwrap();

        let ha_norm: f32 = (ha_raw as f32 - HA_AVG as f32) as f32 / HA_AMP as f32;
        let hb_norm: f32 = (hb_raw as f32 - HB_AVG as f32) as f32 / HB_AMP as f32;
        let hc_norm: f32 = (hc_raw as f32 - HC_AVG as f32) as f32 / HC_AMP as f32;

        let new_angle = tracker.update(ha_norm, hb_norm, hc_norm).unwrap();

        motor_cum_angle_mutex.lock(|cell| cell.set(new_angle));

        let finish_time = embassy_time::Instant::now();

        let deadline_time =
            ticker_initial_time + ticker_duration.checked_mul(iter_idx + 1).unwrap();

        let fail_anim = crate::anim::Pulse::new(
            color::palette::css::PURPLE.discard_alpha(),
            embassy_time::Duration::from_millis(0),
            embassy_time::Duration::from_millis(200),
            embassy_time::Duration::from_millis(400),
            embassy_time::Duration::from_millis(500),
            2,
        );

        let spare_time = if finish_time < deadline_time {
            (deadline_time - finish_time).as_micros() as i32 // On time
        } else {
            -((finish_time - deadline_time).as_micros() as i32) // Late
        };

        if spare_time < 0 {
            // Late
            led_command_ch
                .send(crate::rgb_led::Command::Transient(
                    crate::anim::Animation::Pulse(fail_anim),
                ))
                .await;
            defmt::error!("Motor update loop late by {}", &spare_time);
        }

        ticker.next().await;

        iter_idx += 1;
    }
}

fn wrapped_angle_sub(lhs: f32, rhs: f32) -> f32 {
    let rem_euclid = |lhs: f32, rhs: f32| -> f32 {
        let r = lhs % rhs;
        if r < 0.0 { r + rhs } else { r }
    };
    let raw_diff = lhs - rhs;
    rem_euclid(raw_diff + consts::PI, 2. * consts::PI) - consts::PI
}

pub struct HallAngleTracker {
    cum_rotations: i32,
    offset: fixed::types::I32F32,
    prev_wrapped_angle: Option<f32>,
}

impl HallAngleTracker {
    pub fn new() -> Self {
        Self {
            cum_rotations: 0,
            offset: 0.to_fixed::<I32F32>(),
            prev_wrapped_angle: None,
        }
    }

    pub fn update(
        &mut self,
        ha_norm: f32,
        hb_norm: f32,
        hc_norm: f32,
    ) -> Result<I32F32, &'static str> {
        // Quadrature may fail if the motor is spinning faster than 12,000 RPM
        // (which is faster than its free speed) or the motor quadrature tracker
        // loop isn't running at the required 3 kHz

        use crate::constants::SQRT_3;
        use consts::{PI, TAU};

        // Apply Clarke transformation
        fn clarke_trans(a: f32, b: f32, c: f32) -> f32 {
            let alpha = a;
            let beta = (b - c) / SQRT_3;
            let theta_clarke = libm::atan2f(beta, alpha);
            return theta_clarke;
        }

        // negate to make CCW motor rotation positive
        let new_wrapped_angle = -clarke_trans(ha_norm, hb_norm, hc_norm);

        // handle case of very first reading
        let Some(prev_wrapped_angle) = self.prev_wrapped_angle else {
            self.prev_wrapped_angle = Some(new_wrapped_angle);
            // set the offest so it starts at 0
            self.offset = -new_wrapped_angle.to_fixed::<I32F32>();
            return Ok(0.to_fixed());
        };

        // check if previous angle is within 90 degrees of current angle
        // if not, we can't safely continue tracking. The loop must be tighter
        let wrapped_diff = wrapped_angle_sub(new_wrapped_angle, prev_wrapped_angle);

        if wrapped_diff.abs() > consts::FRAC_PI_2 {
            return Err("Failed to track hall sensor commutation");
        }

        // now we can safely assume the previous angle is within 90 degrees of
        // the current one
        let delta_angle = new_wrapped_angle - prev_wrapped_angle;

        if delta_angle > PI {
            // must have gone from something like -170 to 170, which is
            // decreasing in angle
            self.cum_rotations -= 1;
        } else if delta_angle < -PI {
            // must have gone from something like 170 to -170, which is
            // increasing in angle
            self.cum_rotations += 1;
        }
        self.prev_wrapped_angle = Some(new_wrapped_angle);

        let tau_fixed: I32F32 = TAU.to_fixed::<I32F32>();
        let new_cum_angle = (self.cum_rotations.to_fixed::<I32F32>() * tau_fixed)
            + new_wrapped_angle.to_fixed::<I32F32>();

        Ok(new_cum_angle + self.offset)
    }

    pub fn reset(&mut self, init_angle: I32F32) {
        let Some(prev_wrapped_angle) = self.prev_wrapped_angle else {
            return;
        };
        let tau_fixed: I32F32 = consts::TAU.to_fixed::<I32F32>();
        let cum_angle = (self.cum_rotations.to_fixed::<I32F32>() * tau_fixed)
            + prev_wrapped_angle.to_fixed::<I32F32>();
        self.offset = init_angle - cum_angle;
    }

    pub fn zero(&mut self) {
        self.reset(0.to_fixed());
    }
}
