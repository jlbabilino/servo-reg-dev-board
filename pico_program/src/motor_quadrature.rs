use core::f32::consts;
use embassy_futures::select::{Either, select};
use embassy_rp::adc;
use embassy_time::{Duration, Instant};
use fixed::traits::ToFixed;
use fixed::types::I32F32;

use crate::types::{I32F32Mutex, QuadratureCommandWatchReceiver, QuadratureErrorWatchSender};

#[derive(Copy, Clone, defmt::Format)]
pub struct QuadratureError {}

#[derive(Copy, Clone, defmt::Format)]
pub enum QuadratureCommand {
    #[allow(unused)]
    Zero, // TODO: add feature in manual mode to call this command
    ResetAt(I32F32),
}

#[embassy_executor::task]
pub async fn motor_quadrature_task(
    mut adc: adc::Adc<'static, adc::Blocking>,
    mut hall_a_pin: adc::Channel<'static>,
    mut hall_b_pin: adc::Channel<'static>,
    mut hall_c_pin: adc::Channel<'static>,
    motor_current_position: &'static I32F32Mutex,
    quadrature_error_sender: QuadratureErrorWatchSender,
    mut quadrature_command_receiver: QuadratureCommandWatchReceiver,
) {
    let mut quadrature_loop = async |tracker: &mut HallAngleTracker| -> Result<(), &'static str> {
        const QUADRATURE_LOOP_RATE: Duration = Duration::from_hz(3000);
        let loop_init_time = embassy_time::Instant::now();
        let mut ticker = embassy_time::Ticker::every(QUADRATURE_LOOP_RATE);
        ticker.reset_at(loop_init_time);
        let mut iter_idx: u32 = 0;

        let mut late_timestamp_opt: Option<Instant> = None;

        loop {
            use crate::constants::{HA_AMP, HA_AVG, HB_AMP, HB_AVG, HC_AMP, HC_AVG};

            let ha_raw = adc
                .blocking_read(&mut hall_a_pin)
                .map_err(|_| "Failed to read hall sensor channel a")?;
            let hb_raw = adc
                .blocking_read(&mut hall_b_pin)
                .map_err(|_| "Failed to read hall sensor channel b")?;
            let hc_raw = adc
                .blocking_read(&mut hall_c_pin)
                .map_err(|_| "Failed to read hall sensor channel c")?;

            let ha_norm: f32 = (ha_raw as f32 - HA_AVG as f32) / HA_AMP as f32;
            let hb_norm: f32 = (hb_raw as f32 - HB_AVG as f32) / HB_AMP as f32;
            let hc_norm: f32 = (hc_raw as f32 - HC_AVG as f32) / HC_AMP as f32;
            // defmt::info!("ha = {}", ha_raw);
            // defmt::info!("hb = {}", hb_raw);
            // defmt::info!("hc = {}", hc_raw);

            let new_angle = tracker.update(ha_norm, hb_norm, hc_norm)?;

            motor_current_position.lock(|cell| cell.set(new_angle));

            let finish_time = embassy_time::Instant::now();

            if let Some(relative_deadline_duration) = QUADRATURE_LOOP_RATE.checked_mul(iter_idx + 1)
            {
                let deadline_time = loop_init_time + relative_deadline_duration;

                let spare_time = if finish_time < deadline_time {
                    (deadline_time - finish_time).as_micros() as i32 // On time
                } else {
                    -((finish_time - deadline_time).as_micros() as i32) // Late
                };

                // Timer so it doesn't spam the "motor update loop late" warning
                if let Some(late_timestamp) = late_timestamp_opt
                    && Instant::now() - late_timestamp > Duration::from_secs(1)
                {
                    late_timestamp_opt = None;
                }

                if spare_time < 0 && late_timestamp_opt.is_none() {
                    // Late
                    defmt::warn!("Motor update loop late by {}", &spare_time);
                    late_timestamp_opt = Some(Instant::now());
                }
            } else {
                defmt::error!("Failed to calculate if quadrature loop was late");
            }

            ticker.next().await;
            iter_idx += 1;
        }
    };

    let mut tracker = HallAngleTracker::new(I32F32::ZERO);
    loop {
        match select(
            quadrature_command_receiver.changed(),
            quadrature_loop(&mut tracker),
        )
        .await
        {
            Either::First(quadrature_command) => match quadrature_command {
                QuadratureCommand::Zero => {
                    tracker = HallAngleTracker::new(I32F32::ZERO);
                }
                QuadratureCommand::ResetAt(init_pos) => {
                    tracker = HallAngleTracker::new(init_pos);
                }
            },
            Either::Second(Ok(_)) => {
                defmt::error!("Quadrature loop should never end, check code!");
            }
            Either::Second(Err(msg)) => {
                defmt::error!("Quadrature error: {}", msg);
                quadrature_error_sender.send(QuadratureError {});
                // Reset tracking after error
                tracker = HallAngleTracker::new(I32F32::ZERO);
            }
        };
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

/// Keep track of state of hall sensor rotation tracking. [`init_angle`]
/// assumed to be the angle at the start of tracking. Subsequent polls of the
/// [`update()`] function will determine the angle with the following formula:
/// `cum_angle = wrapped_angle + cum_rotations*2*pi + offset`
pub struct HallAngleTracker {
    cum_rotations: i32,
    offset: fixed::types::I32F32,
    prev_wrapped_angle: Option<f32>,
    init_angle: I32F32,
}

impl HallAngleTracker {
    pub fn new(init_angle: I32F32) -> Self {
        Self {
            cum_rotations: 0,
            offset: 0.to_fixed::<I32F32>(),
            prev_wrapped_angle: None,
            init_angle: init_angle,
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
            libm::atan2f(beta, alpha)
        }

        // negate to make CCW motor rotation positive
        let new_wrapped_angle = -clarke_trans(ha_norm, hb_norm, hc_norm);

        // handle case of very first reading
        let Some(prev_wrapped_angle) = self.prev_wrapped_angle else {
            self.prev_wrapped_angle = Some(new_wrapped_angle);
            // set the offest so it starts at init_angle
            self.offset = self.init_angle - new_wrapped_angle.to_fixed::<I32F32>();
            return Ok(self.init_angle);
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

        Ok(new_wrapped_angle.to_fixed::<I32F32>()
            + self.cum_rotations.to_fixed::<I32F32>() * TAU.to_fixed::<I32F32>()
            + self.offset)
    }

    // pub fn reset(&mut self, init_angle: I32F32) {
    //     let Some(prev_wrapped_angle) = self.prev_wrapped_angle else {
    //         return;
    //     };
    //     let tau_fixed: I32F32 = consts::TAU.to_fixed::<I32F32>();
    //     let cum_angle = (self.cum_rotations.to_fixed::<I32F32>() * tau_fixed)
    //         + prev_wrapped_angle.to_fixed::<I32F32>();
    //     self.offset = init_angle - cum_angle;
    // }

    // pub fn zero(&mut self) {
    //     self.reset(0.to_fixed());
    // }
}
