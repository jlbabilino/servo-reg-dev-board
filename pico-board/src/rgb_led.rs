use embassy_futures::select::Either;
use embassy_rp::pwm;
use fixed::traits::ToFixed;

use crate::data::LED_COMMAND_CH;

/// LED "tick" rate, which is the rate at which an LED animation is played at in Hz.
/// This should be fast enough to prevent visual flicker, but slow enough to prevent
/// consuming too much CPU time.
pub const LED_TICK_PERIOD: embassy_time::Duration = embassy_time::Duration::from_hz(100);

#[derive(Debug, Copy, Clone)]
pub enum Command {
    Transient(crate::anim::Animation),
    Looping(crate::anim::Animation),
}

/// r, g, b are values from 0 to 1 where 0.0 is fully off and 1.0 is maximum brightness
pub fn set_rgb<'b>(
    led_green_a: &mut pwm::Pwm<'b>,
    led_red_a_blue_b: &mut pwm::Pwm<'b>,
    rgb: color::OpaqueColor<color::Srgb>,
) {
    let color_components = rgb.components;
    let r = color_components[0];
    let g = color_components[1];
    let b = color_components[2];

    const DUTY_MAX: u16 = 62499;
    let divider: fixed::FixedU16<fixed::types::extra::U4> = 2.to_fixed();
    // If this DUTY_MAX is pwm `top` value, gives
    // period = (62499 + 1) * 2 = 125k cycles
    // RP2350 runs at 125 MHz -> freq = 125M / 125k = 1 kHz
    // This is a good frequency for LED dimming since
    // it's fast enough to not look like it's flashing

    const R_SCALE: f32 = 1.0;
    const G_SCALE: f32 = 0.65;
    const B_SCALE: f32 = 0.7;
    const ALL_SCALE: f32 = 0.1;

    let r = f32::max(f32::min(r, 1.0), 0.0);
    let g = f32::max(f32::min(g, 1.0), 0.0);
    let b = f32::max(f32::min(b, 1.0), 0.0);
    let r_duty = (62499.0 * r * r * R_SCALE * ALL_SCALE) as u16;
    let g_duty = (62499.0 * g * g * G_SCALE * ALL_SCALE) as u16;
    let b_duty = (62499.0 * b * b * B_SCALE * ALL_SCALE) as u16;

    let mut led_green_a_config = pwm::Config::default();
    led_green_a_config.invert_a = false;
    led_green_a_config.phase_correct = false;
    led_green_a_config.enable = true;
    led_green_a_config.divider = divider;
    led_green_a_config.compare_a = g_duty;
    led_green_a_config.top = DUTY_MAX;

    let mut led_red_a_blue_b_config = pwm::Config::default();
    led_red_a_blue_b_config.invert_a = false;
    led_red_a_blue_b_config.invert_b = false;
    led_red_a_blue_b_config.phase_correct = false;
    led_red_a_blue_b_config.enable = true;
    led_red_a_blue_b_config.divider = divider;
    led_red_a_blue_b_config.compare_a = r_duty;
    led_red_a_blue_b_config.compare_b = b_duty;

    led_green_a.set_config(&led_green_a_config);
    led_red_a_blue_b.set_config(&led_red_a_blue_b_config);
}

#[embassy_executor::task]
pub async fn led_driver_task(
    mut led_green_a: pwm::Pwm<'static>,
    mut led_red_a_blue_b: pwm::Pwm<'static>,
) {
    let mut led_pwm_update_loop =
        async |curr_anim: &crate::anim::Animation,
               is_loop: bool,
               t_anim_start: Option<embassy_time::Instant>| {
            let t_anim_start = match t_anim_start {
                Some(instant) => instant,
                None => embassy_time::Instant::now(),
            };
            let mut led_ticker = embassy_time::Ticker::every(LED_TICK_PERIOD);
            led_ticker.reset_at(t_anim_start);
            let deadline = t_anim_start + curr_anim.duration();

            loop {
                let curr_time = embassy_time::Instant::now();
                if !is_loop && curr_time >= deadline {
                    break;
                }
                let t_rel = curr_time - t_anim_start;
                let color = curr_anim.eval(t_rel);
                set_rgb(&mut led_green_a, &mut led_red_a_blue_b, color);
                led_ticker.next().await;
            }
        };

    let mut looping_anim = crate::anim::Animation::Off;
    // For transient anim, keep track of start time as an Instant
    // That way if a new looping command fires, it won't reset the transient
    // animation's timer
    let mut transient_anim: Option<(crate::anim::Animation, embassy_time::Instant)> = None;
    loop {
        match embassy_futures::select::select(
            LED_COMMAND_CH.receive(),
            match transient_anim {
                // play a transient animation with given start time
                Some(ref value) => led_pwm_update_loop(&value.0, false, Some(value.1)),
                // play a looping animation, and start it now (None)
                None => led_pwm_update_loop(&looping_anim, true, None),
            },
        )
        .await
        {
            Either::First(command) => {
                match command {
                    Command::Transient(new_transient_anim) => {
                        // defmt::debug!("New transient animation");
                        transient_anim = Some((new_transient_anim, embassy_time::Instant::now()));
                    }
                    Command::Looping(new_looping_animation) => {
                        // defmt::debug!("New looping animation");
                        looping_anim = new_looping_animation;
                    }
                };
            }
            Either::Second(_) => {
                // the looping animation will never end, so this must be a
                // transient animation ending. So just pop the transient
                // animation
                transient_anim = None;
            }
        }
    }
}
