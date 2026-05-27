use embassy_rp::pwm;
use fixed::traits::ToFixed;

/// LED "tick" rate, which is the rate at which an LED animation is played at in Hz.
/// This should be fast enough to prevent visual flicker, but slow enough to prevent
/// consuming too much CPU time.
pub const LED_TICK_PERIOD: embassy_time::Duration = embassy_time::Duration::from_hz(100);

pub enum Command {
    Transient(crate::anim::Animation),
    Looping(crate::anim::Animation),
}

/// r, g, b are values from 0 to 1 where 0.0 is fully off and 1.0 is maximum brightness
pub fn set_rgb<'b>(
    led_red_a: &mut pwm::Pwm<'b>,
    led_green_a_blue_b: &mut pwm::Pwm<'b>,
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

    let r = f32::max(f32::min(r, 1.0), 0.0);
    let g = f32::max(f32::min(g, 1.0), 0.0);
    let b = f32::max(f32::min(b, 1.0), 0.0);
    let r_duty = (62499.0 * r * r) as u16;
    let g_duty = (62499.0 * g * g) as u16;
    let b_duty = (62499.0 * b * b) as u16;

    let mut led_red_a_config = pwm::Config::default();
    led_red_a_config.invert_a = true;
    led_red_a_config.phase_correct = false;
    led_red_a_config.enable = true;
    led_red_a_config.divider = divider;
    led_red_a_config.compare_a = r_duty;
    led_red_a_config.top = DUTY_MAX;

    let mut led_green_a_blue_b_config = pwm::Config::default();
    led_green_a_blue_b_config.invert_a = true;
    led_green_a_blue_b_config.invert_b = true;
    led_green_a_blue_b_config.phase_correct = false;
    led_green_a_blue_b_config.enable = true;
    led_green_a_blue_b_config.divider = divider;
    led_green_a_blue_b_config.compare_a = g_duty;
    led_green_a_blue_b_config.compare_b = b_duty;

    led_red_a.set_config(&led_red_a_config);
    led_green_a_blue_b.set_config(&led_green_a_blue_b_config);
}
