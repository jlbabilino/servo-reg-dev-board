#[derive(Debug, Copy, Clone)]
pub enum Animation {
    Off,
    Solid(Solid),
    Pulse(Pulse),
    FadeInFadeOut(FadeInFadeOut),
    Rainbow(Rainbow),
}

use color::{OpaqueColor, Srgb};

use crate::util::{const_checked_add, const_checked_mul, const_checked_sub};

trait ExtFormat: defmt::Format {}

impl Animation {
    pub fn duration(&self) -> embassy_time::Duration {
        match self {
            Self::Off => embassy_time::Duration::from_secs(100000),
            Self::Solid(solid) => solid.duration,
            Self::Pulse(pulse) => pulse.duration,
            Self::FadeInFadeOut(fade_in_fade_out) => fade_in_fade_out.duration,
            Self::Rainbow(rainbow) => rainbow.duration,
        }
    }
    pub fn eval(&self, t: embassy_time::Duration) -> color::OpaqueColor<Srgb> {
        let t = crate::util::rem(t, self.duration());
        match self {
            Self::Off => color::palette::css::BLACK.discard_alpha(),
            Self::Solid(solid) => solid.color,
            Self::Pulse(pulse) => {
                if t < pulse.initial_delay || t >= (pulse.duration - pulse.final_delay) {
                    color::palette::css::BLACK.discard_alpha()
                } else {
                    let t_prime = t - pulse.initial_delay;
                    let t_rel = crate::util::rem(t_prime, pulse.period);
                    if t_rel < pulse.on_width {
                        pulse.color
                    } else {
                        color::palette::css::BLACK.discard_alpha()
                    }
                }
            }
            Self::FadeInFadeOut(fade_in_fade_out) => {
                let t_norm = crate::util::div(t, fade_in_fade_out.duration);
                let brightness = if t_norm < 0.5 {
                    2. * t_norm
                } else {
                    2. * (1. - t_norm)
                };
                fade_in_fade_out.color * brightness
            }
            Self::Rainbow(rainbow) => {
                let t_norm = crate::util::div(t, rainbow.duration);
                let hue = 360. * t_norm;
                let new_color: color::AlphaColor<color::Hsl> =
                    color::AlphaColor::new([hue, 100., 50., 1.]);

                new_color.convert().discard_alpha() / 2. // the / 2 is just to make it dimmer cuz my eyes hurt
            }
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Solid {
    color: OpaqueColor<Srgb>,
    duration: embassy_time::Duration,
}

#[derive(Debug, Copy, Clone)]
pub struct Pulse {
    color: OpaqueColor<Srgb>,
    initial_delay: embassy_time::Duration,
    final_delay: embassy_time::Duration,
    period: embassy_time::Duration,
    on_width: embassy_time::Duration,
    duration: embassy_time::Duration,
}

#[derive(Debug, Copy, Clone)]
pub struct FadeInFadeOut {
    color: color::OpaqueColor<Srgb>,
    duration: embassy_time::Duration,
}

#[derive(Debug, Copy, Clone)]
pub struct Rainbow {
    duration: embassy_time::Duration,
}

impl Solid {
    pub const fn new(color: OpaqueColor<Srgb>, duration: embassy_time::Duration) -> Self {
        core::assert!(duration.as_ticks() >= 1);
        Solid {
            color: color,
            duration: duration,
        }
    }
}

impl Pulse {
    pub const fn new(
        color: OpaqueColor<Srgb>,
        initial_delay: embassy_time::Duration,
        pulse_width: embassy_time::Duration,
        period: embassy_time::Duration,
        final_delay: embassy_time::Duration,
        num_pulses: u64,
    ) -> Self {
        core::assert!(period.as_ticks() >= 1);
        core::assert!(pulse_width.as_ticks() <= period.as_ticks());
        core::assert!(num_pulses >= 1);

        let off_width = const_checked_sub(period, pulse_width).unwrap();
        let on_total_duration = const_checked_mul(pulse_width, num_pulses).unwrap();
        let off_total_duration = const_checked_mul(off_width, num_pulses - 1).unwrap();

        // duration = initial_delay + on_total_duration + off_total_duration + final_delay
        let delays_total = const_checked_add(initial_delay, final_delay).unwrap();
        let on_off_duration = const_checked_add(on_total_duration, off_total_duration).unwrap();
        let duration = const_checked_add(on_off_duration, delays_total).unwrap();

        Pulse {
            color: color,
            initial_delay: initial_delay,
            final_delay: final_delay,
            period: period,
            on_width: pulse_width,
            duration: duration,
        }
    }
}

impl FadeInFadeOut {
    pub const fn new(color: color::OpaqueColor<Srgb>, duration: embassy_time::Duration) -> Self {
        core::assert!(duration.as_ticks() >= 1);

        Self {
            color: color,
            duration: duration,
        }
    }
}

impl Rainbow {
    pub const fn new(duration: embassy_time::Duration) -> Self {
        core::assert!(duration.as_ticks() >= 1);

        Self { duration: duration }
    }
}
