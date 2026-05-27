pub enum Animation {
    Off,
    Solid(Solid),
    Pulse(Pulse),
    FadeInFadeOut(FadeInFadeOut),
    Rainbow(Rainbow),
}

use color::{OpaqueColor, Srgb};

impl Animation {
    pub fn duration(&self) -> embassy_time::Duration {
        match self {
            Self::Off => embassy_time::Duration::MAX,
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

pub struct Solid {
    color: OpaqueColor<Srgb>,
    duration: embassy_time::Duration,
}

pub struct Pulse {
    color: OpaqueColor<Srgb>,
    initial_delay: embassy_time::Duration,
    final_delay: embassy_time::Duration,
    period: embassy_time::Duration,
    num_pulses: u32,
    on_width: embassy_time::Duration,
    off_width: embassy_time::Duration,
    duration: embassy_time::Duration,
}

pub struct FadeInFadeOut {
    color: color::OpaqueColor<Srgb>,
    duration: embassy_time::Duration,
}

pub struct Rainbow {
    duration: embassy_time::Duration,
}

impl Solid {
    pub fn new(color: OpaqueColor<Srgb>, duration: embassy_time::Duration) -> Self {
        defmt::assert!(duration.as_ticks() >= 1);
        Solid {
            color: color,
            duration: duration,
        }
    }
}

impl Pulse {
    pub fn new(
        color: OpaqueColor<Srgb>,
        initial_delay: embassy_time::Duration,
        pulse_width: embassy_time::Duration,
        period: embassy_time::Duration,
        final_delay: embassy_time::Duration,
        num_pulses: u32,
    ) -> Self {
        defmt::assert!(period.as_ticks() >= 1);
        defmt::assert!(pulse_width <= period);
        defmt::assert!(num_pulses >= 1);

        let off_width = period - pulse_width;
        let duration =
            initial_delay + (num_pulses * pulse_width) + (num_pulses - 1) * off_width + final_delay;

        Pulse {
            color: color,
            initial_delay: initial_delay,
            final_delay: final_delay,
            period: period,
            num_pulses: num_pulses,
            on_width: pulse_width,
            off_width: off_width,
            duration: duration,
        }
    }
}

impl FadeInFadeOut {
    pub fn new(color: color::OpaqueColor<Srgb>, duration: embassy_time::Duration) -> Self {
        defmt::assert!(duration.as_ticks() >= 1);

        Self {
            color: color,
            duration: duration,
        }
    }
}

impl Rainbow {
    pub fn new(duration: embassy_time::Duration) -> Self {
        defmt::assert!(duration.as_ticks() >= 1);

        Self { duration: duration }
    }
}
