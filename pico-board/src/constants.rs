use core::f32::consts::PI;

pub const HA_MAX: u16 = 3587;
pub const HB_MAX: u16 = 3618;
pub const HC_MAX: u16 = 3571;

pub const HA_MIN: u16 = 589;
pub const HB_MIN: u16 = 537;
pub const HC_MIN: u16 = 543;

pub const HA_AVG: u16 = (HA_MIN + HA_MAX) / 2;
pub const HB_AVG: u16 = (HB_MIN + HB_MAX) / 2;
pub const HC_AVG: u16 = (HC_MIN + HC_MAX) / 2;

pub const HA_AMP: u16 = (HA_MAX - HA_MIN) / 2;
pub const HB_AMP: u16 = (HB_MAX - HB_MIN) / 2;
pub const HC_AMP: u16 = (HC_MAX - HC_MIN) / 2;

pub const SQRT_3: f32 = 1.732050807568877293527446341505872367_f32;

pub const HEARTBEAT_MAX_ALLOWED: embassy_time::Duration = embassy_time::Duration::from_millis(500);
pub const HEARTBEAT_BYTE: u8 = 0x42;

pub const DISCONNECTED_DISABLED_ANIM: crate::anim::Animation = crate::anim::Animation::Rainbow(
    crate::anim::Rainbow::new(embassy_time::Duration::from_secs(2)),
);

pub const CONNECTED_DISABLED_ANIM: crate::anim::Animation =
    crate::anim::Animation::FadeInFadeOut(crate::anim::FadeInFadeOut::new(
        color::palette::css::BLUE.discard_alpha(),
        embassy_time::Duration::from_secs(3),
    ));

pub const NETWORK_ENABLED_ANIM: crate::anim::Animation =
    crate::anim::Animation::Pulse(crate::anim::Pulse::new(
        color::palette::css::RED.discard_alpha(),
        embassy_time::Duration::from_millis(50),
        embassy_time::Duration::from_millis(50),
        embassy_time::Duration::from_millis(100),
        embassy_time::Duration::from_millis(200),
        2,
    ));

pub const MANUAL_ENABLED_ANIM: crate::anim::Animation =
    crate::anim::Animation::Pulse(crate::anim::Pulse::new(
        color::palette::css::MAGENTA.discard_alpha(),
        embassy_time::Duration::from_millis(50),
        embassy_time::Duration::from_millis(50),
        embassy_time::Duration::from_millis(100),
        embassy_time::Duration::from_millis(200),
        2,
    ));

pub const MANUAL_MODE_1_ANIM: crate::anim::Animation =
    crate::anim::Animation::Pulse(crate::anim::Pulse::new(
        color::palette::css::GREEN.discard_alpha(), // color
        embassy_time::Duration::from_millis(300),   // initial delay
        embassy_time::Duration::from_millis(100),   // pulse width
        embassy_time::Duration::from_millis(200),   // period
        embassy_time::Duration::from_millis(700),   // final delay
        1,                                          // num of pulses
    ));

pub const MANUAL_MODE_2_ANIM: crate::anim::Animation =
    crate::anim::Animation::Pulse(crate::anim::Pulse::new(
        color::palette::css::GREEN.discard_alpha(), // color
        embassy_time::Duration::from_millis(300),   // initial delay
        embassy_time::Duration::from_millis(100),   // pulse width
        embassy_time::Duration::from_millis(200),   // period
        embassy_time::Duration::from_millis(500),   // final delay
        2,                                          // num of pulses
    ));
