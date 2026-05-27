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
