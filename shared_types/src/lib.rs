#![no_std]

use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(
    feature = "postcard-bindgen",
    derive(postcard_bindgen::PostcardBindings)
)]
pub enum CmdFromPC {
    Disable,
    Heartbeat,
    PIDSet(f32, f32, f32),
    PIDGet,
    Enable,
}

#[derive(Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(
    feature = "postcard-bindgen",
    derive(postcard_bindgen::PostcardBindings)
)]
pub enum TelemToPC {
    MotorPosition(f32),
}

#[derive(Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(
    feature = "postcard-bindgen",
    derive(postcard_bindgen::PostcardBindings)
)]
pub enum TelemFromPC {
    MotorPositionSetpoint(f32),
    MotorSpeedSetpoint(f32),
}
