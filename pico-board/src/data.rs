use core::cell::Cell;
use fixed::types::I32F32;

// TODO: consider moving all these to the main.rs file so access can be controlled

use embassy_sync::{
    blocking_mutex::{self, Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
    signal::Signal,
};

use crate::{motor_control::MotorCommand, motor_quadrature::QuadratureError, rgb_led};

/// Indicates current position of the motor as measured by the hall effect
/// sensor. Updated by motor_quadrature
pub static MOTOR_CURRENT_POSITION: Mutex<CriticalSectionRawMutex, Cell<I32F32>> =
    blocking_mutex::Mutex::new(Cell::new(I32F32::const_from_int(0)));

pub static QUADRATURE_ERROR_SIGNAL: Signal<CriticalSectionRawMutex, QuadratureError> =
    Signal::new();

/// Used to send commands to the motor control loop. Commands are awaited in motor_control.
/// For example, you may command the motor to go to position of 100 radians, wait for it
/// to get there, then zero the position, then command it to go to position 0 radians.
pub static MOTOR_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, 16> =
    Channel::new();

pub static LED_COMMAND_CH: Channel<
    CriticalSectionRawMutex,
    rgb_led::Command,
    16, // should be processed instantly but just in case
> = Channel::new();

pub static MOTOR_POSITION_SETPOINT: blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<I32F32>> =
    blocking_mutex::Mutex::new(Cell::new(I32F32::ZERO));

pub static MOTOR_SPEED_SETPOINT: blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<f32>> =
    blocking_mutex::Mutex::new(Cell::new(0.));
