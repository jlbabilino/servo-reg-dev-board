use core::cell::Cell;

use color::OpaqueColor;
use embassy_sync::{
    blocking_mutex::{self, raw::CriticalSectionRawMutex},
    pubsub, watch,
};
use fixed::types::I32F32;

use crate::{
    motor_control::MotorCommand,
    motor_quadrature::{QuadratureCommand, QuadratureError},
    network::{CmdFromPC, NetworkStatus},
    rgb_led,
};

pub type NetworkStatusWatch = watch::Watch<CriticalSectionRawMutex, NetworkStatus, 4>;
pub type NetworkStatusWatchSender =
    watch::Sender<'static, CriticalSectionRawMutex, NetworkStatus, 4>;
pub type NetworkStatusWatchReceiver =
    watch::Receiver<'static, CriticalSectionRawMutex, NetworkStatus, 4>;

pub type CMDFromPCPubSub = pubsub::PubSubChannel<CriticalSectionRawMutex, CmdFromPC, 16, 4, 4>;
pub type CMDFromPCPublisher =
    pubsub::Publisher<'static, CriticalSectionRawMutex, CmdFromPC, 16, 4, 4>;
pub type CMDFromPCSubscriber =
    pubsub::Subscriber<'static, CriticalSectionRawMutex, CmdFromPC, 16, 4, 4>;

pub type I32F32Mutex = blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<I32F32>>;
pub type F32Mutex = blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<f32>>;

pub type QuadratureErrorWatch = watch::Watch<CriticalSectionRawMutex, QuadratureError, 4>;
pub type QuadratureErrorWatchSender =
    watch::Sender<'static, CriticalSectionRawMutex, QuadratureError, 4>;
pub type QuadratureErrorWatchReceiver =
    watch::Receiver<'static, CriticalSectionRawMutex, QuadratureError, 4>;

pub type QuadratureCommandWatch = watch::Watch<CriticalSectionRawMutex, QuadratureCommand, 4>;
pub type QuadratureCommandWatchSender =
    watch::Sender<'static, CriticalSectionRawMutex, QuadratureCommand, 4>;
pub type QuadratureCommandWatchReceiver =
    watch::Receiver<'static, CriticalSectionRawMutex, QuadratureCommand, 4>;

pub type MotorCommandPubSub =
    pubsub::PubSubChannel<CriticalSectionRawMutex, MotorCommand, 16, 4, 4>;
pub type MotorCommandPublisher =
    pubsub::Publisher<'static, CriticalSectionRawMutex, MotorCommand, 16, 4, 4>;
pub type MotorCommandSubscriber =
    pubsub::Subscriber<'static, CriticalSectionRawMutex, MotorCommand, 16, 4, 4>;

pub type LEDCommandPubSub =
    pubsub::PubSubChannel<CriticalSectionRawMutex, rgb_led::Command, 16, 4, 4>;
pub type LEDCommandPublisher =
    pubsub::Publisher<'static, CriticalSectionRawMutex, rgb_led::Command, 16, 4, 4>;
pub type LEDCommandSubscriber =
    pubsub::Subscriber<'static, CriticalSectionRawMutex, rgb_led::Command, 16, 4, 4>;

pub type ButtonWatch = watch::Watch<CriticalSectionRawMutex, bool, 4>;
pub type ButtonWatchSender = watch::Sender<'static, CriticalSectionRawMutex, bool, 4>;
pub type ButtonWatchReceiver = watch::Receiver<'static, CriticalSectionRawMutex, bool, 4>;
