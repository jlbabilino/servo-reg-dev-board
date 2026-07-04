//  ________  ________  ________        ___       ___  ________  ___  ___  ___  ________  ________
// |\   __  \|\   ____\|\   __  \      |\  \     |\  \|\   __  \|\  \|\  \|\  \|\   ___ \|\   ____\
// \ \  \|\  \ \  \___|\ \  \|\  \     \ \  \    \ \  \ \  \|\  \ \  \\\  \ \  \ \  \_|\ \ \  \___|_
//  \ \   ____\ \_____  \ \   ____\     \ \  \    \ \  \ \  \\\  \ \  \\\  \ \  \ \  \ \\ \ \_____  \
//   \ \  \___|\|____|\  \ \  \___|      \ \  \____\ \  \ \  \\\  \ \  \\\  \ \  \ \  \_\\ \|____|\  \
//    \ \__\     ____\_\  \ \__\          \ \_______\ \__\ \_____  \ \_______\ \__\ \_______\____\_\  \
//     \|__|    |\_________\|__|           \|_______|\|__|\|___| \__\|_______|\|__|\|_______|\_________\
//              \|_________|                                    \|__|                       \|_________|
//
// Justin Babilino

#![no_std]
#![no_main]
// #![deny(clippy::unwrap_used)]
// #![deny(clippy::expect_used)]
#![deny(clippy::panic)]

mod anim;
mod buttons;
mod constants;
mod motor_control;
mod motor_quadrature;
mod network;
mod rgb_led;
mod util;

use core::cell::Cell;

use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::peripherals::DMA_CH1;
use embassy_rp::pwm;
use embassy_rp::spi;
use embassy_sync::blocking_mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel;
use embassy_sync::watch::Watch;
use embedded_hal_bus::spi::ExclusiveDevice;
use fixed::types::I32F32;
use w5500_ll::eh1::vdm::W5500;

use embassy_rp::adc;
use embassy_rp::gpio;

use crate::motor_control::MotorCommand;
use crate::motor_quadrature::QuadratureCommand;
use crate::motor_quadrature::QuadratureError;
use crate::network::CmdFromPC;
use crate::network::NetworkStatus;

use {defmt_rtt as _, panic_probe as _};

defmt::timestamp!("[t = {=u64:us} s]", {
    embassy_time::Instant::now().as_micros()
});

// enum OverallState {
//     Disconnected,
//     Disabled,
//     Enabled,
// }

embassy_rp::bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>, embassy_rp::dma::InterruptHandler<DMA_CH1>;
});

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    // ======================== MAP PICO GPIO PINS ===========================
    let led_green_a = pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_18, Default::default());
    let led_red_a_blue_b =
        pwm::Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_16, p.PIN_17, Default::default());

    let adc = adc::Adc::new_blocking(p.ADC, Default::default());

    let hall_a_pin = adc::Channel::new_pin(p.PIN_28, gpio::Pull::None);
    let hall_b_pin = adc::Channel::new_pin(p.PIN_27, gpio::Pull::None);
    let hall_c_pin = adc::Channel::new_pin(p.PIN_26, gpio::Pull::None);

    let esc_stop_pin = gpio::OutputOpenDrain::new(p.PIN_20, gpio::Level::High);
    let esc_brake_pin = gpio::OutputOpenDrain::new(p.PIN_19, gpio::Level::High);
    let esc_dir_pin = gpio::OutputOpenDrain::new(p.PIN_21, gpio::Level::High);

    let esc_pwm_config = motor_control::pwm_config();
    let esc_pwm = pwm::Pwm::new_output_a(p.PWM_SLICE3, p.PIN_22, esc_pwm_config);

    // let button_1 = gpio::Input::new(p.PIN_9, gpio::Pull::Up);
    // let button_2 = gpio::Input::new(p.PIN_8, gpio::Pull::Up);
    // let button_3 = gpio::Input::new(p.PIN_7, gpio::Pull::Up);
    // let button_4 = gpio::Input::new(p.PIN_6, gpio::Pull::Up);

    // Initialize W5500 ethernet module
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 50_000_000;
    let (eth_miso, eth_mosi, eth_clk) = (p.PIN_12, p.PIN_11, p.PIN_10);
    let eth_spi = spi::Spi::new(
        p.SPI1, eth_clk, eth_mosi, eth_miso, p.DMA_CH0, p.DMA_CH1, Irqs, spi_cfg,
    );
    let cs = gpio::Output::new(p.PIN_13, gpio::Level::High);
    let w5500_int = gpio::Input::new(p.PIN_14, gpio::Pull::Up);
    let w5500 = W5500::new(ExclusiveDevice::new(eth_spi, cs, embassy_time::Delay).unwrap());

    // ======================= COMMUNICATION CHANNELS =========================

    static NETWORK_STATUS_IND: Watch<CriticalSectionRawMutex, NetworkStatus, 4> = Watch::new();
    static NETWORK_CMD_FROM_PC_CH: channel::Channel<CriticalSectionRawMutex, CmdFromPC, 16> =
        channel::Channel::new();

    /// Indicates current position of the motor as measured by the hall effect
    /// sensor. Updated by motor_quadrature
    static MOTOR_CURRENT_POSITION: blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<I32F32>> =
        blocking_mutex::Mutex::new(Cell::new(I32F32::ZERO));

    static QUADRATURE_ERROR_WATCH: Watch<CriticalSectionRawMutex, QuadratureError, 4> =
        Watch::new();

    static QUADRATURE_COMMAND_WATCH: Watch<CriticalSectionRawMutex, QuadratureCommand, 4> =
        Watch::new();

    /// Used to send commands to the motor control loop. Commands are awaited in motor_control.
    /// For example, you may command the motor to go to position of 100 radians, wait for it
    /// to get there, then zero the position, then command it to go to position 0 radians.
    static MOTOR_COMMAND_CHANNEL: channel::Channel<CriticalSectionRawMutex, MotorCommand, 16> =
        channel::Channel::new();

    static MOTOR_POSITION_SETPOINT: blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<I32F32>> =
        blocking_mutex::Mutex::new(Cell::new(I32F32::ZERO));

    static MOTOR_SPEED_SETPOINT: blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<f32>> =
        blocking_mutex::Mutex::new(Cell::new(0.));

    pub static LED_COMMAND_CH: channel::Channel<
        CriticalSectionRawMutex,
        rgb_led::Command,
        16, // should be processed instantly but just in case
    > = channel::Channel::new();

    let Some(quadrature_command_receiver) = QUADRATURE_COMMAND_WATCH.receiver() else {
        defmt::error!("Failed to create quadrature command receiver!");
        return;
    };

    // ============================= SPAWN TASKS ==============================

    // A high-frequency loop (~3 kHz) to track the motor's rotation
    // Modifies a mutex to set share the current rotation angle with other tasks
    let Ok(motor_quadrature_task_token) = motor_quadrature::motor_quadrature_task(
        adc,
        hall_a_pin,
        hall_b_pin,
        hall_c_pin,
        &MOTOR_CURRENT_POSITION,
        QUADRATURE_ERROR_WATCH.sender(),
        quadrature_command_receiver,
    ) else {
        defmt::error!("Failed to spawn motor quadrature task!");
        return;
    };

    // Accepts MotorCommand requests to drive the motor to a ceratin speed or
    // position. Can also disable or brake the motor using certain pins.
    let Ok(motor_control_task_token) = motor_control::motor_control_task(
        esc_stop_pin,
        esc_brake_pin,
        esc_dir_pin,
        esc_pwm,
        &MOTOR_CURRENT_POSITION,
        &MOTOR_POSITION_SETPOINT,
        &MOTOR_SPEED_SETPOINT,
        MOTOR_COMMAND_CHANNEL.receiver(),
    ) else {
        defmt::error!("Failed to spawn motor control task!");
        return;
    };

    // Plays animations on the RGB LED built into the board. Can send things like
    // "fade in and out blue indefinitely" or "flash red five times"
    let Ok(led_driver_task_token) =
        rgb_led::led_driver_task(led_green_a, led_red_a_blue_b, LED_COMMAND_CH.receiver())
    else {
        defmt::error!("Failed to spawn rgb led driver task!");
        return;
    };

    // Manages network messages, such as motor feedback to DAQ PC
    let Ok(network_task_token) = network::network_task(
        w5500,
        w5500_int,
        NETWORK_STATUS_IND.sender(),
        NETWORK_CMD_FROM_PC_CH.sender(),
        &MOTOR_CURRENT_POSITION,
        &MOTOR_POSITION_SETPOINT,
        &MOTOR_SPEED_SETPOINT,
    ) else {
        defmt::error!("Failed to spawn ethernet/network task!");
        return;
    };

    // Logging and console printing task
    let Ok(monitor_task_token) = monitor_task(&MOTOR_CURRENT_POSITION) else {
        defmt::error!("Failed to spawn monitor task!");
        return;
    };

    // Actually spawn the tasks now that we know they were all created successfully
    spawner.spawn(motor_quadrature_task_token);
    spawner.spawn(motor_control_task_token);
    spawner.spawn(led_driver_task_token);
    spawner.spawn(network_task_token);
    spawner.spawn(monitor_task_token);
}

#[embassy_executor::task]
async fn monitor_task(
    motor_current_position: &'static blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<I32F32>>,
) {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(1));

    loop {
        let cum_theta: f32 = motor_current_position.lock(|cell| cell.get()).to_num();

        defmt::info!("angle = {} deg", cum_theta * (180. / core::f32::consts::PI),);

        ticker.next().await;
    }
}

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"PWM Control Loop"),
    embassy_rp::binary_info::rp_program_description!(c"your program description"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];
