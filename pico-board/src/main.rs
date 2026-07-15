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
mod manual_control;
mod monitor;
mod motor_control;
mod motor_quadrature;
mod network;
mod rgb_led;
mod types;
mod util;

use core::cell::Cell;

use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::peripherals::DMA_CH1;
use embassy_rp::pwm;
use embassy_rp::spi;
use embassy_sync::blocking_mutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;
use embedded_hal_bus::spi::ExclusiveDevice;
use fixed::types::I32F32;
use w5500_ll::eh1::vdm::W5500;

use embassy_rp::adc;
use embassy_rp::gpio;

use crate::types::ButtonWatch;
use crate::types::CMDFromPCPubSub;
use crate::types::F32Mutex;
use crate::types::I32F32Mutex;
use crate::types::LEDCommandPubSub;
use crate::types::MotorCommandPubSub;
use crate::types::NetworkStatusWatch;
use crate::types::QuadratureCommandWatch;
use crate::types::QuadratureErrorWatch;

use {defmt_rtt as _, panic_probe as _};

defmt::timestamp!("[t = {=u64:us} s]", {
    embassy_time::Instant::now().as_micros()
});

embassy_rp::bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>, embassy_rp::dma::InterruptHandler<DMA_CH1>;
});

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    // ========================================================================
    // ======================== MAP PICO GPIO PINS ============================
    // ========================================================================

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

    let button_1 = gpio::Input::new(p.PIN_9, gpio::Pull::Up);
    let button_2 = gpio::Input::new(p.PIN_8, gpio::Pull::Up);
    let button_3 = gpio::Input::new(p.PIN_7, gpio::Pull::Up);
    let button_4 = gpio::Input::new(p.PIN_6, gpio::Pull::Up);

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

    // ========================================================================
    // ======================= COMMUNICATION CHANNELS =========================
    // ========================================================================

    static NETWORK_STATUS_WATCH: NetworkStatusWatch = Watch::new();
    static CMD_FROM_PC_PUBSUB: CMDFromPCPubSub = PubSubChannel::new();

    /// Indicates current position of the motor as measured by the hall effect
    /// sensor. Updated by motor_quadrature
    static MOTOR_CURRENT_POSITION: I32F32Mutex =
        blocking_mutex::Mutex::new(Cell::new(I32F32::ZERO));

    static QUADRATURE_ERROR_WATCH: QuadratureErrorWatch = Watch::new();
    static QUADRATURE_COMMAND_WATCH: QuadratureCommandWatch = Watch::new();

    /// Used to send commands to the motor control loop. Commands are awaited in motor_control.
    /// For example, you may command the motor to go to position of 100 radians, wait for it
    /// to get there, then zero the position, then command it to go to position 0 radians.
    static MOTOR_COMMAND_PUBSUB: MotorCommandPubSub = PubSubChannel::new();

    static MOTOR_POSITION_SETPOINT: I32F32Mutex =
        blocking_mutex::Mutex::new(Cell::new(I32F32::ZERO));

    static MOTOR_SPEED_SETPOINT: F32Mutex = blocking_mutex::Mutex::new(Cell::new(0.));

    static LED_COMMAND_PUBSUB: LEDCommandPubSub = PubSubChannel::new();

    static BUTTON_1_WATCH: ButtonWatch = Watch::new();
    static BUTTON_2_WATCH: ButtonWatch = Watch::new();
    static BUTTON_3_WATCH: ButtonWatch = Watch::new();
    static BUTTON_4_WATCH: ButtonWatch = Watch::new();

    // ========================================================================
    // ========================== RECEIVERS/SENDERS ===========================
    // ========================================================================

    let Some(quadrature_command_receiver) = QUADRATURE_COMMAND_WATCH.receiver() else {
        defmt::error!("Failed to create quadrature command receiver!");
        return;
    };
    let Some(quadrature_command_monitor) = QUADRATURE_COMMAND_WATCH.receiver() else {
        defmt::error!("Failed to create quadrature command monitor!");
        return;
    };
    let Some(quadrature_error_monitor) = QUADRATURE_ERROR_WATCH.receiver() else {
        defmt::error!("Failed to create quadrature error monitor!");
        return;
    };

    let Some(network_status_receiver) = NETWORK_STATUS_WATCH.receiver() else {
        defmt::error!("Failed to create network status receiver!");
        return;
    };
    let Some(network_status_monitor) = NETWORK_STATUS_WATCH.receiver() else {
        defmt::error!("Failed to create network status monitor!");
        return;
    };

    let Ok(motor_command_subscriber) = MOTOR_COMMAND_PUBSUB.subscriber() else {
        defmt::error!("Failed to create motor command subscriber!");
        return;
    };
    let Ok(motor_command_monitor) = MOTOR_COMMAND_PUBSUB.subscriber() else {
        defmt::error!("Failed to create motor command monitor!");
        return;
    };

    let Ok(led_command_subscriber) = LED_COMMAND_PUBSUB.subscriber() else {
        defmt::error!("Failed to create LED command subscriber!");
        return;
    };
    let Ok(led_command_monitor) = LED_COMMAND_PUBSUB.subscriber() else {
        defmt::error!("Failed to create LED command monitor!");
        return;
    };

    let Ok(cmd_from_pc_publisher) = CMD_FROM_PC_PUBSUB.publisher() else {
        defmt::error!("Failed to create Network CMD from PC publisher!");
        return;
    };
    let Ok(cmd_from_pc_monitor) = CMD_FROM_PC_PUBSUB.subscriber() else {
        defmt::error!("Failed to create Network CMD from PC monitor!");
        return;
    };

    let Some(button_1_receiver) = BUTTON_1_WATCH.receiver() else {
        defmt::error!("Failed to create button 1 receiver");
        return;
    };
    let Some(button_2_receiver) = BUTTON_2_WATCH.receiver() else {
        defmt::error!("Failed to create button 2 receiver");
        return;
    };
    let Some(button_3_receiver) = BUTTON_3_WATCH.receiver() else {
        defmt::error!("Failed to create button 3 receiver");
        return;
    };
    let Some(button_4_receiver) = BUTTON_4_WATCH.receiver() else {
        defmt::error!("Failed to create button 4 receiver");
        return;
    };

    let Ok(led_pub_manual) = LED_COMMAND_PUBSUB.publisher() else {
        defmt::error!("Failed to create LED command publisher for manual task");
        return;
    };
    let Ok(led_pub_network) = LED_COMMAND_PUBSUB.publisher() else {
        defmt::error!("Failed to create LED command publisher for network task");
        return;
    };
    let Ok(motor_cmd_pub) = MOTOR_COMMAND_PUBSUB.publisher() else {
        defmt::error!("Failed to create motor command publisher");
        return;
    };

    // ========================================================================
    // ============================= SPAWN TASKS ==============================
    // ========================================================================

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
        motor_command_subscriber,
    ) else {
        defmt::error!("Failed to spawn motor control task!");
        return;
    };

    // Plays animations on the RGB LED built into the board. Can send things like
    // "fade in and out blue indefinitely" or "flash red five times"
    let Ok(led_driver_task_token) =
        rgb_led::led_driver_task(led_green_a, led_red_a_blue_b, led_command_subscriber)
    else {
        defmt::error!("Failed to spawn rgb led driver task!");
        return;
    };

    // Manages network messages, such as motor feedback to DAQ PC
    let Ok(network_task_token) = network::network_task(
        w5500,
        w5500_int,
        NETWORK_STATUS_WATCH.sender(),
        cmd_from_pc_publisher,
        &MOTOR_CURRENT_POSITION,
        &MOTOR_POSITION_SETPOINT,
        &MOTOR_SPEED_SETPOINT,
        led_pub_network,
    ) else {
        defmt::error!("Failed to spawn ethernet/network task!");
        return;
    };

    // Logging and console printing task
    let Ok(monitor_task_token) = monitor::monitor_task(
        network_status_monitor,
        cmd_from_pc_monitor,
        &MOTOR_CURRENT_POSITION,
        quadrature_error_monitor,
        quadrature_command_monitor,
        motor_command_monitor,
        &MOTOR_POSITION_SETPOINT,
        &MOTOR_SPEED_SETPOINT,
        led_command_monitor,
    ) else {
        defmt::error!("Failed to spawn monitor task!");
        return;
    };

    let Ok(button_1_task_token) = buttons::button_task(button_1, BUTTON_1_WATCH.sender()) else {
        defmt::error!("Failed to spawn button 1 debounce task");
        return;
    };
    let Ok(button_2_task_token) = buttons::button_task(button_2, BUTTON_2_WATCH.sender()) else {
        defmt::error!("Failed to spawn button 2 debounce task");
        return;
    };
    let Ok(button_3_task_token) = buttons::button_task(button_3, BUTTON_3_WATCH.sender()) else {
        defmt::error!("Failed to spawn button 3 debounce task");
        return;
    };
    let Ok(button_4_task_token) = buttons::button_task(button_4, BUTTON_4_WATCH.sender()) else {
        defmt::error!("Failed to spawn button 4 debounce task");
        return;
    };

    let Ok(quick_tests_token) = manual_control::manual_mode_task(
        button_1_receiver,
        button_2_receiver,
        button_3_receiver,
        button_4_receiver,
        led_pub_manual,
        motor_cmd_pub,
        &MOTOR_CURRENT_POSITION,
        network_status_receiver,
    ) else {
        defmt::error!("Failed to spawn quick tests tasks");
        return;
    };

    // Actually spawn the tasks now that we know they were all created successfully
    spawner.spawn(motor_quadrature_task_token);
    spawner.spawn(motor_control_task_token);
    spawner.spawn(led_driver_task_token);
    spawner.spawn(network_task_token);
    spawner.spawn(monitor_task_token);
    spawner.spawn(button_1_task_token);
    spawner.spawn(button_2_task_token);
    spawner.spawn(button_3_task_token);
    spawner.spawn(button_4_task_token);

    spawner.spawn(quick_tests_token);
}

// async fn signal_presses(button_receiver: &mut ButtonWatchReceiver, signal: )

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"PWM Control Loop"),
    embassy_rp::binary_info::rp_program_description!(c"your program description"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];
