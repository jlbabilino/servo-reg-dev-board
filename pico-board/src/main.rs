//  ________  ________  ________        ___       ___  ________  ___  ___  ___  ________  ________
// |\   __  \|\   ____\|\   __  \      |\  \     |\  \|\   __  \|\  \|\  \|\  \|\   ___ \|\   ____\
// \ \  \|\  \ \  \___|\ \  \|\  \     \ \  \    \ \  \ \  \|\  \ \  \\\  \ \  \ \  \_|\ \ \  \___|_
//  \ \   ____\ \_____  \ \   ____\     \ \  \    \ \  \ \  \\\  \ \  \\\  \ \  \ \  \ \\ \ \_____  \
//   \ \  \___|\|____|\  \ \  \___|      \ \  \____\ \  \ \  \\\  \ \  \\\  \ \  \ \  \_\\ \|____|\  \
//    \ \__\     ____\_\  \ \__\          \ \_______\ \__\ \_____  \ \_______\ \__\ \_______\____\_\  \
//     \|__|    |\_________\|__|           \|_______|\|__|\|___| \__\|_______|\|__|\|_______|\_________\
//              \|_________|                                    \|__|                       \|_________|

#![no_std]
#![no_main]

mod anim;
mod constants;
mod motor_control;
mod motor_quadrature;
mod network;
mod rgb_led;
mod util;

use core::cell::Cell;

use embassy_rp::gpio::Level;
use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::peripherals::DMA_CH1;
use embassy_rp::pwm;
use embassy_rp::pwm::SetDutyCycle;
use embassy_rp::spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_hal_bus::spi::ExclusiveDevice;

use embassy_rp::adc;
use embassy_rp::gpio;
use embassy_sync::blocking_mutex::Mutex;

use fixed::types::I32F32;

use anim::Rainbow;
use motor_control::MotorState;

use {defmt_rtt as _, panic_probe as _};

defmt::timestamp!("[t = {=u64:us} s]", {
    embassy_time::Instant::now().as_micros()
});

// enum OverallState {
//     Disconnected,
//     Disabled,
//     Enabled,
// }

static MOTOR_CUM_ANGLE_MUTEX: Mutex<CriticalSectionRawMutex, Cell<I32F32>> =
    Mutex::new(Cell::new(I32F32::const_from_int(0)));

static MOTOR_STATE_SIGNAL: embassy_sync::signal::Signal<CriticalSectionRawMutex, MotorState> =
    Signal::new();

pub static LED_COMMAND_CH: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    rgb_led::Command,
    16, // should be processed instantly but just in case
> = embassy_sync::channel::Channel::new();

embassy_rp::bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>, embassy_rp::dma::InterruptHandler<DMA_CH1>;
});

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    // Bind peripherals
    let mut led_green_a_blue_b =
        pwm::Pwm::new_output_ab(p.PWM_SLICE3, p.PIN_6, p.PIN_7, Default::default());
    let led_red_a = pwm::Pwm::new_output_a(p.PWM_SLICE4, p.PIN_8, Default::default());

    led_green_a_blue_b.set_duty_cycle_fully_on().unwrap();

    let adc = adc::Adc::new_blocking(p.ADC, Default::default());

    let hall_a_pin = adc::Channel::new_pin(p.PIN_28, gpio::Pull::None);
    let hall_b_pin = adc::Channel::new_pin(p.PIN_27, gpio::Pull::None);
    let hall_c_pin = adc::Channel::new_pin(p.PIN_26, gpio::Pull::None);

    let esc_stop_pin = gpio::OutputOpenDrain::new(p.PIN_2, gpio::Level::High);
    let esc_brake_pin = gpio::OutputOpenDrain::new(p.PIN_3, gpio::Level::High);
    let esc_dir_pin = gpio::OutputOpenDrain::new(p.PIN_4, gpio::Level::High);

    let esc_pwm_config = motor_control::pwm_config();
    let esc_pwm = pwm::Pwm::new_output_b(p.PWM_SLICE2, p.PIN_5, esc_pwm_config);

    let test_button_a = gpio::Input::new(p.PIN_14, gpio::Pull::Up);
    let test_button_b = gpio::Input::new(p.PIN_15, gpio::Pull::Up);

    // Initialize W5500 ethernet module
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 50_000_000;

    let (eth_miso, eth_mosi, eth_clk) = (p.PIN_16, p.PIN_19, p.PIN_18);
    let eth_spi = spi::Spi::new(
        p.SPI0, eth_clk, eth_mosi, eth_miso, p.DMA_CH0, p.DMA_CH1, Irqs, spi_cfg,
    );
    let cs = gpio::Output::new(p.PIN_17, gpio::Level::High);

    let w5500_int = gpio::Input::new(p.PIN_21, gpio::Pull::Up);

    use w5500_ll::eh1::vdm::W5500;

    let w5500 = W5500::new(ExclusiveDevice::new(eth_spi, cs, embassy_time::Delay).unwrap());

    // A high-frequency loop (~3 kHz) to track the motor's rotation
    // Modifies a mutex to set share the current rotation angle with other tasks
    spawner.spawn(
        motor_quadrature::motor_quadrature_task(
            adc,
            hall_a_pin,
            hall_b_pin,
            hall_c_pin,
            &MOTOR_CUM_ANGLE_MUTEX,
            &LED_COMMAND_CH,
        )
        .unwrap(),
    );

    // Accepts MotorState requests to drive the motor to a ceratin speed or
    // position. Can also disable or brake the motor using certain pins.
    spawner.spawn(
        motor_control::motor_control_task(
            &MOTOR_STATE_SIGNAL,
            &MOTOR_CUM_ANGLE_MUTEX,
            esc_stop_pin,
            esc_brake_pin,
            esc_dir_pin,
            esc_pwm,
        )
        .unwrap(),
    );

    // Plays animations on the RGB LED built into the board. Can send things like
    // "fade in and out blue indefinitely" or "flash red five times"
    spawner
        .spawn(rgb_led::led_driver_task(&LED_COMMAND_CH, led_red_a, led_green_a_blue_b).unwrap());

    // Manages network messages, such as motor feedback to DAQ PC
    spawner.spawn(network::network_task(w5500, w5500_int).unwrap());

    // Logging and console printing task
    spawner.spawn(monitor_task().unwrap());

    // Quick test of motor functionality
    spawner.spawn(test_motor_ctrl(&MOTOR_STATE_SIGNAL, test_button_a, test_button_b).unwrap());

    // LED_COMMAND_CH
    //     .send(rgb_led::Command::Looping(anim::Animation::FadeInFadeOut(
    //         anim::FadeInFadeOut::new(
    //             color::palette::css::BLUE.discard_alpha(),
    //             embassy_time::Duration::from_secs(3),
    //         ),
    //     )))
    //     .await;

    LED_COMMAND_CH
        .send(rgb_led::Command::Looping(anim::Animation::Rainbow(
            anim::Rainbow::new(embassy_time::Duration::from_secs(2)),
        )))
        .await;
}

#[embassy_executor::task]
async fn test_motor_ctrl(
    motor_state_signal: &'static Signal<CriticalSectionRawMutex, MotorState>,
    test_button_a: gpio::Input<'static>,
    test_button_b: gpio::Input<'static>,
) {
    let disabled_anim = rgb_led::Command::Transient(anim::Animation::Rainbow(Rainbow::new(
        embassy_time::Duration::from_secs(2),
    )));

    let enabled_anim = rgb_led::Command::Transient(anim::Animation::Pulse(anim::Pulse::new(
        color::palette::css::RED.discard_alpha(),
        embassy_time::Duration::from_millis(50),
        embassy_time::Duration::from_millis(50),
        embassy_time::Duration::from_millis(100),
        embassy_time::Duration::from_millis(200),
        2,
    )));
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(40));
    loop {
        match (test_button_a.get_level(), test_button_b.get_level()) {
            (Level::Low, Level::Low) => {
                motor_state_signal.signal(MotorState::Brake);
            }
            (Level::High, Level::Low) => {
                motor_state_signal.signal(MotorState::Speed(0.1));
                LED_COMMAND_CH.send(disabled_anim).await;
            }
            (Level::Low, Level::High) => {
                motor_state_signal.signal(MotorState::Speed(-0.1));
                LED_COMMAND_CH.send(enabled_anim).await;
            }
            (Level::High, Level::High) => {
                motor_state_signal.signal(MotorState::Disabled);
            }
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn monitor_task() {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(20));

    loop {
        let cum_theta: f32 = MOTOR_CUM_ANGLE_MUTEX.lock(|cell| cell.get()).to_num();

        // defmt::info!("angle = {} deg", cum_theta * (180. / core::f32::consts::PI),);

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
