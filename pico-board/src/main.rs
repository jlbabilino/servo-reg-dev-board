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
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]

mod anim;
mod buttons;
mod constants;
mod data;
mod motor_control;
mod motor_quadrature;
mod network;
mod rgb_led;
mod util;

use embassy_futures::select::Either;
use embassy_futures::select::select;
use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::peripherals::DMA_CH1;
use embassy_rp::pwm;
use embassy_rp::spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::Duration;
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;

use embassy_rp::adc;
use embassy_rp::gpio;

use anim::Rainbow;
use motor_control::MotorCommand;

use crate::data::LED_COMMAND_CH;
use crate::data::MOTOR_COMMAND_CHANNEL;
use crate::data::MOTOR_CURRENT_POSITION;
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

static BUTTON_1_WATCH: Watch<CriticalSectionRawMutex, bool, 4> = Watch::new();
static BUTTON_2_SIGNAL: embassy_sync::signal::Signal<CriticalSectionRawMutex, bool> = Signal::new();
static BUTTON_3_SIGNAL: embassy_sync::signal::Signal<CriticalSectionRawMutex, bool> = Signal::new();
static BUTTON_4_SIGNAL: embassy_sync::signal::Signal<CriticalSectionRawMutex, bool> = Signal::new();

static NETWORK_STATUS_IND: Watch<CriticalSectionRawMutex, NetworkStatus, 4> = Watch::new();
static NETWORK_CMD_FROM_PC_CH: channel::Channel<CriticalSectionRawMutex, CmdFromPC, 16> =
    channel::Channel::new();

embassy_rp::bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>, embassy_rp::dma::InterruptHandler<DMA_CH1>;
});

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    // Bind peripherals
    let mut led_green_a = pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_18, Default::default());
    let led_red_a_blue_b =
        pwm::Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_16, p.PIN_17, Default::default());

    // led_green_a_blue_b.set_duty_cycle_fully_on().unwrap();

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

    use w5500_ll::eh1::vdm::W5500;

    let w5500 = W5500::new(ExclusiveDevice::new(eth_spi, cs, embassy_time::Delay).unwrap());

    // A high-frequency loop (~3 kHz) to track the motor's rotation
    // Modifies a mutex to set share the current rotation angle with other tasks
    spawner.spawn(
        motor_quadrature::motor_quadrature_task(adc, hall_a_pin, hall_b_pin, hall_c_pin).unwrap(),
    );

    // Accepts MotorState requests to drive the motor to a ceratin speed or
    // position. Can also disable or brake the motor using certain pins.
    spawner.spawn(
        motor_control::motor_control_task(esc_stop_pin, esc_brake_pin, esc_dir_pin, esc_pwm)
            .unwrap(),
    );

    // Plays animations on the RGB LED built into the board. Can send things like
    // "fade in and out blue indefinitely" or "flash red five times"
    spawner.spawn(rgb_led::led_driver_task(led_green_a, led_red_a_blue_b).unwrap());

    // Manages network messages, such as motor feedback to DAQ PC
    spawner.spawn(
        network::network_task(
            w5500,
            w5500_int,
            NETWORK_STATUS_IND.sender(),
            NETWORK_CMD_FROM_PC_CH.sender(),
        )
        .unwrap(),
    );

    // Logging and console printing task
    spawner.spawn(monitor_task().unwrap());

    // Quick test of motor functionality
    // spawner.spawn(test_motor_ctrl(&MOTOR_STATE_SIGNAL, button_1, button_2).unwrap());

    spawner.spawn(buttons::button_task(button_1, &BUTTON_1_WATCH).unwrap());
    // spawner.spawn(buttons::button_task(button_2, &BUTTON_2_SIGNAL).unwrap());
    // spawner.spawn(buttons::button_task(button_3, &BUTTON_3_SIGNAL).unwrap());
    // spawner.spawn(buttons::button_task(button_4, &BUTTON_4_SIGNAL).unwrap());

    // LED_COMMAND_CH
    //     .send(rgb_led::Command::Looping(anim::Animation::FadeInFadeOut(
    //         anim::FadeInFadeOut::new(
    //             color::palette::css::BLUE.discard_alpha(),
    //             embassy_time::Duration::from_secs(3),
    //         ),
    //     )))
    //     .await;

    // LED_COMMAND_CH
    //     .send(rgb_led::Command::Looping(anim::Animation::Rainbow(
    //         anim::Rainbow::new(embassy_time::Duration::from_secs(2)),
    //     )))
    //     .await;

    // const PRESS_THRESH: Duration = Duration::from_millis(200);
    const HOLD_THRESH: Duration = Duration::from_millis(1000);

    #[derive(Copy, Clone, PartialEq)]
    enum ControlState {
        Disabled,
        Manual1,
        Manual2,
        Network,
    }

    #[derive(defmt::Format)]
    enum ButtonSignal {
        Pressed,
        Held,
    }

    let mut btn_recv = BUTTON_1_WATCH.receiver().expect("Ran out of receivers");
    let mut check_for_presses = async || {
        loop {
            btn_recv.changed_and(|val| *val == true).await;
            let hold_timer = embassy_time::Timer::after(HOLD_THRESH);
            match select(btn_recv.changed_and(|val| *val == false), hold_timer).await {
                Either::First(_) => {
                    // released quick enough to count as a press, not hold
                    defmt::info!("Button pressed!");
                    return ButtonSignal::Pressed;
                }
                Either::Second(_) => {
                    defmt::info!("Button held!");
                    return ButtonSignal::Held;
                }
            };
        }
    };

    LED_COMMAND_CH
        .send(rgb_led::Command::Looping(
            constants::DISCONNECTED_DISABLED_ANIM,
        ))
        .await;
    let mut control_state = ControlState::Disabled;
    loop {
        match select(check_for_presses(), async {
            // let animation = match control_state {
            //     ControlState::Disabled => constants::DISABLED_ANIM,
            //     ControlState::Manual1 => constants::MANUAL_MODE_1_ANIM,
            //     ControlState::Manual2 => constants::MANUAL_MODE_2_ANIM,
            // };
            // LED_COMMAND_CH
            //     .send(rgb_led::Command::Looping(animation))
            //     .await;

            MOTOR_COMMAND_CHANNEL.send(MotorCommand::Disabled).await;
            if control_state == ControlState::Manual1 {
                loop {
                    let button_2_val = button_2.is_low();
                    let button_4_val = button_4.is_low();

                    match (button_2_val, button_4_val) {
                        (false, false) => {
                            MOTOR_COMMAND_CHANNEL.send(MotorCommand::Disabled).await;
                        }
                        (true, false) => {
                            MOTOR_COMMAND_CHANNEL.send(MotorCommand::Speed(0.01)).await;
                        }
                        (false, true) => {
                            MOTOR_COMMAND_CHANNEL.send(MotorCommand::Speed(-0.01)).await;
                        }
                        (true, true) => {
                            MOTOR_COMMAND_CHANNEL.send(MotorCommand::Brake).await;
                        }
                    };
                    Timer::after_millis(10).await;
                }
            }

            loop {
                Timer::after_secs(1).await;
            }
        })
        .await
        {
            Either::First(button_signal) => {
                defmt::info!("Button signal: {}", &button_signal);
                match (control_state, button_signal) {
                    (ControlState::Disabled, ButtonSignal::Pressed) => {
                        defmt::info!("Pressed when disabled -- no change");
                    }
                    (ControlState::Disabled, ButtonSignal::Held) => {
                        defmt::info!("Held when disabled -- switch to manual mode 1");
                        control_state = ControlState::Manual1;
                        LED_COMMAND_CH
                            .send(rgb_led::Command::Looping(constants::MANUAL_MODE_1_ANIM))
                            .await;
                    }
                    (ControlState::Manual1, ButtonSignal::Pressed) => {
                        defmt::info!("Pressed when in manual mode 1 -- disable");
                        control_state = ControlState::Disabled;
                        LED_COMMAND_CH
                            .send(rgb_led::Command::Looping(
                                constants::DISCONNECTED_DISABLED_ANIM,
                            ))
                            .await;
                    }
                    (ControlState::Manual1, ButtonSignal::Held) => {
                        defmt::info!("Held while in manual mode 1 -- switch to manual mode 2");
                        control_state = ControlState::Manual2;
                        LED_COMMAND_CH
                            .send(rgb_led::Command::Looping(constants::MANUAL_MODE_2_ANIM))
                            .await;
                    }
                    (ControlState::Manual2, ButtonSignal::Pressed) => {
                        defmt::info!("Pressed when in manual mode 2 -- disable");
                        control_state = ControlState::Disabled;
                        LED_COMMAND_CH
                            .send(rgb_led::Command::Looping(
                                constants::DISCONNECTED_DISABLED_ANIM,
                            ))
                            .await;
                    }
                    (ControlState::Manual2, ButtonSignal::Held) => {
                        defmt::info!("Held while in manual mode 2 -- switch to manual mode 1");
                        control_state = ControlState::Manual1;
                        LED_COMMAND_CH
                            .send(rgb_led::Command::Looping(constants::MANUAL_MODE_1_ANIM))
                            .await;
                    }
                    (_, _) => {}
                }
            }
            Either::Second(_) => defmt::unreachable!(),
        };
    }

    // let mut idx = 0;
    // loop {
    //     let button_3_val = BUTTON_1_SIGNAL.wait().await;
    //     LED_COMMAND_CH
    //         .send(rgb_led::Command::Looping(anim::Animation::Solid(
    //             anim::Solid::new(
    //                 (if button_3_val {
    //                     color::palette::css::WHITE
    //                 } else {
    //                     color::palette::css::BLACK
    //                 })
    //                 .discard_alpha(),
    //                 embassy_time::Duration::from_secs(10),
    //             ),
    //         )))
    //         .await;
    //     defmt::info!("Button 3 @ {}: {}", &idx, &button_3_val);
    //     idx += 1;
    // }
}

#[embassy_executor::task]
async fn test_motor_ctrl(
    motor_state_signal: &'static Signal<CriticalSectionRawMutex, MotorCommand>,
    mut test_button_a: gpio::Input<'static>,
    mut test_button_b: gpio::Input<'static>,
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
    // let mut curr_pos = 0_i32.to_fixed::<I32F32>();
    let color_list = [
        color::palette::css::RED,
        color::palette::css::GREEN,
        color::palette::css::BLUE,
        color::palette::css::CYAN,
        color::palette::css::MAGENTA,
        color::palette::css::YELLOW,
        color::palette::css::BLACK,
        color::palette::css::WHITE,
        color::palette::css::PURPLE,
        color::palette::css::ORANGE,
    ];
    // loop {
    //     match (test_button_a.get_level(), test_button_b.get_level()) {
    //         (Level::Low, Level::Low) => {
    //             motor_state_signal.signal(MotorState::Position(
    //                 COMMANDED_POS_MUTEX.lock(|cell| cell.get()),
    //             ));
    //         }
    //         (Level::High, Level::Low) => {
    //             motor_state_signal.signal(MotorState::Position(0_i32.to_fixed::<I32F32>()));
    //             LED_COMMAND_CH.send(disabled_anim).await;
    //         }
    //         (Level::Low, Level::High) => {
    //             // motor_state_signal.signal(MotorState::Speed(-0.1));
    //             motor_state_signal.signal(MotorState::Position(62.83_f32.to_fixed::<I32F32>()));
    //             LED_COMMAND_CH.send(enabled_anim).await;
    //         }
    //         (Level::High, Level::High) => {
    //             motor_state_signal.signal(MotorState::Disabled);
    //         }
    //     }
    //     ticker.next().await;
    // }
    for color in color_list.iter().cycle() {
        test_button_a.wait_for_low().await;
        LED_COMMAND_CH
            .send(rgb_led::Command::Looping(anim::Animation::Solid(
                anim::Solid::new(color.discard_alpha(), embassy_time::Duration::from_secs(5)),
            )))
            .await;
        embassy_time::Timer::after_millis(200).await;
    }
}

#[embassy_executor::task]
async fn monitor_task() {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(20));

    loop {
        let cum_theta: f32 = MOTOR_CURRENT_POSITION.lock(|cell| cell.get()).to_num();

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
