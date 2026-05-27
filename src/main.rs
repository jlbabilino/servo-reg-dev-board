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
mod motor_quadrature;
mod network;
mod rgb_led;
mod util;

use core::cell::Cell;
use core::cell::RefCell;

use embassy_futures::select::Either;
use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::peripherals::DMA_CH1;
use embassy_rp::peripherals::SPI0;
use embassy_rp::pwm;
use embassy_rp::pwm::SetDutyCycle;
use embassy_rp::spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_hal_bus::spi::ExclusiveDevice;

use embassy_rp::adc;
use embassy_rp::gpio;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

use fixed::traits::ToFixed as _;
use fixed::types::I32F32;
use static_cell::StaticCell;

use crate::anim::Rainbow;
use crate::motor_quadrature::HallAngleTracker;

use {defmt_rtt as _, panic_probe as _};

enum OverallState {
    Init,
    Enabled,
    Disabled,
}

enum MotorState {
    Disabled,
    Enabled(I32F32),
}

static MOTOR_ROTATION_MUTEX: Mutex<CriticalSectionRawMutex, Cell<I32F32>> =
    Mutex::new(Cell::new(I32F32::const_from_int(0)));

static MOTOR_STATE_SIGNAL: embassy_sync::signal::Signal<CriticalSectionRawMutex, MotorState> =
    Signal::new();

static LED_COMMAND_CH: embassy_sync::channel::Channel<
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

    // Static cells for each peripheral
    static LED_GREEN_A_BLUE_B_CELL: StaticCell<pwm::Pwm<'static>> = StaticCell::new();
    static LED_RED_A_CELL: StaticCell<pwm::Pwm<'static>> = StaticCell::new();

    static ADC_CELL: StaticCell<adc::Adc<'static, adc::Blocking>> = StaticCell::new();

    static HALL_A_PIN_CELL: StaticCell<adc::Channel<'static>> = StaticCell::new();
    static HALL_B_PIN_CELL: StaticCell<adc::Channel<'static>> = StaticCell::new();
    static HALL_C_PIN_CELL: StaticCell<adc::Channel<'static>> = StaticCell::new();

    static ESC_STOP_PIN_CELL: StaticCell<gpio::OutputOpenDrain<'static>> = StaticCell::new();
    static ESC_BRAKE_PIN_CELL: StaticCell<gpio::OutputOpenDrain<'static>> = StaticCell::new();
    static ESC_DIR_PIN_CELL: StaticCell<gpio::OutputOpenDrain<'static>> = StaticCell::new();
    static ESC_PWM_PIN_CELL: StaticCell<pwm::Pwm<'static>> = StaticCell::new();

    static TEST_BUTTON_A_CELL: StaticCell<gpio::Input<'static>> = StaticCell::new();
    static TEST_BUTTON_B_CELL: StaticCell<gpio::Input<'static>> = StaticCell::new();

    static W5500_INT_CELL: StaticCell<gpio::Input<'static>> = StaticCell::new();
    static W5500_CELL: StaticCell<
        W5500<
            ExclusiveDevice<
                embassy_rp::spi::Spi<'static, SPI0, embassy_rp::spi::Async>,
                gpio::Output<'static>,
                embassy_time::Delay,
            >,
        >,
    > = StaticCell::new();

    // Bind peripherals
    let led_green_a_blue_b = LED_GREEN_A_BLUE_B_CELL.init(pwm::Pwm::new_output_ab(
        p.PWM_SLICE3,
        p.PIN_6,
        p.PIN_7,
        Default::default(),
    ));
    let led_red_a = LED_RED_A_CELL.init(pwm::Pwm::new_output_a(
        p.PWM_SLICE4,
        p.PIN_8,
        Default::default(),
    ));

    led_green_a_blue_b.set_duty_cycle_fully_on().unwrap();

    let adc = ADC_CELL.init(adc::Adc::new_blocking(p.ADC, Default::default()));

    let hall_a_pin = HALL_A_PIN_CELL.init(adc::Channel::new_pin(p.PIN_28, gpio::Pull::None));
    let hall_b_pin = HALL_B_PIN_CELL.init(adc::Channel::new_pin(p.PIN_27, gpio::Pull::None));
    let hall_c_pin = HALL_C_PIN_CELL.init(adc::Channel::new_pin(p.PIN_26, gpio::Pull::None));

    let esc_stop_pin =
        ESC_STOP_PIN_CELL.init(gpio::OutputOpenDrain::new(p.PIN_2, gpio::Level::High));
    let esc_brake_pin =
        ESC_BRAKE_PIN_CELL.init(gpio::OutputOpenDrain::new(p.PIN_3, gpio::Level::High));
    let esc_dir_pin = ESC_DIR_PIN_CELL.init(gpio::OutputOpenDrain::new(p.PIN_4, gpio::Level::High));

    let mut esc_pwm_config = pwm::Config::default();
    esc_pwm_config.compare_b = 0; // disable for now
    esc_pwm_config.top = 12499;
    esc_pwm_config.phase_correct = false;
    esc_pwm_config.enable = true;
    let esc_pwm = ESC_PWM_PIN_CELL.init(pwm::Pwm::new_output_b(
        p.PWM_SLICE2,
        p.PIN_5,
        esc_pwm_config,
    ));

    let test_button_a = TEST_BUTTON_A_CELL.init(gpio::Input::new(p.PIN_14, gpio::Pull::Up));
    let test_button_b = TEST_BUTTON_B_CELL.init(gpio::Input::new(p.PIN_15, gpio::Pull::Up));

    // Initialize W5500 ethernet module
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 50_000_000;

    let (eth_miso, eth_mosi, eth_clk) = (p.PIN_16, p.PIN_19, p.PIN_18);
    let eth_spi = spi::Spi::new(
        p.SPI0, eth_clk, eth_mosi, eth_miso, p.DMA_CH0, p.DMA_CH1, Irqs, spi_cfg,
    );
    let cs = gpio::Output::new(p.PIN_17, gpio::Level::High);

    let w5500_int = W5500_INT_CELL.init(gpio::Input::new(p.PIN_21, gpio::Pull::Up));

    use w5500_ll::eh1::vdm::W5500;

    let w5500 = W5500_CELL.init(W5500::new(
        ExclusiveDevice::new(eth_spi, cs, embassy_time::Delay).unwrap(),
    ));

    // Spawn tasks

    // A high-frequency loop (~3 kHz) to track the motor's rotation
    spawner.spawn(motor_quadrature_loop(adc, hall_a_pin, hall_b_pin, hall_c_pin).unwrap());

    // Manages network messages, such as motor feedback to DAQ PC
    spawner.spawn(network::network_task(w5500, w5500_int).unwrap());

    // Logging and console printing task
    spawner.spawn(monitor_task().unwrap());

    // static OVERALL_STATE_CELL: StaticCell<Cell<OverallState>> = StaticCell::new();

    // let overall_state = OVERALL_STATE_CELL.init(Cell::new(OverallState::Init));

    // static OVERALL_STATE_SIGNAL: embassy_sync::signal::Signal<
    //     embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    //     OverallState,
    // > = embassy_sync::signal::Signal::new();

    spawner.spawn(led_manager(&LED_COMMAND_CH, led_red_a, led_green_a_blue_b).unwrap());

    spawner.spawn(
        test_motor_ctrl(
            esc_stop_pin,
            esc_brake_pin,
            esc_dir_pin,
            esc_pwm,
            test_button_a,
            test_button_b,
        )
        .unwrap(),
    );

    LED_COMMAND_CH
        .send(rgb_led::Command::Looping(anim::Animation::Rainbow(
            Rainbow::new(embassy_time::Duration::from_secs(2)),
        )))
        .await;
}

#[embassy_executor::task]
async fn test_motor_ctrl(
    esc_stop_pin: &'static mut gpio::OutputOpenDrain<'static>,
    esc_brake_pin: &'static mut gpio::OutputOpenDrain<'static>,
    esc_dir_pin: &'static mut gpio::OutputOpenDrain<'static>,
    esc_pwm: &'static mut pwm::Pwm<'static>,
    test_button_a: &'static mut gpio::Input<'static>,
    test_button_b: &'static mut gpio::Input<'static>,
) {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(40));
    loop {
        if test_button_a.is_low() {
            esc_dir_pin.set_low();
            esc_pwm.set_duty_cycle_fraction(10, 100).unwrap();
        } else {
            esc_pwm.set_duty_cycle_fraction(0, 100).unwrap();
        }

        if test_button_b.is_low() {
            esc_stop_pin.set_low();
        } else {
            esc_stop_pin.set_high();
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn motor_quadrature_loop(
    adc: &'static mut adc::Adc<'static, adc::Blocking>,
    hall_a_pin: &'static mut adc::Channel<'static>,
    hall_b_pin: &'static mut adc::Channel<'static>,
    hall_c_pin: &'static mut adc::Channel<'static>,
) {
    let mut tracker = HallAngleTracker::new();

    let ticker_duration = embassy_time::Duration::from_hz(3000);
    let ticker_initial_time = embassy_time::Instant::now();
    let mut ticker = embassy_time::Ticker::every(ticker_duration);
    ticker.reset_at(ticker_initial_time);
    let mut iter_idx: u32 = 0;

    // let mut is_first = true;
    // let mut sector_angle = motor_quadrature::SectorAngle(0);
    loop {
        use constants::{HA_AMP, HA_AVG, HB_AMP, HB_AVG, HC_AMP, HC_AVG};

        let ha_raw = adc.blocking_read(hall_a_pin).unwrap();
        let hb_raw = adc.blocking_read(hall_b_pin).unwrap();
        let hc_raw = adc.blocking_read(hall_c_pin).unwrap();

        let ha_norm: f32 = (ha_raw - HA_AVG) as f32 / HA_AMP as f32;
        let hb_norm: f32 = (hb_raw - HB_AVG) as f32 / HB_AMP as f32;
        let hc_norm: f32 = (hc_raw - HC_AVG) as f32 / HC_AMP as f32;

        let new_angle = tracker.update(ha_norm, hb_norm, hc_norm).unwrap();

        MOTOR_ROTATION_MUTEX.lock(|cell| cell.set(new_angle));

        let finish_time = embassy_time::Instant::now();

        let deadline_time =
            ticker_initial_time + ticker_duration.checked_mul(iter_idx + 1).unwrap();

        let fail_anim = anim::Pulse::new(
            color::palette::css::RED.discard_alpha(),
            embassy_time::Duration::from_millis(0),
            embassy_time::Duration::from_millis(200),
            embassy_time::Duration::from_millis(400),
            embassy_time::Duration::from_millis(500),
            2,
        );

        let spare_time = if finish_time < deadline_time {
            (deadline_time - finish_time).as_micros() as i32 // On time
        } else {
            -((finish_time - deadline_time).as_micros() as i32) // Late
        };

        if spare_time < 0 {
            // Late
            LED_COMMAND_CH
                .send(rgb_led::Command::Transient(anim::Animation::Pulse(
                    fail_anim,
                )))
                .await;
            defmt::error!("Motor update loop late by {}", &spare_time);
        }

        ticker.next().await;

        iter_idx += 1;
    }
}

#[embassy_executor::task]
async fn led_manager(
    led_command_signal: &'static embassy_sync::channel::Channel<
        CriticalSectionRawMutex,
        rgb_led::Command,
        16,
    >,
    led_red_a: &'static mut pwm::Pwm<'static>,
    led_green_a_blue_b: &'static mut pwm::Pwm<'static>,
) {
    let mut led_pwm_update_loop =
        async |curr_anim: &anim::Animation,
               is_loop: bool,
               t_anim_start: Option<embassy_time::Instant>| {
            let t_anim_start = match t_anim_start {
                Some(instant) => instant,
                None => embassy_time::Instant::now(),
            };
            let mut led_ticker = embassy_time::Ticker::every(rgb_led::LED_TICK_PERIOD);
            led_ticker.reset_at(t_anim_start);
            let deadline = t_anim_start + curr_anim.duration();

            loop {
                let curr_time = embassy_time::Instant::now();
                if !is_loop && curr_time >= deadline {
                    break;
                }
                let t_rel = curr_time - t_anim_start;
                let color = curr_anim.eval(t_rel);
                rgb_led::set_rgb(led_red_a, led_green_a_blue_b, color);
                led_ticker.next().await;
            }
        };

    let mut looping_anim = anim::Animation::Off;
    // For transient anim, keep track of start time as an Instant
    // That way if a new looping command fires, it won't reset the transient
    // animation's timer
    let mut transient_anim: Option<(anim::Animation, embassy_time::Instant)> = None;
    loop {
        match embassy_futures::select::select(
            led_command_signal.receive(),
            match transient_anim {
                // play a transient animation with given start time
                Some(ref value) => led_pwm_update_loop(&value.0, false, Some(value.1)),
                // play a looping animation, and start it now (None)
                None => led_pwm_update_loop(&looping_anim, true, None),
            },
        )
        .await
        {
            Either::First(command) => {
                match command {
                    rgb_led::Command::Transient(new_transient_anim) => {
                        transient_anim = Some((new_transient_anim, embassy_time::Instant::now()));
                    }
                    rgb_led::Command::Looping(new_looping_animation) => {
                        looping_anim = new_looping_animation;
                    }
                };
            }
            Either::Second(_) => {
                // the looping animation will never end, so this must be a
                // transient animation ending. So just pop the transient
                // animation
                transient_anim = None;
            }
        }
    }
}

#[embassy_executor::task]
async fn monitor_task() {
    // let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(20));

    // loop {
    //     let motor_state = MOTOR_STATE.lock(|state_cell| state_cell.clone().into_inner());

    //     let ha = motor_state.hall_a;
    //     let hb = motor_state.hall_b;
    //     let hc = motor_state.hall_c;
    //     let sector_angle = motor_state.sector_angle;

    //     let ha_norm: f32 = (ha as f32) - (((HA_MIN + HA_MAX) as f32) / 2.0);
    //     let hb_norm: f32 = (hb as f32) - (((HB_MIN + HB_MAX) as f32) / 2.0);
    //     let hc_norm: f32 = (hc as f32) - (((HC_MIN + HC_MAX) as f32) / 2.0);

    //     let ha_norm = ha_norm / (((HA_MAX - HA_MIN) as f32) / 2.0);
    //     let hb_norm = hb_norm / (((HB_MAX - HB_MIN) as f32) / 2.0);
    //     let hc_norm = hc_norm / (((HC_MAX - HC_MIN) as f32) / 2.0);

    //     // Apply Clarke transformation
    //     fn clarke_trans(a: f32, b: f32, c: f32) -> f32 {
    //         let alpha = a;
    //         let beta = (b - c) / SQRT_3;
    //         let theta_clarke = -libm::atan2f(beta, alpha);
    //         return theta_clarke;
    //     }

    //     let sector = sector_angle.to_sector();
    //     let theta_0 = sector_angle.angle_rel_to_zero();

    //     // Clarke transform: (a, b, c) -> (alpha, beta) -> theta
    //     // Case A: inputs are (ha, hb, hc), singularity in sector 4
    //     // Case B: inputs are (hb, hc, ha), singularity in sector 2
    //     // Case C: inputs are (hc, ha, hb), singularity in sector 0

    //     // Case A: singularity in sector 4, before sing:  3*(pi/6), after sing: 15*(pi/6)
    //     // Case B: singularity in sector 2, before sing:   -(pi/6), after sing: 11*(pi/6)
    //     // Case C: singularity in sector 0, before sing: -5*(pi/6), after sing:  7*(pi/6)

    //     fn clarke_case_a(ha: f32, hb: f32, hc: f32, is_after_sing: bool) -> f32 {
    //         clarke_trans(ha, hb, hc) + (3.0 * PI_6) + if is_after_sing { PI_TIMES_2 } else { 0.0 }
    //     }
    //     fn clarke_case_b(ha: f32, hb: f32, hc: f32, is_after_sing: bool) -> f32 {
    //         clarke_trans(hb, hc, ha) - PI_6 + if is_after_sing { PI_TIMES_2 } else { 0.0 }
    //     }
    //     fn clarke_case_c(ha: f32, hb: f32, hc: f32, is_after_sing: bool) -> f32 {
    //         clarke_trans(hc, ha, hb) - (5.0 * PI_6) + if is_after_sing { PI_TIMES_2 } else { 0.0 }
    //     }

    //     // In sector 0: A (before), B (before)
    //     // In sector 1: A (before), B (before), C (after)
    //     // In sector 2: A (before),             C (after)
    //     // In sector 3: A (before), B  (after), C (after)
    //     // In sector 4:             B  (after), C (after)
    //     // In sector 5: A (after),  B  (after), C (after)

    //     let delta_theta = match sector {
    //         motor_quadrature::Sector::S0 => {
    //             (clarke_case_a(ha_norm, hb_norm, hc_norm, false)
    //                 + clarke_case_b(ha_norm, hb_norm, hc_norm, false))
    //                 / 2.0
    //         }
    //         motor_quadrature::Sector::S1 => {
    //             (clarke_case_a(ha_norm, hb_norm, hc_norm, false)
    //                 + clarke_case_b(ha_norm, hb_norm, hc_norm, false)
    //                 + clarke_case_c(ha_norm, hb_norm, hc_norm, true))
    //                 / 3.0
    //         }
    //         motor_quadrature::Sector::S2 => {
    //             (clarke_case_a(ha_norm, hb_norm, hc_norm, false)
    //                 + clarke_case_c(ha_norm, hb_norm, hc_norm, true))
    //                 / 2.0
    //         }
    //         motor_quadrature::Sector::S3 => {
    //             (clarke_case_a(ha_norm, hb_norm, hc_norm, false)
    //                 + clarke_case_b(ha_norm, hb_norm, hc_norm, true)
    //                 + clarke_case_c(ha_norm, hb_norm, hc_norm, true))
    //                 / 3.0
    //         }
    //         motor_quadrature::Sector::S4 => {
    //             (clarke_case_b(ha_norm, hb_norm, hc_norm, true)
    //                 + clarke_case_c(ha_norm, hb_norm, hc_norm, true))
    //                 / 2.0
    //         }
    //         motor_quadrature::Sector::S5 => {
    //             (clarke_case_a(ha_norm, hb_norm, hc_norm, true)
    //                 + clarke_case_b(ha_norm, hb_norm, hc_norm, true)
    //                 + clarke_case_c(ha_norm, hb_norm, hc_norm, true))
    //                 / 3.0
    //         }
    //     };

    //     let cum_theta = theta_0 + delta_theta;

    //     let spare_time = TICKER_SPARE_TIME.load(core::sync::atomic::Ordering::Relaxed);

    //     defmt::info!(
    //         "angle = {}, spare_time = {}",
    //         cum_theta * 180.0 / PI,
    //         spare_time
    //     );

    //     ticker.next().await;
    // }
}

// #[embassy_executor::task]
// async fn ethernet_task(
//     runner: embassy_net_wiznet::Runner<
//         'static,
//         W5500,
//         ExclusiveDevice<
//             embassy_rp::spi::Spi<'static, SPI0, embassy_rp::spi::Async>,
//             embassy_rp::gpio::Output<'static>,
//             embassy_time::Delay,
//         >,
//         embassy_rp::gpio::Input<'static>,
//         embassy_rp::gpio::Output<'static>,
//     >,
// ) -> ! {
//     runner.run().await
// }

// #[embassy_executor::task]
// async fn net_task(
//     mut runner: embassy_net::Runner<'static, embassy_net_wiznet::Device<'static>>,
// ) -> ! {
//     runner.run().await
// }

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"PWM Control Loop"),
    embassy_rp::binary_info::rp_program_description!(c"your program description"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];
