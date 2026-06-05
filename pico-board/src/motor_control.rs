use core::cell::Cell;

use embassy_futures::select::Either;
use embassy_rp::{
    gpio,
    pwm::{self, SetDutyCycle},
};
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    signal::Signal,
};
use fixed::types::I32F32;

pub enum MotorState {
    Disabled,
    Position(I32F32),
    Speed(f32), // 0 = stopped, +1 = max CCW, -1 = max CW
    Brake,
}

const PWM_TOP: u16 = 12499;

pub fn pwm_config() -> pwm::Config {
    let mut esc_pwm_config = pwm::Config::default();
    esc_pwm_config.compare_b = 0; // disable for now
    esc_pwm_config.top = PWM_TOP;
    esc_pwm_config.phase_correct = false;
    esc_pwm_config.enable = true;
    esc_pwm_config
}

#[embassy_executor::task]
pub async fn motor_control_task(
    motor_state_signal: &'static Signal<CriticalSectionRawMutex, MotorState>,
    motor_cum_angle_mutex: &'static Mutex<CriticalSectionRawMutex, Cell<I32F32>>,
    mut esc_stop_pin: gpio::OutputOpenDrain<'static>,
    mut esc_brake_pin: gpio::OutputOpenDrain<'static>,
    mut esc_dir_pin: gpio::OutputOpenDrain<'static>,
    mut esc_pwm: pwm::Pwm<'static>,
) {
    let mut motor_controller = async |state: &MotorState| -> ! {
        match state {
            MotorState::Disabled => {
                esc_stop_pin.set_low();
                esc_brake_pin.set_high();
                esc_dir_pin.set_high();
                esc_pwm.set_duty_cycle(0).unwrap();
                loop {
                    // nothing else to do so just sleep 😴
                    embassy_time::Timer::after(embassy_time::Duration::from_secs(100000)).await;
                }
            }
            MotorState::Brake => {
                esc_stop_pin.set_high();
                esc_brake_pin.set_low();
                esc_dir_pin.set_high();
                esc_pwm.set_duty_cycle(0).unwrap();
                loop {
                    embassy_time::Timer::after(embassy_time::Duration::from_secs(100000)).await;
                }
            }
            MotorState::Speed(speed) => {
                let speed = speed.clamp(-1., 1.);
                esc_stop_pin.set_high();
                esc_brake_pin.set_high();
                set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, speed);
                loop {
                    embassy_time::Timer::after(embassy_time::Duration::from_secs(100000)).await;
                }
            }
            MotorState::Position(target_angle) => {
                // Basic P controller
                let kp: f32 = 0.3;

                let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(200));

                loop {
                    let curr_angle = motor_cum_angle_mutex.lock(|cell| cell.get());
                    let err: f32 = (curr_angle - target_angle).to_num();
                    let commanded_speed = kp * err;
                    set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, commanded_speed);

                    ticker.next().await;
                }
            }
        };
    };

    let mut motor_state = MotorState::Disabled;
    loop {
        match embassy_futures::select::select(
            motor_state_signal.wait(),
            motor_controller(&motor_state),
        )
        .await
        {
            Either::First(new_state) => motor_state = new_state,
            Either::Second(_) => unreachable!(),
        }
    }
}

fn set_motor_speed(
    esc_dir_pin: &mut gpio::OutputOpenDrain<'static>,
    esc_pwm: &mut pwm::Pwm<'static>,
    commanded_speed: f32,
) {
    esc_dir_pin.set_level(if commanded_speed < 0.0 {
        gpio::Level::High
    } else {
        gpio::Level::Low
    });
    esc_pwm
        .set_duty_cycle(((PWM_TOP + 1) as f32 * commanded_speed.abs()) as u16)
        .unwrap();
}
