use core::cell::Cell;

use embassy_futures::select::Either;
use embassy_rp::{
    gpio,
    pwm::{self, SetDutyCycle},
};
use embassy_sync::{
    blocking_mutex::{self, Mutex, raw::CriticalSectionRawMutex},
    signal::Signal,
};
use fixed::types::I32F32;

use crate::{
    data::{
        MOTOR_COMMAND_CHANNEL, MOTOR_CURRENT_POSITION, MOTOR_POSITION_SETPOINT,
        MOTOR_SPEED_SETPOINT,
    },
    util::spin_async,
};

static MOTOR_POSITION_OFFSET: blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<I32F32>> =
    blocking_mutex::Mutex::new(Cell::new(I32F32::ZERO));

pub enum MotorCommand {
    Disabled,
    Position(I32F32),
    PositionRaw,
    Speed(f32), // 0 = stopped, +1 = max CCW, -1 = max CW
    SpeedRaw,
    Brake,
    Zero,
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

pub fn get_motor_current_position() -> I32F32 {
    let offset = MOTOR_POSITION_OFFSET.lock(|cell| cell.get());
    let raw_pos = MOTOR_CURRENT_POSITION.lock(|cell| cell.get());
    raw_pos + offset
}

fn motor_reset_position_at(new_pos: I32F32) {
    let raw_pos = MOTOR_CURRENT_POSITION.lock(|cell| cell.get());
    let new_offset = new_pos - raw_pos;
    MOTOR_POSITION_OFFSET.lock(|cell| cell.set(new_offset));
}

#[embassy_executor::task]
pub async fn motor_control_task(
    mut esc_stop_pin: gpio::OutputOpenDrain<'static>,
    mut esc_brake_pin: gpio::OutputOpenDrain<'static>,
    mut esc_dir_pin: gpio::OutputOpenDrain<'static>,
    mut esc_pwm: pwm::Pwm<'static>,
) {
    let mut motor_controller = async |state: &MotorCommand| -> Result<(), &'static str> {
        match state {
            MotorCommand::Disabled => {
                esc_stop_pin.set_low();
                esc_brake_pin.set_high();
                esc_dir_pin.set_high();
                esc_pwm
                    .set_duty_cycle(0)
                    .map_err(|_| "Failed to set ESC PWM duty cycle")?;
                // nothing else to do so just sleep 😴
                spin_async().await;
            }
            MotorCommand::Brake => {
                esc_stop_pin.set_high();
                esc_brake_pin.set_low();
                esc_dir_pin.set_high();
                esc_pwm
                    .set_duty_cycle(0)
                    .map_err(|_| "Failed to set ESC PWM duty cycle")?;
                spin_async().await;
            }
            MotorCommand::Speed(speed) => {
                let speed = speed.clamp(-1., 1.);
                esc_stop_pin.set_high();
                esc_brake_pin.set_high();
                set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, speed);
                spin_async().await;
            }
            MotorCommand::SpeedRaw => {
                let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(200));
                esc_stop_pin.set_high();
                esc_brake_pin.set_high();
                loop {
                    let speed = MOTOR_SPEED_SETPOINT.lock(|cell| cell.get());
                    let speed = speed.clamp(-1., 1.);
                    set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, speed);
                    ticker.next().await;
                }
            }
            MotorCommand::Position(target_angle) => {
                esc_stop_pin.set_high();
                esc_brake_pin.set_high();
                // Basic P controller
                let kp: f32 = 0.002;

                let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(200));

                loop {
                    let curr_angle = MOTOR_CURRENT_POSITION.lock(|cell| cell.get());
                    let err: f32 = (curr_angle - target_angle).to_num();
                    let commanded_speed = (kp * err).clamp(-0.1, 0.1);
                    // defmt::info!("Commanded speed: {}", &commanded_speed);
                    set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, -commanded_speed);

                    ticker.next().await;
                }
            }
            MotorCommand::PositionRaw => {
                esc_stop_pin.set_high();
                esc_brake_pin.set_high();
                // Basic P controller
                let kp: f32 = 0.002;

                let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(200));

                loop {
                    let curr_angle = MOTOR_CURRENT_POSITION.lock(|cell| cell.get());
                    let target_angle = MOTOR_POSITION_SETPOINT.lock(|cell| cell.get());
                    let err: f32 = (curr_angle - target_angle).to_num();
                    let commanded_speed = (kp * err).clamp(-0.1, 0.1);
                    // defmt::info!("Commanded speed: {}", &commanded_speed);
                    set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, -commanded_speed);

                    ticker.next().await;
                }
            }
            MotorCommand::Zero => {
                motor_reset_position_at(I32F32::ZERO);
                spin_async().await;
            }
        };
    };

    let mut motor_state = MotorCommand::Disabled;
    loop {
        match embassy_futures::select::select(
            MOTOR_COMMAND_CHANNEL.receive(),
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
) -> Result<(), &'static str> {
    let commanded_speed = commanded_speed.clamp(-1.0, 1.0);
    esc_dir_pin.set_level(if commanded_speed < 0.0 {
        gpio::Level::High
    } else {
        gpio::Level::Low
    });
    esc_pwm
        .set_duty_cycle(((PWM_TOP) as f32 * commanded_speed.abs()) as u16)
        .map_err(|_| "Failed to set ESC PWM duty cycle")?;
    Ok(())
}
