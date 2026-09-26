use core::f32::consts::FRAC_PI_2;

use embassy_executor::raw;
use embassy_futures::select::{Either, select};
use embassy_rp::{
    gpio,
    pwm::{self, SetDutyCycle},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::WaitResult, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use fixed::types::I32F32;

use crate::{
    types::{ButtonWatchReceiver, F32Mutex, I32F32Mutex, MotorCommandSubscriber},
    util::spin_async,
};

#[derive(Copy, Clone, defmt::Format)]
pub enum MotorCommand {
    Disabled,
    Position(I32F32),
    PositionRaw,
    Speed(f32), // 0 = stopped, +1 = max CCW, -1 = max CW
    SpeedRaw,
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
    mut esc_stop_pin: gpio::OutputOpenDrain<'static>,
    mut esc_brake_pin: gpio::Output<'static>,
    mut esc_dir_pin: gpio::OutputOpenDrain<'static>,
    mut esc_pwm: pwm::Pwm<'static>,
    motor_current_position: &'static I32F32Mutex,
    motor_position_setpoint: &'static I32F32Mutex,
    motor_speed_setpoint: &'static F32Mutex,
    mut motor_command_subscriber: MotorCommandSubscriber,
    mut button_3_receiver: ButtonWatchReceiver,
) {
    let mut motor_controller = async |state: &MotorCommand| -> Result<(), &'static str> {
        match state {
            MotorCommand::Disabled => {
                esc_stop_pin.set_low();
                esc_brake_pin.set_low();
                esc_dir_pin.set_high();
                esc_pwm
                    .set_duty_cycle(0)
                    .map_err(|_| "Failed to set ESC PWM duty cycle")?;
                // nothing else to do so just sleep 😴
                spin_async().await;
            }
            MotorCommand::Brake => {
                esc_stop_pin.set_high();
                esc_brake_pin.set_high();
                esc_dir_pin.set_high();
                esc_pwm
                    .set_duty_cycle(0)
                    .map_err(|_| "Failed to set ESC PWM duty cycle")?;
                spin_async().await;
            }
            MotorCommand::Speed(speed) => {
                let speed = speed.clamp(-1., 1.);
                esc_stop_pin.set_high();
                esc_brake_pin.set_low();
                set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, speed)?;
                spin_async().await;
            }
            MotorCommand::Position(target_angle) => {
                esc_stop_pin.set_high();
                esc_brake_pin.set_low();
                // Basic P controller
                let kp: f32 = 0.002;

                let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(200));

                loop {
                    let curr_angle = motor_current_position.lock(|cell| cell.get());
                    let err: f32 = (curr_angle - target_angle).to_num();
                    let commanded_speed = (kp * err).clamp(-1.0, 1.0);
                    // defmt::info!("Commanded speed: {}", &commanded_speed);
                    set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, -commanded_speed)?;

                    ticker.next().await;
                }
            }
            MotorCommand::SpeedRaw => {
                let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(20)); // TODO
                esc_stop_pin.set_high();
                esc_brake_pin.set_low();
                loop {
                    let speed = motor_speed_setpoint.lock(|cell| cell.get());
                    let speed = speed.clamp(-1., 1.);
                    defmt::info!("Commanded speed: {}", &speed); // TODO
                    set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, speed)?;
                    ticker.next().await;
                }
            }
            MotorCommand::PositionRaw => {
                esc_stop_pin.set_high();
                esc_brake_pin.set_low();
                // Basic P controller
                // let kp: f32 = 0.004;
                // let ki: f32 = 0.001;
                // let kd: f32 = 0.00003;
                let kp: f32 = 0.0;
                let ki: f32 = 0.0;
                let kd: f32 = 0.0;

                let mut control_loop = async || -> Result<(), &'static str> {
                    let dt = Duration::from_hz(200);
                    let dt_f32 = dt.as_micros() as f32 / 1e6;
                    let mut ticker = Ticker::every(dt);

                    let mut err_int: f32 = 0.0;
                    let mut prev_angle: I32F32 = motor_current_position.lock(|cell| cell.get());

                    loop {
                        let curr_angle = motor_current_position.lock(|cell| cell.get());
                        let target_angle = motor_position_setpoint.lock(|cell| cell.get());
                        let err_angle: f32 = (curr_angle - target_angle).to_num();

                        if err_angle.abs() < 2.0 {
                            set_motor_speed(&mut esc_dir_pin, &mut esc_pwm, 0.0)?;
                            err_int = 0.0; // Clear integral term inside deadband
                        } else {
                            err_int += err_angle * dt_f32;
                            // Anti-windup clamp on integral term
                            err_int = err_int.clamp(-1.0, 1.0);

                            let err_d = (curr_angle - prev_angle).to_num::<f32>() / dt_f32;

                            let pid_out = -kp * err_angle - ki * err_int - kd * err_d;

                            // Use experimentally determined minimum speeds to
                            // overcome friction
                            let feedforward = if pid_out > 0.0 {
                                // if clockwise
                                0.008 + 0.000617 * f32::max(curr_angle.to_num::<f32>() + 420.0, 0.0)
                            } else {
                                // if counterclockwise
                                -(0.008
                                    + 0.000524 * f32::max(curr_angle.to_num::<f32>() + 420.0, 0.0))
                            };

                            let commanded_speed = feedforward + pid_out;

                            set_motor_speed(
                                &mut esc_dir_pin,
                                &mut esc_pwm,
                                commanded_speed.clamp(-1.0, 1.0),
                            )?;
                        }

                        prev_angle = curr_angle;

                        ticker.next().await;
                    }
                };

                control_loop().await?;
                spin_async().await;
            }
        };
    };

    let mut motor_state = MotorCommand::Disabled;
    loop {
        match embassy_futures::select::select(
            motor_command_subscriber.next_message(),
            motor_controller(&motor_state),
        )
        .await
        {
            Either::First(WaitResult::Message(new_state)) => motor_state = new_state,
            Either::First(WaitResult::Lagged(num_msg)) => {
                defmt::error!("Motor command pubsub lagged! Missed {} messages", num_msg);
            }
            Either::Second(Ok(_)) => {
                // Should not be possible
                defmt::error!("Motor controller loop should NOT finish!");
            }
            Either::Second(Err(msg)) => {
                defmt::error!("Motor controller loop failed: {}", msg);
            }
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
