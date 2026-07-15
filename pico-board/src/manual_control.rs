// Manual mode operations

use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;
use fixed::{traits::ToFixed, types::I32F32};

use crate::{
    buttons::{self, ButtonPressed},
    constants,
    motor_control::MotorCommand,
    network::NetworkStatus,
    rgb_led,
    types::{
        ButtonWatchReceiver, I32F32Mutex, LEDCommandPublisher, MotorCommandPublisher,
        NetworkStatusWatchReceiver,
    },
};

#[embassy_executor::task]
pub async fn manual_mode_task(
    mut button_1_receiver: ButtonWatchReceiver,
    mut button_2_receiver: ButtonWatchReceiver,
    mut button_3_receiver: ButtonWatchReceiver,
    mut button_4_receiver: ButtonWatchReceiver,
    mut led_pub: LEDCommandPublisher,
    mut motor_cmd_pub: MotorCommandPublisher,
    motor_current_position: &'static I32F32Mutex,
    mut network_status_receiver: NetworkStatusWatchReceiver,
) {
    loop {
        // Wait until network is disconnected to start manual mode
        network_status_receiver
            .changed_and(|val| *val == NetworkStatus::Disconnected)
            .await;

        match select(
            network_status_receiver.changed_and(|val| *val == NetworkStatus::Connected),
            handle_disabled_loop(
                &mut button_1_receiver,
                &mut button_2_receiver,
                &mut button_3_receiver,
                &mut button_4_receiver,
                &mut led_pub,
                &mut motor_cmd_pub,
                motor_current_position,
            ),
        )
        .await
        {
            Either::First(_) => {
                // Network connection was established, so kill manual mode
            }
            Either::Second(_) => {
                defmt::error!("Manual mode disabled loop should never end!");
            }
        }

        // Reset motor in case network connection was established during operation
        motor_cmd_pub.publish(MotorCommand::Disabled).await;
    }
}

async fn handle_disabled_loop(
    button_1_receiver: &mut ButtonWatchReceiver,
    button_2_receiver: &mut ButtonWatchReceiver,
    button_3_receiver: &mut ButtonWatchReceiver,
    button_4_receiver: &mut ButtonWatchReceiver,
    led_pub: &mut LEDCommandPublisher,
    motor_cmd_pub: &mut MotorCommandPublisher,
    motor_current_position: &'static I32F32Mutex,
) {
    loop {
        // Disabled
        motor_cmd_pub.publish(MotorCommand::Disabled).await;
        led_pub
            .publish(rgb_led::Command::Looping(
                constants::DISCONNECTED_DISABLED_ANIM,
            ))
            .await;
        let disabled_waiter = buttons::button_pressed_short_or_long(button_1_receiver);

        let disabled_code = async || {
            defmt::info!("Disabled!");
            loop {
                Timer::after_secs(1).await;
            }
        };

        match select(disabled_waiter, disabled_code()).await {
            Either::First(ButtonPressed::Long) => {
                // If long pressed, move to the next thing
                handle_manual_mode(
                    button_1_receiver,
                    button_2_receiver,
                    button_3_receiver,
                    button_4_receiver,
                    led_pub,
                    motor_cmd_pub,
                    motor_current_position,
                )
                .await;
            }
            Either::First(ButtonPressed::Short) => {
                // If short pressed, you are "disabling" it but it's
                // already disabled, so yeah
                continue;
            }
            Either::Second(_) => {
                defmt::error!("Disabled code written incorrectly!");
                continue;
            }
        };
    }
}

async fn handle_manual_mode(
    button_1_receiver: &mut ButtonWatchReceiver,
    button_2_receiver: &mut ButtonWatchReceiver,
    button_3_receiver: &mut ButtonWatchReceiver,
    button_4_receiver: &mut ButtonWatchReceiver,
    led_pub: &mut LEDCommandPublisher,
    motor_cmd_pub: &mut MotorCommandPublisher,
    motor_current_position: &'static I32F32Mutex,
) {
    loop {
        // Mode 1 - Speed Control
        motor_cmd_pub.publish(MotorCommand::Disabled).await;
        led_pub
            .publish(rgb_led::Command::Looping(constants::MANUAL_MODE_1_ANIM))
            .await;

        let mode_1_waiter = buttons::button_pressed_short_or_long(button_1_receiver);

        let mut mode_1_code = async || {
            defmt::info!("Mode 1!");
            loop {
                let left_button = button_2_receiver.get().await;
                let right_button = button_4_receiver.get().await;

                let motor_command = match (left_button, right_button) {
                    (true, false) => {
                        // Move motor clockwise
                        MotorCommand::Speed(0.1)
                    }
                    (false, true) => {
                        // Move motor counter-clockwise
                        MotorCommand::Speed(-0.1)
                    }
                    (false, false) => MotorCommand::Disabled,
                    (true, true) => MotorCommand::Brake,
                };

                motor_cmd_pub.publish(motor_command).await;

                select(button_2_receiver.changed(), button_4_receiver.changed()).await;
            }
        };

        match select(mode_1_waiter, mode_1_code()).await {
            Either::First(ButtonPressed::Long) => {
                // If long pressed, move to the next thing
            }
            Either::First(ButtonPressed::Short) => {
                // If short pressed, you are "disabling"
                return;
            }
            Either::Second(_) => {
                defmt::error!("Mode 1 code written incorrectly!");
                return;
            }
        };

        // Mode 2 - Position control
        motor_cmd_pub.publish(MotorCommand::Disabled).await;
        led_pub
            .publish(rgb_led::Command::Looping(constants::MANUAL_MODE_2_ANIM))
            .await;

        let mode_2_waiter = buttons::button_pressed_short_or_long(button_1_receiver);

        let left_signal: Signal<NoopRawMutex, u32> = Signal::new();
        let right_signal: Signal<NoopRawMutex, u32> = Signal::new();

        let left_typematic =
            buttons::signal_button_presses_typematic(button_2_receiver, &left_signal);
        let right_typematic =
            buttons::signal_button_presses_typematic(button_4_receiver, &right_signal);

        let mode_2_actual_code = async || {
            defmt::info!("Mode 2!");
            let mut current_pos = motor_current_position.lock(|val| val.get());
            let increment: I32F32 = 6.28319.to_fixed();

            loop {
                motor_cmd_pub
                    .publish(MotorCommand::Position(current_pos))
                    .await;
                match select(left_signal.wait(), right_signal.wait()).await {
                    Either::First(_) => current_pos += increment,
                    Either::Second(_) => current_pos -= increment,
                };
            }
        };

        let mode_2_code =
            embassy_futures::select::select3(left_typematic, right_typematic, mode_2_actual_code());

        match select(mode_2_waiter, mode_2_code).await {
            Either::First(ButtonPressed::Long) => {
                // If long pressed, move to the next thing
            }
            Either::First(ButtonPressed::Short) => {
                // If short pressed, you are "disabling"
                return;
            }
            Either::Second(_) => {
                defmt::error!("Mode 2 code written incorrectly!");
                return;
            }
        };

        // Mode 3
        led_pub
            .publish(rgb_led::Command::Looping(constants::MANUAL_MODE_3_ANIM))
            .await;

        let mode_3_waiter = buttons::button_pressed_short_or_long(button_1_receiver);

        let mode_3_code = async || {
            defmt::info!("Mode 3!");
            loop {
                Timer::after_secs(1).await;
            }
        };

        match select(mode_3_waiter, mode_3_code()).await {
            Either::First(ButtonPressed::Long) => {
                // If long pressed, move to the next thing
            }
            Either::First(ButtonPressed::Short) => {
                // If short pressed, you are "disabling"
                return;
            }
            Either::Second(_) => {
                defmt::error!("Mode 3 code written incorrectly!");
                return;
            }
        };
    }
}

async fn report_button(button_receiver: &mut ButtonWatchReceiver, num: u16) {
    let new_value = button_receiver.changed().await;
    defmt::info!("Button {}: {:?}", num, new_value);
}
