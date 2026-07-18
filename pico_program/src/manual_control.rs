// Manual mode operations

use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::WaitResult, signal::Signal};
use embassy_time::Timer;
use fixed::{traits::ToFixed, types::I32F32};
use shared_types::{CmdFromPC, ResponseToPC};

use crate::{
    buttons::{self, ButtonPressed},
    constants,
    motor_control::MotorCommand,
    motor_quadrature::QuadratureCommand,
    network::NetworkStatus,
    rgb_led,
    types::{
        ButtonWatchReceiver, CMDFromPCSubscriber, I32F32Mutex, LEDCommandPublisher,
        MotorCommandPublisher, NetworkStatusWatchReceiver, QuadratureCommandWatchSender,
        QuadratureErrorWatchReceiver, ResponseToPCPublisher,
    },
    util::spin_async,
};

// TODO: Change name of this to "state manager" or something
#[embassy_executor::task]
pub async fn manual_mode_task(
    mut button_1_receiver: ButtonWatchReceiver,
    mut button_2_receiver: ButtonWatchReceiver,
    mut button_3_receiver: ButtonWatchReceiver,
    mut button_4_receiver: ButtonWatchReceiver,
    mut led_pub: LEDCommandPublisher,
    mut motor_cmd_pub: MotorCommandPublisher,
    mut quad_cmd_sender: QuadratureCommandWatchSender,
    mut quad_err_receiver: QuadratureErrorWatchReceiver,
    motor_current_position: &'static I32F32Mutex,
    mut network_status_receiver: NetworkStatusWatchReceiver,
    mut cmd_from_pc_subscriber: CMDFromPCSubscriber,
    mut resp_to_pc_pub: ResponseToPCPublisher,
) {
    loop {
        match network_status_receiver.get().await {
            NetworkStatus::Connected => {
                match select(
                    network_status_receiver.changed_and(|val| *val == NetworkStatus::Disconnected),
                    handle_network_loop(
                        &mut led_pub,
                        &mut motor_cmd_pub,
                        &mut quad_cmd_sender,
                        &mut quad_err_receiver,
                        motor_current_position,
                        &mut cmd_from_pc_subscriber,
                        &mut resp_to_pc_pub,
                    ),
                )
                .await
                {
                    Either::First(_) => {
                        // Network connection was dropped
                    }
                    Either::Second(_) => {
                        defmt::error!("Network loop should never end!");
                    }
                }
            }
            NetworkStatus::Disconnected => {
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
            }
        }

        // Reset motor in case of abrupt close of network connection or manual mode.
        // Should get disabled when looping back to the network or manual loop
        // But this will act instantly (as a redundancy)
        motor_cmd_pub.publish(MotorCommand::Disabled).await;
    }
}

async fn handle_network_loop(
    led_pub: &mut LEDCommandPublisher,
    motor_cmd_pub: &mut MotorCommandPublisher,
    quad_cmd_sender: &mut QuadratureCommandWatchSender,
    quad_err_receiver: &mut QuadratureErrorWatchReceiver,
    motor_current_position: &'static I32F32Mutex,
    cmd_from_pc_subscriber: &mut CMDFromPCSubscriber,
    resp_to_pc_pub: &mut ResponseToPCPublisher,
) -> ! {
    #[derive(PartialEq, Copy, Clone)]
    enum NetworkDriveState {
        Disabled,
        SpeedControl,
        PositionControl,
    }

    let state_signal = Signal::<NoopRawMutex, NetworkDriveState>::new();
    let reset_position_signal = Signal::<NoopRawMutex, I32F32>::new();

    let wait_for_new_state = async |prev_state: NetworkDriveState| {
        loop {
            let new_state = state_signal.wait().await;

            if new_state != prev_state {
                return new_state;
            }
        }
    };

    let position_reset_loop = async || -> ! {
        loop {
            let reset_pos = reset_position_signal.wait().await;
            quad_cmd_sender.send(QuadratureCommand::ResetAt(reset_pos));
        }
    };

    let state_loop = async || {
        // Start out disabled, can be changed once network sends a packet to change it
        let mut state = NetworkDriveState::Disabled;

        let state_loop_inner = async |state: NetworkDriveState| -> ! {
            match state {
                NetworkDriveState::Disabled => {
                    motor_cmd_pub.publish(MotorCommand::Disabled).await;
                    resp_to_pc_pub.publish(ResponseToPC::Disabled).await;
                    led_pub
                        .publish(rgb_led::Command::Looping(
                            constants::CONNECTED_DISABLED_ANIM,
                        ))
                        .await;
                    spin_async().await;
                }
                NetworkDriveState::SpeedControl => {
                    motor_cmd_pub.publish(MotorCommand::SpeedRaw).await;
                    resp_to_pc_pub
                        .publish(ResponseToPC::EnabledSpeedControl)
                        .await;
                    led_pub
                        .publish(rgb_led::Command::Looping(constants::NETWORK_ENABLED_ANIM))
                        .await;
                    spin_async().await;
                }
                NetworkDriveState::PositionControl => {
                    motor_cmd_pub.publish(MotorCommand::PositionRaw).await;
                    resp_to_pc_pub
                        .publish(ResponseToPC::EnabledPositionControl)
                        .await;
                    led_pub
                        .publish(rgb_led::Command::Looping(constants::NETWORK_ENABLED_ANIM))
                        .await;
                    spin_async().await;
                }
            };
        };

        loop {
            match select(wait_for_new_state(state), state_loop_inner(state)).await {
                Either::First(next_state) => {
                    state = next_state;
                }
                Either::Second(_) => {
                    defmt::error!("State inner loop should never end! Check code");
                }
            }
        }
    };

    let mut outer_loop = async || -> ! {
        loop {
            match select3(
                state_loop(),
                position_reset_loop(),
                quad_err_receiver.changed(),
            )
            .await
            {
                Either3::First(_) => {
                    defmt::error!("Network state loop should never end! Check code.");
                }
                Either3::Second(_) => {
                    defmt::error!(
                        "Network state position reset loop should never end! Check code."
                    );
                }
                Either3::Third(_) => {
                    led_pub
                        .publish(rgb_led::Command::Transient(
                            constants::QUADRATURE_ERROR_ANIM,
                        ))
                        .await;
                    defmt::error!(
                        "Resetting network driven state to disabled after quadrature error!"
                    );
                }
            };
        }
    };

    let mut cmd_receiver_loop = async || -> ! {
        loop {
            let WaitResult::Message(packet) = cmd_from_pc_subscriber.next_message().await else {
                defmt::error!("CMD from PC subscriber lagged!");
                continue;
            };

            match packet {
                CmdFromPC::Disable => state_signal.signal(NetworkDriveState::Disabled),
                CmdFromPC::Heartbeat => {}
                CmdFromPC::EnablePositionControl => {
                    state_signal.signal(NetworkDriveState::PositionControl)
                }
                CmdFromPC::EnableSpeedControl => {
                    state_signal.signal(NetworkDriveState::SpeedControl)
                }
                CmdFromPC::ResetPosition(new_pos) => {
                    reset_position_signal.signal(new_pos.to_fixed())
                }
            }
        }
    };

    loop {
        select(outer_loop(), cmd_receiver_loop()).await;

        defmt::error!("Neither outer loop nor cmd receiver loop should end! Check code");
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
) -> ! {
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

// TODO: Handle quad err receiver for manual mode (should boot back to disabled
// with some kind of transient animation and defmt message)
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
