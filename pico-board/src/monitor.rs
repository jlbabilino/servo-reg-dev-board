use embassy_futures::select;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Ticker};

use crate::types::{
    CMDFromPCSubscriber, F32Mutex, I32F32Mutex, LEDCommandSubscriber, MotorCommandSubscriber,
    NetworkStatusWatchReceiver, QuadratureCommandWatchReceiver, QuadratureErrorWatchReceiver,
};

#[embassy_executor::task]
pub async fn monitor_task(
    mut network_status_watch_receiver: NetworkStatusWatchReceiver,
    mut cmd_from_pc_subscriber: CMDFromPCSubscriber,
    motor_current_position: &'static I32F32Mutex,
    mut quadrature_error_watch_receiver: QuadratureErrorWatchReceiver,
    mut quadrature_command_watch_receiver: QuadratureCommandWatchReceiver,
    mut motor_command_subscriber: MotorCommandSubscriber,
    motor_position_setpoint: &'static I32F32Mutex,
    motor_speed_setpoint: &'static F32Mutex,
    mut led_command_subscriber: LEDCommandSubscriber,
) {
    loop {
        let parallel_future = select::select6(
            report_network_status(&mut network_status_watch_receiver),
            report_cmd_from_pc(&mut cmd_from_pc_subscriber),
            report_quadrature_error(&mut quadrature_error_watch_receiver),
            report_quadrature_command(&mut quadrature_command_watch_receiver),
            report_motor_command(&mut motor_command_subscriber),
            report_mutex_periodic(
                &motor_current_position,
                &motor_position_setpoint,
                &motor_speed_setpoint,
            ),
        );
        let parallel_future = select::select(
            parallel_future,
            report_led_command(&mut led_command_subscriber),
        );
        parallel_future.await;
    }
}

async fn report_network_status(network_status_watch_receiver: &mut NetworkStatusWatchReceiver) {
    let new_value = network_status_watch_receiver.changed().await;
    defmt::info!("network status: {:?}", new_value);
}

async fn report_cmd_from_pc(cmd_from_pc_subscriber: &mut CMDFromPCSubscriber) {
    let result = cmd_from_pc_subscriber.next_message().await;
    match result {
        WaitResult::Lagged(num_msg) => {
            defmt::error!("CMD from PC pubsub lagged! Missed {} messages", num_msg);
        }
        WaitResult::Message(new_value) => {
            defmt::info!("CMD from PC: {:?}", new_value);
        }
    }
}

async fn report_quadrature_error(
    quadrature_error_watch_receiver: &mut QuadratureErrorWatchReceiver,
) {
    let new_value = quadrature_error_watch_receiver.changed().await;
    defmt::info!("Quadrature error received");
}

async fn report_quadrature_command(
    quadrature_command_watch_receiver: &mut QuadratureCommandWatchReceiver,
) {
    let new_value = quadrature_command_watch_receiver.changed().await;
    defmt::info!("Quadrature command: {:?}", new_value);
}

async fn report_motor_command(motor_command_subscriber: &mut MotorCommandSubscriber) {
    let result = motor_command_subscriber.next_message().await;
    match result {
        WaitResult::Lagged(num_msg) => {
            defmt::error!("Motor command pubsub lagged! Missed {} messages", num_msg);
        }
        WaitResult::Message(new_value) => {
            defmt::info!("Motor command: {:?}", new_value);
        }
    }
}

async fn report_led_command(led_command_subscriber: &mut LEDCommandSubscriber) {
    let result = led_command_subscriber.next_message().await;
    match result {
        WaitResult::Lagged(num_msg) => {
            defmt::error!("LED command pubsub lagged! Missed {} messages", num_msg);
        }
        WaitResult::Message(new_value) => {
            defmt::info!("LED Command: {:?}", new_value);
        }
    }
}

async fn report_mutex_periodic(
    motor_current_position: &'static I32F32Mutex,
    motor_position_setpoint: &'static I32F32Mutex,
    motor_speed_setpoint: &'static F32Mutex,
) {
    let mut ticker = Ticker::every(Duration::from_hz(10));

    loop {
        let motor_current_position_value: f32 =
            motor_current_position.lock(|cell| cell.get()).to_num();
        let motor_position_setpoint_value: f32 =
            motor_position_setpoint.lock(|cell| cell.get()).to_num();
        let motor_speed_setpoint_value: f32 = motor_speed_setpoint.lock(|cell| cell.get());

        // defmt::info!(
        //     "motor current position: {} deg",
        //     motor_current_position_value * (180. / core::f32::consts::PI)
        // );
        // defmt::info!("motor position setpoint: {}", motor_position_setpoint_value);
        // defmt::info!("motor speed setpoint: {}", motor_speed_setpoint_value);

        ticker.next().await;
    }
}
