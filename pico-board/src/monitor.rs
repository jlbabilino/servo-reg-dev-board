use core::cell::Cell;

use embassy_sync::{
    blocking_mutex::{self, raw::CriticalSectionRawMutex},
    watch,
};
use fixed::types::I32F32;

use crate::network::NetworkStatus;

#[embassy_executor::task]
pub async fn monitor_task(
    network_status_ind_monitor: watch::Receiver<'static, CriticalSectionRawMutex, NetworkStatus, 4>,
    motor_current_position: &'static blocking_mutex::Mutex<CriticalSectionRawMutex, Cell<I32F32>>,
) {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_hz(1));

    loop {
        let cum_theta: f32 = motor_current_position.lock(|cell| cell.get()).to_num();

        defmt::info!("angle = {} deg", cum_theta * (180. / core::f32::consts::PI),);

        ticker.next().await;
    }
}
