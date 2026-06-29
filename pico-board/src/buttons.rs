// Handles button software debouncing

use embassy_futures::select::{Either, select};
use embassy_rp::gpio::{self, Level};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::Timer;

#[embassy_executor::task(pool_size = 4)]
pub async fn button_task(
    mut button: gpio::Input<'static>,
    watch: &'static Watch<CriticalSectionRawMutex, bool, 4>,
) -> ! {
    const DEBOUNCE_MS: u64 = 50;

    let sender = watch.sender();

    loop {
        // Wait until pressed
        button.wait_for_low().await;
        sender.send(true);

        // After a certain amount of time, start waiting for it to be released
        Timer::after_millis(DEBOUNCE_MS).await;
        button.wait_for_high().await;
        sender.send(false);

        Timer::after_millis(DEBOUNCE_MS).await;
    }
}
