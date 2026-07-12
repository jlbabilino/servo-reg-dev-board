// Handles button software debouncing
use embassy_rp::gpio;
use embassy_time::Timer;

use crate::types::ButtonWatchSender;

#[embassy_executor::task(pool_size = 4)]
pub async fn button_task(mut button: gpio::Input<'static>, sender: ButtonWatchSender) -> ! {
    const DEBOUNCE_MS: u64 = 50;

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
