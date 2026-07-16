use embassy_futures::select::Either;
// Handles button software debouncing
use embassy_rp::gpio;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;

use crate::types::{ButtonWatchReceiver, ButtonWatchSender};

#[embassy_executor::task(pool_size = 4)]
pub async fn button_task(mut button: gpio::Input<'static>, sender: ButtonWatchSender) -> ! {
    const DEBOUNCE_MS: u64 = 50;

    // Have to make sure not to be pressing the buttons when code starts up
    sender.send(button.is_low());

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

pub enum ButtonPressed {
    Short,
    Long,
}

pub async fn button_pressed_short_or_long(
    button_receiver: &mut ButtonWatchReceiver,
) -> ButtonPressed {
    // wait until released (false)
    button_receiver.get_and(|val| !*val).await;

    // wait until pressed (true)
    button_receiver.changed_and(|val| *val).await;

    let until_released = button_receiver.changed_and(|val| !*val);
    let until_considered_long = Timer::after_millis(1000);

    match embassy_futures::select::select(until_released, until_considered_long).await {
        Either::First(_) => {
            // button was released before long threshold
            ButtonPressed::Short
        }
        Either::Second(_) => {
            // button was held until threshold
            ButtonPressed::Long
        }
    }
}

pub async fn signal_button_presses_typematic(
    button_receiver: &mut ButtonWatchReceiver,
    signal: &Signal<NoopRawMutex, u32>,
) -> ! {
    let typematic_sequence = async || -> ! {
        let mut i = 0;
        signal.signal(i);
        i += 1;
        Timer::after_millis(1000).await;
        loop {
            signal.signal(i);
            i += 1;
            Timer::after_millis(100).await;
        }
    };

    loop {
        // wait until pressed, but if already pressed, go ahead and start
        button_receiver.get_and(|val| *val).await;

        let until_released = button_receiver.changed_and(|val| !*val);

        match embassy_futures::select::select(until_released, typematic_sequence()).await {
            Either::First(_) => {
                continue;
            }
            Either::Second(_) => {
                // Should never end
                defmt::error!("Button typematic not implemented correctly!");
                continue;
            }
        }
    }
}
