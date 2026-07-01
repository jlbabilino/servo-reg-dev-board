use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub fn rem(lhs: embassy_time::Duration, rhs: embassy_time::Duration) -> embassy_time::Duration {
    let lhs_ticks = lhs.as_ticks();
    let rhs_ticks = rhs.as_ticks();

    let rem_ticks = lhs_ticks % rhs_ticks;

    embassy_time::Duration::from_ticks(rem_ticks)
}

// pub fn div_int(lhs: embassy_time::Duration, rhs: embassy_time::Duration) -> u64 {
//     let lhs_ticks = lhs.as_ticks();
//     let rhs_ticks = rhs.as_ticks();

//     let div_ticks = lhs_ticks / rhs_ticks;

//     div_ticks
// }

pub fn div(lhs: embassy_time::Duration, rhs: embassy_time::Duration) -> f32 {
    let lhs_ticks = lhs.as_ticks();
    let rhs_ticks = rhs.as_ticks();

    let div_ticks = (lhs_ticks as f32) / (rhs_ticks as f32);

    div_ticks
}

pub const fn const_checked_sub(
    lhs: embassy_time::Duration,
    rhs: embassy_time::Duration,
) -> Option<embassy_time::Duration> {
    let Some(result) = lhs.as_ticks().checked_sub(rhs.as_ticks()) else {
        return None;
    };
    Some(embassy_time::Duration::from_ticks(result))
}

pub const fn const_checked_add(
    lhs: embassy_time::Duration,
    rhs: embassy_time::Duration,
) -> Option<embassy_time::Duration> {
    let Some(result) = lhs.as_ticks().checked_add(rhs.as_ticks()) else {
        return None;
    };
    Some(embassy_time::Duration::from_ticks(result))
}

pub const fn const_checked_mul(
    lhs: embassy_time::Duration,
    rhs: u64,
) -> Option<embassy_time::Duration> {
    let Some(result) = lhs.as_ticks().checked_mul(rhs) else {
        return None;
    };
    Some(embassy_time::Duration::from_ticks(result))
}

pub async fn spin_async() -> ! {
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(100000)).await;
    }
}

pub struct BoolSignal {
    signal: embassy_sync::signal::Signal<CriticalSectionRawMutex, bool>,
}

impl BoolSignal {
    pub const fn new() -> Self {
        BoolSignal {
            signal: embassy_sync::signal::Signal::<CriticalSectionRawMutex, bool>::new(),
        }
    }

    pub async fn wait_for_any_edge(&self) -> bool {
        self.signal.wait().await
    }

    pub async fn wait_for_high(&self) {
        while !self.signal.wait().await {}
    }
}
