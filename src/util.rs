pub fn rem(lhs: embassy_time::Duration, rhs: embassy_time::Duration) -> embassy_time::Duration {
    let lhs_ticks = lhs.as_ticks();
    let rhs_ticks = rhs.as_ticks();

    let rem_ticks = lhs_ticks % rhs_ticks;

    embassy_time::Duration::from_ticks(rem_ticks)
}

pub fn div_int(lhs: embassy_time::Duration, rhs: embassy_time::Duration) -> u64 {
    let lhs_ticks = lhs.as_ticks();
    let rhs_ticks = rhs.as_ticks();

    let div_ticks = lhs_ticks / rhs_ticks;

    div_ticks
}

pub fn div(lhs: embassy_time::Duration, rhs: embassy_time::Duration) -> f32 {
    let lhs_ticks = lhs.as_ticks();
    let rhs_ticks = rhs.as_ticks();

    let div_ticks = (lhs_ticks as f32) / (rhs_ticks as f32);

    div_ticks
}
