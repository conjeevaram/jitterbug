//! timer and scheduler knobs for control thread using c library

pub fn set_timer_slack_ns(ns: u64) {
    let r = unsafe {
        libc::prctl(
            libc::PR_SET_TIMERSLACK,
            ns as libc::c_ulong,
            0 as libc::c_ulong,
            0 as libc::c_ulong,
            0 as libc::c_ulong,
        )
    };
    if r != 0 {
        eprintln!("PR_SET_TIMERSLACK failed");
    }
}

pub fn timer_slack_ns() -> i64 {
    unsafe {
        libc::prctl(
            libc::PR_GET_TIMERSLACK,
            0 as libc::c_ulong,
            0 as libc::c_ulong,
            0 as libc::c_ulong,
            0 as libc::c_ulong,
        ) as i64
    }
}
