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

pub fn cpu() -> i32 {
    unsafe { libc::sched_getcpu() }
}

pub fn thread_cpu_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

pub fn pin_to_core(core: usize) {
    let r = unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_SET(core, &mut set);
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set)
    };
    if r != 0 {
        eprintln!("pin to core failed")
    }
}
