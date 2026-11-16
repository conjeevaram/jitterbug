//! fixed rate leader to follower teleop

use crate::{bus, safety};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::sleep;
use std::time::{Duration, Instant};

const IDS: [u8; 6] = [1, 2, 3, 4, 5, 6];
const MAX_STEP: u16 = 10;
const N: usize = 6;
const ACTIVE: [usize; N] = [0, 1, 2, 3, 4, 5];
const PERIOD: Duration = Duration::from_micros(5000);
const MAX_FAILS: u32 = 3;

pub static STOP: AtomicBool = AtomicBool::new(false);

pub struct Tick {
    pub late_us: u128,
    pub read_us: u128,
    pub write_us: u128,
}

pub fn run(
    leader: &mut bus::Bus,
    follower: &mut bus::Bus,
    ticks: u32,
    log: &mut Vec<Tick>,
) -> Result<(), bus::BusError> {
    let mut lim = [(0u16, 0u16); 6];
    for (i, &id) in IDS.iter().enumerate() {
        lim[i] = follower.read_limits(id)?;
    }
    let mut l = [0u16; 6];
    let mut goal = [0u16; 6];
    follower.sync_read_positions(&IDS, &mut goal)?;

    let mut ids = [0u8; N];
    let mut goals = [0u16; N];
    for (n, &j) in ACTIVE.iter().enumerate() {
        ids[n] = IDS[j];
        follower.write_goal(IDS[j], goal[j])?;
        follower.set_torque(IDS[j], true)?;
    }

    let mut fails = 0u32;
    let t0 = Instant::now();
    for k in 0..ticks {
        if STOP.load(Ordering::Relaxed) {
            break;
        }
        let deadline = t0 + PERIOD * k;
        let now = Instant::now();
        if deadline > now {
            sleep(deadline - now);
        }
        let woke = Instant::now();
        match leader.sync_read_positions(&IDS, &mut l) {
            Ok(_) => fails = 0,
            Err(e) => {
                fails += 1;
                if fails >= MAX_FAILS {
                    return Err(e);
                }
                continue;
            }
        }
        let read = Instant::now();
        for (n, &j) in ACTIVE.iter().enumerate() {
            goal[j] = safety::shape(l[j], goal[j], lim[j].0, lim[j].1, MAX_STEP);
            goals[n] = goal[j];
        }
        follower.sync_write_goals(&ids, &goals)?;
        let written = Instant::now();
        log.push(Tick {
            late_us: woke.saturating_duration_since(deadline).as_micros(),
            read_us: read.duration_since(woke).as_micros(),
            write_us: written.duration_since(read).as_micros(),
        });
    }
    Ok(())
}

pub fn release(follower: &mut bus::Bus) -> Result<(), bus::BusError> {
    let mut res = Ok(());
    for &j in ACTIVE.iter() {
        if let Err(e) = follower.set_torque(IDS[j], false) {
            res = Err(e);
        }
    }
    res
}
