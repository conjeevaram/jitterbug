//! fixed rate leader to follower teleop

mod command;
mod pacer;
mod session;
#[cfg(test)]
mod tests;

use crate::bus::BusError;
use crate::rt;
use crate::servo::Servo;
use crate::stimulus::Stimulus;
use pacer::Pacer;
use session::Session;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

const IDS: [u8; 6] = [1, 2, 3, 4, 5, 6];
pub const MAX_SPEED: u64 = 2000; // servo ticks/second (10 per tick at 200hz)
const N: usize = IDS.len();
const ACTIVE: [usize; N] = [0, 1, 2, 3, 4, 5];

pub static STOP: AtomicBool = AtomicBool::new(false);

pub struct Cfg {
    pub max_speed: u64,
    pub stimulus: Option<Stimulus>,
}

pub struct Tick {
    pub late_us: u128,
    pub read_us: u128,
    pub write_us: u128,
    pub core: i32,
    pub missed: u32,
    pub trk: u16,
    pub e2e: u16,
    pub leader: [u16; N],
    pub goal: [u16; N],
    pub meas: [u16; N],
}

pub fn run<S: Servo>(
    leader: &mut S,
    follower: &mut S,
    ticks: u32,
    period: Duration,
    cfg: Cfg,
    log: &mut Vec<Tick>,
) -> Result<(), BusError> {
    let mut session = Session::start(follower, period, cfg, ticks)?;
    let mut pacer = Pacer::new(period, ticks);
    let cpu0 = rt::thread_cpu_ns();
    while !STOP.load(Ordering::Relaxed) && !pacer.done() {
        let wake = pacer.wait();
        if let Some(tick) = session.step(leader, follower, &wake)? {
            log.push(tick);
        }
    }
    eprintln!("missed ticks: {}", pacer.skipped);
    eprintln!("follower read fails: {}", session.follower_read_fails);
    let cpu = rt::thread_cpu_ns() - cpu0;
    eprintln!(
        "cpu: {:.1}% of wall",
        100.0 * cpu as f64 / pacer.elapsed().as_nanos() as f64
    );
    Ok(())
}

pub fn release<S: Servo>(follower: &mut S) -> Result<(), BusError> {
    let mut res = Ok(());
    for &j in ACTIVE.iter() {
        if let Err(e) = follower.set_torque(IDS[j], false) {
            res = Err(e);
        }
    }
    res
}
