//! fixed rate leader to follower teleop

use crate::bus::BusError;
use crate::rt;
use crate::safety;
use crate::servo::Servo;
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
    pub core: i32,
    pub missed: u32,
}

fn missed_ticks(deadline: Instant, now: Instant, period: Duration) -> u32 {
    (now.saturating_duration_since(deadline).as_nanos() / period.as_nanos()) as u32
}

pub fn run<S: Servo>(
    leader: &mut S,
    follower: &mut S,
    ticks: u32,
    log: &mut Vec<Tick>,
) -> Result<(), BusError> {
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
    let mut skipped = 0u32;
    let mut k = 0u32;
    let t0 = Instant::now();
    while k < ticks {
        if STOP.load(Ordering::Relaxed) {
            break;
        }
        let deadline = t0 + PERIOD * k;
        let now = Instant::now();
        if deadline > now {
            sleep(deadline - now);
        }

        let woke = Instant::now();
        let core = rt::cpu();
        let missed = missed_ticks(deadline, woke, PERIOD);
        k += 1 + missed;
        skipped += missed;

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
            core,
            missed,
        });
    }
    eprintln!("missed ticks: {skipped}");
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

#[cfg(test)]
mod tests {
    use super::*;

    fn err() -> BusError {
        BusError::Io(std::io::Error::new(std::io::ErrorKind::TimedOut, "mock"))
    }

    struct Mock {
        pos: [u16; 6],
        lim: (u16, u16),
        script: Vec<bool>, // leader read outcomes (pass or fail)
        reads: usize,
        fail_writes: bool,
        torque_fail: Option<u8>,
        goals: Vec<[u16; 6]>,
        torque: Vec<(u8, bool)>,
    }

    impl Mock {
        fn new(pos: u16) -> Self {
            Mock {
                pos: [pos; 6],
                lim: (0, 4095),
                script: vec![],
                reads: 0,
                fail_writes: false,
                torque_fail: None,
                goals: vec![],
                torque: vec![],
            }
        }
    }

    impl Servo for Mock {
        fn read_limits(&mut self, _id: u8) -> Result<(u16, u16), BusError> {
            Ok(self.lim)
        }
        fn sync_read_positions(&mut self, ids: &[u8], out: &mut [u16]) -> Result<(), BusError> {
            let ok = self.script.get(self.reads).copied().unwrap_or(true);
            self.reads += 1;
            if !ok {
                return Err(err());
            }
            out[..ids.len()].copy_from_slice(&self.pos[..ids.len()]);
            Ok(())
        }
        fn sync_write_goals(&mut self, _ids: &[u8], goals: &[u16]) -> Result<(), BusError> {
            if self.fail_writes {
                return Err(err());
            }
            let mut g = [0u16; 6];
            g[..goals.len()].copy_from_slice(goals);
            self.goals.push(g);
            Ok(())
        }
        fn write_goal(&mut self, _id: u8, _pos: u16) -> Result<(), BusError> {
            Ok(())
        }
        fn set_torque(&mut self, id: u8, on: bool) -> Result<(), BusError> {
            self.torque.push((id, on));
            if self.torque_fail == Some(id) {
                return Err(err());
            }
            Ok(())
        }
    }

    #[test]
    fn aborts_after_three_consecutive_leader_failures() {
        let mut leader = Mock::new(2000);
        leader.script = vec![false; 3];
        let mut follower = Mock::new(2000);
        let mut log = Vec::new();
        assert!(run(&mut leader, &mut follower, 20, &mut log).is_err());
        assert_eq!(leader.reads, 3);
        assert!(follower.goals.is_empty());
    }

    #[test]
    fn success_resets_the_failure_count() {
        let mut leader = Mock::new(2000);
        leader.script = vec![false, false, true, false, false];
        let mut follower = Mock::new(2000);
        let mut log = Vec::new();
        assert!(run(&mut leader, &mut follower, 10, &mut log).is_ok());
        assert_eq!(follower.goals.len(), 6); // 10 ticks 4 skipped
    }

    #[test]
    fn follower_write_failure_aborts_immediately() {
        let mut leader = Mock::new(2000);
        let mut follower = Mock::new(2000);
        follower.fail_writes = true;
        let mut log = Vec::new();
        assert!(run(&mut leader, &mut follower, 10, &mut log).is_err());
        assert_eq!(leader.reads, 1);
    }

    #[test]
    fn goals_respect_limits_and_step_cap() {
        let mut leader = Mock::new(4000); // way past follower's limit
        let mut follower = Mock::new(2000);
        follower.lim = (0, 3000);
        let mut log = Vec::new();
        assert!(run(&mut leader, &mut follower, 120, &mut log).is_ok());
        let mut prev = [2000u16; 6];
        for g in &follower.goals {
            for j in 0..6 {
                assert!(g[j] <= 3000);
                assert!(g[j].abs_diff(prev[j]) <= MAX_STEP);
            }
            prev = *g;
        }
        assert_eq!(prev, [3000; 6]);
    }

    #[test]
    fn release_tries_every_joint_even_if_one_fails() {
        let mut follower = Mock::new(2000);
        follower.torque_fail = Some(3);
        assert!(release(&mut follower).is_err());
        assert_eq!(follower.torque.iter().filter(|(_, on)| !on).count(), 6);
    }

    #[test]
    fn counts_whole_periods_behind() {
        let t = Instant::now();
        let ms = Duration::from_millis;
        assert_eq!(missed_ticks(t + ms(10), t, ms(5)), 0);
        assert_eq!(missed_ticks(t, t + ms(4), ms(5)), 0);
        assert_eq!(missed_ticks(t, t + ms(12), ms(5)), 2);
    }
}
