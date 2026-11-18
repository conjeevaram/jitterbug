//! bulk of teleop, state that lives across ticks, we also define here what happens inside a tick

use super::command::Command;
use super::pacer::Wake;
use super::{ACTIVE, Cfg, IDS, N, Tick};
use crate::bus::BusError;
use crate::rt;
use crate::safety;
use crate::servo::Servo;
use std::time::{Duration, Instant};

pub fn max_step(speed: u64, period: Duration) -> u16 {
    (speed * period.as_micros() as u64 / 1_000_000).max(1) as u16
}

/// worst err between two poses
fn max_err(a: &[u16; N], b: &[u16; N]) -> u16 {
    let mut max_val = 0;

    for &j in &ACTIVE {
        let diff = a[j].abs_diff(b[j]);
        if diff > max_val {
            max_val = diff;
        }
    }
    max_val
}

const MAX_FAILS: u32 = 3;

/// we can tolerate short bursts of leader read failure
#[derive(Default)]
struct FailGuard {
    fails: u32,
    first: Option<Instant>,
}

impl FailGuard {
    /// true - read is usable, false - skip this tick, err - too many consecutive failures
    fn usable(&mut self, read: Result<(), BusError>, now: Instant) -> Result<bool, BusError> {
        let Err(e) = read else {
            *self = FailGuard::default();
            return Ok(true);
        };
        self.fails += 1;
        let since = *self.first.get_or_insert(now);
        if self.fails < MAX_FAILS {
            return Ok(false);
        }
        eprintln!(
            "abort: {} failed reads over {} us",
            self.fails,
            since.elapsed().as_micros()
        );
        Err(e)
    }
}

pub struct Session {
    lim: [(u16, u16); N],
    ids: [u8; N],
    goal: [u16; N],
    max_step: u16,
    guard: FailGuard,
    command: Command,
    pub follower_read_fails: u32,
}

impl Session {
    /// read limits and curr follower pose then hold that pose with torque on
    pub fn start<S: Servo>(
        follower: &mut S,
        period: Duration,
        cfg: Cfg,
        ticks: u32,
    ) -> Result<Session, BusError> {
        let mut lim = [(0u16, 0u16); N];
        for (i, &id) in IDS.iter().enumerate() {
            lim[i] = follower.read_limits(id)?;
        }
        let mut goal = [0u16; N];
        follower.sync_read_positions(&IDS, &mut goal)?;

        let command = match cfg.stimulus {
            None => match cfg.lead_ms {
                None => Command::Leader,
                Some(ms) => Command::lead(ms / 1e3 / period.as_secs_f64()),
            },
            Some(signal) => {
                if !signal.fits(goal[signal.joint], lim[signal.joint]) {
                    eprintln!("stimulus: +- {} ticks oustide joint lims", signal.amp);
                    let kind = std::io::ErrorKind::InvalidInput;
                    return Err(BusError::Io(std::io::Error::new(kind, "stimulus range")));
                }
                Command::Stimulus {
                    signal,
                    hold: goal,
                    total_s: ticks as f64 * period.as_secs_f64(),
                }
            }
        };

        let mut ids = [0u8; N];
        for (n, &j) in ACTIVE.iter().enumerate() {
            ids[n] = IDS[j];
            follower.write_goal(IDS[j], goal[j])?;
            follower.set_torque(IDS[j], true)?;
        }
        Ok(Session {
            lim,
            ids,
            goal,
            max_step: max_step(cfg.max_speed, period),
            guard: FailGuard::default(),
            command,
            follower_read_fails: 0,
        })
    }

    /// for some tick: read both arms, shape and write goals, or do nothing if we're skipping the tick
    pub fn step<S: Servo>(
        &mut self,
        leader: &mut S,
        follower: &mut S,
        wake: &Wake,
    ) -> Result<Option<Tick>, BusError> {
        let core = rt::cpu();
        let mut l = [0u16; N];
        let mut meas = [0u16; N];
        let lb = leader.begin_sync_read(&IDS);
        let fb = follower.begin_sync_read(&IDS);
        let lr = lb.and_then(|_| leader.finish_sync_read(&IDS, &mut l));
        let fr = fb.and_then(|_| follower.finish_sync_read(&IDS, &mut meas));
        if !self.guard.usable(lr, wake.woke)? {
            return Ok(None);
        }
        let read = Instant::now();

        let (trk, e2e) = match fr {
            Ok(()) => (max_err(&self.goal, &meas), max_err(&l, &meas)),
            Err(_) => {
                self.follower_read_fails += 1;
                meas = [u16::MAX; N];
                (u16::MAX, u16::MAX)
            }
        };

        let target = self.command.target(&l, wake.t);
        let mut goals = [0u16; N];
        for (n, &j) in ACTIVE.iter().enumerate() {
            let (lo, hi) = self.lim[j];
            self.goal[j] = safety::shape(target[j], self.goal[j], lo, hi, self.max_step);
            goals[n] = self.goal[j];
        }
        follower.sync_write_goals(&self.ids, &goals)?;
        let written = Instant::now();

        Ok(Some(Tick {
            late_us: wake.late.as_micros(),
            read_us: read.duration_since(wake.woke).as_micros(),
            write_us: written.duration_since(read).as_micros(),
            core,
            missed: wake.missed,
            trk,
            e2e,
            leader: l,
            goal: self.goal,
            meas,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn step_scales_with_period() {
        assert_eq!(max_step(2000, Duration::from_micros(5000)), 10);
        assert_eq!(max_step(2000, Duration::from_micros(3003)), 6);
        assert_eq!(max_step(2000, Duration::from_micros(1)), 1);
    }
}
