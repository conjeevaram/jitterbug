//! self explanatory

use super::session::max_step;
use super::*;
use crate::stimulus::Wave;

const P: Duration = Duration::from_millis(5);

fn run<S: Servo>(
    leader: &mut S,
    follower: &mut S,
    ticks: u32,
    period: Duration,
    log: &mut Vec<Tick>,
) -> Result<(), BusError> {
    let cfg = Cfg {
        max_speed: MAX_SPEED,
        stimulus: None,
    };
    super::run(leader, follower, ticks, period, cfg, log)
}

fn stimulus(joint: usize, amp: f64, wave: Wave) -> Cfg {
    Cfg {
        max_speed: 100_000,
        stimulus: Some(Stimulus { joint, amp, wave }),
    }
}

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
    fn begin_sync_read(&mut self, _ids: &[u8]) -> Result<(), BusError> {
        Ok(())
    }
    fn finish_sync_read(&mut self, ids: &[u8], out: &mut [u16]) -> Result<(), BusError> {
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
    assert!(run(&mut leader, &mut follower, 20, P, &mut log).is_err());
    assert_eq!(leader.reads, 3);
    assert!(follower.goals.is_empty());
}

#[test]
fn success_resets_the_failure_count() {
    let mut leader = Mock::new(2000);
    leader.script = vec![false, false, true, false, false];
    let mut follower = Mock::new(2000);
    let mut log = Vec::new();
    assert!(
        run(
            &mut leader,
            &mut follower,
            10,
            Duration::from_millis(20),
            &mut log
        )
        .is_ok()
    );
    assert_eq!(follower.goals.len(), 6); // 10 ticks 4 skipped
}

#[test]
fn follower_write_failure_aborts_immediately() {
    let mut leader = Mock::new(2000);
    let mut follower = Mock::new(2000);
    follower.fail_writes = true;
    let mut log = Vec::new();
    assert!(run(&mut leader, &mut follower, 10, P, &mut log).is_err());
    assert_eq!(leader.reads, 1);
}

#[test]
fn goals_respect_limits_and_step_cap() {
    let mut leader = Mock::new(4000); // way past follower's limit
    let mut follower = Mock::new(2000);
    follower.lim = (0, 3000);
    let mut log = Vec::new();
    assert!(run(&mut leader, &mut follower, 120, P, &mut log).is_ok());
    let mut prev = [2000u16; 6];
    for g in &follower.goals {
        for j in 0..6 {
            assert!(g[j] <= 3000);
            assert!(g[j].abs_diff(prev[j]) <= max_step(MAX_SPEED, P));
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
fn logs_tracking_and_teleop_error() {
    let mut leader = Mock::new(2100);
    let mut follower = Mock::new(2000);
    let mut log = Vec::new();
    run(&mut leader, &mut follower, 3, P, &mut log).unwrap();
    assert_eq!((log[0].trk, log[0].e2e), (0, 100));
    assert_eq!(log[1].trk, 10); // goal moves one capped step, follower doesnt
}

#[test]
fn follower_read_failure_is_logged_not_fatal() {
    let mut leader = Mock::new(2000);
    let mut follower = Mock::new(2000);
    follower.script = vec![true, false]; // startup read passes, first tick fails
    let mut log = Vec::new();
    assert!(run(&mut leader, &mut follower, 3, P, &mut log).is_ok());
    assert_eq!(log[0].trk, u16::MAX);
    assert_ne!(log[1].trk, u16::MAX);
}

#[test]
fn stimulus_moves_one_joint_and_holds_the_rest() {
    let mut leader = Mock::new(3000); // ignored during a stimulus
    let mut follower = Mock::new(2000);
    let mut log = Vec::new();
    let cfg = stimulus(2, 50.0, Wave::Chirp { f0: 2.0, f1: 5.0 });
    super::run(&mut leader, &mut follower, 200, P, cfg, &mut log).unwrap();
    for g in &follower.goals {
        for j in [0, 1, 3, 4, 5] {
            assert_eq!(g[j], 2000);
        }
        assert!(g[2].abs_diff(2000) <= 50);
    }
    assert!(follower.goals.iter().any(|g| g[2] > 2020));
    assert!(follower.goals.iter().any(|g| g[2] < 1980));
}

#[test]
fn stimulus_refuses_to_start_near_a_limit() {
    let mut leader = Mock::new(2000);
    let mut follower = Mock::new(2000);
    follower.lim = (1980, 4095);
    let mut log = Vec::new();
    let cfg = stimulus(0, 50.0, Wave::Steps { hold: 1.0 });
    assert!(super::run(&mut leader, &mut follower, 10, P, cfg, &mut log).is_err());
    assert!(follower.goals.is_empty() && follower.torque.is_empty());
}
