//! defining follower target pose

use super::N;
use crate::stimulus::Stimulus;

pub enum Command {
    Leader,
    Stimulus {
        signal: Stimulus,
        hold: [u16; N], // one joint uses signal, the rest hold start pose
        total_s: f64,
    },
}

impl Command {
    pub fn target(&self, leader: &[u16; N], t: f64) -> [u16; N] {
        match self {
            Command::Leader => *leader,
            Command::Stimulus {
                signal,
                hold,
                total_s,
            } => {
                let mut target = *hold;
                let j = signal.joint;
                target[j] = (hold[j] as f64 + signal.offset(t, *total_s)).round() as u16;
                target
            }
        }
    }
}
