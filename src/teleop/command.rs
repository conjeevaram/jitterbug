//! defining follower target pose

use super::N;
use crate::stimulus::Stimulus;
use std::collections::VecDeque;

/// size of leader pose window used to estimate its speed for lead compensation
const LEAD_WINDOW: usize = 10;

pub enum Command {
    Leader,
    Lead {
        ahead: f64,               // how many ticks to look ahead
        past: VecDeque<[u16; N]>, // recent leader poses, oldest first
    },
    Stimulus {
        signal: Stimulus,
        hold: [u16; N], // one joint uses signal, the rest hold start pose
        total_s: f64,
    },
}

impl Command {
    pub fn lead(ahead: f64) -> Command {
        Command::Lead {
            ahead,
            past: VecDeque::with_capacity(LEAD_WINDOW + 1),
        }
    }

    pub fn target(&mut self, leader: &[u16; N], t: f64) -> [u16; N] {
        match self {
            Command::Leader => *leader,
            Command::Lead { ahead, past } => {
                past.push_back(*leader);
                let old = *past.front().unwrap();
                if past.len() > LEAD_WINDOW {
                    past.pop_front();
                }
                let mut target = *leader;
                for j in 0..N {
                    let per_tick = (leader[j] as f64 - old[j] as f64) / LEAD_WINDOW as f64;
                    target[j] = (leader[j] as f64 + per_tick * *ahead).round() as u16;
                }
                target
            }
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
