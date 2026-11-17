//! scripted test signals for sysid, replaces the leader for one joint.
//! goal is to be able to use this to fit models (using lr) to the servo behaviour

pub enum Wave {
    Chirp { f0: f64, f1: f64 }, // freqs(hz) swept linearly
    Steps { hold: f64 },        // hold for 'hold' seconds at some level
}

pub struct Stimulus {
    pub joint: usize,
    pub amp: f64,
    pub wave: Wave,
}

impl Stimulus {
    /// offset from start pose
    pub fn offset(&self, t: f64, total: f64) -> f64 {
        const LEVELS: [f64; 6] = [1.0, -1.0, 0.5, -0.5, 0.25, -0.25];
        match self.wave {
            Wave::Chirp { f0, f1 } => {
                let phase = std::f64::consts::TAU * (f0 * t + (f1 - f0) * t * t / (2.0 * total));
                self.amp * phase.sin()
            }
            Wave::Steps { hold } => self.amp * LEVELS[(t / hold) as usize % LEVELS.len()],
        }
    }

    /// does the signal stay inside the joint limits?
    pub fn fits(&self, start: u16, (lo, hi): (u16, u16)) -> bool {
        start as f64 - self.amp >= lo as f64 && start as f64 + self.amp <= hi as f64
    }

    /// reject setups that would get clipped by speed cap
    pub fn check(&self, hz: u64, max_speed: u64) -> Result<(), String> {
        if let Wave::Chirp { f1, .. } = self.wave {
            if f1 * 2.0 >= hz as f64 {
                return Err("f1 must be < loop rate / 2".into());
            }
            let peak = std::f64::consts::TAU * f1 * self.amp;
            if peak > max_speed as f64 {
                return Err(format!(
                    "chirp peaks at {peak:.0}, raise max speed or lower f1 or amp"
                ));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chirp(amp: f64, f1: f64) -> Stimulus {
        Stimulus {
            joint: 0,
            amp,
            wave: Wave::Chirp { f0: 0.3, f1 },
        }
    }

    #[test]
    fn chirp_starts_at_the_start_pose_and_stays_within_amp() {
        let s = chirp(100.0, 12.0);
        assert_eq!(s.offset(0.0, 30.0), 0.0);
        for k in 0..6000 {
            assert!(s.offset(k as f64 * 0.005, 30.0).abs() <= 100.0);
        }
    }

    #[test]
    fn check_rejects_a_chirp_the_speed_cap_would_clip_or_alias() {
        assert!(chirp(100.0, 12.0).check(200, 8000).is_ok());
        assert!(chirp(100.0, 12.0).check(200, 2000).is_err());
        assert!(chirp(100.0, 120.0).check(200, 1_000_000).is_err()); // above freq limit (nyquist freq)
    }
}
