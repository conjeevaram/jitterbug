//! absolute grid pacing, designed so that lateness never accumulates (tick k due at t0 + k*period)

use std::thread::sleep;
use std::time::{Duration, Instant};

pub struct Wake {
    pub woke: Instant,
    pub late: Duration,
    pub missed: u32,
    pub t: f64,
}

pub struct Pacer {
    t0: Instant,
    period: Duration,
    ticks: u32,
    k: u32,
    pub skipped: u32,
}

/// how many full periods late the wake was
fn missed_ticks(late: Duration, period: Duration) -> u32 {
    (late.as_nanos() / period.as_nanos()) as u32
}

impl Pacer {
    pub fn new(period: Duration, ticks: u32) -> Pacer {
        Pacer {
            t0: Instant::now(),
            period,
            ticks,
            k: 0,
            skipped: 0,
        }
    }

    pub fn done(&self) -> bool {
        self.k >= self.ticks
    }

    pub fn elapsed(&self) -> Duration {
        self.t0.elapsed()
    }

    /// sleep until the next deadline, if a tick is alr a full period late, skip it too
    pub fn wait(&mut self) -> Wake {
        let t = self.period * self.k;
        let deadline = self.t0 + t;
        let now = Instant::now();
        if deadline > now {
            sleep(deadline - now);
        }

        let woke = Instant::now();
        let late = woke.saturating_duration_since(deadline);
        let missed = missed_ticks(late, self.period);
        self.k += 1 + missed;
        self.skipped += missed;
        Wake {
            woke,
            late,
            missed,
            t: t.as_secs_f64(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_whole_periods_behind() {
        let ms = Duration::from_millis;
        assert_eq!(missed_ticks(ms(0), ms(5)), 0);
        assert_eq!(missed_ticks(ms(4), ms(5)), 0);
        assert_eq!(missed_ticks(ms(12), ms(5)), 2);
    }
}
