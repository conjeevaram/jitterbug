mod bus;
mod protocol;
mod rt;
mod safety;
mod servo;
mod teleop;

use std::fs::File;
use std::io::Write;
use std::time::{Duration, Instant};

const KNOWN_FLAGS: [&str; 1] = ["hz"];

fn write_csv(path: &str, log: &[teleop::Tick]) {
    let mut f = File::create(path).expect("create csv");
    write!(f, "late_us,read_us,write_us,core,missed,trk,e2e").unwrap();
    for p in ["l", "g", "m"] {
        for j in 1..=6 {
            write!(f, ",{p}{j}").unwrap();
        }
    }
    writeln!(f).unwrap();
    for t in log {
        write!(
            f,
            "{},{},{},{},{},{},{}",
            t.late_us, t.read_us, t.write_us, t.core, t.missed, t.trk, t.e2e,
        )
        .unwrap();
        for v in t.leader.iter().chain(&t.goal).chain(&t.meas) {
            write!(f, ",{v}").unwrap();
        }
        writeln!(f).unwrap();
    }
}

/// finds `--name=val` flag and produces its val
fn flag<T: std::str::FromStr>(args: &[String], name: &str) -> Option<T> {
    let prefix = format!("--{name}=");
    args.iter()
        .find_map(|a| a.strip_prefix(&prefix))
        .map(|v| v.parse().unwrap_or_else(|_| panic!("bad --{name}: {v}")))
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (flags, pos): (Vec<_>, Vec<_>) = args.iter().cloned().partition(|a| a.starts_with("--"));
    for f in &flags {
        let name = f.trim_start_matches("--").split('=').next().unwrap_or("");
        assert!(KNOWN_FLAGS.contains(&name), "unknown flag {f}");
    }
    if pos.len() != 4 {
        eprintln!("usage: jitterbug <leader> <follower> <ticks> <out.csv> [--hz=N]");
        std::process::exit(2);
    }
    let ticks: u32 = pos[2].parse().expect("ticks");
    let out = &pos[3];

    rt::set_timer_slack_ns(1); // kernel default is 50, 1 ns gives very little room for error
    eprintln!("timer slack ns: {}", rt::timer_slack_ns());

    let hz: u64 = flag(&flags, "hz").unwrap_or(200);
    assert!((1..=10_000).contains(&hz), "hz out of range");
    let period = Duration::from_nanos(1_000_000_000 / hz);
    eprintln!("rate hz: {hz}");

    let mut leader = bus::Bus::open(&pos[0]).expect("open leader");
    let mut follower = bus::Bus::open(&pos[1]).expect("open follower");
    let mut log = Vec::with_capacity(ticks as usize);

    ctrlc::set_handler(|| teleop::STOP.store(true, std::sync::atomic::Ordering::Relaxed))
        .expect("ctrlc handler");
    let result = teleop::run(&mut leader, &mut follower, ticks, period, &mut log);
    let r0 = Instant::now();
    let off = teleop::release(&mut follower);
    eprintln!("release took {} us", r0.elapsed().as_micros());
    write_csv(out, &log);
    result.expect("run");
    off.expect("torque off");
}
