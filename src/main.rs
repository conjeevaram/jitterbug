mod bus;
mod protocol;
mod rt;
mod safety;
mod servo;
mod teleop;

use std::fs::File;
use std::io::Write;
use std::time::{Duration, Instant};

fn write_csv(path: &str, log: &[teleop::Tick]) {
    let mut f = File::create(path).expect("create csv");
    writeln!(f, "late_us,read_us,write_us,core,missed").unwrap();
    for t in log {
        writeln!(
            f,
            "{},{},{},{},{}",
            t.late_us, t.read_us, t.write_us, t.core, t.missed
        )
        .unwrap();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 4 {
        eprintln!("usage: jitterbug <leader> <follower> <ticks> <out.csv> [slack_ns][hz]");
        std::process::exit(2);
    }
    let ticks: u32 = args[2].parse().expect("ticks");
    let out = &args[3];
    if let Some(s) = args.get(4) {
        rt::set_timer_slack_ns(s.parse().expect("slack_ns"));
    }
    eprintln!("timer slack ns: {}", rt::timer_slack_ns());

    let hz: u64 = args.get(5).map_or(200, |s| s.parse().expect("hz"));
    assert!((1..=10_000).contains(&hz), "hz out of range");
    let period = Duration::from_nanos(1_000_000_000 / hz);
    eprintln!("rate hz: {hz}");

    let mut leader = bus::Bus::open(&args[0]).expect("open leader");
    let mut follower = bus::Bus::open(&args[1]).expect("open follower");
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
