mod bus;
mod protocol;

use std::f64::consts::PI;
use std::fs::File;
use std::io::Write;
use std::thread::sleep;
use std::time::{Duration, Instant};

const ID: u8 = 6;

fn run(bus: &mut bus::Bus, log: &mut Vec<[u128; 5]>) -> Result<(), bus::BusError> {
    let ids = [1, 2, 3, 4, 5, 6];
    let mut pos = [0u16; 6];
    bus.sync_read_positions(&ids, &mut pos)?;
    let center = pos[5];
    assert!(center < 3300, "unexpected gripper position");

    bus.write_goal(ID, center)?;
    bus.set_torque(ID, true)?;

    let period = Duration::from_micros(5000);
    let t0 = Instant::now();
    for k in 0..2000 {
        let deadline = t0 + period * k as u32;
        let now = Instant::now();
        if deadline > now {
            sleep(deadline - now);
        }
        let woke = Instant::now();

        let t = k as f64 * 0.005;
        let goal = (center as f64 + 60.0 * (2.0 * PI * 0.5 * t).sin()) as u16;
        bus.sync_write_goals(&[ID], &[goal])?;
        let written = Instant::now();
        bus.sync_read_positions(&ids, &mut pos)?;
        let read = Instant::now();

        log.push([
            woke.saturating_duration_since(deadline).as_micros(),
            written.duration_since(woke).as_micros(),
            read.duration_since(written).as_micros(),
            goal as u128,
            pos[5] as u128,
        ]);
    }
    Ok(())
}

fn main() {
    let path = std::env::args().nth(1).expect("usage: jitterbug <port>");
    let mut bus = bus::Bus::open(&path).expect("open port");
    let mut log = Vec::with_capacity(2000);

    let result = run(&mut bus, &mut log);
    bus.set_torque(ID, false).expect("torque off");

    let mut f = File::create("ticks.csv").expect("create csv");
    writeln!(f, "late_us,write_us,read_us,goal,present").unwrap();
    for r in &log {
        writeln!(f, "{},{},{},{},{}", r[0], r[1], r[2], r[3], r[4]).unwrap();
    }
    result.expect("run");
}
