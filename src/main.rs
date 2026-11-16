mod bus;
mod protocol;
mod safety;
mod teleop;

use std::fs::File;
use std::io::Write;

fn write_csv(path: &str, log: &[teleop::Tick]) {
    let mut f = File::create(path).expect("create csv");
    writeln!(f, "late_us,read_us,write_us").unwrap();
    for t in log {
        writeln!(f, "{},{},{}", t.late_us, t.read_us, t.write_us).unwrap();
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let leader_path = args.next().expect("usage: jitterbug <leader> <follower>");
    let follower_path = args.next().expect("usage: jitterbug <leader> <follower>");
    let mut leader = bus::Bus::open(&leader_path).expect("open leader");
    let mut follower = bus::Bus::open(&follower_path).expect("open follower");
    let mut log = Vec::with_capacity(teleop::TICKS as usize);

    let result = teleop::run(&mut leader, &mut follower, &mut log);
    let off = teleop::release(&mut follower);
    write_csv("teleop.csv", &log);
    result.expect("run");
    off.expect("torque off");
}
