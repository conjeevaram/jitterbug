mod bus;
mod protocol;

use std::time::Instant;

fn main() {
    let path = std::env::args().nth(1).expect("usage: jitterbug <port>");
    let mut bus = bus::Bus::open(&path).expect("open port");

    let ids = [1, 2, 3, 4, 5, 6];
    let mut pos = [0u16; 6];
    let mut times = Vec::new();
    for _ in 0..1000 {
        let t = Instant::now();
        bus.sync_read_positions(&ids, &mut pos).expect("sync read");
        times.push(t.elapsed().as_micros());
    }
    times.sort();
    println!("{pos:?}");
    println!(
        "p50 {} us p99 {} us max {} us",
        times[500], times[990], times[999]
    );
}
