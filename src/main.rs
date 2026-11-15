mod bus;
mod protocol;

fn main() {
    let path = std::env::args().nth(1).expect("usage: jitterbug <port>");
    let mut bus = bus::Bus::open(&path).expect("open port");
    for id in 1..=6 {
        match bus.ping_raw(id) {
            Ok(bytes) => println!("id {id}: {bytes:02X?}"),
            Err(e) => println!("id {id}: error {e}"),
        }
    }
}
