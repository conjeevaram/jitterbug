mod bus;
mod protocol;

use std::thread::sleep;
use std::time::Duration;

fn gripper_test(bus: &mut bus::Bus) -> Result<(), bus::BusError> {
    let id = 6;
    let start = bus.read_position(id)?;
    println!("start {start}");
    assert!(start < 3300, "unexpected gripper position");

    bus.write_goal(id, start)?;
    bus.set_torque(id, true)?;
    sleep(Duration::from_millis(500));

    bus.write_goal(id, start + 100)?;
    sleep(Duration::from_millis(1000));
    println!("moved {}", bus.read_position(id)?);

    bus.write_goal(id, start)?;
    sleep(Duration::from_millis(1000));
    println!("back {}", bus.read_position(id)?);
    Ok(())
}

fn main() {
    let path = std::env::args().nth(1).expect("usage: jitterbug <port>");
    let mut bus = bus::Bus::open(&path).expect("open port");

    let result = gripper_test(&mut bus);
    bus.set_torque(6, false).expect("torque off");
    result.expect("gripper test");
}
