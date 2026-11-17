//! the bus operations for teleop, also enables mock servos for hardwareless testing

use crate::bus::{Bus, BusError};

pub trait Servo {
    fn read_limits(&mut self, id: u8) -> Result<(u16, u16), BusError>;
    fn begin_sync_read(&mut self, ids: &[u8]) -> Result<(), BusError>;
    fn finish_sync_read(&mut self, ids: &[u8], out: &mut [u16]) -> Result<(), BusError>;
    fn sync_read_positions(&mut self, ids: &[u8], out: &mut [u16]) -> Result<(), BusError> {
        self.begin_sync_read(ids)?;
        self.finish_sync_read(ids, out)
    }
    fn sync_write_goals(&mut self, ids: &[u8], goals: &[u16]) -> Result<(), BusError>;
    fn write_goal(&mut self, id: u8, pos: u16) -> Result<(), BusError>;
    fn set_torque(&mut self, id: u8, on: bool) -> Result<(), BusError>;
}

impl Servo for Bus {
    fn read_limits(&mut self, id: u8) -> Result<(u16, u16), BusError> {
        Bus::read_limits(self, id)
    }
    fn begin_sync_read(&mut self, ids: &[u8]) -> Result<(), BusError> {
        Bus::begin_sync_read(self, ids)
    }
    fn finish_sync_read(&mut self, ids: &[u8], out: &mut [u16]) -> Result<(), BusError> {
        Bus::finish_sync_read(self, ids, out)
    }
    fn sync_write_goals(&mut self, ids: &[u8], goals: &[u16]) -> Result<(), BusError> {
        Bus::sync_write_goals(self, ids, goals)
    }
    fn write_goal(&mut self, id: u8, pos: u16) -> Result<(), BusError> {
        Bus::write_goal(self, id, pos)
    }
    fn set_torque(&mut self, id: u8, on: bool) -> Result<(), BusError> {
        Bus::set_torque(self, id, on)
    }
}
