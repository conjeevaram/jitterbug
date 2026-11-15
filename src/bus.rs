//! serial transport for the serov bus

use std::io::{Read, Write};
use std::time::Duration;

use crate::protocol;

const CURR_POS: u8 = 56;
const TORQUE_ENABLE: u8 = 40;
const GOAL_POS: u8 = 42;

#[derive(Debug)]
pub enum BusError {
    Io(std::io::Error),
    Port(serialport::Error),
    Decode(protocol::DecodeError),
    Servo(u8),
    UnexpectedId(u8),
}

impl From<std::io::Error> for BusError {
    fn from(e: std::io::Error) -> Self {
        BusError::Io(e)
    }
}

impl From<serialport::Error> for BusError {
    fn from(e: serialport::Error) -> Self {
        BusError::Port(e)
    }
}

impl From<protocol::DecodeError> for BusError {
    fn from(e: protocol::DecodeError) -> Self {
        BusError::Decode(e)
    }
}

pub struct Bus {
    port: Box<dyn serialport::SerialPort>, // heap allocated
}

impl Bus {
    pub fn open(path: &str) -> serialport::Result<Bus> {
        let port = serialport::new(path, 1_000_000)
            .timeout(Duration::from_millis(20))
            .open()?; // returns early if a call fails, otherwise unwraps val
        Ok(Bus { port })
    }

    ///sends one instruction, reads the reply and returns its length
    fn send(&mut self, id: u8, instr: u8, params: &[u8]) -> Result<(), BusError> {
        let mut frame = [0u8; 32];
        let n = protocol::encode(id, instr, params, &mut frame);
        self.port.clear(serialport::ClearBuffer::Input)?;
        self.port.write_all(&frame[..n])?;
        self.port.flush()?;
        Ok(())
    }

    /// reads one reply frame and gives its length
    fn recv(&mut self, reply: &mut [u8]) -> Result<usize, BusError> {
        self.port.read_exact(&mut reply[..4])?;
        let rest = reply[3] as usize;
        if rest + 4 > reply.len() {
            return Err(BusError::Decode(protocol::DecodeError::BadLength));
        }
        self.port.read_exact(&mut reply[4..4 + rest])?;
        Ok(rest + 4)
    }

    pub fn read_position(&mut self, id: u8) -> Result<u16, BusError> {
        let mut reply = [0u8; 16];
        self.send(id, protocol::READ, &[CURR_POS, 2])?;
        let n = self.recv(&mut reply)?;
        let status = protocol::decode(&reply[..n])?;
        parse_position(&status)
    }

    pub fn sync_read_positions(&mut self, ids: &[u8], out: &mut [u16]) -> Result<(), BusError> {
        let mut params = [0u8; 18];
        params[0] = CURR_POS;
        params[1] = 2;
        params[2..2 + ids.len()].copy_from_slice(ids);
        self.send(
            protocol::BROADCAST,
            protocol::SYNC_READ,
            &params[..2 + ids.len()],
        )?;

        let mut reply = [0u8; 16];
        for (k, &id) in ids.iter().enumerate() {
            let n = self.recv(&mut reply)?;
            let status = protocol::decode(&reply[..n])?;
            if status.id != id {
                return Err(BusError::UnexpectedId(status.id));
            }
            out[k] = parse_position(&status)?;
        }
        Ok(())
    }

    fn write(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), BusError> {
        let mut params = [0u8; 8];
        params[0] = addr;
        params[1..1 + data.len()].copy_from_slice(data);
        self.send(id, protocol::WRITE, &params[..1 + data.len()])?;

        let mut reply = [0u8; 16];
        let n = self.recv(&mut reply)?;
        let status = protocol::decode(&reply[..n])?;
        if status.id != id {
            return Err(BusError::UnexpectedId(status.id));
        }
        if status.error != 0 {
            return Err(BusError::Servo(status.error));
        }
        Ok(())
    }

    pub fn set_torque(&mut self, id: u8, on: bool) -> Result<(), BusError> {
        self.write(id, TORQUE_ENABLE, &[on as u8])
    }

    pub fn write_goal(&mut self, id: u8, pos: u16) -> Result<(), BusError> {
        self.write(id, GOAL_POS, &pos.to_le_bytes())
    }
}

fn parse_position(status: &protocol::Status) -> Result<u16, BusError> {
    if status.error != 0 {
        return Err(BusError::Servo(status.error));
    }
    if status.params.len() != 2 {
        return Err(BusError::Decode(protocol::DecodeError::BadLength));
    }
    Ok(u16::from_le_bytes([status.params[0], status.params[1]]))
}
