//! serial transport for the serov bus

use std::io::{Read, Write};
use std::time::Duration;

use crate::protocol;

const CURR_POS: u8 = 56;

#[derive(Debug)]
pub enum BusError {
    Io(std::io::Error),
    Port(serialport::Error),
    Decode(protocol::DecodeError),
    Servo(u8),
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
    fn transact(
        &mut self,
        id: u8,
        instr: u8,
        params: &[u8],
        reply: &mut [u8],
    ) -> Result<usize, BusError> {
        let mut frame = [0u8; 32];
        let n = protocol::encode(id, instr, params, &mut frame);
        self.port.clear(serialport::ClearBuffer::Input)?;
        self.port.write_all(&frame[..n])?;
        self.port.flush()?;

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
        let n = self.transact(id, protocol::READ, &[CURR_POS, 2], &mut reply)?;
        let status = protocol::decode(&reply[..n])?;
        if status.error != 0 {
            return Err(BusError::Servo(status.error));
        }
        if status.params.len() != 2 {
            return Err(BusError::Decode(protocol::DecodeError::BadLength));
        }
        Ok(u16::from_le_bytes([status.params[0], status.params[1]]))
    }
}
