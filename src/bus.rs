//! serial transport for the serov bus

use std::io::{ErrorKind, Read, Write};
use std::time::Duration;

use crate::protocol;

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

    /// sends a ping and returns bytes that arrive before the line goes idle
    pub fn ping_raw(&mut self, id: u8) -> std::io::Result<Vec<u8>> {
        let mut frame = [0u8; 16];
        let n = protocol::encode(id, protocol::PING, &[], &mut frame);
        self.port.clear(serialport::ClearBuffer::Input)?;
        self.port.write_all(&frame[..n])?;
        self.port.flush()?;

        let mut got = Vec::new();
        let mut buf = [0u8; 32];
        loop {
            match self.port.read(&mut buf) {
                Ok(k) => got.extend_from_slice(&buf[..k]),
                Err(e) if e.kind() == ErrorKind::TimedOut => break,
                Err(e) => return Err(e),
            }
        }
        Ok(got)
    }
}
