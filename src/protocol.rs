//! servo bus packet encoding and decoding for the st3215

pub const READ: u8 = 0x02;
pub const WRITE: u8 = 0x03;
pub const SYNC_READ: u8 = 0x82;
pub const SYNC_WRITE: u8 = 0x83;
pub const BROADCAST: u8 = 0xFE;

/// checksum over id, length, instruction and params excluding FF FF header
pub fn checksum(bytes: &[u8]) -> u8 {
    let mut sum: u8 = 0;
    for &b in bytes {
        sum = sum.wrapping_add(b);
    }
    !sum
}

/// writes 'FF FF id len instr params ... chk' into 'out' and returns frame len
pub fn encode(id: u8, instr: u8, params: &[u8], out: &mut [u8]) -> usize {
    let n = params.len() + 6;
    out[0] = 0xff;
    out[1] = 0xff;
    out[2] = id;
    out[3] = (params.len() + 2) as u8;
    out[4] = instr;
    out[5..n - 1].copy_from_slice(params);
    out[n - 1] = checksum(&out[2..n - 1]);
    n
}

#[derive(Debug, PartialEq)]
pub struct Status<'a> {
    pub id: u8,
    pub error: u8,
    pub params: &'a [u8],
}

#[derive(Debug, PartialEq)]
pub enum DecodeError {
    TooShort,
    BadHeader,
    BadLength,
    BadChecksum,
}

///parses 'FF FF id len err params ... chk' from a fram
pub fn decode(data: &[u8]) -> Result<Status<'_>, DecodeError> {
    if data.len() < 6 {
        return Err(DecodeError::TooShort);
    }
    if data[0] != 0xFF || data[1] != 0xFF {
        return Err(DecodeError::BadHeader);
    }
    let len = data[3] as usize;
    if len < 2 || data.len() != len + 4 {
        return Err(DecodeError::BadLength);
    }
    let n = data.len();
    if checksum(&data[2..n - 1]) != data[n - 1] {
        return Err(DecodeError::BadChecksum);
    }
    Ok(Status {
        id: data[2],
        error: data[4],
        params: &data[5..n - 1],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_pos_id1() {
        let mut buf = [0u8; 16];
        let n = encode(1, READ, &[56, 2], &mut buf);
        assert_eq!(
            &buf[..n],
            &[0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE][..]
        );
    }

    #[test]
    fn write_pos_id1() {
        let mut buf = [0u8; 16];
        let n = encode(1, WRITE, &[42, 0x00, 0x08], &mut buf);
        assert_eq!(
            &buf[..n],
            &[0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC4][..]
        );
    }

    #[test]
    fn decode_ping_reply() {
        let data = [0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let s = decode(&data).unwrap();
        assert_eq!(s.id, 1);
        assert_eq!(s.error, 0);
        assert!(s.params.is_empty());
    }

    #[test]
    fn decode_read_reply() {
        let data = [0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2];
        let s = decode(&data).unwrap();
        assert_eq!(s.params, &[0x00, 0x08][..]);
    }

    #[test]
    fn decode_bad_checksum() {
        let data = [0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFD];
        assert_eq!(decode(&data), Err(DecodeError::BadChecksum));
    }

    #[test]
    fn decode_too_short() {
        assert_eq!(decode(&[0xFF, 0xFF, 0x01]), Err(DecodeError::TooShort));
    }

    #[test]
    fn sync_write_goals() {
        let params = [
            42, 2, 1, 0x00, 0x08, 2, 0x01, 0x08, 3, 0x02, 0x08, 4, 0x03, 0x08, 5, 0x04, 0x08, 6,
            0x05, 0x08,
        ];
        let mut buf = [0u8; 32];
        let n = encode(BROADCAST, SYNC_WRITE, &params, &mut buf);
        assert_eq!(
            &buf[..n],
            &[
                0xFF, 0xFF, 0xFE, 0x16, 0x83, 0x2A, 0x02, 1, 0x00, 0x08, 2, 0x01, 0x08, 3, 0x02,
                0x08, 4, 0x03, 0x08, 5, 0x04, 0x08, 6, 0x05, 0x08, 0xE8
            ][..]
        );
    }
}
