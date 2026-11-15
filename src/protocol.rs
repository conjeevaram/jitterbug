//! servo bus packet encoding and decoding for the st3215

pub const PING: u8 = 0x01;

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ping_id1() {
        let mut buf = [0u8; 16];
        let n = encode(1, PING, &[], &mut buf);
        assert_eq!(&buf[..n], &[0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB][..]);
    }
}
