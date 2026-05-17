//! Various utilities that don't fit in other modules.

/// Calculate an 8-bit CRC for some arbitrary data.
#[must_use]
pub fn crc8(arr: &[u8]) -> u8 {
    let mut crc = 0x00;
    for element in arr {
        crc ^= element;
        for _ in 0..8 {
            if crc & 0x80 > 0 {
                crc = (crc << 1) ^ 0xd5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_small_data() {
        let data = [0xff, 0x2d, 0x29, 0x23, 0x22, 0x32, 0x59];
        assert_eq!(crc8(&data), 181);
    }

    #[test]
    fn crc_large_data() {
        let data = [95, 26, 74, 21, 23, 89, 62, 90, 40, 55, 74, 35, 29, 57, 59, 77, 15, 87, 2, 60, 74, 38, 18, 33, 60, 85, 49, 80, 0, 66, 91, 17, 70, 2, 92, 43, 44, 28, 95, 68, 8, 31, 40, 9, 1, 86, 6, 67, 21, 84, 50, 56, 33, 8, 98, 77, 17, 73, 17, 53, 17, 11, 5, 46, 50, 34, 67, 79, 1, 29, 11, 18, 0, 68, 23, 87, 5, 84, 58, 40, 55, 12, 85, 25, 69, 14, 32, 85, 34, 61, 79, 13, 98, 40, 14, 84, 69, 90, 54, 1];
        assert_eq!(crc8(&data), 175);
    }
}
