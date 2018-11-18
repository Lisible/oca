/*
* MIT License
*
* Copyright (c) 2018 Clément SIBILLE
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

use gb::memory::memory::*;

pub struct Ram {
    data: [u8; 0x2000]
}

impl Ram {
    pub fn new() -> Ram {
        Ram {
            data: [0; 0x2000]
        }
    }
}

impl ReadMemory for Ram {
    fn read_8bit(&self, address: usize) -> u8 {
        self.data[address]
    }

    fn read_8bit_signed(&self, address: usize) -> i8 {
        self.read_8bit(address) as i8
    }

    fn read_16bit(&self, address: usize) -> u16 {
        (self.read_8bit(address) as u16) | ((self.read_8bit(address + 1) as u16) << 8)
    }
}

impl WriteMemory for Ram {
    fn write_8bit(&mut self, address: usize, value: u8) {
        self.data[address] = value;
    }

    fn write_8bit_signed(&mut self, address: usize, value: i8) {
        self.write_8bit(address, value as u8);
    }

    fn write_16bit(&mut self, address: usize, value: u16) {
        self.write_8bit(address, ((value << 8) >> 8) as u8);
        self.write_8bit(address + 1, (value >> 8) as u8);
    }
}

#[cfg(test)]
mod test {

    use super::*;

    #[test]
    fn can_read_8bit_from_ram() {
        let mut ram = Ram::new();
        ram.data[0xA5] = 0x23;

        assert_eq!(ram.read_8bit(0xA5), 0x23);
    }

    #[test]
    fn can_read_8bit_signed_from_ram() {
        let mut ram = Ram::new();
        ram.data[0xA5] = 0x23;

        assert_eq!(ram.read_8bit_signed(0xA5), 0x23 as i8);
    }

    #[test]
    fn can_read_16bit_from_ram() {
        let mut ram = Ram::new();
        ram.data[0xA5] = 0x23;
        ram.data[0xA6] = 0x45;

        assert_eq!(ram.read_16bit(0xA5), 0x4523);
    }

    #[test]
    fn can_write_8bit_to_ram() {
        let mut ram = Ram::new();
        ram.write_8bit(0xA5, 0x23);

        assert_eq!(ram.data[0xA5], 0x23);
    }

    #[test]
    fn can_write_8bit_signed_to_ram() {
        let mut ram = Ram::new();
        ram.write_8bit_signed(0xA5, 0x23i8);

        assert_eq!(ram.data[0xA5] as i8, 0x23i8);
    }

    #[test]
    fn can_write_16bit_to_ram() {
        let mut ram = Ram::new();
        ram.write_16bit(0xA5, 0x2345);

        assert_eq!(ram.data[0xA5], 0x45);
        assert_eq!(ram.data[0xA6], 0x23);
    }

}