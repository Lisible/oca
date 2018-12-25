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

///
/// Allows access to the different part of the system memory
///
pub struct MemoryBus {
    cartridge: [u8; 0x8000],
    ram: [u8; 0x2000],
    high_ram: [u8; 0x80],
    oam: [u8; 0xA0],
    io: [u8; 0x4D]
}

impl MemoryBus {
    pub fn new() -> MemoryBus {
        MemoryBus {
            cartridge: [0; 0x8000],
            ram: [0; 0x2000],
            high_ram: [0; 0x80],
            oam:  [0; 0xA0],
            io:  [0; 0x4D]
        }
    }

    pub fn load_rom(&mut self, rom_data: [u8; 0x8000]) {
        self.cartridge.copy_from_slice(&rom_data[..0x8000]);
    }

    pub fn dump_io_register(&self, address: u16) -> u8 {
        self.read_8bit(address)
    }
}

impl ReadMemory for MemoryBus {
    fn read_8bit(&self, address: u16) -> u8 {
        if address < 0x8000 {
            self.cartridge.read_8bit(address)
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.read_8bit(address - 0xC000)
        } else if (address >= 0xE000) && (address < 0xFE00) {
            self.ram.read_8bit(address - 0xE000)
        } else if (address >= 0xFE00) && (address < 0xFEA0) {
            self.oam.read_8bit(address - 0xFE00)
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.read_8bit(address - 0xFF80)
        } else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.read_8bit(address - 0xFF00)
        } else if (address == 0xFFFF) {
            self.io.read_8bit(0xFF4C)
        }  else {
            panic!("Unmapped memory access: {:X}", address)
        }
    }

    fn read_8bit_signed(&self, address: u16) -> i8 {
        self.read_8bit(address) as i8
    }

    fn read_16bit(&self, address: u16) -> u16 {
        if address < 0x8000 {
            self.cartridge.read_16bit(address)
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.read_16bit(address - 0xC000)
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.read_16bit(address - 0xE000)
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.read_16bit(address - 0xFF80)
        } else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.read_16bit(address - 0xFF00)
        } else if (address == 0xFFFF) {
            self.io.read_16bit(0xFF4C)
        } else {
            panic!("Unmapped memory access")
        }
    }
}

impl WriteMemory for MemoryBus {
    fn write_8bit(&mut self, address: u16, value: u8) {
        if address < 0x8000 {
            panic!("Cartridge ROM is read-only !!!")
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.write_8bit(address - 0xC000, value);
        } else if (address >= 0xE000) && (address < 0xFE00) {
            self.ram.write_8bit(address - 0xE000, value);
        } else if (address >= 0xFE00) && (address < 0xFEA0) {
            self.oam.write_8bit(address - 0xFE00, value);
        }  else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.write_8bit(address - 0xFF80, value);
        } else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.write_8bit(address - 0xFF00, value);
        } else if (address == 0xFFFF) {
            self.io.write_8bit(0xFF4C, value)
        }  else {
            panic!("Unmapped memory access: 0x{:X}", address)
        }
    }

    fn write_8bit_signed(&mut self, address: u16, value: i8) {
        if address < 0x8000 {
            panic!("Cartridge ROM is read-only !!!")
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.write_8bit_signed(address - 0xC000, value);
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.write_8bit_signed(address - 0xE000, value);
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.write_8bit_signed(address - 0xFF80, value);
        }  else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.write_8bit_signed(address - 0xFF00, value);
        } else if (address == 0xFFFF) {
            self.io.write_8bit_signed(0xFF4C, value)
        }  else {
            panic!("Unmapped memory access")
        }
    }

    fn write_16bit(&mut self, address: u16, value: u16) {
        if address < 0x8000 {
            panic!("Cartridge ROM is read-only !!!")
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.write_16bit(address - 0xC000, value);
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.write_16bit(address - 0xE000, value);
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.write_16bit(address - 0xFF80, value);
        }  else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.write_16bit(address - 0xFF00, value);
        } else if (address == 0xFFFF) {
            self.io.write_16bit(0xFF4C, value)
        }  else {
            panic!("Unmapped memory access")
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn can_read_8bit_from_ram() {
        let mut bus = MemoryBus::new();
        bus.ram.write_8bit(0x10, 0x11);

        assert_eq!(bus.read_8bit(0xC010), 0x11);
        assert_eq!(bus.read_8bit(0xE010), 0x11);
    }

    #[test]
    fn can_read_8bit_signed_from_ram() {
        let mut bus = MemoryBus::new();
        bus.ram.write_8bit_signed(0x10, 0x11i8);

        assert_eq!(bus.read_8bit_signed(0xC010), 0x11i8);
        assert_eq!(bus.read_8bit_signed(0xE010), 0x11i8);
    }

    #[test]
    fn can_read_16bit_from_ram() {
        let mut bus = MemoryBus::new();
        bus.ram.write_16bit(0x10, 0x1122);

        assert_eq!(bus.read_16bit(0xC010), 0x1122);
        assert_eq!(bus.read_16bit(0xE010), 0x1122);
    }

    #[test]
    fn can_write_8bit_to_ram() {
        let mut bus = MemoryBus::new();
        bus.write_8bit(0xC010, 0x11);
        assert_eq!(bus.ram.read_8bit(0x10), 0x11);


        bus.write_8bit(0xE010, 0x12);
        assert_eq!(bus.ram.read_8bit(0x10), 0x12);
    }

    #[test]
    fn can_write_8bit_signed_to_ram() {
        let mut bus = MemoryBus::new();

        bus.write_8bit_signed(0xC010, 0x11i8);
        assert_eq!(bus.ram.read_8bit_signed(0x10), 0x11i8);

        bus.write_8bit_signed(0xC010, 0x12i8);
        assert_eq!(bus.ram.read_8bit_signed(0x10), 0x12i8);
    }

    #[test]
    fn can_write_16bit_to_ram() {
        let mut bus = MemoryBus::new();

        bus.write_16bit(0xC010, 0x1122);
        assert_eq!(bus.ram.read_16bit(0x10), 0x1122);

        bus.write_16bit(0xE010, 0x1123);
        assert_eq!(bus.ram.read_16bit(0x10), 0x1123);
    }

    #[test]
    fn can_read_8bit_from_high_ram() {
        let mut bus = MemoryBus::new();
        bus.high_ram.write_8bit(0x10, 0x11);

        assert_eq!(bus.read_8bit(0xFF90), 0x11);
    }

    #[test]
    fn can_read_8bit_signed_from_high_ram() {
        let mut bus = MemoryBus::new();
        bus.high_ram.write_8bit_signed(0x10, 0x11i8);

        assert_eq!(bus.read_8bit_signed(0xFF90), 0x11i8);
    }

    #[test]
    fn can_read_16bit_from_high_ram() {
        let mut bus = MemoryBus::new();
        bus.high_ram.write_16bit(0x10, 0x1122);

        assert_eq!(bus.read_16bit(0xFF90), 0x1122);
    }

    #[test]
    fn can_write_8bit_to_high_ram() {
        let mut bus = MemoryBus::new();

        bus.write_8bit(0xFF90, 0x11);
        assert_eq!(bus.high_ram.read_8bit(0x10), 0x11);
    }

    #[test]
    fn can_write_8bit_signed_to_high_ram() {
        let mut bus = MemoryBus::new();

        bus.write_8bit_signed(0xFF90, 0x11i8);
        assert_eq!(bus.high_ram.read_8bit_signed(0x10), 0x11i8);
    }

    #[test]
    fn can_write_16bit_to_high_ram() {
        let mut bus = MemoryBus::new();

        bus.write_16bit(0xFF90, 0x1122);
        assert_eq!(bus.high_ram.read_16bit(0x10), 0x1122);
    }

    #[test]
    fn can_read_8bit_from_io() {
        let mut bus = MemoryBus::new();
        bus.io.write_8bit(0x10, 0x11);

        assert_eq!(bus.read_8bit(0xFF10), 0x11);
    }

    #[test]
    fn can_read_8bit_signed_from_io() {
        let mut bus = MemoryBus::new();
        bus.io.write_8bit_signed(0x10, 0x11i8);

        assert_eq!(bus.read_8bit_signed(0xFF10), 0x11i8);
    }

    #[test]
    fn can_read_16bit_from_io() {
        let mut bus = MemoryBus::new();
        bus.io.write_16bit(0x10, 0x1122);

        assert_eq!(bus.read_16bit(0xFF10), 0x1122);
    }

    #[test]
    fn can_write_8bit_to_io() {
        let mut bus = MemoryBus::new();

        bus.write_8bit(0xFF10, 0x11);
        assert_eq!(bus.io.read_8bit(0x10), 0x11);
    }

    #[test]
    fn can_write_8bit_signed_to_io() {
        let mut bus = MemoryBus::new();

        bus.write_8bit_signed(0xFF10, 0x11i8);
        assert_eq!(bus.io.read_8bit_signed(0x10), 0x11i8);
    }

    #[test]
    fn can_write_16bit_to_io() {
        let mut bus = MemoryBus::new();

        bus.write_16bit(0xFF10, 0x1122);
        assert_eq!(bus.io.read_16bit(0x10), 0x1122);
    }
}
