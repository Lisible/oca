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

use std::usize;
use gb::memory::memory::*;

pub struct Cartridge {
    rom: [u8; 0x8000],
}

#[derive(PartialEq, Debug)]
enum CartridgeType {
    RomOnly,
    RomMbc1,
    RomMbc1Ram,
    RomMbc1RamBattery,
    RomMbc2,
    RomMbc2Battery,
    RomRam,
    RomRamBattery,
    RomMm01,
    RomMm01Sram,
    RomMm01SramBattery,
    RomMbc3TimerBattery,
    RomMbc3TimerRamBattery,
    RomMbc3,
    RomMbc3Ram,
    RomMbc3RamBattery,
    RomMbc5,
    RomMbc5Ram,
    RomMbc5RamBattery,
    RomMbc5Rumble,
    RomMbc5RumbleSram,
    RomMbc5RumbleSramBattery,
    PocketCamera,
    BandaiTama5,
    HudsonHuc3,
    HudsonHuc1
}

impl Cartridge {
    ///
    /// Creates a cartridge from a byte array
    ///
    pub fn from_bytes(bytes: [u8; 0x8000]) -> Cartridge {
        Cartridge {
            rom: bytes,
        }
    }

    ///
    /// Returns:
    /// - The type of cartridge
    ///
    pub fn get_type(&self) -> CartridgeType {
        match self.read_8bit(0x0147) {
            0x0 => CartridgeType::RomOnly,
            0x1 => CartridgeType::RomMbc1,
            0x2 => CartridgeType::RomMbc1Ram,
            0x3 => CartridgeType::RomMbc1RamBattery,
            0x5 => CartridgeType::RomMbc2,
            0x6 => CartridgeType::RomMbc2Battery,
            0x8 => CartridgeType::RomRam,
            0x9 => CartridgeType::RomRamBattery,
            0xB => CartridgeType::RomMm01,
            0xC => CartridgeType::RomMm01Sram,
            0xD => CartridgeType::RomMm01SramBattery,
            0xF => CartridgeType::RomMbc3TimerBattery,
            0x10 => CartridgeType::RomMbc3TimerRamBattery,
            0x11 => CartridgeType::RomMbc3,
            0x12 => CartridgeType::RomMbc3Ram,
            0x13 => CartridgeType::RomMbc3RamBattery,
            0x19 => CartridgeType::RomMbc5,
            0x1A => CartridgeType::RomMbc5Ram,
            0x1B => CartridgeType::RomMbc5RamBattery,
            0x1C => CartridgeType::RomMbc5Rumble,
            0x1D => CartridgeType::RomMbc5RumbleSram,
            0x1E => CartridgeType::RomMbc5RumbleSramBattery,
            0x1F => CartridgeType::PocketCamera,
            0xFD => CartridgeType::BandaiTama5,
            0xFE => CartridgeType::HudsonHuc3,
            0xFF => CartridgeType::HudsonHuc1,
            _ => panic!("Unknown cartridge type")
        }
    }
}

impl ReadMemory for Cartridge {
    fn read_8bit(&self, address: usize) -> u8 {
        self.rom[address]
    }

    fn read_8bit_signed(&self, address: usize) -> i8 {
        self.read_8bit(address) as i8
    }

    fn read_16bit(&self, address: usize) -> u16 {
        let second = self.read_8bit(address);
        let first = self.read_8bit(address.wrapping_add(1));

        ((first as u16) << 8) | second as u16
    }
}

#[cfg(test)]
impl WriteMemory for Cartridge {
    fn write_8bit(&mut self, address: usize, value: u8) {
        self.rom[address] = value;
    }

    fn write_8bit_signed(&mut self, address: usize, value: i8) {
        self.write_8bit(address, value as u8);
    }

    fn write_16bit(&mut self, address: usize, value: u16) {
        self.write_8bit(address, (((value as u16) << 8) >> 8) as u8);
        self.write_8bit(address+1, ((value as u16) >> 8) as u8);
    }
}


#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn can_create_cartridge_from_bytes() {
        let mut bytes : [u8; 0x8000] = [0; 0x8000];

        for i in 0..0x8000 {
            bytes[i] = i as u8;
        }

        let cartridge = Cartridge::from_bytes(bytes);
        assert_eq!(cartridge.rom[0x8], 0x8);
    }

    #[test]
    fn can_read_8bit_from_cartridge() {
        let mut bytes : [u8; 0x8000] = [0; 0x8000];

        for i in 0..0x8000 {
            bytes[i] = i as u8;
        }

        let cartridge = Cartridge::from_bytes(bytes);
        assert_eq!(cartridge.read_8bit(0x8), 0x8);
    }

    #[test]
    fn can_read_16bit_from_cartridge() {
        let mut bytes : [u8; 0x8000] = [0; 0x8000];

        for i in (0..0x8000).step_by(2) {
            bytes[i] = 0xCD;
            bytes[i+1] = 0xAB;
        }

        let cartridge = Cartridge::from_bytes(bytes);
        assert_eq!(cartridge.read_16bit(0x0), 0xABCD);
    }

    #[test]
    fn can_get_cartridge_type() {
        let mut bytes : [u8; 0x8000] = [0; 0x8000];

        for i in 0..0x8000 {
            bytes[i] = i as u8;
        }
        bytes[0x147] = 0x8;

        let cartridge = Cartridge::from_bytes(bytes);
        assert_eq!(cartridge.get_type(), CartridgeType::RomRam);
    }
}