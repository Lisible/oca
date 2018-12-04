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

use gb::memory::cartridge::Cartridge;
use gb::memory::ram::Ram;
use gb::memory::high_ram::HighRam;
use gb::memory::io::IO;
use gb::memory::memory::*;
use std::rc::Rc;
use std::cell::RefCell;

///
/// Allows access to the different part of the system memory
///
pub struct MemoryBus {
    cartridge: Rc<RefCell<Cartridge>>,
    ram: Rc<RefCell<Ram>>,
    high_ram: Rc<RefCell<HighRam>>,
    io: Rc<RefCell<IO>>
}

impl MemoryBus {
    pub fn new(cartridge: Rc<RefCell<Cartridge>>,
               ram: Rc<RefCell<Ram>>,
               high_ram: Rc<RefCell<HighRam>>,
               io: Rc<RefCell<IO>>) -> MemoryBus {
        MemoryBus {
            cartridge,
            ram,
            high_ram,
            io
        }
    }


}

impl ReadMemory for MemoryBus {
    fn read_8bit(&self, address: usize) -> u8 {
        if address < 0x8000 {
            self.cartridge.borrow().read_8bit(address)
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.borrow().read_8bit(address - 0xC000)
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.borrow().read_8bit(address - 0xE000)
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.borrow().read_8bit(address - 0xFF80)
        }  else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.borrow_mut().read_8bit(address - 0xFF00)
        } else {
            panic!("Unmapped memory access")
        }
    }

    fn read_8bit_signed(&self, address: usize) -> i8 {
        if address < 0x8000 {
            self.cartridge.borrow().read_8bit_signed(address)
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.borrow().read_8bit_signed(address - 0xC000)
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.borrow().read_8bit_signed(address - 0xE000)
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.borrow().read_8bit_signed(address - 0xFF80)
        }  else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.borrow_mut().read_8bit_signed(address - 0xFF00)
        } else {
            panic!("Unmapped memory access")
        }
    }

    fn read_16bit(&self, address: usize) -> u16 {
        if address < 0x8000 {
            self.cartridge.borrow().read_16bit(address)
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.borrow().read_16bit(address - 0xC000)
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.borrow().read_16bit(address - 0xE000)
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.borrow().read_16bit(address - 0xFF80)
        } else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.borrow_mut().read_16bit(address - 0xFF00)
        } else {
            panic!("Unmapped memory access")
        }
    }
}

impl WriteMemory for MemoryBus {
    fn write_8bit(&mut self, address: usize, value: u8) {
        if address < 0x8000 {
            panic!("Cartridge ROM is read-only !!!")
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.borrow_mut().write_8bit(address - 0xC000, value)
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.borrow_mut().write_8bit(address - 0xE000, value)
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.borrow_mut().write_8bit(address - 0xFF80, value)
        } else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.borrow_mut().write_8bit(address - 0xFF00, value)
        } else {
            panic!("Unmapped memory access")
        }
    }

    fn write_8bit_signed(&mut self, address: usize, value: i8) {
        if address < 0x8000 {
            panic!("Cartridge ROM is read-only !!!")
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.borrow_mut().write_8bit_signed(address - 0xC000, value)
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.borrow_mut().write_8bit_signed(address - 0xE000, value)
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.borrow_mut().write_8bit_signed(address - 0xFF80, value)
        }  else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.borrow_mut().write_8bit_signed(address - 0xFF00, value)
        } else {
            panic!("Unmapped memory access")
        }
    }

    fn write_16bit(&mut self, address: usize, value: u16) {
        if address < 0x8000 {
            panic!("Cartridge ROM is read-only !!!")
        } else if (address >= 0xC000) && (address < 0xE000) {
            self.ram.borrow_mut().write_16bit(address - 0xC000, value)
        } else if (address >= 0xE000) && (address < 0xF000) {
            self.ram.borrow_mut().write_16bit(address - 0xE000, value)
        } else if (address >= 0xFF80) && (address < 0xFFFF) {
            self.high_ram.borrow_mut().write_16bit(address - 0xFF80, value)
        }  else if (address >= 0xFF00) && (address < 0xFF4C) {
            self.io.borrow_mut().write_16bit(address - 0xFF00, value)
        } else {
            panic!("Unmapped memory access")
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    fn create_bus() -> MemoryBus {
        let cartridge = Rc::new(RefCell::new(Cartridge::from_bytes([0;0x8000])));
        let ram = Rc::new(RefCell::new(Ram::new()));
        let high_ram = Rc::new(RefCell::new(HighRam::new()));
        let io = Rc::new(RefCell::new(IO::new()));

        MemoryBus::new(cartridge.clone(),
                       ram.clone(),
                       high_ram.clone(),
                       io.clone())
    }

    #[test]
    fn can_read_8bit_from_ram() {
        let mut bus = create_bus();
        bus.ram.borrow_mut().write_8bit(0x10, 0x11);

        assert_eq!(bus.read_8bit(0xC010), 0x11);
        assert_eq!(bus.read_8bit(0xE010), 0x11);
    }

    #[test]
    fn can_read_8bit_signed_from_ram() {
        let mut bus = create_bus();
        bus.ram.borrow_mut().write_8bit_signed(0x10, 0x11i8);

        assert_eq!(bus.read_8bit_signed(0xC010), 0x11i8);
        assert_eq!(bus.read_8bit_signed(0xE010), 0x11i8);
    }

    #[test]
    fn can_read_16bit_from_ram() {
        let mut bus = create_bus();
        bus.ram.borrow_mut().write_16bit(0x10, 0x1122);

        assert_eq!(bus.read_16bit(0xC010), 0x1122);
        assert_eq!(bus.read_16bit(0xE010), 0x1122);
    }

    #[test]
    fn can_write_8bit_to_ram() {
        let mut bus = create_bus();
        bus.write_8bit(0xC010, 0x11);
        assert_eq!(bus.ram.borrow().read_8bit(0x10), 0x11);


        bus.write_8bit(0xE010, 0x12);
        assert_eq!(bus.ram.borrow().read_8bit(0x10), 0x12);
    }

    #[test]
    fn can_write_8bit_signed_to_ram() {
        let mut bus = create_bus();

        bus.write_8bit_signed(0xC010, 0x11i8);
        assert_eq!(bus.ram.borrow().read_8bit_signed(0x10), 0x11i8);

        bus.write_8bit_signed(0xC010, 0x12i8);
        assert_eq!(bus.ram.borrow().read_8bit_signed(0x10), 0x12i8);
    }

    #[test]
    fn can_write_16bit_to_ram() {
        let mut bus = create_bus();

        bus.write_16bit(0xC010, 0x1122);
        assert_eq!(bus.ram.borrow().read_16bit(0x10), 0x1122);

        bus.write_16bit(0xE010, 0x1123);
        assert_eq!(bus.ram.borrow().read_16bit(0x10), 0x1123);
    }

    #[test]
    fn can_read_8bit_from_high_ram() {
        let mut bus = create_bus();
        bus.high_ram.borrow_mut().write_8bit(0x10, 0x11);

        assert_eq!(bus.read_8bit(0xFF90), 0x11);
    }

    #[test]
    fn can_read_8bit_signed_from_high_ram() {
        let mut bus = create_bus();
        bus.high_ram.borrow_mut().write_8bit_signed(0x10, 0x11i8);

        assert_eq!(bus.read_8bit_signed(0xFF90), 0x11i8);
    }

    #[test]
    fn can_read_16bit_from_high_ram() {
        let mut bus = create_bus();
        bus.high_ram.borrow_mut().write_16bit(0x10, 0x1122);

        assert_eq!(bus.read_16bit(0xFF90), 0x1122);
    }

    #[test]
    fn can_write_8bit_to_high_ram() {
        let mut bus = create_bus();

        bus.write_8bit(0xFF90, 0x11);
        assert_eq!(bus.high_ram.borrow().read_8bit(0x10), 0x11);
    }

    #[test]
    fn can_write_8bit_signed_to_high_ram() {
        let mut bus = create_bus();

        bus.write_8bit_signed(0xFF90, 0x11i8);
        assert_eq!(bus.high_ram.borrow().read_8bit_signed(0x10), 0x11i8);
    }

    #[test]
    fn can_write_16bit_to_high_ram() {
        let mut bus = create_bus();

        bus.write_16bit(0xFF90, 0x1122);
        assert_eq!(bus.high_ram.borrow().read_16bit(0x10), 0x1122);
    }

    #[test]
    fn can_read_8bit_from_io() {
        let mut bus = create_bus();
        bus.io.borrow_mut().write_8bit(0x10, 0x11);

        assert_eq!(bus.read_8bit(0xFF10), 0x11);
    }

    #[test]
    fn can_read_8bit_signed_from_io() {
        let mut bus = create_bus();
        bus.io.borrow_mut().write_8bit_signed(0x10, 0x11i8);

        assert_eq!(bus.read_8bit_signed(0xFF10), 0x11i8);
    }

    #[test]
    fn can_read_16bit_from_io() {
        let mut bus = create_bus();
        bus.io.borrow_mut().write_16bit(0x10, 0x1122);

        assert_eq!(bus.read_16bit(0xFF10), 0x1122);
    }

    #[test]
    fn can_write_8bit_to_io() {
        let mut bus = create_bus();

        bus.write_8bit(0xFF10, 0x11);
        assert_eq!(bus.io.borrow().read_8bit(0x10), 0x11);
    }

    #[test]
    fn can_write_8bit_signed_to_io() {
        let mut bus = create_bus();

        bus.write_8bit_signed(0xFF10, 0x11i8);
        assert_eq!(bus.io.borrow().read_8bit_signed(0x10), 0x11i8);
    }

    #[test]
    fn can_write_16bit_to_io() {
        let mut bus = create_bus();

        bus.write_16bit(0xFF10, 0x1122);
        assert_eq!(bus.io.borrow().read_16bit(0x10), 0x1122);
    }
}
