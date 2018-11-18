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
use gb::memory::memory::*;
use std::rc::Rc;
use std::cell::RefCell;

///
/// Allows access to the different part of the system memory
///
pub struct MemoryBus {
    cartridge: Rc<RefCell<Cartridge>>,
    ram: Rc<RefCell<Ram>>
}

impl MemoryBus {
    pub fn new(cartridge: Rc<RefCell<Cartridge>>, ram: Rc<RefCell<Ram>>) -> MemoryBus {
        MemoryBus {
            cartridge,
            ram
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
        } else {
            panic!("Unmapped memory access")
        }
    }
}
