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
use std::rc::Rc;
use std::cell::RefCell;

///
/// Allows access to the different part of the system memory
///
pub struct MemoryBus {
    cartridge: Rc<RefCell<Cartridge>>,
}

impl MemoryBus {
    pub fn new(cartridge: Rc<RefCell<Cartridge>>) -> MemoryBus {
        MemoryBus {
            cartridge
        }
    }

    pub fn read_8bit(&self, address: usize) -> u8 {
        if address < 0x8000 {
            self.cartridge.borrow().read_8bit(address)
        } else {
            panic!("Unmapped memory access")
        }
    }

    pub fn write_8bit(&self, address: usize, value: u8) {
        if address < 0x8000 {
            #[cfg(test)]
            {
                self.cartridge.borrow_mut().write(address, value)
            }

            #[cfg(not(test))]
            {
                panic!("Cartridge ROM is read-only !!!")
            }
        } else {
            panic!("Unmapped memory access")
        }
    }

    pub fn read_8bit_signed(&self, address: usize) -> i8 {
        if address < 0x8000 {
            self.cartridge.borrow().read_8bit(address) as i8
        } else {
            panic!("Unmapped memory access")
        }
    }

    pub fn read_16bit(&self, address: usize) -> u16 {
        if address < 0x8000 {
            self.cartridge.borrow().read_16bit(address)
        } else {
            panic!("Unmapped memory access")
        }
    }
}
