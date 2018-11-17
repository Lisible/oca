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

use gb::cpu::register_8bit::Register8Bit;
use gb::cpu::register::Register;
use std::rc::*;
use std::cell::RefCell;

///
/// Represents an 8bit bi-register: two 8 bit registers used as one 16 bit register
///
pub struct BiRegister8Bit {
    first: Rc<RefCell<Register8Bit>>,
    second: Rc<RefCell<Register8Bit>>,
}

impl BiRegister8Bit {
    pub fn new(first: Rc<RefCell<Register8Bit>>, second: Rc<RefCell<Register8Bit>>) -> BiRegister8Bit {
        BiRegister8Bit {
            first,
            second
        }
    }
}

impl Register<u16> for BiRegister8Bit {
    fn write(&mut self, value: u16) {
        self.first.borrow_mut().write((value >> 8) as u8);
        self.second.borrow_mut().write(((value << 8) >> 8) as u8);
    }
    fn read(&self) -> u16 {
        ((self.first.borrow().read() as u16) << 8) | (self.second.borrow().read() as u16)
    }

    fn increment(&mut self) {
        self.write(self.read()+1);
    }
    fn decrement(&mut self) {
        self.write(self.read()-1);
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn can_write_to_bi_register() {
        let first = Rc::new(RefCell::new(Register8Bit::new()));
        let second = Rc::new(RefCell::new(Register8Bit::new()));

        let mut bi_register = BiRegister8Bit::new(Rc::clone(&first), Rc::clone(&second));

        bi_register.write(0xF0A5);

        assert_eq!(first.borrow().read(), 0xF0);
        assert_eq!(second.borrow().read(), 0xA5);
    }

    #[test]
    fn can_read_from_bi_register() {
        let first = Rc::new(RefCell::new(Register8Bit::new()));
        let second = Rc::new(RefCell::new(Register8Bit::new()));
        first.borrow_mut().write(0xAB);
        second.borrow_mut().write(0xCD);

        let bi_register = BiRegister8Bit::new(Rc::clone(&first), Rc::clone(&second));

        assert_eq!(bi_register.read(), 0xABCD);
    }
}