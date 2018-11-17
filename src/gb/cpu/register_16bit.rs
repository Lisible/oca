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

use std::u16;

use gb::cpu::register::*;

///
/// Represents an 16-bit register
///
#[derive(Clone)]
pub struct Register16Bit {
    value: u16
}

impl Register16Bit {
    pub fn new() -> Register16Bit {
        Register16Bit {
            value: 0
        }
    }
}

impl Register<u16> for Register16Bit {
    fn write(&mut self, value: u16) {
        self.value = value;
    }
    fn read(&self) -> u16 {
        self.value
    }
    fn increment(&mut self, value: u16) {
        self.value += value
    }
    fn decrement(&mut self, value: u16) {
        self.value -= value;
    }
}