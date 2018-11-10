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

use gb::register_identifier::*;

pub struct CPU {

}

impl CPU {
    pub fn new() -> CPU {
        CPU {

        }
    }
}

#[cfg(test)]
mod test {

    use super::*;

    #[test]
    fn can_write_to_register() {
        let cpu = CPU::new();
        cpu.write_register(RegisterIdentifier::A, 0x15);

        assert_eq!(cpu.registers[RegisterIdentifier::A], 0x15);
    }

    #[test]
    fn can_read_from_register() {
        let mut cpu = CPU::new();
        cpu.registers[RegisterIdentifier::A] = 0x15;

        assert_eq!(cpu.read_register(RegisterIdentifier::A), 0x15);
    }
}

