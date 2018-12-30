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

pub trait BitManipulation {
    fn is_bit_set(&self, bit: u8) -> bool;
    fn set_bit(&self, bit: u8) -> u8;
    fn reset_bit(&self, bit: u8) -> u8;
    fn get_bit(&self, bit: u8) -> u8;
}

impl BitManipulation for u8 {
    fn is_bit_set(&self, bit: u8) -> bool {
        return self.get_bit(bit) != 0;
    }
    fn set_bit(&self, bit: u8) -> u8 {
        self | (1 << bit)
    }

    fn reset_bit(&self, bit: u8) -> u8 {
        self & !(1 << bit)
    }

    fn get_bit(&self, bit: u8) -> u8 {
        return self & (1 << bit);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_bit_set() {
        let value: u8 = 0b00000100;
        assert_eq!(value.is_bit_set(2), true);
    }

    #[test]
    fn set_bit() {
        let value: u8 = 0b00000000;
        let value2 = value.set_bit(3);

        assert_eq!(value2, 0b00001000);
    }

    #[test]
    fn reset_bit() {
        let value: u8 = 0b00000011;
        let value2 = value.reset_bit(1);

        assert_eq!(value2, 0b00000001);
    }

    #[test]
    fn get_bit() {
        let value: u8 = 0b00000011;
        let bit = value.get_bit(1);

        assert_eq!(bit, 0b00000010);
    }
}