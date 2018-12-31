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

///
/// Trait for readable memory
///
pub trait ReadMemory {
    ///
    /// Reads 8-bit unsigned data from the memory at the given address
    ///
    /// Params:
    /// - address: usize = The address to read at
    ///
    /// Returns:
    /// - The read byte
    ///
    fn read_8bit(&self, address: u16) -> u8;
    ///
    /// Reads 8-bit signed data from the memory at the given address
    ///
    /// Params:
    /// - address: usize = The address to read at
    ///
    /// Returns:
    /// - The read byte
    ///
    fn read_8bit_signed(&self, address: u16) -> i8;
    ///
    /// Reads 16-bit unsigned data from the memory at the given address
    ///
    /// Params:
    /// - address: usize = The address to read at
    ///
    /// Returns:
    /// - The read bytes
    ///
    fn read_16bit(&self, address: u16) -> u16;
}

///
/// Trait for writable memory
///
pub trait WriteMemory {
    ///
    /// Writes 8-bit unsigned data to the memory at the given address
    ///
    /// Params:
    /// - address: usize = The address to write at
    ///
    fn write_8bit(&mut self, address: u16, value: u8);
    ///
    /// Writes 8-bit signed data to the memory at the given address
    ///
    /// Params:
    /// - address: usize = The address to write at
    ///
    fn write_8bit_signed(&mut self, address: u16, value: i8);
    ///
    /// Writes 16-bit unsigned data to the memory at the given address
    ///
    /// Params:
    /// - address: usize = The address to write at
    ///
    fn write_16bit(&mut self, address: u16, value: u16);
}


///
/// Implementation of ReadMemory for u8 arrays
///
impl ReadMemory for [u8] {
    fn read_8bit(&self, address: u16) -> u8 {
        self[address as usize]
    }


    fn read_8bit_signed(&self, address: u16) -> i8 {
        self.read_8bit(address) as i8
    }


    fn read_16bit(&self, address: u16) -> u16 {
        self[address as usize] as u16 | ((self[(address+1) as usize] as u16) << 8)
    }
}

///
/// Implementation of WriteMemory for u8 arrays
///
impl WriteMemory for [u8] {
    fn write_8bit(&mut self, address: u16, value: u8) {
        self[address as usize] = value;
    }


    fn write_8bit_signed(&mut self, address: u16, value: i8) {
        self.write_8bit(address, value as u8);
    }


    fn write_16bit(&mut self, address: u16, value: u16) {
        self.write_8bit(address, ((value << 8) >> 8) as u8);
        self.write_8bit(address + 1, (value >> 8) as u8);
    }
}