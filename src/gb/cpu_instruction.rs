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
/// Variant type for the CPU instructions callback functions
///
enum CPUInstructionCallback {
    VoidCallback(Fn() -> void),
    U8Callback(Fn(u8) -> void),
    I8Callback(Fn(i8) -> void),
    U16Callback(Fn(u16) -> void),
    I16Callback(Fn(i16) -> void),
}


///
/// Represents a CPU instruciton
///
pub struct CPUInstruction {
    ///
    /// The disassembly name of the instruction
    ///
    name: String,
    ///
    /// The length of the operand in bytes
    ///
    operand_length: u8,
    ///
    /// The callback function of the instruction
    ///
    function: CPUInstructionCallback
}

pub const CPU_INSTRUCTIONS : [CPUInstruction; 1] = [
    CPUInstruction{
        name: "NOP",
        operand_length: 0,
        function: CPUInstructionCallback::VoidCallback(||{}),
    }
];

