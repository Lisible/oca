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
use gb::register::Register;
use gb::register_8bit::Register8Bit;
use gb::bi_register_8bit::BiRegister8Bit;
use std::collections::HashMap;
use std::rc::Rc;
use std::cell::RefCell;

const OP_NAMES: [&'static str; 256] = [
    "NOP","LD","LD","INC","INC","DEC","LD","RLCA",
    "LD","ADD","LD","DEC","INC","DEC","LD","RRCA",
    "STOP","LD","LD","INC","INC","DEC","LD","RLA",
    "JR","ADD","LD","DEC","INC","DEC","LD","RRA",
    "JR","LD","LD","INC","INC","DEC","LD","DAA",
    "JR","ADD","LD","DEC","INC","DEC","LD","CPL",
    "JR","LD","LD","INC","INC","DEC","LD","SCF",
    "JR","ADD","LD","DEC","INC","DEC","LD","CCF",
    "LD","LD","LD","LD","LD","LD","LD","LD",
    "LD","LD","LD","LD","LD","LD","LD","LD",
    "LD","LD","LD","LD","LD","LD","LD","LD",
    "LD","LD","LD","LD","LD","LD","LD","LD",
    "LD","LD","LD","LD","LD","LD","LD","LD",
    "LD","LD","LD","LD","LD","LD","LD","LD",
    "LD","LD","LD","LD","LD","LD","HALT","LD",
    "LD","LD","LD","LD","LD","LD","LD","LD",
    "ADD","ADD","ADD","ADD","ADD","ADD","ADD","ADD",
    "ADC","ADC","ADC","ADC","ADC","ADC","ADC","ADC",
    "SUB","SUB","SUB","SUB","SUB","SUB","SUB","SUB",
    "SBC","SBC","SBC","SBC","SBC","SBC","SBC","SBC",
    "AND","AND","AND","AND","AND","AND","AND","AND",
    "XOR","XOR","XOR","XOR","XOR","XOR","XOR","XOR",
    "OR","OR","OR","OR","OR","OR","OR","OR",
    "CP","CP","CP","CP","CP","CP","CP","CP",
    "RET","POP","JP", "JP","CALL","PUSH","ADD","RST",
    "RET","RET","JP","PREFIX","CALL","CALL","ADC","RST",
    "RET","POP","JP","UNKOWN","CALL","PUSH","SUB","RST",
    "RET","RETI","JP","UNKOWN","CALL","UNKOWN","SBC","RST",
    "LDH","POP","LD","UNKOWN","UNKOWN","PUSH","AND","RST",
    "ADD","JP","LD","UNKOWN","UNKOWN","UNKOWN","XOR","RST",
    "LDH","POP","LD","DI","UNKOWN","PUSH","OR","RST",
    "LD","LD","LD","EI","UNKOWN","UNKOWN","CP","RST"
];

const OP_CYCLES: [u8; 256] = [
    4, 12, 8, 8, 4, 4, 8, 4, 20, 8, 8, 8, 4, 4, 8, 4,
    4, 12, 8, 8, 4, 4, 8, 4, 12, 8, 8, 8, 4, 4, 8, 4,
    8, 12, 8, 8, 4, 4, 8, 4, 8, 8, 8, 8, 4, 4, 8, 4,
    8, 12, 8, 8, 12, 12, 12, 4, 8, 8, 8, 8, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    8, 8, 8, 8, 8, 8, 4, 8, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    8, 12, 12, 16, 12, 16, 8, 16, 8, 16, 12, 4, 12, 24, 8, 16,
    8, 12, 12, 0, 12, 16, 8, 16, 8, 16, 12, 0, 12, 0, 8, 16,
    12, 12, 8, 0, 0, 16, 8, 16, 16, 4, 16, 0, 0, 0, 8, 16,
    12, 12, 8, 4, 0, 16, 8, 16, 12, 8, 16, 4, 0, 0, 8, 16
];

const OP_SIZES: [u8; 256] = [
    1, 3, 1, 1, 1, 1, 2, 1, 3, 1, 1, 1, 1, 1, 2, 1,
    2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
    2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
    2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 3, 3, 3, 1, 2, 1, 1, 1, 3, 1, 3, 3, 2, 1,
    1, 1, 3, 0, 3, 1, 2, 1, 1, 1, 3, 0, 3, 0, 2, 1,
    2, 1, 2, 0, 0, 1, 2, 1, 2, 1, 3, 0, 0, 0, 2, 1,
    2, 1, 2, 1, 0, 1, 2, 1, 2, 1, 3, 1, 0, 0, 2, 1
];


///
/// Represents the GameBoy CPU
///
pub struct CPU {
    ///
    /// Contains the 8 registers of the GameBoy CPU (A, B, C, D, E, F, H, L)
    ///
    registers: HashMap<RegisterIdentifier, Rc<RefCell<Register8Bit>>>,
    ///
    /// Contains the 4 "bi-registers" of the GameBoy CPU
    /// A "bi-register" is two registers used as a bigger one
    /// The GameBoy CPU has 4 bi-registers: AF, BC, DE, HL
    ///
    bi_registers: HashMap<BiRegisterIdentifier, BiRegister8Bit>,

    ///
    /// The stack pointer, points to the current stack position
    ///
    stack_pointer: u16,
    ///
    /// Program counter, points to the next instruction to be executed
    ///
    program_counter: u16,

    cpu_instructions: [CPUInstruction; 256],
}

impl CPU {
    pub fn new() -> CPU {
        let mut registers = HashMap::new();
        registers.insert(RegisterIdentifier::A, Rc::new(RefCell::new(Register8Bit::new())));
        registers.insert(RegisterIdentifier::B, Rc::new(RefCell::new(Register8Bit::new())));
        registers.insert(RegisterIdentifier::C, Rc::new(RefCell::new(Register8Bit::new())));
        registers.insert(RegisterIdentifier::D, Rc::new(RefCell::new(Register8Bit::new())));
        registers.insert(RegisterIdentifier::E, Rc::new(RefCell::new(Register8Bit::new())));
        registers.insert(RegisterIdentifier::F, Rc::new(RefCell::new(Register8Bit::new())));
        registers.insert(RegisterIdentifier::H, Rc::new(RefCell::new(Register8Bit::new())));
        registers.insert(RegisterIdentifier::L, Rc::new(RefCell::new(Register8Bit::new())));

        let mut bi_registers = HashMap::new();
        bi_registers.insert(BiRegisterIdentifier::AF,
                            BiRegister8Bit::new(Rc::clone(&registers[&RegisterIdentifier::F]),
                                                Rc::clone(&registers[&RegisterIdentifier::A])));
        bi_registers.insert(BiRegisterIdentifier::BC,
                            BiRegister8Bit::new(Rc::clone(&registers[&RegisterIdentifier::C]),
                                                Rc::clone(&registers[&RegisterIdentifier::B])));
        bi_registers.insert(BiRegisterIdentifier::DE,
                            BiRegister8Bit::new(Rc::clone(&registers[&RegisterIdentifier::E]),
                                                Rc::clone(&registers[&RegisterIdentifier::D])));
        bi_registers.insert(BiRegisterIdentifier::HL,
                            BiRegister8Bit::new(Rc::clone(&registers[&RegisterIdentifier::L]),
                                                Rc::clone(&registers[&RegisterIdentifier::H])));

        CPU {
            registers,
            bi_registers,
            stack_pointer: 0,
            program_counter: 0,
            cpu_instructions: CPU::create_instruction_array(),
        }
    }

    ///
    /// Sets the registers up for running the emulator
    /// 
    pub fn initialize(&mut self) {
        self.registers.iter_mut().for_each(|(_, r)| r.borrow_mut().write(0x00));
        self.program_counter = 0x100;
        self.stack_pointer = 0xFFFE;
    }

    ///
    /// Writes a value into a register
    ///
    /// Params:
    /// - register_identifier: RegisterIdentifier = The identifier of the register to write to
    /// - value: u8 = The value to write into the register
    ///
    fn write_register(&mut self, register_identifier: RegisterIdentifier, value: u8) {
        self.registers.get_mut(&register_identifier).unwrap().borrow_mut().write(value);
    }
    ///
    /// Reads a value from a register
    ///
    /// Params:
    /// - register_identifier: RegisterIdentifier = The identifier of the register to read from
    ///
    /// Return:
    /// - The register value
    ///
    fn read_register(&self, register_identifier: RegisterIdentifier) -> u8 {
        self.registers.get(&register_identifier).unwrap().borrow().read()
    }

    ///
    /// Writes a value into a bi-register
    ///
    /// Params:
    /// - register_identifier: RegisterIdentifier = The identifier of the bi-register to write to
    /// - value: u16 = The value to write into the register
    ///
    fn write_bi_register(&mut self, register_identifier: BiRegisterIdentifier, value: u16) {
        self.bi_registers.get_mut(&register_identifier).unwrap().write(value);
    }
    ///
    /// Reads a value from a bi-register
    ///
    /// Params:
    /// - register_identifier: RegisterIdentifier = The identifier of the bi-register to read from
    ///
    /// Return:
    /// - The bi-register value
    ///
    fn read_bi_register(&self, register_identifier: BiRegisterIdentifier) -> u16 {
        self.bi_registers.get(&register_identifier).unwrap().read()
    }
}

#[cfg(test)]
mod test {

    use super::*;

    #[test]
    fn can_write_to_register() {
        let mut cpu = CPU::new();
        cpu.write_register(RegisterIdentifier::A, 0x15);

        assert_eq!(cpu.registers.get(&RegisterIdentifier::A).unwrap().borrow().read(), 0x15);
    }

    #[test]
    fn can_read_from_register() {
        let mut cpu = CPU::new();
        cpu.registers.get_mut(&RegisterIdentifier::A).unwrap().borrow_mut().write(0x05);

        assert_eq!(cpu.read_register(RegisterIdentifier::A), 0x05);
    }

    #[test]
    fn can_write_to_bi_register() {
        let mut cpu = CPU::new();
        cpu.write_bi_register(BiRegisterIdentifier::AF, 0x1234);

        assert_eq!(cpu.bi_registers.get(&BiRegisterIdentifier::AF).unwrap().read(), 0x1234);
    }

    #[test]
    fn can_read_from_bi_register() {
        let mut cpu = CPU::new();
        cpu.bi_registers.get_mut(&BiRegisterIdentifier::AF).unwrap().write(0x1234);

        assert_eq!(cpu.read_bi_register(BiRegisterIdentifier::AF), 0x1234);
    }
}

