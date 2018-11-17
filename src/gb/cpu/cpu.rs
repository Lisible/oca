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

use std::usize;

use gb::cpu::register::Register;
use gb::cpu::register_identifier::*;
use gb::cpu::register_8bit::*;
use gb::cpu::register_16bit::*;
use gb::cpu::bi_register_8bit::*;
use gb::memory::memory_bus::MemoryBus;
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

enum CPUFlag {
    Z,
    N,
    H,
    C
}

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
    stack_pointer: Register16Bit,
    ///
    /// Program counter, points to the next instruction to be executed
    ///
    program_counter: Register16Bit,

    ///
    /// A reference to the memory bus used by the CPU
    ///
    memory_bus: Rc<RefCell<MemoryBus>>
}

impl CPU {

    pub fn new(memory_bus : Rc<RefCell<MemoryBus>>) -> CPU {
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
            stack_pointer: Register16Bit::new(),
            program_counter: Register16Bit::new(),
            memory_bus
        }
    }

    ///
    /// Params:
    /// - flag: CPUFlag = The identifier of the desired flag
    ///
    /// Returns:
    /// - The value of the flag
    ///
    fn get_flag(&self, flag: CPUFlag) -> bool {
        let register_f_value = self.registers[&RegisterIdentifier::F].borrow().read();
        match flag {
            CPUFlag::Z => (register_f_value & (1 << 7)) != 0,
            CPUFlag::N => (register_f_value & (1 << 6)) != 0,
            CPUFlag::H => (register_f_value & (1 << 5)) != 0,
            CPUFlag::C => (register_f_value & (1 << 4)) != 0,
        }
    }

    ///
    /// Sets a flag
    ///
    /// Params:
    /// - flag: CPUFlag = The flag to set
    ///
    fn set_flag(&mut self, flag: CPUFlag) {
        let mut register_f = self.registers[&RegisterIdentifier::F].borrow_mut();
        let register_f_value = register_f.read();

        match flag {
            CPUFlag::Z => register_f.write(register_f_value | (1 << 7)),
            CPUFlag::N => register_f.write(register_f_value | (1 << 6)),
            CPUFlag::H => register_f.write(register_f_value | (1 << 5)),
            CPUFlag::C => register_f.write(register_f_value | (1 << 4)),
        }
    }

    ///
    /// Unsets a flag
    ///
    /// Params:
    /// - flag: CPUFlag = The flag to unset
    ///
    fn unset_flag(&mut self, flag: CPUFlag) {
        let mut register_f = self.registers[&RegisterIdentifier::F].borrow_mut();
        let register_f_value = register_f.read();

        match flag {
            CPUFlag::Z => register_f.write(register_f_value & !(1 << 7)),
            CPUFlag::N => register_f.write(register_f_value & !(1 << 6)),
            CPUFlag::H => register_f.write(register_f_value & !(1 << 5)),
            CPUFlag::C => register_f.write(register_f_value & !(1 << 4)),
        }
    }

    ///
    /// Sets the registers up for running the emulator
    /// 
    pub fn initialize(&mut self) {
        self.registers.iter_mut().for_each(|(_, r)| r.borrow_mut().write(0x00));
        self.program_counter.write(0x0);
        self.stack_pointer.write(0xFFFE);
    }

    ///
    /// Emulates a single CPU step
    ///
    pub fn step(&mut self) {
        let mut cycles = 0;

        let opcode = self.program_counter.read();
        self.program_counter.increment(1);
        match opcode {
            // NOP
            0x00 => cycles += self.nop(),
            // LD BC,d16
            0x01 => cycles += self.ld_bi_register_d16(BiRegisterIdentifier::BC),
            // LD (BC),A
            0x02 => cycles += self.ld_bi_register_a(BiRegisterIdentifier::BC),
            // INC BC
            0x03 => cycles += self.inc_bi_register(BiRegisterIdentifier::BC),
            // INC B
            0x04 => cycles += self.inc_register(RegisterIdentifier::B),
            // DEC B
            0x05 => cycles += self.dec_register(RegisterIdentifier::B),
            // LD B,d8
            0x06 => cycles += self.ld_register_d8(RegisterIdentifier::B),
            /*// RLCA
            0x07 => cycles += self.rlca(),
            // LD (a16),SP
            0x08 => cycles += self.ld_a16_sp(),
            // ADD HL, BC
            0x09 => cycles += self.add_bi_register_bi_register(),
            // LD A,(BC)
            0x10 => cycles += self.ld_a_bi_register(),*/


            // LD DE,d16
            0x11 => cycles += self.ld_bi_register_d16(BiRegisterIdentifier::DE),
            // LD (BC),A
            0x12 => cycles += self.ld_bi_register_a(BiRegisterIdentifier::DE),
            // LD HL,d16
            0x21 => cycles += self.ld_bi_register_d16(BiRegisterIdentifier::HL),
            // LD SP,d16
            0x31 => cycles += self.ld_sp_d16(),

            _ => panic!("Unimplemented instruction")
        }
    }



    fn nop(&mut self) -> u32 {
        4
    }

    fn ld_bi_register_d16(&mut self, register_identifier: BiRegisterIdentifier) -> u32 {
        let value = self.memory_bus.borrow().read_16bit(self.program_counter.read() as usize);
        self.program_counter.increment(2);

        self.write_bi_register(register_identifier, value);
        12
    }

    fn ld_sp_d16(&mut self) -> u32 {
        let value = self.memory_bus.borrow().read_16bit(self.program_counter.read() as usize);
        self.program_counter.increment(2);

        self.program_counter.write(value);
        12
    }

    fn ld_bi_register_a(&mut self, register_identifier: BiRegisterIdentifier) -> u32 {
        let value = self.read_register(RegisterIdentifier::A);
        self.write_bi_register(register_identifier, value as u16);
        8
    }

    fn inc_bi_register(&mut self, register_identifier: BiRegisterIdentifier) -> u32 {
        self.bi_registers.get_mut(&register_identifier).unwrap().increment(1);
        8
    }

    fn inc_register(&mut self, register_identifier: RegisterIdentifier) -> u32 {
        self.registers[&register_identifier].borrow_mut().increment(1);
        4
    }

    fn dec_register(&mut self, register_identifier: RegisterIdentifier) -> u32 {
        self.registers[&register_identifier].borrow_mut().decrement(1);
        4
    }

    fn ld_register_d8(&mut self, register_identifier: RegisterIdentifier) -> u32 {
        let value = self.memory_bus.borrow().read_8bit(self.program_counter.read() as usize);
        self.program_counter.increment(1);

        self.write_register(register_identifier, value);
        8
    }

    fn rlca(&mut self) {

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
    use gb::memory::cartridge::*;

    fn create_cpu() -> CPU {
        let cartridge = Rc::new(RefCell::new(Cartridge::from_bytes([0; 0x8000])));
        let memory_bus = Rc::new(RefCell::new(MemoryBus::new(cartridge.clone())));
        CPU::new(memory_bus.clone())
    }

    #[test]
    fn can_write_to_register() {
        let mut cpu = create_cpu();
        cpu.write_register(RegisterIdentifier::A, 0x15);

        assert_eq!(cpu.registers.get(&RegisterIdentifier::A).unwrap().borrow().read(), 0x15);
    }

    #[test]
    fn can_read_from_register() {
        let mut cpu = create_cpu();
        cpu.registers.get_mut(&RegisterIdentifier::A).unwrap().borrow_mut().write(0x05);

        assert_eq!(cpu.read_register(RegisterIdentifier::A), 0x05);
    }

    #[test]
    fn can_write_to_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(BiRegisterIdentifier::AF, 0x1234);

        assert_eq!(cpu.bi_registers.get(&BiRegisterIdentifier::AF).unwrap().read(), 0x1234);
    }

    #[test]
    fn can_read_from_bi_register() {
        let mut cpu = create_cpu();
        cpu.bi_registers.get_mut(&BiRegisterIdentifier::AF).unwrap().write(0x1234);

        assert_eq!(cpu.read_bi_register(BiRegisterIdentifier::AF), 0x1234);
    }

    #[test]
    fn can_set_flag() {
        let mut cpu = create_cpu();

        {
            let register_f = cpu.registers[&RegisterIdentifier::F].borrow();
            assert_eq!(register_f.read(), 0x00);
        }

        cpu.set_flag(CPUFlag::Z);

        {
            let register_f = cpu.registers[&RegisterIdentifier::F].borrow();
            assert_eq!(register_f.read(), 0b10000000);
        }
    }

    #[test]
    fn can_unset_flag() {
        let mut cpu = create_cpu();

        {
            let mut register_f = cpu.registers[&RegisterIdentifier::F].borrow_mut();
            register_f.write(0b01000000);
            assert_eq!(register_f.read(), 0b01000000);
        }

        cpu.unset_flag(CPUFlag::N);

        {
            let mut register_f = cpu.registers[&RegisterIdentifier::F].borrow_mut();
            assert_eq!(register_f.read(), 0b00000000);
        }
    }

    #[test]
    fn can_get_flag() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::H);

        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
}

