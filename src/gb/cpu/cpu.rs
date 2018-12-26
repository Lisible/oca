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
use gb::memory::memory::*;
use std::collections::HashMap;
use std::rc::Rc;
use std::cell::RefCell;

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
    memory_bus: Rc<RefCell<MemoryBus>>,

    ///
    /// The interrupt master enable flag
    ///
    interrupt_master_enable: bool,
    ///
    /// Delay before disabling interrupts (number of instructions)
    ///
    interrupt_disable_delay: u8,
    ///
    /// Delay before enabling interrupts (number of instructions)
    ///
    interrupt_enable_delay: u8,

    stopped: bool,
    halted: bool,

}

enum Operand8Bit<'a> {
    Register(&'a RegisterIdentifier),
    IndirectBiRegister(&'a BiRegisterIdentifier),
    IndirectBiRegisterIncrement(&'a BiRegisterIdentifier),
    IndirectBiRegisterDecrement(&'a BiRegisterIdentifier),
    IndirectAddress,
    DirectIOAddress,
    IndirectRegisterIO(&'a RegisterIdentifier),
    Direct8Bit,
}

enum Operand16Bit<'a> {
    BiRegister(&'a BiRegisterIdentifier),
    IndirectBiRegister(&'a BiRegisterIdentifier),
    IndirectBiRegisterIncrement(&'a BiRegisterIdentifier),
    IndirectBiRegisterDecrement(&'a BiRegisterIdentifier),
    IndirectAddress,
    Direct16Bit,
    StackPointer,
    StackPointerRelative,
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

        let mut cpu = CPU {
            registers,
            bi_registers,
            stack_pointer: Register16Bit::new(),
            program_counter: Register16Bit::new(),
            memory_bus,
            interrupt_master_enable: false,
            interrupt_disable_delay: 0,
            interrupt_enable_delay: 0,
            stopped: false,
            halted: false
        };
        cpu.initialize();

        cpu
    }

    pub fn dump_state(&self) -> CPUState {
        let pc = self.program_counter.read();
        let opcode = self.memory_bus.borrow().read_8bit(pc);

        CPUState {
            a: self.read_register(&RegisterIdentifier::A),
            b: self.read_register(&RegisterIdentifier::B),
            c: self.read_register(&RegisterIdentifier::C),
            d: self.read_register(&RegisterIdentifier::D),
            e: self.read_register(&RegisterIdentifier::E),
            f: self.read_register(&RegisterIdentifier::F),
            h: self.read_register(&RegisterIdentifier::H),
            l: self.read_register(&RegisterIdentifier::L),
            stack_pointer: self.stack_pointer.read(),
            program_counter: pc,
            opcode,
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
    /// - value: bool = The value to set the flag to
    ///
    fn set_flag(&mut self, flag: CPUFlag, value: bool) {
        let mut register_f = self.registers[&RegisterIdentifier::F].borrow_mut();
        let register_f_value = register_f.read();

        if value == true {
            match flag {
                CPUFlag::Z => register_f.write(register_f_value | (1 << 7)),
                CPUFlag::N => register_f.write(register_f_value | (1 << 6)),
                CPUFlag::H => register_f.write(register_f_value | (1 << 5)),
                CPUFlag::C => register_f.write(register_f_value | (1 << 4)),
            }
        } else {
            match flag {
                CPUFlag::Z => register_f.write(register_f_value & !(1 << 7)),
                CPUFlag::N => register_f.write(register_f_value & !(1 << 6)),
                CPUFlag::H => register_f.write(register_f_value & !(1 << 5)),
                CPUFlag::C => register_f.write(register_f_value & !(1 << 4)),
            }
        }
    }

    ///
    /// Sets the registers up for running the emulator
    /// 
    pub fn initialize(&mut self) {
        self.registers.iter_mut().for_each(|(_, r)| r.borrow_mut().write(0x00));
        self.program_counter.write(0x100);
        self.stack_pointer.write(0xFFFE);
        self.write_register(&RegisterIdentifier::A, 0x01);
        self.write_register(&RegisterIdentifier::F, 0xB0);
        self.write_register(&RegisterIdentifier::B, 0x00);
        self.write_register(&RegisterIdentifier::C, 0x13);
        self.write_register(&RegisterIdentifier::D, 0x00);
        self.write_register(&RegisterIdentifier::E, 0xD8);
        self.write_register(&RegisterIdentifier::H, 0x01);
        self.write_register(&RegisterIdentifier::L, 0x4D);
        self.memory_bus.borrow_mut().write_8bit(0xFF05, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF06, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF07, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF10, 0x80);
        self.memory_bus.borrow_mut().write_8bit(0xFF11, 0xBF);
        self.memory_bus.borrow_mut().write_8bit(0xFF12, 0xF3);
        self.memory_bus.borrow_mut().write_8bit(0xFF14, 0xBF);
        self.memory_bus.borrow_mut().write_8bit(0xFF16, 0x3F);
        self.memory_bus.borrow_mut().write_8bit(0xFF17, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF19, 0xBF);
        self.memory_bus.borrow_mut().write_8bit(0xFF1A, 0x7F);
        self.memory_bus.borrow_mut().write_8bit(0xFF1B, 0xFF);
        self.memory_bus.borrow_mut().write_8bit(0xFF1C, 0x9F);
        self.memory_bus.borrow_mut().write_8bit(0xFF1E, 0xBF);
        self.memory_bus.borrow_mut().write_8bit(0xFF20, 0xFF);
        self.memory_bus.borrow_mut().write_8bit(0xFF21, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF22, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF23, 0xBF);
        self.memory_bus.borrow_mut().write_8bit(0xFF24, 0x77);
        self.memory_bus.borrow_mut().write_8bit(0xFF25, 0xF3);
        self.memory_bus.borrow_mut().write_8bit(0xFF26, 0xF1);
        self.memory_bus.borrow_mut().write_8bit(0xFF40, 0x91);
        self.memory_bus.borrow_mut().write_8bit(0xFF42, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF43, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF45, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF47, 0xFC);
        self.memory_bus.borrow_mut().write_8bit(0xFF48, 0xFF);
        self.memory_bus.borrow_mut().write_8bit(0xFF49, 0xFF);
        self.memory_bus.borrow_mut().write_8bit(0xFF4A, 0x00);
        self.memory_bus.borrow_mut().write_8bit(0xFF4B, 0x00);
    }

    ///
    /// Emulates a single CPU step
    ///
    pub fn step(&mut self) {
        let mut cycles = 0;
        let pc = self.program_counter.read();
        let opcode = self.memory_bus.borrow().read_8bit(pc);

        self.program_counter.increment(1);
        match opcode {
            0x00 => cycles += self.nop(),
            0x01 => cycles += { self.ld16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC), Operand16Bit::Direct16Bit); 12 },
            0x02 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::BC), Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x03 => cycles += { self.inc16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC)); 8 },
            0x04 => cycles += { self.inc8(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x05 => cycles += { self.dec8(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x06 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::Direct8Bit); 8 },
            0x07 => cycles += self.rlca(),
            0x08 => cycles += { self.ld16(Operand16Bit::IndirectAddress, Operand16Bit::StackPointer); 20 },
            0x09 => cycles += { self.add16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::BiRegister(&BiRegisterIdentifier::BC)); 8},
            0x0A => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::BC)); 8 },
            0x0B => cycles += { self.dec16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC)); 8 },
            0x0C => cycles += { self.inc8(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x0D => cycles += { self.dec8(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x0E => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::Direct8Bit); 8 },
            0x0F => cycles += self.rrca(),
            0x10 => cycles += self.stop_0(),
            0x11 => cycles += { self.ld16(Operand16Bit::BiRegister(&BiRegisterIdentifier::DE), Operand16Bit::Direct16Bit); 12 },
            0x12 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::DE), Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x13 => cycles += { self.inc16(Operand16Bit::BiRegister(&BiRegisterIdentifier::DE)); 8 },
            0x14 => cycles += { self.inc8(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x15 => cycles += { self.dec8(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x16 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::Direct8Bit); 8 },
            0x17 => cycles += self.rla(),
            0x18 => cycles += self.jr_r8(),
            0x19 => cycles += { self.add16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::BiRegister(&BiRegisterIdentifier::DE)); 8},
            0x1A => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::DE)); 8 },
            0x1B => cycles += { self.dec16(Operand16Bit::BiRegister(&BiRegisterIdentifier::DE)); 8 },
            0x1C => cycles += { self.inc8(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x1D => cycles += { self.dec8(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x1E => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::Direct8Bit); 8 },
            0x1F => cycles += self.rra(),
            0x20 => cycles += self.jr_flag_r8(CPUFlag::Z, false),
            0x21 => cycles += { self.ld16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::Direct16Bit); 12 },
            0x22 => cycles += { self.ld8(Operand8Bit::IndirectBiRegisterIncrement(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x23 => cycles += { self.inc16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x24 => cycles += { self.inc8(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x25 => cycles += { self.dec8(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x26 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::Direct8Bit); 8 },
            0x27 => cycles += self.daa(),
            0x28 => cycles += self.jr_flag_r8(CPUFlag::Z, true),
            0x29 => cycles += { self.add16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::BiRegister(&BiRegisterIdentifier::HL)); 8},
            0x2A => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::IndirectBiRegisterIncrement(&BiRegisterIdentifier::HL)); 8 },
            0x2B => cycles += { self.dec16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x2C => cycles += { self.inc8(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x2D => cycles += { self.dec8(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x2E => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::Direct8Bit); 8 },
            0x2F => cycles += self.cpl(),
            0x30 => cycles += self.jr_flag_r8(CPUFlag::C, false),
            0x31 => cycles += { self.ld16(Operand16Bit::StackPointer, Operand16Bit::Direct16Bit); 12 },
            0x32 => cycles += { self.ld8(Operand8Bit::IndirectBiRegisterDecrement(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x33 => cycles += { self.inc16(Operand16Bit::StackPointer); 8 },
            0x34 => cycles += { self.inc8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 12 },
            0x35 => cycles += { self.dec8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 12 },
            0x36 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL), Operand8Bit::Direct8Bit); 12 },
            0x37 => cycles += self.scf(),
            0x38 => cycles += self.jr_flag_r8(CPUFlag::C, true),
            0x39 => cycles += { self.add16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::StackPointer); 8},
            0x3A => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::IndirectBiRegisterDecrement(&BiRegisterIdentifier::HL)); 8 },
            0x3B => cycles += { self.dec16(Operand16Bit::StackPointer); 8 },
            0x3C => cycles += { self.inc8(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x3D => cycles += { self.dec8(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x3E => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Direct8Bit); 8 },
            0x3F => cycles += self.ccf(),
            0x40 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x41 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x42 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x43 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x44 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x45 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x46 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x47 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::B), Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x48 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x49 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x4A => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x4B => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x4C => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x4D => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x4E => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x4F => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::C), Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x50 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x51 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x52 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x53 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x54 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x55 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x56 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x57 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::D), Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x58 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x59 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x5A => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x5B => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x5C => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x5D => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x5E => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x5F => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::E), Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x60 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x61 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x62 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x63 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x64 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x65 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x66 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x67 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::H), Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x68 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x69 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x6A => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x6B => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x6C => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x6D => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x6E => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x6F => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::L), Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x70 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x71 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x72 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x73 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x74 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x75 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x76 => cycles += self.halt(),
            0x77 => cycles += { self.ld8(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL), Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x78 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x79 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x7A => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x7B => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x7C => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x7D => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x7E => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x7F => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x80 => cycles += { self.add(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x81 => cycles += { self.add(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x82 => cycles += { self.add(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x83 => cycles += { self.add(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x84 => cycles += { self.add(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x85 => cycles += { self.add(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x86 => cycles += { self.add(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x87 => cycles += { self.add(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x88 => cycles += { self.adc(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x89 => cycles += { self.adc(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x8A => cycles += { self.adc(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x8B => cycles += { self.adc(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x8C => cycles += { self.adc(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x8D => cycles += { self.adc(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x8E => cycles += { self.adc(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x8F => cycles += { self.adc(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x90 => cycles += { self.sub(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x91 => cycles += { self.sub(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x92 => cycles += { self.sub(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x93 => cycles += { self.sub(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x94 => cycles += { self.sub(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x95 => cycles += { self.sub(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x96 => cycles += { self.sub(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x97 => cycles += { self.sub(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0x98 => cycles += { self.sbc(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0x99 => cycles += { self.sbc(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0x9A => cycles += { self.sbc(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0x9B => cycles += { self.sbc(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0x9C => cycles += { self.sbc(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0x9D => cycles += { self.sbc(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0x9E => cycles += { self.sbc(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0x9F => cycles += { self.sbc(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0xA0 => cycles += { self.and(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0xA1 => cycles += { self.and(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0xA2 => cycles += { self.and(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0xA3 => cycles += { self.and(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0xA4 => cycles += { self.and(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0xA5 => cycles += { self.and(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0xA6 => cycles += { self.and(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0xA7 => cycles += { self.and(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0xA8 => cycles += { self.xor(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0xA9 => cycles += { self.xor(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0xAA => cycles += { self.xor(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0xAB => cycles += { self.xor(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0xAC => cycles += { self.xor(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0xAD => cycles += { self.xor(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0xAE => cycles += { self.xor(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0xAF => cycles += { self.xor(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0xB0 => cycles += { self.or(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0xB1 => cycles += { self.or(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0xB2 => cycles += { self.or(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0xB3 => cycles += { self.or(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0xB4 => cycles += { self.or(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0xB5 => cycles += { self.or(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0xB6 => cycles += { self.or(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0xB7 => cycles += { self.or(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0xB8 => cycles += { self.cp(Operand8Bit::Register(&RegisterIdentifier::B)); 4 },
            0xB9 => cycles += { self.cp(Operand8Bit::Register(&RegisterIdentifier::C)); 4 },
            0xBA => cycles += { self.cp(Operand8Bit::Register(&RegisterIdentifier::D)); 4 },
            0xBB => cycles += { self.cp(Operand8Bit::Register(&RegisterIdentifier::E)); 4 },
            0xBC => cycles += { self.cp(Operand8Bit::Register(&RegisterIdentifier::H)); 4 },
            0xBD => cycles += { self.cp(Operand8Bit::Register(&RegisterIdentifier::L)); 4 },
            0xBE => cycles += { self.cp(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 8 },
            0xBF => cycles += { self.cp(Operand8Bit::Register(&RegisterIdentifier::A)); 4 },
            0xC0 => cycles += self.ret_flag(CPUFlag::Z, false),
            0xC1 => cycles += { self.pop16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC)); 12 },
            0xC2 => cycles += self.jp_flag_a16(CPUFlag::Z, false),
            0xC3 => cycles += self.jp_a16(),
            0xC4 => cycles += self.call_flag_a16(CPUFlag::Z, false),
            0xC5 => cycles += { self.push16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC)); 12 },
            0xC6 => cycles += { self.add(Operand8Bit::Direct8Bit); 8 },
            0xC7 => cycles += self.rst(0x00),
            0xC8 => cycles += self.ret_flag(CPUFlag::Z, true),
            0xC9 => cycles += self.ret(),
            0xCA => cycles += self.jp_flag_a16(CPUFlag::Z, true),
            0xCB => cycles += self.cb_instruction(),
            0xCC => cycles += self.call_flag_a16(CPUFlag::Z, true),
            0xCD => cycles += self.call_a16(),
            0xCE => cycles += { self.adc(Operand8Bit::Direct8Bit); 8 },
            0xCF => cycles += self.rst(0x08),
            0xD0 => cycles += self.ret_flag(CPUFlag::C, false),
            0xC1 => cycles += { self.pop16(Operand16Bit::BiRegister(&BiRegisterIdentifier::DE)); 12 },
            0xD2 => cycles += self.jp_flag_a16(CPUFlag::C, false),
            0xD4 => cycles += self.call_flag_a16(CPUFlag::C, false),
            0xD5 => cycles += { self.push16(Operand16Bit::BiRegister(&BiRegisterIdentifier::DE)); 12 },
            0xD6 => cycles += { self.sub(Operand8Bit::Direct8Bit); 8 },
            0xD7 => cycles += self.rst(0x10),
            0xD8 => cycles += self.ret_flag(CPUFlag::C, true),
            0xD9 => cycles += self.reti(),
            0xDA => cycles += self.jp_flag_a16(CPUFlag::C, true),
            0xDC => cycles += self.call_flag_a16(CPUFlag::C, true),
            0xDE => cycles += { self.sbc(Operand8Bit::Direct8Bit); 8 },
            0xDF => cycles += self.rst(0x18),
            0xE0 => cycles += { self.ld8(Operand8Bit::DirectIOAddress, Operand8Bit::Register(&RegisterIdentifier::A)); 12 },
            0xE1 => cycles += { self.pop16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL)); 12 },
            0xE2 => cycles += { self.ld8(Operand8Bit::IndirectRegisterIO(&RegisterIdentifier::C), Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0xE5 => cycles += { self.push16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL)); 12 },
            0xE6 => cycles += { self.and(Operand8Bit::Direct8Bit); 8 },
            0xE7 => cycles += self.rst(0x20),
            0xE8 => cycles += self.add_sp_r8(),
            0xE9 => cycles += self.jp_bi_register_ptr(&BiRegisterIdentifier::HL),
            0xEA => cycles += { self.ld8(Operand8Bit::IndirectAddress, Operand8Bit::Register(&RegisterIdentifier::A)); 16 },
            0xEE => cycles += { self.xor(Operand8Bit::Direct8Bit); 8 },
            0xEF => cycles += self.rst(0x28),
            0xF0 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::DirectIOAddress); 12 },
            0xF1 => cycles += { self.pop16(Operand16Bit::BiRegister(&BiRegisterIdentifier::AF)); 12 },
            0xF2 => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::IndirectRegisterIO(&RegisterIdentifier::C)); 8 },
            0xF3 => cycles += self.di(),
            0xF5 => cycles += { self.push16(Operand16Bit::BiRegister(&BiRegisterIdentifier::AF)); 12 },
            0xF6 => cycles += { self.or(Operand8Bit::Direct8Bit); 8 },
            0xF7 => cycles += self.rst(0x30),
            0xF8 => cycles += { self.ld16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::StackPointerRelative); 12 },
            0xF9 => cycles += { self.ld16(Operand16Bit::StackPointer, Operand16Bit::BiRegister(&BiRegisterIdentifier::HL)); 8 },
            0xFA => cycles += { self.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::IndirectAddress); 16 },
            0xFB => cycles += self.ei(),
            0xFE => cycles += { self.cp(Operand8Bit::Direct8Bit); 8 },
            0xFF => cycles += self.rst(0x38),
            _ => panic!("Unimplemented instruction")
        }
    }

    fn cb_instruction(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let opcode = self.memory_bus.borrow().read_8bit(pc);
        match opcode {
            0x00 => { self.rlc(Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x01 => { self.rlc(Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x02 => { self.rlc(Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x03 => { self.rlc(Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x04 => { self.rlc(Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x05 => { self.rlc(Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x06 => { self.rlc(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 },
            0x07 => { self.rlc(Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x08 => { self.rrc(Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x09 => { self.rrc(Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x0A => { self.rrc(Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x0B => { self.rrc(Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x0C => { self.rrc(Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x0D => { self.rrc(Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x0E => { self.rrc(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 },
            0x0F => { self.rrc(Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x10 => { self.rl(Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x11 => { self.rl(Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x12 => { self.rl(Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x13 => { self.rl(Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x14 => { self.rl(Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x15 => { self.rl(Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x16 => { self.rl(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 },
            0x17 => { self.rl(Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x18 => { self.rr(Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x19 => { self.rr(Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x1A => { self.rr(Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x1B => { self.rr(Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x1C => { self.rr(Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x1D => { self.rr(Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x1E => { self.rr(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 },
            0x1F => { self.rr(Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x20 => { self.sla(Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x21 => { self.sla(Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x22 => { self.sla(Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x23 => { self.sla(Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x24 => { self.sla(Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x25 => { self.sla(Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x26 => { self.sla(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 },
            0x27 => { self.sla(Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x28 => { self.sra(Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x29 => { self.sra(Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x2A => { self.sra(Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x2B => { self.sra(Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x2C => { self.sra(Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x2D => { self.sra(Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x2E => { self.sra(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 },
            0x2F => { self.sra(Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x30 => { self.swap(Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x31 => { self.swap(Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x32 => { self.swap(Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x33 => { self.swap(Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x34 => { self.swap(Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x35 => { self.swap(Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x36 => { self.swap(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 },
            0x37 => { self.swap(Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x38 => { self.srl(Operand8Bit::Register(&RegisterIdentifier::B)); 8 },
            0x39 => { self.srl(Operand8Bit::Register(&RegisterIdentifier::C)); 8 },
            0x3A => { self.srl(Operand8Bit::Register(&RegisterIdentifier::D)); 8 },
            0x3B => { self.srl(Operand8Bit::Register(&RegisterIdentifier::E)); 8 },
            0x3C => { self.srl(Operand8Bit::Register(&RegisterIdentifier::H)); 8 },
            0x3D => { self.srl(Operand8Bit::Register(&RegisterIdentifier::L)); 8 },
            0x3E => { self.srl(Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 },
            0x3F => { self.srl(Operand8Bit::Register(&RegisterIdentifier::A)); 8 },
            0x40 => { self.bit(0, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x41 => { self.bit(0, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x42 => { self.bit(0, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x43 => { self.bit(0, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x44 => { self.bit(0, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x45 => { self.bit(0, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x46 => { self.bit(0, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x47 => { self.bit(0, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x48 => { self.bit(1, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x49 => { self.bit(1, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x4A => { self.bit(1, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x4B => { self.bit(1, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x4C => { self.bit(1, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x4D => { self.bit(1, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x4E => { self.bit(1, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x4F => { self.bit(1, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x50 => { self.bit(2, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x51 => { self.bit(2, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x52 => { self.bit(2, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x53 => { self.bit(2, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x54 => { self.bit(2, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x55 => { self.bit(2, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x56 => { self.bit(2, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x57 => { self.bit(2, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x58 => { self.bit(3, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x59 => { self.bit(3, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x5A => { self.bit(3, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x5B => { self.bit(3, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x5C => { self.bit(3, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x5D => { self.bit(3, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x5E => { self.bit(3, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x5F => { self.bit(3, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x60 => { self.bit(4, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x61 => { self.bit(4, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x62 => { self.bit(4, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x63 => { self.bit(4, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x64 => { self.bit(4, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x65 => { self.bit(4, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x66 => { self.bit(4, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x67 => { self.bit(4, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x68 => { self.bit(5, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x69 => { self.bit(5, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x6A => { self.bit(5, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x6B => { self.bit(5, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x6C => { self.bit(5, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x6D => { self.bit(5, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x6E => { self.bit(5, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x6F => { self.bit(5, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x70 => { self.bit(6, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x71 => { self.bit(6, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x72 => { self.bit(6, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x73 => { self.bit(6, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x74 => { self.bit(6, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x75 => { self.bit(6, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x76 => { self.bit(6, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x77 => { self.bit(6, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x78 => { self.bit(7, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x79 => { self.bit(7, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x7A => { self.bit(7, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x7B => { self.bit(7, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x7C => { self.bit(7, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x7D => { self.bit(7, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x7E => { self.bit(7, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x7F => { self.bit(7, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x80 => { self.res(0, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x81 => { self.res(0, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x82 => { self.res(0, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x83 => { self.res(0, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x84 => { self.res(0, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x85 => { self.res(0, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x86 => { self.res(0, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x87 => { self.res(0, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x88 => { self.res(1, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x89 => { self.res(1, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x8A => { self.res(1, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x8B => { self.res(1, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x8C => { self.res(1, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x8D => { self.res(1, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x8E => { self.res(1, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x8F => { self.res(1, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x90 => { self.res(2, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x91 => { self.res(2, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x92 => { self.res(2, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x93 => { self.res(2, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x94 => { self.res(2, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x95 => { self.res(2, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x96 => { self.res(2, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x97 => { self.res(2, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0x98 => { self.res(3, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0x99 => { self.res(3, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0x9A => { self.res(3, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0x9B => { self.res(3, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0x9C => { self.res(3, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0x9D => { self.res(3, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0x9E => { self.res(3, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0x9F => { self.res(3, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xA0 => { self.res(4, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xA1 => { self.res(4, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xA2 => { self.res(4, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xA3 => { self.res(4, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xA4 => { self.res(4, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xA5 => { self.res(4, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xA6 => { self.res(4, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xA7 => { self.res(4, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xA8 => { self.res(5, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xA9 => { self.res(5, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xAA => { self.res(5, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xAB => { self.res(5, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xAC => { self.res(5, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xAD => { self.res(5, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xAE => { self.res(5, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xAF => { self.res(5, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xB0 => { self.res(6, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xB1 => { self.res(6, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xB2 => { self.res(6, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xB3 => { self.res(6, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xB4 => { self.res(6, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xB5 => { self.res(6, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xB6 => { self.res(6, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xB7 => { self.res(6, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xB8 => { self.res(7, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xB9 => { self.res(7, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xBA => { self.res(7, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xBB => { self.res(7, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xBC => { self.res(7, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xBD => { self.res(7, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xBE => { self.res(7, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xBF => { self.res(7, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xC0 => { self.set(0, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xC1 => { self.set(0, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xC2 => { self.set(0, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xC3 => { self.set(0, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xC4 => { self.set(0, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xC5 => { self.set(0, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xC6 => { self.set(0, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xC7 => { self.set(0, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xC8 => { self.set(1, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xC9 => { self.set(1, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xCA => { self.set(1, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xCB => { self.set(1, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xCC => { self.set(1, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xCD => { self.set(1, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xCE => { self.set(1, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xCF => { self.set(1, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xD0 => { self.set(2, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xD1 => { self.set(2, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xD2 => { self.set(2, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xD3 => { self.set(2, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xD4 => { self.set(2, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xD5 => { self.set(2, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xD6 => { self.set(2, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xD7 => { self.set(2, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xD8 => { self.set(3, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xD9 => { self.set(3, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xDA => { self.set(3, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xDB => { self.set(3, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xDC => { self.set(3, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xDD => { self.set(3, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xDE => { self.set(3, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xDF => { self.set(3, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xE0 => { self.set(4, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xE1 => { self.set(4, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xE2 => { self.set(4, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xE3 => { self.set(4, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xE4 => { self.set(4, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xE5 => { self.set(4, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xE6 => { self.set(4, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xE7 => { self.set(4, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xE8 => { self.set(5, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xE9 => { self.set(5, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xEA => { self.set(5, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xEB => { self.set(5, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xEC => { self.set(5, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xED => { self.set(5, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xEE => { self.set(5, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xEF => { self.set(5, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xF0 => { self.set(6, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xF1 => { self.set(6, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xF2 => { self.set(6, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xF3 => { self.set(6, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xF4 => { self.set(6, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xF5 => { self.set(6, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xF6 => { self.set(6, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xF7 => { self.set(6, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            0xF8 => { self.set(7, Operand8Bit::Register(&RegisterIdentifier::B)); 8 }
            0xF9 => { self.set(7, Operand8Bit::Register(&RegisterIdentifier::C)); 8 }
            0xFA => { self.set(7, Operand8Bit::Register(&RegisterIdentifier::D)); 8 }
            0xFB => { self.set(7, Operand8Bit::Register(&RegisterIdentifier::E)); 8 }
            0xFC => { self.set(7, Operand8Bit::Register(&RegisterIdentifier::H)); 8 }
            0xFD => { self.set(7, Operand8Bit::Register(&RegisterIdentifier::L)); 8 }
            0xFE => { self.set(7, Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::HL)); 16 }
            0xFF => { self.set(7, Operand8Bit::Register(&RegisterIdentifier::A)); 8 }
            _ => panic!("Unimplemented CB instruction")
        }
    }

    fn read_value_from_8bit_operand(&mut self, source: &Operand8Bit) ->  u8 {
        match source {
            Operand8Bit::Direct8Bit => {
                let pc = self.program_counter.read();
                let value = self.memory_bus.borrow().read_8bit(pc);
                self.program_counter.increment(1);

                value
            },
            Operand8Bit::Register(identifier) => {
                self.read_register(identifier)
            },
            Operand8Bit::IndirectBiRegister(identifier) |
            Operand8Bit::IndirectBiRegisterIncrement(identifier) |
            Operand8Bit::IndirectBiRegisterDecrement(identifier) => {
                let address = self.read_bi_register(identifier);

                if let Operand8Bit::IndirectBiRegisterIncrement(identifier) = source {
                    self.write_bi_register(identifier, address.wrapping_add(1));
                } else if let Operand8Bit::IndirectBiRegisterDecrement(identifier) = source {
                    self.write_bi_register(identifier, address.wrapping_sub(1));
                }

                self.memory_bus.borrow().read_8bit(address)
            },
            Operand8Bit::IndirectRegisterIO(identifier) => {
                let offset = self.read_register(identifier);
                let resulting_address = 0xFF00 + offset as u16;
                self.memory_bus.borrow().read_8bit(resulting_address)
            }
            Operand8Bit::IndirectAddress => {
                let pc = self.program_counter.read();
                let address = self.memory_bus.borrow().read_16bit(pc);
                self.program_counter.increment(2);
                self.memory_bus.borrow().read_8bit(address)
            },
            Operand8Bit::DirectIOAddress => {
                let pc = self.program_counter.read();
                let offset = self.memory_bus.borrow().read_8bit_signed(pc);
                let resulting_address = 0xFF00 + offset as u16;
                self.program_counter.increment(1);
                self.memory_bus.borrow().read_8bit(resulting_address)
            }
            _ => panic!("Not an 8bit source")
        }
    }
    fn read_value_from_16bit_operand(&mut self, source: &Operand16Bit) -> u16 {
        match source {
            Operand16Bit::Direct16Bit => {
                let pc = self.program_counter.read();
                let value = self.memory_bus.borrow().read_16bit(pc);
                self.program_counter.increment(2);

                value
            },
            Operand16Bit::BiRegister(identifier) => {
                self.read_bi_register(identifier)
            },
            Operand16Bit::IndirectBiRegister(identifier) |
            Operand16Bit::IndirectBiRegisterIncrement(identifier) |
            Operand16Bit::IndirectBiRegisterDecrement(identifier) => {
                let address = self.read_bi_register(identifier);

                if let Operand16Bit::IndirectBiRegisterIncrement(identifier) = source {
                    self.write_bi_register(identifier, address.wrapping_add(1));
                } else if let Operand16Bit::IndirectBiRegisterDecrement(identifier) = source {
                    self.write_bi_register(identifier, address.wrapping_sub(1));
                }

                self.memory_bus.borrow().read_16bit(address)
            },
            Operand16Bit::IndirectAddress => {
                let pc = self.program_counter.read();
                let address = self.memory_bus.borrow().read_16bit(pc);
                self.program_counter.increment(2);
                self.memory_bus.borrow().read_16bit(address)
            },
            Operand16Bit::StackPointer => {
                self.stack_pointer.read()
            },
            StackPointerRelative => {
                let pc = self.program_counter.read();
                let offset = self.memory_bus.borrow().read_8bit_signed(pc);
                self.program_counter.increment(1);

                let sp = self.stack_pointer.read();

                (sp as i32 + offset as i32) as u16
            }
            _ => panic!("Not a 16bit source")
        }
    }
    fn write_value_to_8bit_operand(&mut self, destination: &Operand8Bit, value: u8) {
        match destination {
            Operand8Bit::Register(identifier) => {
                self.write_register(identifier, value);
            },
            Operand8Bit::IndirectBiRegister(identifier) |
            Operand8Bit::IndirectBiRegisterIncrement(identifier) |
            Operand8Bit::IndirectBiRegisterDecrement(identifier) => {
                let address = self.read_bi_register(identifier);

                if let Operand8Bit::IndirectBiRegisterIncrement(identifier) = destination {
                    self.write_bi_register(identifier, address.wrapping_add(1));
                } else if let Operand8Bit::IndirectBiRegisterDecrement(identifier) = destination {
                    self.write_bi_register(identifier, address.wrapping_sub(1));
                };

                self.memory_bus.borrow_mut().write_8bit(address, value);
            },
            Operand8Bit::IndirectRegisterIO(identifier) => {
                let offset = self.read_register(identifier);
                let resulting_address = 0xFF00u16.wrapping_add(offset as u16);
                self.memory_bus.borrow_mut().write_8bit(resulting_address, value)
            },
            Operand8Bit::DirectIOAddress => {
                let pc = self.program_counter.read();
                let offset = self.memory_bus.borrow().read_8bit(pc);
                let resulting_address = 0xFF00u16.wrapping_add(offset as u16);
                self.program_counter.increment(1);
                self.memory_bus.borrow_mut().write_8bit(resulting_address, value);
            }
            _ => panic!("Not an 8bit destination")
        }
    }
    fn write_value_to_16bit_operand(&mut self, destination: &Operand16Bit, value: u16) {
        match destination {
            Operand16Bit::BiRegister(identifier) => {
                self.write_bi_register(identifier, value);
            },
            Operand16Bit::IndirectBiRegister(identifier) |
            Operand16Bit::IndirectBiRegisterIncrement(identifier) |
            Operand16Bit::IndirectBiRegisterDecrement(identifier) => {
                let address = self.read_bi_register(identifier);

                if let Operand16Bit::IndirectBiRegisterIncrement(identifier) = destination {
                    self.write_bi_register(identifier, address.wrapping_add(1));
                } else if let Operand16Bit::IndirectBiRegisterDecrement(identifier) = destination {
                    self.write_bi_register(identifier, address.wrapping_sub(1));
                };

                self.memory_bus.borrow_mut().write_16bit(address, value);
            },
            Operand16Bit::StackPointer => {
                self.stack_pointer.write(value);
            },
            StackPointerRelative => {
                let pc = self.program_counter.read();
                let offset = self.memory_bus.borrow().read_8bit_signed(pc);
                self.program_counter.increment(1);

                let sp = self.stack_pointer.read();
                self.memory_bus.borrow_mut().write_16bit((sp as i32 + offset as i32) as u16, value);
            }
            _ => panic!("Not a 16bit destination")
        }
    }



    // Control instrucitons

    fn nop(&mut self) -> u32 {
        4
    }

    fn stop_0(&mut self) -> u32 {
        self.program_counter.increment(1);
        self.stopped = true;
        4
    }

    fn halt(&mut self) -> u32 {
        self.halted = true;
        1
    }

    fn ei(&mut self) -> u32 {
        self.interrupt_enable_delay = 1;
        4
    }

    fn di(&mut self) -> u32 {
        self.interrupt_disable_delay = 1;
        4
    }

    // 8-bit load/move

    fn ld8(&mut self, destination: Operand8Bit, source: Operand8Bit) {
        let value = self.read_value_from_8bit_operand(&source);
        self.write_value_to_8bit_operand(&destination, value);
    }

    // 16-bit load/move

    fn ld16(&mut self, destination: Operand16Bit, source: Operand16Bit) {
        let value = self.read_value_from_16bit_operand(&source);
        self.write_value_to_16bit_operand(&destination, value);
    }

    fn push16(&mut self, operand: Operand16Bit) -> u32 {
        let value = self.read_value_from_16bit_operand(&operand);
        let sp = self.stack_pointer.read();
        self.stack_pointer.decrement(2);
        self.memory_bus.borrow_mut().write_16bit(sp, value);
        16
    }

    fn pop16(&mut self, operand: Operand16Bit) -> u32 {
        self.stack_pointer.increment(2);
        let sp = self.stack_pointer.read();
        let value = self.memory_bus.borrow().read_16bit(sp);
        self.write_value_to_16bit_operand(&operand, value);
        12
    }



    // 8-bit ALU

    fn inc8(&mut self, operand: Operand8Bit) {
        let lhs = self.read_value_from_8bit_operand(&operand);
        let sum = lhs.wrapping_add(1);

        self.set_flag(CPUFlag::Z, sum == 0);
        self.set_flag(CPUFlag::H, (sum & 0xF) < (lhs & 0xF));
        self.set_flag(CPUFlag::N, false);

        self.write_value_to_8bit_operand(&operand, sum);
    }

    fn dec8(&mut self, operand: Operand8Bit) {
        let lhs = self.read_value_from_8bit_operand(&operand);
        let difference = lhs.wrapping_sub(1);

        self.set_flag(CPUFlag::Z, difference == 0);
        self.set_flag(CPUFlag::H, (difference & 0xF) <= (lhs & 0xF));
        self.set_flag(CPUFlag::N, true);

        self.write_value_to_8bit_operand(&operand, difference);
    }

    fn daa(&mut self) -> u32 {
        use std::u8;

        let mut register_a = self.read_register(&RegisterIdentifier::A);

        if !self.get_flag(CPUFlag::N) {
            if self.get_flag(CPUFlag::C) || register_a > 0x99 {
                register_a = register_a.wrapping_add(0x60);
                self.set_flag(CPUFlag::C, true);
            }

            if self.get_flag(CPUFlag::H) || (register_a & 0x0f) > 0x09 {
                register_a = register_a.wrapping_add(0x06);
            }
        }
            else {
                if self.get_flag(CPUFlag::C) {
                    register_a = register_a.wrapping_sub(0x60);
                }
                if self.get_flag(CPUFlag::H) {
                    register_a = register_a.wrapping_sub(0x06);
                }
            }

        self.write_register(&RegisterIdentifier::A, register_a);
        self.set_flag(CPUFlag::Z, register_a == 0);
        self.set_flag(CPUFlag::H, false);

        4
    }

    fn cpl(&mut self) -> u32 {
        let na = !self.read_register(&RegisterIdentifier::A);
        self.write_register(&RegisterIdentifier::A, na);
        4
    }

    fn scf(&mut self) -> u32 {
        self.set_flag(CPUFlag::C, true);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        4
    }

    fn ccf(&mut self) -> u32 {
        let c = self.get_flag(CPUFlag::C);
        self.set_flag(CPUFlag::C, !c);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        4
    }

    fn add(&mut self, operand: Operand8Bit) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_value_from_8bit_operand(&operand);

        let sum = (lhs as u16).wrapping_add(rhs as u16);

        self.set_flag(CPUFlag::Z, sum as u8 == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, (sum as u8 & 0x0F) < (lhs & 0x0F));
        self.set_flag(CPUFlag::C, sum > 0xFF);

        self.write_register(&RegisterIdentifier::A, sum as u8);
    }

    fn adc(&mut self, operand: Operand8Bit) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_value_from_8bit_operand(&operand);
        let rhs = rhs.wrapping_add(if self.get_flag(CPUFlag::C) {1}  else {0});

        let sum = (lhs as u16).wrapping_add(rhs as u16);

        self.set_flag(CPUFlag::Z, sum as u8 == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, (sum as u8 & 0x0F) < (lhs & 0x0F));
        self.set_flag(CPUFlag::C, sum > 0xFF);

        self.write_register(&RegisterIdentifier::A, sum as u8);
    }

    fn sub(&mut self, operand: Operand8Bit) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_value_from_8bit_operand(&operand);
        let difference = (lhs as u16).wrapping_sub(rhs as u16);

        self.set_flag(CPUFlag::Z, difference == 0);
        self.set_flag(CPUFlag::N, true);
        self.set_flag(CPUFlag::H, (rhs & 0x0F) > (lhs & 0x0F));
        self.set_flag(CPUFlag::C, rhs > lhs);

        self.write_register(&RegisterIdentifier::A, difference as u8);
    }

    fn sbc(&mut self, operand: Operand8Bit) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_value_from_8bit_operand(&operand);
        let rhs = rhs.wrapping_add(if self.get_flag(CPUFlag::C) {1}  else {0});
        let difference = (lhs as u16).wrapping_sub(rhs as u16);

        self.set_flag(CPUFlag::Z, difference == 0);
        self.set_flag(CPUFlag::N, true);
        self.set_flag(CPUFlag::H, (rhs & 0x0F) > (lhs & 0x0F));
        self.set_flag(CPUFlag::C, rhs > lhs);

        self.write_register(&RegisterIdentifier::A, difference as u8);
    }

    fn and(&mut self, operand: Operand8Bit) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_value_from_8bit_operand(&operand);
        let result = lhs & rhs;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, true);
        self.set_flag(CPUFlag::C, false);

        self.write_register(&RegisterIdentifier::A, result);
    }

    fn xor(&mut self, operand: Operand8Bit) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_value_from_8bit_operand(&operand);
        let result = lhs ^ rhs;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, false);

        self.write_register(&RegisterIdentifier::A, result);
    }

    fn or(&mut self, operand: Operand8Bit) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_value_from_8bit_operand(&operand);
        let result = lhs | rhs;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, false);

        self.write_register(&RegisterIdentifier::A, result);
    }

    fn cp(&mut self, operand: Operand8Bit) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_value_from_8bit_operand(&operand);
        let difference = (lhs as u16).wrapping_sub(rhs as u16);

        self.set_flag(CPUFlag::Z, difference == 0);
        self.set_flag(CPUFlag::N, true);
        self.set_flag(CPUFlag::H, (rhs & 0x0F) > (lhs & 0x0F));
        self.set_flag(CPUFlag::C, rhs > lhs);
    }

    // 16-bit ALU
    fn inc16(&mut self, operand: Operand16Bit) {
        let value = self.read_value_from_16bit_operand(&operand);
        self.write_value_to_16bit_operand(&operand, value.wrapping_add(1));
    }
    fn dec16(&mut self, operand: Operand16Bit) {
        let value = self.read_value_from_16bit_operand(&operand);
        self.write_value_to_16bit_operand(&operand, value.wrapping_sub(1));
    }
    fn add16(&mut self, first_operand: Operand16Bit, second_operand: Operand16Bit) {
        let lhs = self.read_value_from_16bit_operand(&first_operand);
        let rhs = self.read_value_from_16bit_operand(&second_operand);
        let sum = (lhs as u32).wrapping_add(rhs as u32);

        if let Operand16Bit::StackPointer = first_operand {
            self.set_flag(CPUFlag::Z, false);
        }

        self.set_flag(CPUFlag::H, ((sum & 0x0FFF) as u16) < (lhs & 0x0FFF));
        self.set_flag(CPUFlag::C, sum > 0xFFFF);
        self.set_flag(CPUFlag::N, false);

        self.write_value_to_16bit_operand(&first_operand, sum as u16);
    }

    fn reti(&mut self) -> u32 {
        let sp = self.stack_pointer.read();
        let value = self.memory_bus.borrow().read_16bit(sp);
        self.stack_pointer.increment(2);
        self.program_counter.write(value);
        self.interrupt_master_enable = true;
        self.interrupt_enable_delay = 0;
        8
    }

    fn rlca(&mut self) -> u32 {
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);

        let a = self.read_register(&RegisterIdentifier::A);
        self.set_flag(CPUFlag::C, (a & 0x80) != 0);

        let result = (a << 1) | ((a & 0x80) >> 7);
        self.set_flag(CPUFlag::Z, result == 0);

        self.write_register(&RegisterIdentifier::A, result);
        4
    }

    fn rla(&mut self) -> u32 {
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);

        let carry_flag = self.get_flag(CPUFlag::C) as u8;

        let a = self.read_register(&RegisterIdentifier::A);
        self.set_flag(CPUFlag::C, (a & 0x80) != 0);

        let result = ((a << 1) & 0xFF) | carry_flag;
        self.set_flag(CPUFlag::Z, result == 0);

        self.write_register(&RegisterIdentifier::A, result);
        4
    }

    fn rrca(&mut self) -> u32 {
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);

        let a = self.read_register(&RegisterIdentifier::A);
        self.set_flag(CPUFlag::C, (a & 0x01) != 0);

        let mut result = (a >> 1);
        if (a & 0x01) != 0 {
            result = result | 0x80;
        }

        self.set_flag(CPUFlag::Z, result == 0);

        self.write_register(&RegisterIdentifier::A, result);
        4
    }

    fn rra(&mut self) -> u32 {
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);

        let carry_flag = self.get_flag(CPUFlag::C) as u8;

        let a = self.read_register(&RegisterIdentifier::A);
        self.set_flag(CPUFlag::C, (a & 0x01) != 0);

        let result = ((a >> 1) & 0xFF) | (carry_flag << 7);
        self.set_flag(CPUFlag::Z, result == 0);

        self.write_register(&RegisterIdentifier::A, result);

        4
    }

    fn jr_r8(&mut self) -> u32 {
        let value = self.memory_bus.borrow().read_8bit_signed(self.program_counter.read());
        self.program_counter.increment(1);

        let pc = self.program_counter.read();
        self.program_counter.write((pc as i32 + value as i32 - 1) as u16);
        12
    }

    fn jr_flag_r8(&mut self, flag: CPUFlag, jump_if_true: bool) -> u32 {
        let value = self.memory_bus.borrow().read_8bit_signed(self.program_counter.read());
        self.program_counter.increment(1);

        let pc = self.program_counter.read();

        if self.get_flag(flag) == jump_if_true {
            self.program_counter.write((pc as i32 + value as i32) as u16);
        }

        8
    }

    fn jp_a16(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let address = self.memory_bus.borrow().read_16bit(pc);
        self.program_counter.write(address);
        12
    }

    fn jp_flag_a16(&mut self, cpu_flag: CPUFlag, if_set: bool) -> u32 {
        let cycles;

        if self.get_flag(cpu_flag) == if_set {
            self.jp_a16();
            cycles = 16;
        } else {
            cycles = 12;
        }

        cycles
    }

    fn jp_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        self.program_counter.write(address);
        4
    }

    fn ret(&mut self) -> u32 {
        let sp = self.stack_pointer.read();
        let value = self.memory_bus.borrow().read_16bit(sp);
        self.stack_pointer.increment(2);
        self.program_counter.write(value);
        8
    }

    fn ret_flag(&mut self, cpu_flag: CPUFlag, if_set: bool) -> u32 {
        let cycles;

        if self.get_flag(cpu_flag) == if_set {
            self.ret();
            cycles = 20;
        } else {
            cycles = 8;
        }

        cycles
    }

    fn call_a16(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let address = self.memory_bus.borrow().read_16bit(pc);
        self.program_counter.increment(2);

        let sp = self.stack_pointer.read();
        let next_instruction_address = self.program_counter.read();
        self.memory_bus.borrow_mut().write_16bit(sp, next_instruction_address);
        self.stack_pointer.decrement(2);

        self.program_counter.write(address);
        12
    }

    fn call_flag_a16(&mut self, flag: CPUFlag, value: bool) -> u32 {
        if self.get_flag(flag) == value {
            self.call_a16();
            24
        } else {
            self.program_counter.increment(2);
            12
        }
    }

    fn rst(&mut self, address: u8) -> u32 {
        let pc = self.program_counter.read();
        let sp = self.stack_pointer.read();
        self.memory_bus.borrow_mut().write_16bit(sp, pc);
        self.stack_pointer.decrement(2);

        self.program_counter.write(0x0000 + address as u16);
        32
    }


    // 8-bit rotations/shifts and bit instructions

    fn rlc(&mut self, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        let result = (value << 1) | ((value & 0x80) >> 7);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x80 != 0);

        result
    }

    fn rl(&mut self, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        let result = (value << 1) | (self.get_flag(CPUFlag::C) as u8);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x80 != 0);

        result
    }

    fn sla(&mut self, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        let result = value << 1;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x80 != 0);

        result
    }

    fn sra(&mut self, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        let result = (value & 0x80) | (value >> 1);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x01 != 0);

        result
    }

    fn srl(&mut self, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        let result = value >> 1;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x01 != 0);

        result
    }

    fn rrc(&mut self, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        let result = (value >> 1) | ((value & 1) << 7);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 1 != 0);

        result
    }

    fn rr(&mut self, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        let result = (value >> 1) | ((self.get_flag(CPUFlag::C) as u8) << 7);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 1 != 0);

        result
    }

    fn swap(&mut self, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        let result = (value << 4) | (value >> 4);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, false);

        result
    }

    fn bit(&mut self, bit: u8, operand: Operand8Bit) {
        let value = self.read_value_from_8bit_operand(&operand);
        self.set_flag(CPUFlag::Z, (value & (1 << bit)) == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, true);
    }

    fn res(&mut self, bit: u8, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        return value & !(1 << bit)
    }

    fn set(&mut self, bit: u8, operand: Operand8Bit) -> u8 {
        let value = self.read_value_from_8bit_operand(&operand);
        return value | (1 << bit)
    }

    fn add_sp_r8(&mut self) -> u32 {
        let lhs = self.stack_pointer.read();
        let pc = self.program_counter.read();
        let rhs = self.memory_bus.borrow().read_8bit_signed(pc);
        self.program_counter.increment(1);

        let sum = (lhs as u32).wrapping_add(rhs as u32);

        self.set_flag(CPUFlag::H, ((sum & 0x0FFF) as u16) < (lhs & 0x0FFF));
        self.set_flag(CPUFlag::C, sum > 0xFFFF);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::Z, false);

        self.stack_pointer.write(sum as u16);
        16
    }


    ///
    /// Writes a value into a register
    ///
    /// Params:
    /// - register_identifier: &RegisterIdentifier = The identifier of the register to write to
    /// - value: u8 = The value to write into the register
    ///
    fn write_register(&mut self, register_identifier: &RegisterIdentifier, value: u8) {
        self.registers.get_mut(&register_identifier).unwrap().borrow_mut().write(value);
    }
    ///
    /// Reads a value from a register
    ///
    /// Params:
    /// - register_identifier: &RegisterIdentifier = The identifier of the register to read from
    ///
    /// Return:
    /// - The register value
    ///
    fn read_register(&self, register_identifier: &RegisterIdentifier) -> u8 {
        self.registers.get(&register_identifier).unwrap().borrow().read()
    }

    ///
    /// Writes a value into a bi-register
    ///
    /// Params:
    /// - register_identifier: &RegisterIdentifier = The identifier of the bi-register to write to
    /// - value: u16 = The value to write into the register
    ///
    fn write_bi_register(&mut self, register_identifier: &BiRegisterIdentifier, value: u16) {
        self.bi_registers.get_mut(&register_identifier).unwrap().write(value);
    }
    ///
    /// Reads a value from a bi-register
    ///
    /// Params:
    /// - register_identifier: &RegisterIdentifier = The identifier of the bi-register to read from
    ///
    /// Return:
    /// - The bi-register value
    ///
    fn read_bi_register(&self, register_identifier: &BiRegisterIdentifier) -> u16 {
        self.bi_registers.get(&register_identifier).unwrap().read()
    }
}

pub struct CPUState {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub f: u8,
    pub h: u8,
    pub l: u8,
    pub stack_pointer: u16,
    pub program_counter: u16,
    pub opcode: u8
}

#[cfg(test)]
mod test {

    use super::*;

    fn create_cpu() -> CPU {
        let memory_bus = Rc::new(RefCell::new(MemoryBus::new()));
        let mut cpu = CPU::new(memory_bus.clone());
        cpu.initialize();
        cpu
    }

    #[test]
    fn can_write_to_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0x15);

        assert_eq!(cpu.registers.get(&RegisterIdentifier::A).unwrap().borrow().read(), 0x15);
    }
    #[test]
    fn can_read_from_register() {
        let mut cpu = create_cpu();
        cpu.registers.get_mut(&RegisterIdentifier::A).unwrap().borrow_mut().write(0x05);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x05);
    }
    #[test]
    fn can_write_to_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::AF, 0x1234);

        assert_eq!(cpu.bi_registers.get(&BiRegisterIdentifier::AF).unwrap().read(), 0x1234);
    }
    #[test]
    fn can_read_from_bi_register() {
        let mut cpu = create_cpu();
        cpu.bi_registers.get_mut(&BiRegisterIdentifier::AF).unwrap().write(0x1234);

        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::AF), 0x1234);
    }
    #[test]
    fn can_set_flag() {
        let mut cpu = create_cpu();

        {
            let mut register_f = cpu.registers[&RegisterIdentifier::F].borrow_mut();
            register_f.write(0b01000000);
            assert_eq!(register_f.read(), 0b01000000);
        }

        cpu.set_flag(CPUFlag::N, false);

        {
            let mut register_f = cpu.registers[&RegisterIdentifier::F].borrow_mut();
            assert_eq!(register_f.read(), 0b00000000);
        }
    }
    #[test]
    fn can_get_flag() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, true);
        cpu.set_flag(CPUFlag::C, false);

        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn can_read_8bit_value_from_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0x77);
        let value = cpu.read_value_from_8bit_operand(&Operand8Bit::Register(&RegisterIdentifier::A));
        assert_eq!(value, 0x77);
    }
    #[test]
    fn can_read_8bit_value_from_direct() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC222);
        cpu.memory_bus.borrow_mut().write_8bit(0xC222, 0x77);
        let value = cpu.read_value_from_8bit_operand(&Operand8Bit::Direct8Bit);
        assert_eq!(value, 0x77);
    }
    #[test]
    fn can_read_8bit_value_from_direct_address() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC222);
        cpu.memory_bus.borrow_mut().write_16bit(0xC222, 0xC245);
        cpu.memory_bus.borrow_mut().write_8bit(0xC245, 0x77);
        let value = cpu.read_value_from_8bit_operand(&Operand8Bit::IndirectAddress);
        assert_eq!(value, 0x77);
    }
    #[test]
    fn can_read_8bit_value_from_indirect_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.memory_bus.borrow_mut().write_8bit(0xC245, 0x77);
        let value = cpu.read_value_from_8bit_operand(&Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::DE));
        assert_eq!(value, 0x77);
    }
    #[test]
    fn can_read_8bit_value_from_indirect_bi_register_increment() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.memory_bus.borrow_mut().write_8bit(0xC245, 0x77);
        let value = cpu.read_value_from_8bit_operand(&Operand8Bit::IndirectBiRegisterIncrement(&BiRegisterIdentifier::DE));
        assert_eq!(value, 0x77);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::DE), 0xC246);
    }
    #[test]
    fn can_read_8bit_value_from_indirect_bi_register_decrement() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.memory_bus.borrow_mut().write_8bit(0xC245, 0x77);
        let value = cpu.read_value_from_8bit_operand(&Operand8Bit::IndirectBiRegisterDecrement(&BiRegisterIdentifier::DE));
        assert_eq!(value, 0x77);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::DE), 0xC244);
    }
    #[test]
    fn can_read_8bit_value_from_direct_io_address() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC233);
        cpu.memory_bus.borrow_mut().write_8bit(0xC233, 0x10);
        cpu.memory_bus.borrow_mut().write_8bit(0xFF10, 0x77);
        let value = cpu.read_value_from_8bit_operand(&Operand8Bit::DirectIOAddress);
        assert_eq!(value, 0x77);
    }
    #[test]
    fn can_read_8bit_value_from_indirect_register_io() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::C, 0x10);
        cpu.memory_bus.borrow_mut().write_8bit(0xFF10, 0x77);
        let value = cpu.read_value_from_8bit_operand(&Operand8Bit::IndirectRegisterIO(&RegisterIdentifier::C));
        assert_eq!(value, 0x77);
    }
    #[test]
    fn can_write_8bit_value_to_register() {
        let mut cpu = create_cpu();
        let value = 0x77;
        cpu.write_value_to_8bit_operand(&Operand8Bit::Register(&RegisterIdentifier::A), 0x77);
        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x77);
    }
    #[test]
    fn can_write_8bit_value_to_indirect_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.write_value_to_8bit_operand(&Operand8Bit::IndirectBiRegister(&BiRegisterIdentifier::DE), 0x77);
        let value = cpu.memory_bus.borrow().read_8bit(0xC245);
        assert_eq!(value, 0x77);
    }
    #[test]
    fn can_write_8bit_value_to_indirect_bi_register_increment() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.write_value_to_8bit_operand(&Operand8Bit::IndirectBiRegisterIncrement(&BiRegisterIdentifier::DE), 0x77);
        let value = cpu.memory_bus.borrow().read_8bit(0xC245);
        assert_eq!(value, 0x77);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::DE), 0xC246);
    }
    #[test]
    fn can_write_8bit_value_to_indirect_bi_register_decrement() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.write_value_to_8bit_operand(&Operand8Bit::IndirectBiRegisterDecrement(&BiRegisterIdentifier::DE), 0x77);
        let value = cpu.memory_bus.borrow().read_8bit(0xC245);
        assert_eq!(value, 0x77);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::DE), 0xC244);
    }
    #[test]
    fn can_write_8bit_value_to_indirect_register_io() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::C, 0x10);
        cpu.write_value_to_8bit_operand(&Operand8Bit::IndirectRegisterIO(&RegisterIdentifier::C), 0x77);
        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xFF10), 0x77);
    }
    #[test]
    fn can_read_16bit_value_from_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x7777);
        let value = cpu.read_value_from_16bit_operand(&Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));
        assert_eq!(value, 0x7777);
    }
    #[test]
    fn can_read_16bit_value_from_direct() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_16bit(0xC023, 0x7777);
        let value = cpu.read_value_from_16bit_operand(&Operand16Bit::Direct16Bit);
        assert_eq!(value, 0x7777);
    }
    #[test]
    fn can_read_16bit_value_from_sp() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xFFEE);
        let value = cpu.read_value_from_16bit_operand(&Operand16Bit::StackPointer);
        assert_eq!(value, 0xFFEE);
    }
    #[test]
    fn can_read_16bit_value_from_sp_relative() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xF000);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit_signed(0xC023, -5);
        let value = cpu.read_value_from_16bit_operand(&Operand16Bit::StackPointerRelative);
        assert_eq!(value, 0xEFFB);
    }
    #[test]
    fn can_read_16bit_value_from_indirect_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0xC777);
        cpu.memory_bus.borrow_mut().write_16bit(0xC777, 0x7777);
        let value = cpu.read_value_from_16bit_operand(&Operand16Bit::IndirectBiRegister(&BiRegisterIdentifier::BC));
        assert_eq!(value, 0x7777);
    }
    #[test]
    fn can_read_16bit_value_from_indirect_bi_register_increment() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0xC777);
        cpu.memory_bus.borrow_mut().write_16bit(0xC777, 0x7777);
        let value = cpu.read_value_from_16bit_operand(&Operand16Bit::IndirectBiRegisterIncrement(&BiRegisterIdentifier::BC));
        assert_eq!(value, 0x7777);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0xC778);
    }
    #[test]
    fn can_read_16bit_value_from_indirect_bi_register_decrement() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0xC777);
        cpu.memory_bus.borrow_mut().write_16bit(0xC777, 0x7777);
        let value = cpu.read_value_from_16bit_operand(&Operand16Bit::IndirectBiRegisterDecrement(&BiRegisterIdentifier::BC));
        assert_eq!(value, 0x7777);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0xC776);
    }
    #[test]
    fn can_read_16bit_value_from_indirect_address() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_16bit(0xC023, 0xC777);
        cpu.memory_bus.borrow_mut().write_16bit(0xC777, 0x7777);
        let value = cpu.read_value_from_16bit_operand(&Operand16Bit::IndirectAddress);
        assert_eq!(value, 0x7777);
    }
    #[test]
    fn can_write_16bit_value_to_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_value_to_16bit_operand(&Operand16Bit::BiRegister(&BiRegisterIdentifier::BC), 0x7777);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0x7777);
    }
    #[test]
    fn can_write_16bit_value_to_indirect_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.write_value_to_16bit_operand(&Operand16Bit::IndirectBiRegister(&BiRegisterIdentifier::DE), 0x7777);
        let value = cpu.memory_bus.borrow().read_8bit(0xC245);
        assert_eq!(value, 0x7777);
    }
    #[test]
    fn can_write_16bit_value_to_indirect_bi_register_increment() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.write_value_to_16bit_operand(&Operand16Bit::IndirectBiRegisterIncrement(&BiRegisterIdentifier::DE), 0x7777);
        let value = cpu.memory_bus.borrow().read_8bit(0xC245);
        assert_eq!(value, 0x7777);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::DE), 0xC246);
    }
    #[test]
    fn can_write_16bit_value_to_indirect_bi_register_decrement() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0xC245);
        cpu.write_value_to_16bit_operand(&Operand16Bit::IndirectBiRegisterDecrement(&BiRegisterIdentifier::DE), 0x7777);
        let value = cpu.memory_bus.borrow().read_8bit(0xC245);
        assert_eq!(value, 0x7777);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::DE), 0xC244);
    }
    #[test]
    fn can_write_16bit_value_to_sp() {
        let mut cpu = create_cpu();
        cpu.write_value_to_16bit_operand(&Operand16Bit::StackPointer, 0x7777);
        assert_eq!(cpu.stack_pointer.read(), 0x7777);
    }

    #[test]
    fn instruction_ld8() {
        let mut cpu = create_cpu();

        cpu.write_register(&RegisterIdentifier::A, 0xFF);
        cpu.write_register(&RegisterIdentifier::B, 0x25);

        cpu.ld8(Operand8Bit::Register(&RegisterIdentifier::A), Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x25);
    }

    #[test]
    fn instruction_ld16() {
        let mut cpu = create_cpu();

        cpu.write_bi_register(&BiRegisterIdentifier::AF, 0xFFFF);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x2536);

        cpu.ld16(Operand16Bit::BiRegister(&BiRegisterIdentifier::AF), Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::AF), 0x2536);
    }
    #[test]
    fn instruction_push16() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xFFFE);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x1234);

        cpu.push16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.stack_pointer.read(), 0xFFFC);
        assert_eq!(cpu.memory_bus.borrow().read_16bit(0xFFFE), 0x1234);
    }
    #[test]
    fn instruction_pop16() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xFFFC);
        cpu.memory_bus.borrow_mut().write_16bit(0xFFFE, 0x1234);

        cpu.pop16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0x1234);
    }

    #[test]
    fn instruction_inc8() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x0A);

        cpu.inc8(Operand8Bit::Register(&RegisterIdentifier::A));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x0B);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_inc8_0() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0xFF);

        cpu.inc8(Operand8Bit::Register(&RegisterIdentifier::A));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x00);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_inc8_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x0F);

        cpu.inc8(Operand8Bit::Register(&RegisterIdentifier::A));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x10);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_dec8() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x0B);

        cpu.dec8(Operand8Bit::Register(&RegisterIdentifier::A));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x0A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_dec8_0() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x01);

        cpu.dec8(Operand8Bit::Register(&RegisterIdentifier::A));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x00);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_dec8_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x10);

        cpu.dec8(Operand8Bit::Register(&RegisterIdentifier::A));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x0F);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_daa() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);
        cpu.write_register(&RegisterIdentifier::A, 0xC0);


        cpu.daa();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x20);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }
    #[test]
    fn instruction_daa_2() {
        use std::u8;

        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, true);
        cpu.write_register(&RegisterIdentifier::A, 0x20);

        cpu.daa();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x180 as u8);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }
    #[test]
    fn instruction_cpl() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b01010101);

        cpu.cpl();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10101010);
    }
    #[test]
    fn instruction_scf() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::N, true);

        cpu.scf();

        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
    }
    #[test]
    fn instruction_ccf_1() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, true);
        cpu.set_flag(CPUFlag::N, true);

        cpu.ccf();

        assert_eq!(cpu.get_flag(CPUFlag::C), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
    }
    #[test]
    fn instruction_ccf_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.ccf();

        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
    }
    #[test]
    fn instruction_add() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x05);
        cpu.write_register(&RegisterIdentifier::B, 0x02);

        cpu.add(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x07);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_add_0() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0xFF);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.add(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x00);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }
    #[test]
    fn instruction_add_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x0F);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.add(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x10);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_adc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, true);

        cpu.write_register(&RegisterIdentifier::A, 0x05);
        cpu.write_register(&RegisterIdentifier::B, 0x02);

        cpu.adc(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x08);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_adc_0() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, true);

        cpu.write_register(&RegisterIdentifier::A, 0xFE);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.adc(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x00);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }
    #[test]
    fn instruction_adc_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, true);

        cpu.write_register(&RegisterIdentifier::A, 0x0E);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.adc(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x10);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_sub() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x05);
        cpu.write_register(&RegisterIdentifier::B, 0x02);

        cpu.sub(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x03);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_sub_0() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x01);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.sub(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x00);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_sub_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x10);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.sub(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x0F);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_sbc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, true);

        cpu.write_register(&RegisterIdentifier::A, 0x05);
        cpu.write_register(&RegisterIdentifier::B, 0x02);

        cpu.sbc(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x02);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_sbc_0() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, true);

        cpu.write_register(&RegisterIdentifier::A, 0x02);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.sbc(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x00);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_sbc_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, true);

        cpu.write_register(&RegisterIdentifier::A, 0x11);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.sbc(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x0F);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_and() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_register(&RegisterIdentifier::B, 0b01111000);

        cpu.and(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00100000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_or() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_register(&RegisterIdentifier::B, 0b01111000);

        cpu.or(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01111010);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_xor() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_register(&RegisterIdentifier::B, 0b01111000);

        cpu.xor(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01011010);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_cp() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 5);
        cpu.write_register(&RegisterIdentifier::B, 2);

        cpu.cp(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_cp_0() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 5);
        cpu.write_register(&RegisterIdentifier::B, 5);

        cpu.cp(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_cp_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0x10);
        cpu.write_register(&RegisterIdentifier::B, 0x01);

        cpu.cp(Operand8Bit::Register(&RegisterIdentifier::B));

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_inc16() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0xF551);

        cpu.inc16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0xF552);
    }
    #[test]
    fn instruction_dec16() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0xF551);

        cpu.dec16(Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0xF550);
    }
    #[test]
    fn instruction_add16() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0x1023);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x0005);

        cpu.add16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::HL), 0x1028);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_add16_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0x1FFF);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x0001);

        cpu.add16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::HL), 0x2000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_add16_c() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xFFFF);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x0001);

        cpu.add16(Operand16Bit::BiRegister(&BiRegisterIdentifier::HL), Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::HL), 0x0000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }
    #[test]
    fn instruction_add16_sp() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, true);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.stack_pointer.write(0x1023);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x0005);

        cpu.add16(Operand16Bit::StackPointer, Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.stack_pointer.read(), 0x1028);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_add16_sp_hc() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.stack_pointer.write(0x1FFF);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x0001);

        cpu.add16(Operand16Bit::StackPointer, Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.stack_pointer.read(), 0x2000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }
    #[test]
    fn instruction_add16_sp_c() {
        let mut cpu = create_cpu();

        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.stack_pointer.write(0xFFFF);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x0001);

        cpu.add16(Operand16Bit::StackPointer, Operand16Bit::BiRegister(&BiRegisterIdentifier::BC));

        assert_eq!(cpu.stack_pointer.read(), 0x0000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rla_carry0() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rla();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10100010);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }
    #[test]
    fn instruction_rlca() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, true);
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rlca();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10100011);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }
    #[test]
    fn instruction_rlca_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, true);
        cpu.write_register(&RegisterIdentifier::A, 0b01010001);
        cpu.rlca();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10100010);
        assert_eq!(cpu.get_flag(CPUFlag::C), false)
    }
    #[test]
    fn instruction_rla_carry1() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, true);
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rla();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10100011);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }
    #[test]
    fn instruction_rrca() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, true);
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rrca();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b11101000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }
    #[test]
    fn instruction_rrca_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, true);
        cpu.write_register(&RegisterIdentifier::A, 0b11010000);
        cpu.rrca();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01101000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false)
    }
    #[test]
    fn instruction_rra_carry0() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rra();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01101000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }
    #[test]
    fn instruction_rra_carry1() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, true);
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rra();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b11101000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }

    #[test]
    fn instruction_jr_r8() {
        let mut cpu = create_cpu();
        cpu.memory_bus.borrow_mut().write_8bit(0xC002, 0x05);
        cpu.program_counter.write(0xC002);

        cpu.jr_r8();

        let pc = cpu.program_counter.read();
        assert_eq!(pc, 0xC007);
    }

    #[test]
    fn instruction_stop_0() {
        let mut cpu = create_cpu();

        cpu.stop_0();
        assert_eq!(cpu.stopped, true);
    }

    #[test]
    fn instruction_jr_flag_r8() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, true);
        cpu.memory_bus.borrow_mut().write_16bit(0xC035, 0x2);
        cpu.program_counter.write(0xC035);

        cpu.jr_flag_r8(CPUFlag::Z, true);

        let current_address = cpu.program_counter.read();
        assert_eq!(current_address, 0xC038);
    }

    #[test]
    fn instruction_halt() {
        let mut cpu = create_cpu();
        assert_eq!(cpu.halted, false);

        cpu.halt();

        assert_eq!(cpu.halted, true);
    }

    #[test]
    fn instruction_jp_a16() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC035);
        cpu.memory_bus.borrow_mut().write_16bit(0xC035, 0xC999);

        cpu.jp_a16();

        assert_eq!(cpu.program_counter.read(), 0xC999);
    }

    #[test]
    fn instruction_jp_flag_a16() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.program_counter.write(0xC035);
        cpu.memory_bus.borrow_mut().write_16bit(0xC035, 0xC999);

        cpu.jp_flag_a16(CPUFlag::Z, false);

        assert_eq!(cpu.program_counter.read(), 0xC999);
    }

    #[test]
    fn instruction_jp_flag_a16_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, true);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);
        cpu.program_counter.write(0xC035);
        cpu.memory_bus.borrow_mut().write_16bit(0xC035, 0xC999);

        cpu.jp_flag_a16(CPUFlag::Z, true);

        assert_eq!(cpu.program_counter.read(), 0xC999);
    }

    #[test]
    fn instruction_ret() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xFFFC);
        cpu.memory_bus.borrow_mut().write_16bit(0xFFFC, 0xC090);

        cpu.ret();

        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
        assert_eq!(cpu.program_counter.read(), 0xC090);
    }

    #[test]
    fn instruction_ret_flag() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, true);
        cpu.stack_pointer.write(0xFFFC);
        cpu.memory_bus.borrow_mut().write_16bit(0xFFFC, 0xC090);

        cpu.ret_flag(CPUFlag::C, true);

        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
        assert_eq!(cpu.program_counter.read(), 0xC090);
    }

    #[test]
    fn instruction_ret_flag_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, false);

        cpu.stack_pointer.write(0xFFFC);
        cpu.memory_bus.borrow_mut().write_16bit(0xFFFC, 0xC090);

        cpu.ret_flag(CPUFlag::C, false);

        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
        assert_eq!(cpu.program_counter.read(), 0xC090);
    }

    #[test]
    fn instruction_ret_flag_3() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C, false)
        ;
        cpu.stack_pointer.write(0xFFFC);
        cpu.memory_bus.borrow_mut().write_16bit(0xFFFC, 0xC090);

        cpu.ret_flag(CPUFlag::C, true);

        assert_ne!(cpu.stack_pointer.read(), 0xFFFE);
        assert_ne!(cpu.program_counter.read(), 0xC090);
    }

    #[test]
    fn instruction_call_a16() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC000);
        cpu.memory_bus.borrow_mut().write_16bit(0xC000, 0xC999);

        cpu.call_a16();

        assert_eq!(cpu.memory_bus.borrow_mut().read_16bit(0xFFFE), 0xC002);
        assert_eq!(cpu.stack_pointer.read(), 0xFFFC);
        assert_eq!(cpu.program_counter.read(), 0xC999);
    }

    #[test]
    fn instruction_call_flag_a16() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC000);
        cpu.memory_bus.borrow_mut().write_16bit(0xC000, 0xC999);
        cpu.set_flag(CPUFlag::Z, true);

        cpu.call_flag_a16(CPUFlag::Z, true);

        assert_eq!(cpu.memory_bus.borrow_mut().read_16bit(0xFFFE), 0xC002);
        assert_eq!(cpu.stack_pointer.read(), 0xFFFC);
        assert_eq!(cpu.program_counter.read(), 0xC999);
    }

    #[test]
    fn instruction_call_flag_a16_2() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC000);
        cpu.memory_bus.borrow_mut().write_16bit(0xC000, 0xC999);
        cpu.set_flag(CPUFlag::Z, false);

        cpu.call_flag_a16(CPUFlag::Z, true);

        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
        assert_eq!(cpu.program_counter.read(), 0xC002);
    }

    #[test]
    fn instruction_rst() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC023);

        cpu.rst(0x08);

        assert_eq!(cpu.stack_pointer.read(), 0xFFFC);
        assert_eq!(cpu.memory_bus.borrow().read_16bit(0xFFFE), 0xC023);
        assert_eq!(cpu.program_counter.read(), 0x0008);
    }

    #[test]
    fn instruction_jp_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC825);

        cpu.jp_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.program_counter.read(), 0xC825);
    }

    #[test]
    fn instruction_di() {
        let mut cpu = create_cpu();

        cpu.di();

        assert_eq!(cpu.interrupt_disable_delay, 1);
    }

    #[test]
    fn instruction_ei() {
        let mut cpu = create_cpu();

        cpu.ei();

        assert_eq!(cpu.interrupt_enable_delay, 1);
    }

    #[test]
    fn instruction_reti() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xFFFC);
        cpu.memory_bus.borrow_mut().write_16bit(0xFFFC, 0xC024);

        cpu.reti();

        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
        assert_eq!(cpu.program_counter.read(), 0xC024);
        assert_eq!(cpu.interrupt_master_enable, true);
    }
}

