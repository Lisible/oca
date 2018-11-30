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

    stopped: bool,
    halted: bool,
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
            memory_bus,
            stopped: false,
            halted: false
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
            0x01 => cycles += self.ld_bi_register_d16(&BiRegisterIdentifier::BC),
            // LD (BC),A
            0x02 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::BC, &RegisterIdentifier::A),
            // INC BC
            0x03 => cycles += self.inc_bi_register(&BiRegisterIdentifier::BC),
            // INC B
            0x04 => cycles += self.inc_register(&RegisterIdentifier::B),
            // DEC B
            0x05 => cycles += self.dec_register(&RegisterIdentifier::B),
            // LD B,d8
            0x06 => cycles += self.ld_register_d8(&RegisterIdentifier::B),
            // RLCA
            0x07 => cycles += self.rlca(),
            // LD (a16),SP
            0x08 => cycles += self.ld_a16_ptr_sp(),
            // ADD HL, BC
            0x09 => cycles += self.add_bi_register_bi_register(&BiRegisterIdentifier::HL, &BiRegisterIdentifier::BC),
            // LD A,(BC)
            0x0A => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::A, &BiRegisterIdentifier::BC),
            // DEC BC
            0x0B => cycles += self.dec_bi_register(&BiRegisterIdentifier::BC),
            // INC C
            0x0C => cycles += self.inc_register(&RegisterIdentifier::C),
            // DEC C
            0x0D => cycles += self.dec_register(&RegisterIdentifier::C),
            // LD C,d8
            0x0E => cycles += self.ld_register_d8(&RegisterIdentifier::C),
            // RRCA
            0x0F => cycles += self.rrca(),
            // STOP 0
            0x10 => cycles += self.stop_0(),
            // LD DE,d16
            0x11 => cycles += self.ld_bi_register_d16(&BiRegisterIdentifier::DE),
            // LD (DE),A
            0x12 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::DE, &RegisterIdentifier::A),
            // INC DE
            0x13 => cycles += self.inc_bi_register(&BiRegisterIdentifier::DE),
            // INC D
            0x14 => cycles += self.inc_register(&RegisterIdentifier::D),
            // DEC D
            0x15 => cycles += self.dec_register(&RegisterIdentifier::D),
            // LD D,d8
            0x16 => cycles += self.ld_register_d8(&RegisterIdentifier::D),
            // RLA
            0x17 => cycles += self.rla(),
            // JR r8
            0x18 => cycles += self.jr_r8(),
            // ADD HL,DE
            0x19 => cycles += self.add_bi_register_bi_register(&BiRegisterIdentifier::HL, &BiRegisterIdentifier::DE),
            // LD A,(DE)
            0x1A => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::A, &BiRegisterIdentifier::DE),
            // DEC DE
            0x1B => cycles += self.dec_bi_register(&BiRegisterIdentifier::DE),
            // INC E
            0x1C => cycles += self.inc_register(&RegisterIdentifier::E),
            // DEC E
            0x1D => cycles += self.dec_register(&RegisterIdentifier::E),
            // LD E,d8
            0x1E => cycles += self.ld_register_d8(&RegisterIdentifier::E),
            // RRA
            0x1F => cycles += self.rra(),
            // JR NZ,r8
            0x20 => cycles += self.jr_flag_r8(CPUFlag::Z, false),
            // LD HL,d16
            0x21 => cycles += self.ld_bi_register_d16(&BiRegisterIdentifier::HL),
            // LD (HL+),A
            0x22 => cycles += self.ldi_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::A),
            // INC HL
            0x23 => cycles += self.inc_bi_register(&BiRegisterIdentifier::HL),
            // INC H
            0x24 => cycles += self.inc_register(&RegisterIdentifier::H),
            // DEC H
            0x25 => cycles += self.dec_register(&RegisterIdentifier::H),
            // LD H,d8
            0x26 => cycles += self.ld_register_d8(&RegisterIdentifier::H),
            // DAA
            0x27 => cycles += self.daa(),
            // JR Z,r8
            0x28 => cycles += self.jr_flag_r8(CPUFlag::Z, true),
            // ADD HL,HL
            0x29 => cycles += self.add_bi_register_bi_register(&BiRegisterIdentifier::HL, &BiRegisterIdentifier::HL),
            // LD A,(HL+)
            0x2A => cycles += self.ldi_register_bi_register_ptr(&RegisterIdentifier::A, &BiRegisterIdentifier::HL),
            // DEC HL
            0x2B => cycles += self.dec_bi_register(&BiRegisterIdentifier::HL),
            // INC L
            0x2C => cycles += self.inc_register(&RegisterIdentifier::L),
            // DEC L
            0x2D => cycles += self.dec_register(&RegisterIdentifier::L),
            // LD L,d8
            0x2E => cycles += self.ld_register_d8(&RegisterIdentifier::L),
            // CPL
            0x2F => cycles += self.cpl(),
            // JR NC,r8
            0x30 => cycles += self.jr_flag_r8(CPUFlag::C, false),
            // LD SP,d16
            0x31 => cycles += self.ld_sp_d16(),
            // LD (HL-),A
            0x32 => cycles += self.ldd_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::A),
            // INC SP
            0x33 => cycles += self.inc_sp(),
            // INC (HL)
            0x34 => cycles += self.inc_bi_register_ptr(&BiRegisterIdentifier::HL),
            // DEC (HL)
            0x35 => cycles += self.dec_bi_register_ptr(&BiRegisterIdentifier::HL),
            // LD (HL),d8
            0x36 => cycles += self.ld_bi_register_ptr_d8(&BiRegisterIdentifier::HL),
            // SCF
            0x37 => cycles += self.scf(),
            // JR C,r8
            0x38 => cycles += self.jr_flag_r8(CPUFlag::C, true),
            // ADD HL,SP
            0x39 => cycles += self.add_bi_register_sp(&BiRegisterIdentifier::HL),
            // LD A,(HL-)
            0x3A => cycles += self.ldd_register_bi_register_ptr(&RegisterIdentifier::A, &BiRegisterIdentifier::HL),
            // DEC SP
            0x3B => cycles += self.dec_sp(),
            // INC A
            0x3C => cycles += self.inc_register(&RegisterIdentifier::A),
            // DEC A
            0x3D => cycles += self.dec_register(&RegisterIdentifier::A),
            // LD A,d8
            0x3E => cycles += self.ld_register_d8(&RegisterIdentifier::A),
            // CCF
            0x3F => cycles += self.ccf(),
            // LD B,B
			0x40 => cycles += self.ld_register_register(&RegisterIdentifier::B, &RegisterIdentifier::B),
            // LD B,C
            0x41 => cycles += self.ld_register_register(&RegisterIdentifier::B, &RegisterIdentifier::C),
            // LD B,D
            0x42 => cycles += self.ld_register_register(&RegisterIdentifier::B, &RegisterIdentifier::D),
            // LD B,E
            0x43 => cycles += self.ld_register_register(&RegisterIdentifier::B, &RegisterIdentifier::E),
            // LD B,H
            0x44 => cycles += self.ld_register_register(&RegisterIdentifier::B, &RegisterIdentifier::H),
            // LD B,L
            0x45 => cycles += self.ld_register_register(&RegisterIdentifier::B, &RegisterIdentifier::L),
            // LD B,(HL)
            0x46 => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::B, &BiRegisterIdentifier::HL),
            // LD B,A
            0x47 => cycles += self.ld_register_register(&RegisterIdentifier::B, &RegisterIdentifier::A),
            // LD C,B
            0x48 => cycles += self.ld_register_register(&RegisterIdentifier::C, &RegisterIdentifier::B),
            // LD C,C
            0x49 => cycles += self.ld_register_register(&RegisterIdentifier::C, &RegisterIdentifier::C),
            // LD C,D
        	0x4A => cycles += self.ld_register_register(&RegisterIdentifier::C, &RegisterIdentifier::D),
            // LD C,E
        	0x4B => cycles += self.ld_register_register(&RegisterIdentifier::C, &RegisterIdentifier::E),
            // LD C,H
        	0x4C => cycles += self.ld_register_register(&RegisterIdentifier::C, &RegisterIdentifier::H),
            // LD C,L
        	0x4D => cycles += self.ld_register_register(&RegisterIdentifier::C, &RegisterIdentifier::L),
            // LD C,(HL)
        	0x4E => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::C, &BiRegisterIdentifier::HL),
            // LD C,A
        	0x4F => cycles += self.ld_register_register(&RegisterIdentifier::C, &RegisterIdentifier::A),
            // LD D,B
        	0x50 => cycles += self.ld_register_register(&RegisterIdentifier::D, &RegisterIdentifier::B),
            // LD D,C
        	0x51 => cycles += self.ld_register_register(&RegisterIdentifier::D, &RegisterIdentifier::C),
            // LD D,D
        	0x52 => cycles += self.ld_register_register(&RegisterIdentifier::D, &RegisterIdentifier::D),
            // LD D,E
        	0x53 => cycles += self.ld_register_register(&RegisterIdentifier::D, &RegisterIdentifier::E),
            // LD D,H
        	0x54 => cycles += self.ld_register_register(&RegisterIdentifier::D, &RegisterIdentifier::H),
            // LD D,L
        	0x55 => cycles += self.ld_register_register(&RegisterIdentifier::D, &RegisterIdentifier::L),
            // LD D,(HL)
        	0x56 => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::D, &BiRegisterIdentifier::HL),
            // LD D,A
        	0x57 => cycles += self.ld_register_register(&RegisterIdentifier::D, &RegisterIdentifier::A),
            // LD E,B
        	0x58 => cycles += self.ld_register_register(&RegisterIdentifier::E, &RegisterIdentifier::B),
            // LD E,C
        	0x59 => cycles += self.ld_register_register(&RegisterIdentifier::E, &RegisterIdentifier::C),
            // LD E,D
        	0x5A => cycles += self.ld_register_register(&RegisterIdentifier::E, &RegisterIdentifier::D),
            // LD E,E
        	0x5B => cycles += self.ld_register_register(&RegisterIdentifier::E, &RegisterIdentifier::E),
            // LD E,H
        	0x5C => cycles += self.ld_register_register(&RegisterIdentifier::E, &RegisterIdentifier::H),
            // LD E,L
        	0x5D => cycles += self.ld_register_register(&RegisterIdentifier::E, &RegisterIdentifier::L),
            // LD E,(HL)
        	0x5E => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::E, &BiRegisterIdentifier::HL),
            // LD E,A
        	0x5F => cycles += self.ld_register_register(&RegisterIdentifier::E, &RegisterIdentifier::A),
            // LD H,B
        	0x60 => cycles += self.ld_register_register(&RegisterIdentifier::H, &RegisterIdentifier::B),
            // LD H,C
        	0x61 => cycles += self.ld_register_register(&RegisterIdentifier::H, &RegisterIdentifier::C),
            // LD H,D
        	0x62 => cycles += self.ld_register_register(&RegisterIdentifier::H, &RegisterIdentifier::D),
            // LD H,E
        	0x63 => cycles += self.ld_register_register(&RegisterIdentifier::H, &RegisterIdentifier::E),
            // LD H,H
        	0x64 => cycles += self.ld_register_register(&RegisterIdentifier::H, &RegisterIdentifier::H),
            // LD H,L
        	0x65 => cycles += self.ld_register_register(&RegisterIdentifier::H, &RegisterIdentifier::L),
            // LD H,(HL)
        	0x66 => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::H, &BiRegisterIdentifier::HL),
            // LD H,A
        	0x67 => cycles += self.ld_register_register(&RegisterIdentifier::H, &RegisterIdentifier::A),
            // LD L,B
        	0x68 => cycles += self.ld_register_register(&RegisterIdentifier::L, &RegisterIdentifier::B),
            // LD L,C
        	0x69 => cycles += self.ld_register_register(&RegisterIdentifier::L, &RegisterIdentifier::C),
            // LD L,D
        	0x6A => cycles += self.ld_register_register(&RegisterIdentifier::L, &RegisterIdentifier::D),
            // LD L,E
        	0x6B => cycles += self.ld_register_register(&RegisterIdentifier::L, &RegisterIdentifier::E),
            // LD L,H
        	0x6C => cycles += self.ld_register_register(&RegisterIdentifier::L, &RegisterIdentifier::H),
            // LD L,L
        	0x6D => cycles += self.ld_register_register(&RegisterIdentifier::L, &RegisterIdentifier::L),
            // LD L,(HL)
        	0x6E => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::L, &BiRegisterIdentifier::HL),
            // LD L,A
        	0x6F => cycles += self.ld_register_register(&RegisterIdentifier::L, &RegisterIdentifier::A),
            // LD (HL),B
        	0x70 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::B),
            // LD (HL),C
        	0x71 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::C),
            // LD (HL),D
        	0x72 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::D),
            // LD (HL),E
        	0x73 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::E),
            // LD (HL),H
        	0x74 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::H),
            // LD (HL),L
        	0x75 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::L),
            // HALT
        	0x76 => cycles += self.halt(),
            // LD (HL),A
        	0x77 => cycles += self.ld_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::A),
            // LD A,B
        	0x78 => cycles += self.ld_register_register(&RegisterIdentifier::A, &RegisterIdentifier::B),
            // LD A,C
        	0x79 => cycles += self.ld_register_register(&RegisterIdentifier::A, &RegisterIdentifier::C),
            // LD A,D
        	0x7A => cycles += self.ld_register_register(&RegisterIdentifier::A, &RegisterIdentifier::D),
            // LD A,E
        	0x7B => cycles += self.ld_register_register(&RegisterIdentifier::A, &RegisterIdentifier::E),
            // LD A,H
        	0x7C => cycles += self.ld_register_register(&RegisterIdentifier::A, &RegisterIdentifier::H),
            // LD A,L
        	0x7D => cycles += self.ld_register_register(&RegisterIdentifier::A, &RegisterIdentifier::L),
            // LD A,(HL)
        	0x7E => cycles += self.ld_register_bi_register_ptr(&RegisterIdentifier::A, &BiRegisterIdentifier::HL),
            // LD A,A
        	0x7F => cycles += self.ld_register_register(&RegisterIdentifier::A, &RegisterIdentifier::A),
            // ADD A,B
			0x80 => cycles += self.add_register(&RegisterIdentifier::B),
            // ADD A,C
            0x81 => cycles += self.add_register(&RegisterIdentifier::C),
            // ADD A,D
            0x82 => cycles += self.add_register(&RegisterIdentifier::D),
            // ADD A,E
            0x83 => cycles += self.add_register(&RegisterIdentifier::E),
            // ADD A,H
            0x84 => cycles += self.add_register(&RegisterIdentifier::H),
            // ADD A,L
            0x85 => cycles += self.add_register(&RegisterIdentifier::L),
            // ADD A,(HL)
            0x86 => cycles += self.add_bi_register_ptr(&BiRegisterIdentifier::HL),
            // ADD A,A
            0x87 => cycles += self.add_register(&RegisterIdentifier::A),
            // ADC A,B
			0x88 => cycles += self.adc_register(&RegisterIdentifier::B),
            // ADC A,C
            0x89 => cycles += self.adc_register(&RegisterIdentifier::C),
            // ADC A,D
            0x8A => cycles += self.adc_register(&RegisterIdentifier::D),
            // ADC A,E
            0x8B => cycles += self.adc_register(&RegisterIdentifier::E),
            // ADC A,H
            0x8C => cycles += self.adc_register(&RegisterIdentifier::H),
            // ADC A,L
            0x8D => cycles += self.adc_register(&RegisterIdentifier::L),
            // ADC A,(HL)
            0x8E => cycles += self.adc_bi_register_ptr(&BiRegisterIdentifier::HL),
            // ADC A,A
            0x8F => cycles += self.adc_register(&RegisterIdentifier::A),
            // SUB B
			/*0x90 => cycles += self.sub_register(&RegisterIdentifier::B),
            // SUB C
            0x91 => cycles += self.sub_register(&RegisterIdentifier::C),
            // SUB D
            0x92 => cycles += self.sub_register(&RegisterIdentifier::D),
            // SUB E
            0x93 => cycles += self.sub_register(&RegisterIdentifier::E),
            // SUB H
            0x94 => cycles += self.sub_register(&RegisterIdentifier::H),
            // SUB L
            0x95 => cycles += self.sub_register(&RegisterIdentifier::L),
            // SUB (HL)
            0x96 => cycles += self.sub_bi_register_ptr(&BiRegisterIdentifier::HL),
            // SUB A
			0x97 => cycles += self.sub_register(&RegisterIdentifier::A),
            // SBC A,B
			0x98 => cycles += self.sbc_register_register(&RegisterIdentifier::A, &RegisterIdentifier::B),
            // SBC A,C
			0x99 => cycles += self.sbc_register_register(&RegisterIdentifier::A, &RegisterIdentifier::C),
            // SBC A,D
			0x9A => cycles += self.sbc_register_register(&RegisterIdentifier::A, &RegisterIdentifier::D),
            // SBC A,E
			0x9B => cycles += self.sbc_register_register(&RegisterIdentifier::A, &RegisterIdentifier::E),
            // SBC A,H
			0x9C => cycles += self.sbc_register_register(&RegisterIdentifier::A, &RegisterIdentifier::H),
            // SBC A,L
			0x9D => cycles += self.sbc_register_register(&RegisterIdentifier::A, &RegisterIdentifier::L),
            // SBC A,(
			0x9E => cycles += self.sbc_register_register(&RegisterIdentifier::A, &BiRegisterIdentifier::HL),
            // SBC A,A
			0x9F => cycles += self.sbc_register_register(&RegisterIdentifier::A, &RegisterIdentifier::A),
            // AND B
			0xA0 => cycles += self.and_register(&RegisterIdentifier::B),
            // AND C
            0xA1 => cycles += self.and_register(&RegisterIdentifier::C),
            // AND D
            0xA2 => cycles += self.and_register(&RegisterIdentifier::D),
            // AND E
            0xA3 => cycles += self.and_register(&RegisterIdentifier::E),
            // AND H
            0xA4 => cycles += self.and_register(&RegisterIdentifier::H),
            // AND L
            0xA5 => cycles += self.and_register(&RegisterIdentifier::L),
            // AND (HL)
            0xA6 => cycles += self.and_bi_register_ptr(&BiRegisterIdentifier::HL),
            // AND A
            0xA7 => cycles += self.and_register(&RegisterIdentifier::A),
            // XOR B
            0xA8 => cycles += self.xor_register(&RegisterIdentifier::B),
            // XOR C
            0xA9 => cycles += self.xor_register(&RegisterIdentifier::C),
            // XOR D
            0xAA => cycles += self.xor_register(&RegisterIdentifier::D),
            // XOR E
            0xAB => cycles += self.xor_register(&RegisterIdentifier::E),
            // XOR H
            0xAC => cycles += self.xor_register(&RegisterIdentifier::H),
            // XOR L
            0xAD => cycles += self.xor_register(&RegisterIdentifier::L),
            // XOR (HL)
            0xAE => cycles += self.xor_bi_register_ptr(&BiRegisterIdentifier::HL),
            // XOR A
            0xAF => cycles += self.xor_register(&RegisterIdentifier::A),
            // OR B
            0xB0 => cycles += self.or_register(&RegisterIdentifier::B),
            // OR C
            0xB1 => cycles += self.or_register(&RegisterIdentifier::C),
            // OR D
            0xB2 => cycles += self.or_register(&RegisterIdentifier::D),
            // OR E
            0xB3 => cycles += self.or_register(&RegisterIdentifier::E),
            // OR H
            0xB4 => cycles += self.or_register(&RegisterIdentifier::H),
            // OR L
            0xB5 => cycles += self.or_register(&RegisterIdentifier::L),
            // OR (HL)
            0xB6 => cycles += self.or_bi_register_ptr(&BiRegisterIdentifier::HL),
            // OR A
            0xB7 => cycles += self.or_register(&RegisterIdentifier::A),
            // CP B
            0xB8 => cycles += self.cp_register(&RegisterIdentifier::B),
            // CP C
            0xB9 => cycles += self.cp_register(&RegisterIdentifier::C),
            // CP D
            0xBA => cycles += self.cp_register(&RegisterIdentifier::D),
            // CP E
            0xBB => cycles += self.cp_register(&RegisterIdentifier::E),
            // CP H
            0xBC => cycles += self.cp_register(&RegisterIdentifier::H),
            // CP L
            0xBD => cycles += self.cp_register(&RegisterIdentifier::L),
            // CP (HL)
            0xBE => cycles += self.cp_bi_register_ptr(&BiRegisterIdentifier::HL),
            // CP A
            0xBF => cycles += self.cp_register(&RegisterIdentifier::A),*/
            _ => panic!("Unimplemented instruction")
        }
    }



    fn nop(&mut self) -> u32 {
        4
    }

    fn ld_bi_register_d16(&mut self, register_identifier: &BiRegisterIdentifier) -> u32 {
        let value = self.memory_bus.borrow().read_16bit(self.program_counter.read() as usize);
        self.program_counter.increment(2);

        self.write_bi_register(register_identifier, value);
        12
    }

    fn ld_sp_d16(&mut self) -> u32 {
        let value = self.memory_bus.borrow().read_16bit(self.program_counter.read() as usize);
        self.program_counter.increment(2);
        self.stack_pointer.write(value);
        12
    }

    fn ld_bi_register_register(&mut self,
                               bi_register_identifier: &BiRegisterIdentifier,
                               register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        self.write_bi_register(bi_register_identifier, value as u16);
        8
    }

    fn inc_bi_register(&mut self, register_identifier: &BiRegisterIdentifier) -> u32 {
        self.bi_registers.get_mut(&register_identifier).unwrap().increment(1);
        8
    }

    fn dec_bi_register(&mut self, register_identifier: &BiRegisterIdentifier) -> u32 {
        self.bi_registers.get_mut(&register_identifier).unwrap().decrement(1);
        8
    }

    fn inc_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let lhs = self.read_register(register_identifier);
        let rhs = 1;
        let sum = lhs.wrapping_add(rhs);

        if sum == 0 {
            self.set_flag(CPUFlag::Z);
        } else {
            self.unset_flag(CPUFlag::Z);
        }

        if (sum & 0xF) < (lhs & 0xF) {
            self.set_flag(CPUFlag::H);
        } else {
            self.unset_flag(CPUFlag::H);
        }

        self.write_register(register_identifier, sum);
        self.unset_flag(CPUFlag::N);
        4
    }

    fn dec_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let lhs = self.read_register(register_identifier);
        let rhs = 1;
        let difference = lhs.wrapping_sub(rhs);

        if difference == 0 {
            self.set_flag(CPUFlag::Z);
        } else {
            self.unset_flag(CPUFlag::Z);
        }

        if (difference & 0xF) <= (lhs & 0xF) {
            self.set_flag(CPUFlag::H);
        } else {
            self.unset_flag(CPUFlag::H);
        }

        self.write_register(register_identifier, difference);
        self.set_flag(CPUFlag::N);
        4
    }

    fn ld_register_d8(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.memory_bus.borrow().read_8bit(self.program_counter.read() as usize);
        self.program_counter.increment(1);

        self.write_register(register_identifier, value);
        8
    }

    fn rlca(&mut self) -> u32 {
        self.unset_flag(CPUFlag::Z);
        self.unset_flag(CPUFlag::N);
        self.unset_flag(CPUFlag::H);
        self.unset_flag(CPUFlag::C);



        let mut value = 0;
        {
            let register_a = self.registers.get(&RegisterIdentifier::A).unwrap().borrow();
            value = register_a.read();
        }


        {
            if (value & 0x80) != 0 {
                self.set_flag(CPUFlag::C);
            } else {
                self.unset_flag(CPUFlag::C);
            }
        }

        let mut register_a = self.registers.get_mut(&RegisterIdentifier::A).unwrap().borrow_mut();
        let new_value = (value << 1) | ((value & 0x80) >> 7);
        register_a.write(new_value);


        4
    }



    fn rla(&mut self) -> u32 {
        let carry_flag = self.get_flag(CPUFlag::C) as u8;

        let mut value;

        {
            let register_a = self.registers.get(&RegisterIdentifier::A).unwrap().borrow();
            value = register_a.read();
        }


        {
            if (value & 0x80) != 0 {
                self.set_flag(CPUFlag::C);
            }
            else {
                self.unset_flag(CPUFlag::C);
            }
        }

        {
            let mut register_a = self.registers.get(&RegisterIdentifier::A).unwrap().borrow_mut();
            let new_value = ((value << 1) & 0xFF) | carry_flag;
            register_a.write(new_value);
        }

        self.unset_flag(CPUFlag::Z);
        self.unset_flag(CPUFlag::N);
        self.unset_flag(CPUFlag::H);

        4
    }

    fn rrca(&mut self) -> u32 {
        self.unset_flag(CPUFlag::Z);
        self.unset_flag(CPUFlag::N);
        self.unset_flag(CPUFlag::H);
        self.unset_flag(CPUFlag::C);

        let mut value = 0;

        {
            let register_a = self.registers.get(&RegisterIdentifier::A).unwrap().borrow();
            value = register_a.read();
        }


        {
            if (value & 0x01) != 0 {
                self.set_flag(CPUFlag::C);
            } else {
                self.unset_flag(CPUFlag::C);
            }
        }

        let mut register_a = self.registers.get_mut(&RegisterIdentifier::A).unwrap().borrow_mut();
        let mut new_value = (value >> 1);
        if (value & 0x01) != 0 {
            new_value = new_value | 0x80;
        }

        register_a.write(new_value);
        4
    }

    fn rra(&mut self) -> u32 {
        let carry_flag = self.get_flag(CPUFlag::C) as u8;

        let mut value;

        {
            let register_a = self.registers.get(&RegisterIdentifier::A).unwrap().borrow();
            value = register_a.read();
        }


        {
            if (value & 0x01) != 0 {
                self.set_flag(CPUFlag::C);
            }
            else {
                self.unset_flag(CPUFlag::C);
            }
        }

        {
            let mut register_a = self.registers.get(&RegisterIdentifier::A).unwrap().borrow_mut();
            let new_value = ((value >> 1) & 0xFF) | (carry_flag << 7);
            register_a.write(new_value);
        }

        self.unset_flag(CPUFlag::Z);
        self.unset_flag(CPUFlag::N);
        self.unset_flag(CPUFlag::H);

        4
    }

    fn ld_a16_sp(&mut self) -> u32 {
        let value = self.memory_bus.borrow().read_16bit(self.program_counter.read() as usize);
        self.program_counter.increment(2);
        self.stack_pointer.write(value);

        20
    }

    fn ld_a16_ptr_sp(&mut self) -> u32 {
        let mut address = self.memory_bus.borrow().read_16bit(self.program_counter.read() as usize);
        self.program_counter.increment(2);
        let value = self.stack_pointer.read();
        self.memory_bus.borrow_mut().write_16bit(address as usize, value);
        20
    }

    fn add_bi_register_bi_register(&mut self,
                                   first_register_identifier: &BiRegisterIdentifier,
                                   second_register_identifier: &BiRegisterIdentifier) -> u32 {
        use std::u32;

        let lhs = self.read_bi_register(first_register_identifier);
        let rhs = self.read_bi_register(second_register_identifier);
        let sum = (lhs as u32).wrapping_add(rhs as u32);

        if ((sum & 0x0FFF) as u16) < (lhs & 0x0FFF) {
            self.set_flag(CPUFlag::H);
        } else {
            self.unset_flag(CPUFlag::H);
        }

        if sum > 0xFFFF {
            self.set_flag(CPUFlag::C);
        } else {
            self.unset_flag(CPUFlag::C);
        }

        self.write_bi_register(first_register_identifier, lhs.wrapping_add(rhs));
        self.unset_flag(CPUFlag::N);

        8
    }



    fn add_bi_register_sp(&mut self,
                          bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        use std::u32;

        let lhs = self.read_bi_register(bi_register_identifier);
        let rhs = self.stack_pointer.read();
        let sum = (lhs as u32).wrapping_add(rhs as u32);

        if ((sum & 0x0FFF) as u16) < (lhs & 0x0FFF) {
            self.set_flag(CPUFlag::H);
        }  else {
            self.unset_flag(CPUFlag::H);
        }

        if sum > 0xFFFF {
            self.set_flag(CPUFlag::C);
        }  else {
            self.unset_flag(CPUFlag::C);
        }

        self.write_bi_register(bi_register_identifier, lhs.wrapping_add(rhs));
        self.unset_flag(CPUFlag::N);

        8
    }

    fn ld_register_bi_register_ptr(&mut self,
                                   register_identifier: &RegisterIdentifier,
                                   bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        self.write_register(register_identifier, value);

        8
    }

    fn stop_0(&mut self) -> u32 {
        self.program_counter.increment(1);
        self.stopped = true;
        4
    }

    fn jr_r8(&mut self) -> u32 {
        let value = self.memory_bus.borrow().read_8bit_signed(self.program_counter.read() as usize);
        self.program_counter.increment(1);

        let pc = self.program_counter.read();
        self.program_counter.write((pc as i32 + value as i32) as u16);
        12
    }

    fn ld_bi_register_ptr_register(&mut self,
                                   bi_register_identifier: &BiRegisterIdentifier,
                                   register_identifier: &RegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier) as usize;
        let value = self.read_register(register_identifier);

        self.memory_bus.borrow_mut().write_8bit(address, value);
        8
    }

    fn jr_flag_r8(&mut self, flag: CPUFlag, jump_if_true: bool) -> u32 {
        let value = self.memory_bus.borrow().read_8bit_signed(self.program_counter.read() as usize);
        self.program_counter.increment(1);

        let pc = self.program_counter.read();

        if self.get_flag(flag) == jump_if_true {
            self.program_counter.write((pc as i32 + value as i32) as u16);
        }

        8
    }

    fn ldi_bi_register_ptr_register(&mut self,
                                    bi_register_identifier: &BiRegisterIdentifier,
                                    register_identifier: &RegisterIdentifier) -> u32 {
        self.ld_bi_register_ptr_register(bi_register_identifier, register_identifier);
        self.inc_bi_register(bi_register_identifier);

        8
    }

    fn daa(&mut self) -> u32 {
        use std::u8;

        let mut register_a = self.read_register(&RegisterIdentifier::A);

        if !self.get_flag(CPUFlag::N) {
            if self.get_flag(CPUFlag::C) || register_a > 0x99 {
                register_a = register_a.wrapping_add(0x60);
                self.set_flag(CPUFlag::C);
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
        if register_a == 0 {
            self.set_flag(CPUFlag::Z);
        } else {
            self.unset_flag(CPUFlag::Z);
        }

        self.unset_flag(CPUFlag::H);

        4
    }

    fn cpl(&mut self) -> u32 {
        let na = !self.read_register(&RegisterIdentifier::A);
        self.write_register(&RegisterIdentifier::A, na);
        4
    }

    fn ldi_register_bi_register_ptr(&mut self,
                                    register_identifier: &RegisterIdentifier,
                                    bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        self.ld_register_bi_register_ptr(register_identifier, bi_register_identifier);
        self.inc_bi_register(bi_register_identifier);
        8
    }

    fn ldd_bi_register_ptr_register(&mut self,
                                    bi_register_identifier: &BiRegisterIdentifier,
                                    register_identifier: &RegisterIdentifier) -> u32 {
        self.ld_bi_register_ptr_register(bi_register_identifier, register_identifier);
        self.dec_bi_register(bi_register_identifier);
        8
    }

    fn ldd_register_bi_register_ptr(&mut self,
                                    register_identifier: &RegisterIdentifier,
                                    bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        self.ld_register_bi_register_ptr(register_identifier, bi_register_identifier);
        self.dec_bi_register(bi_register_identifier);
        8
    }

    fn inc_sp(&mut self) -> u32 {
        self.stack_pointer.increment(1);
        8
    }

    fn inc_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let lhs = self.memory_bus.borrow().read_8bit(address as usize);
        let rhs = 1;
        let sum = lhs.wrapping_add(rhs);

        if sum == 0 {
            self.set_flag(CPUFlag::Z);
        }  else {
            self.unset_flag(CPUFlag::Z);
        }

        if (sum & 0x0F) < (lhs & 0x0F) {
            self.set_flag(CPUFlag::H);
        }  else {
            self.unset_flag(CPUFlag::H);
        }

        self.memory_bus.borrow_mut().write_8bit(address as usize, sum);
        self.unset_flag(CPUFlag::N);
        12
    }

    fn dec_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let lhs = self.memory_bus.borrow().read_8bit(address as usize);
        let rhs = 1;
        let difference = lhs.wrapping_sub(rhs);

        if difference == 0 {
            self.set_flag(CPUFlag::Z);
        }

        if (difference & 0x0F) <= (lhs & 0x0F) {
            self.set_flag(CPUFlag::H);
        }

        self.memory_bus.borrow_mut().write_8bit(address as usize, difference);
        self.set_flag(CPUFlag::N);
        12
    }

    fn ld_bi_register_ptr_d8(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let value = self.memory_bus.borrow().read_8bit(self.program_counter.read() as usize);
        self.program_counter.increment(1);
        let address = self.read_bi_register(bi_register_identifier);
        self.memory_bus.borrow_mut().write_8bit(address as usize, value);
        12
    }

    fn dec_sp(&mut self) -> u32 {
        self.stack_pointer.decrement(1);
        8
    }

    fn scf(&mut self) -> u32 {
        self.set_flag(CPUFlag::C);
        self.unset_flag(CPUFlag::N);
        self.unset_flag(CPUFlag::H);
        4
    }

    fn ccf(&mut self) -> u32 {
        if self.get_flag(CPUFlag::C) {
            self.unset_flag(CPUFlag::C);
        }
        else {
            self.set_flag(CPUFlag::C);
        }
        self.unset_flag(CPUFlag::N);
        self.unset_flag(CPUFlag::H);
        4
    }

    fn ld_register_register(&mut self,
                            first_register_identifier: &RegisterIdentifier,
                            second_register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(second_register_identifier);
        self.write_register(first_register_identifier, value);
        4
    }

    fn halt(&mut self) -> u32 {
        self.halted = true;
        1
    }

    fn add_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = self.read_register(register_identifier);
        let sum = (lhs as u16).wrapping_add(rhs as u16);

        if sum as u8 == 0 {
            self.set_flag(CPUFlag::Z);
        } else {
            self.unset_flag(CPUFlag::Z);
        }

        if (sum as u8 & 0x0F) < (lhs & 0x0F) {
            self.set_flag(CPUFlag::H);
        } else {
            self.unset_flag(CPUFlag::H);
        }

        if sum > 0xFF {
            self.set_flag(CPUFlag::C);
        } else {
            self.unset_flag(CPUFlag::C);
        }

        self.write_register(&RegisterIdentifier::A, sum as u8);

        self.unset_flag(CPUFlag::N);
        4
    }

    fn add_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let address = self.read_bi_register(bi_register_identifier);
        let rhs = self.memory_bus.borrow().read_8bit(address as usize);
        let sum : u16 = (lhs as u16).wrapping_add(rhs as u16);

        if sum as u8 == 0 {
            self.set_flag(CPUFlag::Z);
        } else {
            self.unset_flag(CPUFlag::Z);
        }

        if (sum as u8 & 0x0F) < (lhs & 0x0F) {
            self.set_flag(CPUFlag::H);
        } else {
            self.unset_flag(CPUFlag::H);
        }

        if sum > 0xFF {
            self.set_flag(CPUFlag::C);
        } else {
            self.unset_flag(CPUFlag::C);
        }

        self.write_register(&RegisterIdentifier::A, sum as u8);

        self.unset_flag(CPUFlag::N);
        8
    }

    fn adc_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs;
        if self.get_flag(CPUFlag::C) {
            rhs = self.read_register(register_identifier).wrapping_add(1);
        }
        else {
            rhs = self.read_register(register_identifier);
        }

        let sum = (lhs as u16).wrapping_add(rhs as u16);

        if sum as u8 == 0 {
            self.set_flag(CPUFlag::Z);
        } else {
            self.unset_flag(CPUFlag::Z);
        }

        if (sum as u8 & 0x0F) < (lhs & 0x0F) {
            self.set_flag(CPUFlag::H);
        } else {
            self.unset_flag(CPUFlag::H);
        }

        if sum > 0xFF {
            self.set_flag(CPUFlag::C);
        } else {
            self.unset_flag(CPUFlag::C);
        }

        self.write_register(&RegisterIdentifier::A, sum as u8);

        self.unset_flag(CPUFlag::N);
        4
    }

    fn adc_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let address = self.read_bi_register(bi_register_identifier);
        let rhs;
        if self.get_flag(CPUFlag::C) {
            rhs = self.memory_bus.borrow().read_8bit(address as usize).wrapping_add(1);
        }
        else {
            rhs = self.memory_bus.borrow().read_8bit(address as usize);
        }

        let sum = (lhs as u16).wrapping_add(rhs as u16);

        if sum as u8 == 0 {
            self.set_flag(CPUFlag::Z);
        } else {
            self.unset_flag(CPUFlag::Z);
        }

        if (sum as u8 & 0x0F) < (lhs & 0x0F) {
            self.set_flag(CPUFlag::H);
        } else {
            self.unset_flag(CPUFlag::H);
        }

        if sum > 0xFF {
            self.set_flag(CPUFlag::C);
        } else {
            self.unset_flag(CPUFlag::C);
        }

        self.write_register(&RegisterIdentifier::A, sum as u8);

        self.unset_flag(CPUFlag::N);
        4
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

#[cfg(test)]
mod test {

    use super::*;
    use gb::memory::cartridge::*;
    use gb::memory::ram::*;

    fn create_cpu() -> CPU {
        let cartridge = Rc::new(RefCell::new(Cartridge::from_bytes([0; 0x8000])));
        let ram = Rc::new(RefCell::new(Ram::new()));
        let memory_bus = Rc::new(RefCell::new(MemoryBus::new(cartridge.clone(), ram.clone())));
        CPU::new(memory_bus.clone())
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

    #[test]
    fn instruction_rla_carry0() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rla();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10100010);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }

    #[test]
    fn instruction_rlca() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C);
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rlca();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10100011);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }

    #[test]
    fn instruction_rlca_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C);
        cpu.write_register(&RegisterIdentifier::A, 0b01010001);
        cpu.rlca();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10100010);
        assert_eq!(cpu.get_flag(CPUFlag::C), false)
    }

    #[test]
    fn instruction_rla_carry1() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C);
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rla();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10100011);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }

    #[test]
    fn instruction_rrca() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C);
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rrca();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b11101000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }

    #[test]
    fn instruction_rrca_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C);
        cpu.write_register(&RegisterIdentifier::A, 0b11010000);
        cpu.rrca();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01101000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false)
    }

    #[test]
    fn instruction_rra_carry0() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b11010001);
        cpu.rra();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01101000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }

    #[test]
    fn instruction_rra_carry1() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C);
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
        assert_eq!(pc, 0xC008);
    }

    #[test]
    fn instruction_ld_bi_register_d16() {
        let mut cpu = create_cpu();
        cpu.memory_bus.borrow_mut().write_16bit(0xC002, 0x3412);
        cpu.program_counter.write(0xC002);

        cpu.ld_bi_register_d16(&BiRegisterIdentifier::BC);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0x3412);
    }

    #[test]
    fn instruction_ld_sp_d16() {
        let mut cpu = create_cpu();
        cpu.memory_bus.borrow_mut().write_16bit(0xC002, 0x3412);
        cpu.program_counter.write(0xC002);

        cpu.ld_sp_d16();
        assert_eq!(cpu.stack_pointer.read(), 0x3412);
    }

    #[test]
    fn instruction_ld_bi_register_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::E, 0x35);

        cpu.ld_bi_register_register(&BiRegisterIdentifier::BC, &RegisterIdentifier::E);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0x0035);
    }

    #[test]
    fn instruction_inc_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x754F);

        cpu.inc_bi_register(&BiRegisterIdentifier::BC);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0x7550);
    }

    #[test]
    fn instruction_dec_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x754F);

        cpu.dec_bi_register(&BiRegisterIdentifier::BC);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0x754E);
    }

    #[test]
    fn instruction_inc_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::B, 0x75);

        cpu.inc_register(&RegisterIdentifier::B);
        assert_eq!(cpu.read_register(&RegisterIdentifier::B), 0x76);
    }

    #[test]
    fn instruction_dec_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::B, 0x75);

        cpu.dec_register(&RegisterIdentifier::B);
        assert_eq!(cpu.read_register(&RegisterIdentifier::B), 0x74);
    }

    #[test]
    fn instruction_ld_register_d8() {
        let mut cpu = create_cpu();
        cpu.memory_bus.borrow_mut().write_8bit(0xC002, 0x5A);
        cpu.program_counter.write(0xC002);

        cpu.ld_register_d8(&RegisterIdentifier::B);
        assert_eq!(cpu.read_register(&RegisterIdentifier::B), 0x5A);
    }

    #[test]
    fn instruction_ld_a16_sp() {
        let mut cpu = create_cpu();
        cpu.memory_bus.borrow_mut().write_16bit(0xC002, 0x5AFD);
        cpu.program_counter.write(0xC002);

        cpu.ld_a16_sp();
        assert_eq!(cpu.stack_pointer.read(), 0x5AFD)
    }

    #[test]
    fn instruction_add_bi_register_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x05FB);
        cpu.write_bi_register(&BiRegisterIdentifier::DE, 0x0221);

        cpu.add_bi_register_bi_register(&BiRegisterIdentifier::BC, &BiRegisterIdentifier::DE);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0x081C);
    }

    #[test]
    fn instruction_stop_0() {
        let mut cpu = create_cpu();

        cpu.stop_0();
        assert_eq!(cpu.stopped, true);
    }

    #[test]
    fn instruction_ld_bi_register_ptr_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0x53);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0xC023);

        cpu.ld_bi_register_ptr_register(&BiRegisterIdentifier::BC, &RegisterIdentifier::A);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0x53);
    }

    #[test]
    fn instruction_ld_a16_ptr_sp() {
        let mut cpu = create_cpu();
        cpu.memory_bus.borrow_mut().write_16bit(0xC023, 0xC054);
        cpu.stack_pointer.write(0x25);
        cpu.program_counter.write(0xC023);

        cpu.ld_a16_ptr_sp();


        let value = cpu.memory_bus.borrow().read_16bit(0xC054);
        assert_eq!(value, 0x25);
    }

    #[test]
    fn instruction_ld_register_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0xC035);
        cpu.memory_bus.borrow_mut().write_8bit(0xC035, 0x28);

        cpu.ld_register_bi_register_ptr(&RegisterIdentifier::A, &BiRegisterIdentifier::BC);

        let value = cpu.read_register(&RegisterIdentifier::A);
        assert_eq!(value, 0x28);
    }

    #[test]
    fn instruction_jr_flag_r8() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z);
        cpu.memory_bus.borrow_mut().write_16bit(0xC035, 0x2);
        cpu.program_counter.write(0xC035);

        cpu.jr_flag_r8(CPUFlag::Z, true);

        let current_address = cpu.program_counter.read();
        assert_eq!(current_address, 0xC038);
    }

    #[test]
    fn instruction_ldi_bi_register_ptr_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0x8);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC321);

        cpu.ldi_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::A);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC321), 0x8);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::HL), 0xC322);
    }

    #[test]
    fn instruction_daa() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0xC0);

        cpu.daa();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x20);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }

    #[test]
    fn instruction_daa_2() {
        use std::u8;

        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C);
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
    fn instruction_ldi_register_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC020);
        cpu.memory_bus.borrow_mut().write_8bit(0xC020, 0x23);

        cpu.ldi_register_bi_register_ptr(&RegisterIdentifier::A, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x23);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::HL), 0xC021);
    }

    #[test]
    fn instruction_ldd_bi_register_ptr_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0x18);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC020);

        cpu.ldd_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::A);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC020), 0x18);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::HL), 0xC01F);
    }

    #[test]
    fn instruction_inc_sp() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xABCD);

        cpu.inc_sp();

        assert_eq!(cpu.stack_pointer.read(), 0xABCE);
    }

    #[test]
    fn instruction_inc_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC021);
        cpu.memory_bus.borrow_mut().write_8bit(0xC021, 0xA);

        cpu.inc_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC021), 0xB);
    }

    #[test]
    fn instruction_dec_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC021);
        cpu.memory_bus.borrow_mut().write_8bit(0xC021, 0xA);

        cpu.dec_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC021), 0x9);
    }

    #[test]
    fn instruction_ld_bi_register_ptr_d8() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC020);
        cpu.memory_bus.borrow_mut().write_8bit(0xC020, 0x5A);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC030);

        cpu.ld_bi_register_ptr_d8(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC030), 0x5A);
        assert_eq!(cpu.program_counter.read(), 0xC021);
    }

    #[test]
    fn instruction_dec_sp() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xC035);

        cpu.dec_sp();

        assert_eq!(cpu.stack_pointer.read(), 0xC034);
    }

    #[test]
    fn instruction_scf() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::N);

        cpu.scf();

        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
    }

    #[test]
    fn instruction_ccf_1() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::C);
        cpu.set_flag(CPUFlag::N);

        cpu.ccf();

        assert_eq!(cpu.get_flag(CPUFlag::C), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
    }

    #[test]
    fn instruction_ccf_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::N);

        cpu.ccf();

        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
    }

    #[test]
    fn instruction_add_bi_register_sp() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::N);
        cpu.stack_pointer.write(0b1101110110011001);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0b0101010111011001);

        cpu.add_bi_register_sp(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
    }

    #[test]
    fn instruction_ldd_register_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC200);
        cpu.memory_bus.borrow_mut().write_8bit(0xC200, 0x55);

        cpu.ldd_register_bi_register_ptr(&RegisterIdentifier::A,
                                         &BiRegisterIdentifier::HL);


        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x55);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::HL), 0xC1FF);
    }

    #[test]
    fn instruction_ld_register_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::B, 0x67);

        cpu.ld_register_register(&RegisterIdentifier::A, &RegisterIdentifier::B);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x67);
    }

    #[test]
    fn instruction_halt() {
        let mut cpu = create_cpu();
        assert_eq!(cpu.halted, false);

        cpu.halt();

        assert_eq!(cpu.halted, true);
    }

    #[test]
    fn instruction_add_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_register(&RegisterIdentifier::B, 0b00000101);

        cpu.add_register(&RegisterIdentifier::B);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00010000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_add_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC036);
        cpu.memory_bus.borrow_mut().write_8bit(0xC036, 0b00000101);

        cpu.add_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00010000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_adc_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_register(&RegisterIdentifier::B, 0b00000101);
        cpu.set_flag(CPUFlag::C);

        cpu.adc_register(&RegisterIdentifier::B);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00010001);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_adc_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC036);
        cpu.memory_bus.borrow_mut().write_8bit(0xC036, 0b00000101);

        cpu.set_flag(CPUFlag::C);

        cpu.adc_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00010001);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }


}

