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
    /// The interrupt enable register
    ///
    interrupt_enable_register: Register8Bit,
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
            interrupt_enable_register: Register8Bit::new(),
            interrupt_master_enable: false,
            interrupt_disable_delay: 0,
            interrupt_enable_delay: 0,
            stopped: false,
            halted: false
        };
        cpu.initialize();

        cpu
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
        self.interrupt_enable_register.write(0x00);
    }

    ///
    /// Emulates a single CPU step
    ///
    pub fn step(&mut self) {
        let mut cycles = 0;

        let pc = self.program_counter.read();
        let opcode = self.memory_bus.borrow().read_8bit(pc as usize);
        println!("opcode: 0x{:X}, pc: 0x{:X}", opcode, pc);
        println!("A: 0x{:X}, B: 0x{:X}, C: 0x{:X}, D: 0x{:X}, E: 0x{:X}, F: 0x{:X}, H: 0x{:X}, L: 0x{:X}",
                 self.read_register(&RegisterIdentifier::A),
                 self.read_register(&RegisterIdentifier::B),
                 self.read_register(&RegisterIdentifier::C),
                 self.read_register(&RegisterIdentifier::D),
                 self.read_register(&RegisterIdentifier::E),
                 self.read_register(&RegisterIdentifier::F),
                 self.read_register(&RegisterIdentifier::H),
                 self.read_register(&RegisterIdentifier::L));
        println!("Z: {}", self.get_flag(CPUFlag::Z));
        println!("N: {}", self.get_flag(CPUFlag::N));
        println!("H: {}", self.get_flag(CPUFlag::H));
        println!("C: {}", self.get_flag(CPUFlag::C));

        println!();
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
            0x90 => cycles += self.sub_register(&RegisterIdentifier::B),
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
            0x98 => cycles += self.sbc_register(&RegisterIdentifier::B),
            // SBC A,C
            0x99 => cycles += self.sbc_register(&RegisterIdentifier::C),
            // SBC A,D
            0x9A => cycles += self.sbc_register(&RegisterIdentifier::D),
            // SBC A,E
            0x9B => cycles += self.sbc_register(&RegisterIdentifier::E),
            // SBC A,H
            0x9C => cycles += self.sbc_register(&RegisterIdentifier::H),
            // SBC A,L
            0x9D => cycles += self.sbc_register(&RegisterIdentifier::L),
            // SBC A,(HL)
            0x9E => cycles += self.sbc_bi_register_ptr(&BiRegisterIdentifier::HL),
            // SBC A,A
            0x9F => cycles += self.sbc_register(&RegisterIdentifier::A),
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
            0xBF => cycles += self.cp_register(&RegisterIdentifier::A),
            // RET NZ
            0xC0 => cycles += self.ret_flag(CPUFlag::Z, false),
            // POP BC
            0xC1 => cycles += self.pop_bi_register(&BiRegisterIdentifier::BC),
            // JP NZ,a16
            0xC2 => cycles += self.jp_flag_a16(CPUFlag::Z, false),
            // JP a16
            0xC3 => cycles += self.jp_a16(),
            // CALL NZ,a16
            0xC4 => cycles += self.call_flag_a16(CPUFlag::Z, false),
            // PUSH BC
            0xC5 => cycles += self.push_bi_register(&BiRegisterIdentifier::BC),
            // ADD A,d8
            0xC6 => cycles += self.add_d8(),
            // RST 00H
            0xC7 => cycles += self.rst(0x00),
            // RET Z
            0xC8 => cycles += self.ret_flag(CPUFlag::Z, true),
            // RET
            0xC9 => cycles += self.ret(),
            // JP Z,a16
            0xCA => cycles += self.jp_flag_a16(CPUFlag::Z, true),
            // PREFIX CB
            0xCB => cycles += self.cb_instruction(),
            // CALL Z,a16
            0xCC => cycles += self.call_flag_a16(CPUFlag::Z, true),
            // CALL a16
            0xCD => cycles += self.call_a16(),
            // ADC A,d8
            0xCE => cycles += self.adc_d8(),
            // RST 08H
            0xCF => cycles += self.rst(0x08),
            // RET NC
            0xD0 => cycles += self.ret_flag(CPUFlag::C, false),
            // POP DE
            0xC1 => cycles += self.pop_bi_register(&BiRegisterIdentifier::DE),
            // JP NC,a16
            0xD2 => cycles += self.jp_flag_a16(CPUFlag::C, false),
            // CALL NC,a16
            0xD4 => cycles += self.call_flag_a16(CPUFlag::C, false),
            // PUSH DE
            0xD5 => cycles += self.push_bi_register(&BiRegisterIdentifier::DE),
            // SUB d8
            0xD6 => cycles += self.sub_d8(),
            // RST 10H
            0xD7 => cycles += self.rst(0x10),
            // RET C
            0xD8 => cycles += self.ret_flag(CPUFlag::C, true),
            // RETI
            0xD9 => cycles += self.reti(),
            // JP C,a16
            0xDA => cycles += self.jp_flag_a16(CPUFlag::C, true),
            // CALL C,a16
            0xDC => cycles += self.call_flag_a16(CPUFlag::C, true),
            // SBC A,d8
            0xDE => cycles += self.sbc_d8(),
            // RST 18H
            0xDF => cycles += self.rst(0x18),
            // LDH (a8),A
            0xE0 => cycles += self.ldh_a8_ptr_register(&RegisterIdentifier::A),
            // POP HL
            0xE1 => cycles += self.pop_bi_register(&BiRegisterIdentifier::HL),
            // LD (C),A
            0xE2 => cycles += self.ld_register_ptr_register(&RegisterIdentifier::C, &RegisterIdentifier::A),
            // PUSH HL
            0xE5 => cycles += self.push_bi_register(&BiRegisterIdentifier::HL),
            // AND d8
            0xE6 => cycles += self.and_d8(),
            // RST 20H
            0xE7 => cycles += self.rst(0x20),
            // ADD SP,r8
            0xE8 => cycles += self.add_sp_r8(),
            // JP (HL)
            0xE9 => cycles += self.jp_bi_register_ptr(&BiRegisterIdentifier::HL),
            // LD (a16),A
            0xEA => cycles += self.ld_a16_ptr_register(&RegisterIdentifier::A),
            // XOR d8
            0xEE => cycles += self.and_d8(),
            // RST 28H
            0xEF => cycles += self.rst(0x28),
            // LDH A,(a8)
            0xF0 => cycles += self.ldh_register_a8_ptr(&RegisterIdentifier::A),
            // POP AF
            0xF1 => cycles += self.pop_bi_register(&BiRegisterIdentifier::AF),
            // LD A,(C)
            0xF2 => cycles += self.ld_register_register_ptr(&RegisterIdentifier::A, &RegisterIdentifier::C),
            // DI
            0xF3 => cycles += self.di(),
            // PUSH AF
            0xF5 => cycles += self.push_bi_register(&BiRegisterIdentifier::AF),
            // OR d8
            0xF6 => cycles += self.or_d8(),
            // RST 30H
            0xF7 => cycles += self.rst(0x30),
            // LD HL,SP+r8
            0xF8 => cycles += self.ld_bi_register_sppr8(&BiRegisterIdentifier::HL),
            // LD SP,HL
            0xF9 => cycles += self.ld_sp_bi_register(&BiRegisterIdentifier::HL),
            // LD A,(a16)
            0xFA => cycles += self.ld_register_a16_ptr(&RegisterIdentifier::A),
            // EI
            0xFB => cycles += self.ei(),
            // CP d8
            0xFE => cycles += self.cp_d8(),
            // RST 38H
            0xFF => cycles += self.rst(0x38),
            _ => panic!("Unimplemented instruction")
        }
    }

    fn cb_instruction(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let opcode = self.memory_bus.borrow().read_8bit(pc as usize);
        match opcode {
            0x00 => self.rlc_register(&RegisterIdentifier::B),
            0x01 => self.rlc_register(&RegisterIdentifier::C),
            0x02 => self.rlc_register(&RegisterIdentifier::D),
            0x03 => self.rlc_register(&RegisterIdentifier::E),
            0x04 => self.rlc_register(&RegisterIdentifier::H),
            0x05 => self.rlc_register(&RegisterIdentifier::L),
            0x06 => self.rlc_bi_register_ptr(&BiRegisterIdentifier::HL),
            0x07 => self.rlc_register(&RegisterIdentifier::A),
            0x08 => self.rrc_register(&RegisterIdentifier::B),
            0x09 => self.rrc_register(&RegisterIdentifier::C),
            0x0A => self.rrc_register(&RegisterIdentifier::D),
            0x0B => self.rrc_register(&RegisterIdentifier::E),
            0x0C => self.rrc_register(&RegisterIdentifier::H),
            0x0D => self.rrc_register(&RegisterIdentifier::L),
            0x0E => self.rrc_bi_register_ptr(&BiRegisterIdentifier::HL),
            0x0F => self.rrc_register(&RegisterIdentifier::A),
            0x10 => self.rl_register(&RegisterIdentifier::B),
            0x11 => self.rl_register(&RegisterIdentifier::C),
            0x12 => self.rl_register(&RegisterIdentifier::D),
            0x13 => self.rl_register(&RegisterIdentifier::E),
            0x14 => self.rl_register(&RegisterIdentifier::H),
            0x15 => self.rl_register(&RegisterIdentifier::L),
            0x16 => self.rl_bi_register_ptr(&BiRegisterIdentifier::HL),
            0x17 => self.rl_register(&RegisterIdentifier::A),
            0x18 => self.rr_register(&RegisterIdentifier::B),
            0x19 => self.rr_register(&RegisterIdentifier::C),
            0x1A => self.rr_register(&RegisterIdentifier::D),
            0x1B => self.rr_register(&RegisterIdentifier::E),
            0x1C => self.rr_register(&RegisterIdentifier::H),
            0x1D => self.rr_register(&RegisterIdentifier::L),
            0x1E => self.rr_bi_register_ptr(&BiRegisterIdentifier::HL),
            0x1F => self.rr_register(&RegisterIdentifier::A),
            0x20 => self.sla_register(&RegisterIdentifier::B),
            0x21 => self.sla_register(&RegisterIdentifier::C),
            0x22 => self.sla_register(&RegisterIdentifier::D),
            0x23 => self.sla_register(&RegisterIdentifier::E),
            0x24 => self.sla_register(&RegisterIdentifier::H),
            0x25 => self.sla_register(&RegisterIdentifier::L),
            0x26 => self.sla_bi_register_ptr(&BiRegisterIdentifier::HL),
            0x27 => self.sla_register(&RegisterIdentifier::A),
            0x28 => self.sra_register(&RegisterIdentifier::B),
            0x29 => self.sra_register(&RegisterIdentifier::C),
            0x2A => self.sra_register(&RegisterIdentifier::D),
            0x2B => self.sra_register(&RegisterIdentifier::E),
            0x2C => self.sra_register(&RegisterIdentifier::H),
            0x2D => self.sra_register(&RegisterIdentifier::L),
            0x2E => self.sra_bi_register_ptr(&BiRegisterIdentifier::HL),
            0x2F => self.sra_register(&RegisterIdentifier::A),
            0x30 => self.swap_register(&RegisterIdentifier::B),
            0x31 => self.swap_register(&RegisterIdentifier::C),
            0x32 => self.swap_register(&RegisterIdentifier::D),
            0x33 => self.swap_register(&RegisterIdentifier::E),
            0x34 => self.swap_register(&RegisterIdentifier::H),
            0x35 => self.swap_register(&RegisterIdentifier::L),
            0x36 => self.swap_bi_register_ptr(&BiRegisterIdentifier::HL),
            0x37 => self.swap_register(&RegisterIdentifier::A),
            0x38 => self.srl_register(&RegisterIdentifier::B),
            0x39 => self.srl_register(&RegisterIdentifier::C),
            0x3A => self.srl_register(&RegisterIdentifier::D),
            0x3B => self.srl_register(&RegisterIdentifier::E),
            0x3C => self.srl_register(&RegisterIdentifier::H),
            0x3D => self.srl_register(&RegisterIdentifier::L),
            0x3E => self.srl_bi_register_ptr(&BiRegisterIdentifier::HL),
            0x3F => self.srl_register(&RegisterIdentifier::A),
            0x40 => self.bit_register(0, &RegisterIdentifier::B),
            0x41 => self.bit_register(0, &RegisterIdentifier::C),
            0x42 => self.bit_register(0, &RegisterIdentifier::D),
            0x43 => self.bit_register(0, &RegisterIdentifier::E),
            0x44 => self.bit_register(0, &RegisterIdentifier::H),
            0x45 => self.bit_register(0, &RegisterIdentifier::L),
            0x46 => self.bit_bi_register_ptr(0, &BiRegisterIdentifier::HL),
            0x47 => self.bit_register(0, &RegisterIdentifier::A),
            0x48 => self.bit_register(1, &RegisterIdentifier::B),
            0x49 => self.bit_register(1, &RegisterIdentifier::C),
            0x4A => self.bit_register(1, &RegisterIdentifier::D),
            0x4B => self.bit_register(1, &RegisterIdentifier::E),
            0x4C => self.bit_register(1, &RegisterIdentifier::H),
            0x4D => self.bit_register(1, &RegisterIdentifier::L),
            0x4E => self.bit_bi_register_ptr(1, &BiRegisterIdentifier::HL),
            0x4F => self.bit_register(1, &RegisterIdentifier::A),
            0x50 => self.bit_register(2, &RegisterIdentifier::B),
            0x51 => self.bit_register(2, &RegisterIdentifier::C),
            0x52 => self.bit_register(2, &RegisterIdentifier::D),
            0x53 => self.bit_register(2, &RegisterIdentifier::E),
            0x54 => self.bit_register(2, &RegisterIdentifier::H),
            0x55 => self.bit_register(2, &RegisterIdentifier::L),
            0x56 => self.bit_bi_register_ptr(2, &BiRegisterIdentifier::HL),
            0x57 => self.bit_register(2, &RegisterIdentifier::A),
            0x58 => self.bit_register(3, &RegisterIdentifier::B),
            0x59 => self.bit_register(3, &RegisterIdentifier::C),
            0x5A => self.bit_register(3, &RegisterIdentifier::D),
            0x5B => self.bit_register(3,&RegisterIdentifier::E),
            0x5C => self.bit_register(3, &RegisterIdentifier::H),
            0x5D => self.bit_register(3, &RegisterIdentifier::L),
            0x5E => self.bit_bi_register_ptr(3, &BiRegisterIdentifier::HL),
            0x5F => self.bit_register(3, &RegisterIdentifier::A),
            0x60 => self.bit_register(4, &RegisterIdentifier::B),
            0x61 => self.bit_register(4, &RegisterIdentifier::C),
            0x62 => self.bit_register(4, &RegisterIdentifier::D),
            0x63 => self.bit_register(4, &RegisterIdentifier::E),
            0x64 => self.bit_register(4, &RegisterIdentifier::H),
            0x65 => self.bit_register(4, &RegisterIdentifier::L),
            0x66 => self.bit_bi_register_ptr(4, &BiRegisterIdentifier::HL),
            0x67 => self.bit_register(4, &RegisterIdentifier::A),
            0x68 => self.bit_register(5, &RegisterIdentifier::B),
            0x69 => self.bit_register(5, &RegisterIdentifier::C),
            0x6A => self.bit_register(5, &RegisterIdentifier::D),
            0x6B => self.bit_register(5, &RegisterIdentifier::E),
            0x6C => self.bit_register(5, &RegisterIdentifier::H),
            0x6D => self.bit_register(5, &RegisterIdentifier::L),
            0x6E => self.bit_bi_register_ptr(5, &BiRegisterIdentifier::HL),
            0x6F => self.bit_register(5, &RegisterIdentifier::A),
            0x70 => self.bit_register(6, &RegisterIdentifier::B),
            0x71 => self.bit_register(6, &RegisterIdentifier::C),
            0x72 => self.bit_register(6, &RegisterIdentifier::D),
            0x73 => self.bit_register(6, &RegisterIdentifier::E),
            0x74 => self.bit_register(6, &RegisterIdentifier::H),
            0x75 => self.bit_register(6, &RegisterIdentifier::L),
            0x76 => self.bit_bi_register_ptr(6, &BiRegisterIdentifier::HL),
            0x77 => self.bit_register(6, &RegisterIdentifier::A),
            0x78 => self.bit_register(7, &RegisterIdentifier::B),
            0x79 => self.bit_register(7, &RegisterIdentifier::C),
            0x7A => self.bit_register(7, &RegisterIdentifier::D),
            0x7B => self.bit_register(7, &RegisterIdentifier::E),
            0x7C => self.bit_register(7, &RegisterIdentifier::H),
            0x7D => self.bit_register(7, &RegisterIdentifier::L),
            0x7E => self.bit_bi_register_ptr(7, &BiRegisterIdentifier::HL),
            0x7F => self.bit_register(7, &RegisterIdentifier::A),
            0x80 => self.res_register(0, &RegisterIdentifier::B),
            0x81 => self.res_register(0, &RegisterIdentifier::C),
            0x82 => self.res_register(0, &RegisterIdentifier::D),
            0x83 => self.res_register(0, &RegisterIdentifier::E),
            0x84 => self.res_register(0, &RegisterIdentifier::H),
            0x85 => self.res_register(0, &RegisterIdentifier::L),
            0x86 => self.res_bi_register_ptr(0, &BiRegisterIdentifier::HL),
            0x87 => self.res_register(0, &RegisterIdentifier::A),
            0x88 => self.res_register(1, &RegisterIdentifier::B),
            0x89 => self.res_register(1, &RegisterIdentifier::C),
            0x8A => self.res_register(1, &RegisterIdentifier::D),
            0x8B => self.res_register(1, &RegisterIdentifier::E),
            0x8C => self.res_register(1, &RegisterIdentifier::H),
            0x8D => self.res_register(1, &RegisterIdentifier::L),
            0x8E => self.res_bi_register_ptr(1, &BiRegisterIdentifier::HL),
            0x8F => self.res_register(1, &RegisterIdentifier::A),
            0x90 => self.res_register(2, &RegisterIdentifier::B),
            0x91 => self.res_register(2, &RegisterIdentifier::C),
            0x92 => self.res_register(2, &RegisterIdentifier::D),
            0x93 => self.res_register(2, &RegisterIdentifier::E),
            0x94 => self.res_register(2, &RegisterIdentifier::H),
            0x95 => self.res_register(2, &RegisterIdentifier::L),
            0x96 => self.res_bi_register_ptr(2, &BiRegisterIdentifier::HL),
            0x97 => self.res_register(2, &RegisterIdentifier::A),
            0x98 => self.res_register(3, &RegisterIdentifier::B),
            0x99 => self.res_register(3, &RegisterIdentifier::C),
            0x9A => self.res_register(3, &RegisterIdentifier::D),
            0x9B => self.res_register(3,&RegisterIdentifier::E),
            0x9C => self.res_register(3, &RegisterIdentifier::H),
            0x9D => self.res_register(3, &RegisterIdentifier::L),
            0x9E => self.res_bi_register_ptr(3, &BiRegisterIdentifier::HL),
            0x9F => self.res_register(3, &RegisterIdentifier::A),
            0xA0 => self.res_register(4, &RegisterIdentifier::B),
            0xA1 => self.res_register(4, &RegisterIdentifier::C),
            0xA2 => self.res_register(4, &RegisterIdentifier::D),
            0xA3 => self.res_register(4, &RegisterIdentifier::E),
            0xA4 => self.res_register(4, &RegisterIdentifier::H),
            0xA5 => self.res_register(4, &RegisterIdentifier::L),
            0xA6 => self.res_bi_register_ptr(4, &BiRegisterIdentifier::HL),
            0xA7 => self.res_register(4, &RegisterIdentifier::A),
            0xA8 => self.res_register(5, &RegisterIdentifier::B),
            0xA9 => self.res_register(5, &RegisterIdentifier::C),
            0xAA => self.res_register(5, &RegisterIdentifier::D),
            0xAB => self.res_register(5, &RegisterIdentifier::E),
            0xAC => self.res_register(5, &RegisterIdentifier::H),
            0xAD => self.res_register(5, &RegisterIdentifier::L),
            0xAE => self.res_bi_register_ptr(5, &BiRegisterIdentifier::HL),
            0xAF => self.res_register(5, &RegisterIdentifier::A),
            0xB0 => self.res_register(6, &RegisterIdentifier::B),
            0xB1 => self.res_register(6, &RegisterIdentifier::C),
            0xB2 => self.res_register(6, &RegisterIdentifier::D),
            0xB3 => self.res_register(6, &RegisterIdentifier::E),
            0xB4 => self.res_register(6, &RegisterIdentifier::H),
            0xB5 => self.res_register(6, &RegisterIdentifier::L),
            0xB6 => self.res_bi_register_ptr(6, &BiRegisterIdentifier::HL),
            0xB7 => self.res_register(6, &RegisterIdentifier::A),
            0xB8 => self.res_register(7, &RegisterIdentifier::B),
            0xB9 => self.res_register(7, &RegisterIdentifier::C),
            0xBA => self.res_register(7, &RegisterIdentifier::D),
            0xBB => self.res_register(7, &RegisterIdentifier::E),
            0xBC => self.res_register(7, &RegisterIdentifier::H),
            0xBD => self.res_register(7, &RegisterIdentifier::L),
            0xBE => self.res_bi_register_ptr(7, &BiRegisterIdentifier::HL),
            0xBF => self.res_register(7, &RegisterIdentifier::A),
            0xC0 => self.set_register(0, &RegisterIdentifier::B),
            0xC1 => self.set_register(0, &RegisterIdentifier::C),
            0xC2 => self.set_register(0, &RegisterIdentifier::D),
            0xC3 => self.set_register(0, &RegisterIdentifier::E),
            0xC4 => self.set_register(0, &RegisterIdentifier::H),
            0xC5 => self.set_register(0, &RegisterIdentifier::L),
            0xC6 => self.set_bi_register_ptr(0, &BiRegisterIdentifier::HL),
            0xC7 => self.set_register(0, &RegisterIdentifier::A),
            0xC8 => self.set_register(1, &RegisterIdentifier::B),
            0xC9 => self.set_register(1, &RegisterIdentifier::C),
            0xCA => self.set_register(1, &RegisterIdentifier::D),
            0xCB => self.set_register(1, &RegisterIdentifier::E),
            0xCC => self.set_register(1, &RegisterIdentifier::H),
            0xCD => self.set_register(1, &RegisterIdentifier::L),
            0xCE => self.set_bi_register_ptr(1, &BiRegisterIdentifier::HL),
            0xCF => self.set_register(1, &RegisterIdentifier::A),
            0xD0 => self.set_register(2, &RegisterIdentifier::B),
            0xD1 => self.set_register(2, &RegisterIdentifier::C),
            0xD2 => self.set_register(2, &RegisterIdentifier::D),
            0xD3 => self.set_register(2, &RegisterIdentifier::E),
            0xD4 => self.set_register(2, &RegisterIdentifier::H),
            0xD5 => self.set_register(2, &RegisterIdentifier::L),
            0xD6 => self.set_bi_register_ptr(2, &BiRegisterIdentifier::HL),
            0xD7 => self.set_register(2, &RegisterIdentifier::A),
            0xD8 => self.set_register(3, &RegisterIdentifier::B),
            0xD9 => self.set_register(3, &RegisterIdentifier::C),
            0xDA => self.set_register(3, &RegisterIdentifier::D),
            0xDB => self.set_register(3,&RegisterIdentifier::E),
            0xDC => self.set_register(3, &RegisterIdentifier::H),
            0xDD => self.set_register(3, &RegisterIdentifier::L),
            0xDE => self.set_bi_register_ptr(3, &BiRegisterIdentifier::HL),
            0xDF => self.set_register(3, &RegisterIdentifier::A),
            0xE0 => self.set_register(4, &RegisterIdentifier::B),
            0xE1 => self.set_register(4, &RegisterIdentifier::C),
            0xE2 => self.set_register(4, &RegisterIdentifier::D),
            0xE3 => self.set_register(4, &RegisterIdentifier::E),
            0xE4 => self.set_register(4, &RegisterIdentifier::H),
            0xE5 => self.set_register(4, &RegisterIdentifier::L),
            0xE6 => self.set_bi_register_ptr(4, &BiRegisterIdentifier::HL),
            0xE7 => self.set_register(4, &RegisterIdentifier::A),
            0xE8 => self.set_register(5, &RegisterIdentifier::B),
            0xE9 => self.set_register(5, &RegisterIdentifier::C),
            0xEA => self.set_register(5, &RegisterIdentifier::D),
            0xEB => self.set_register(5, &RegisterIdentifier::E),
            0xEC => self.set_register(5, &RegisterIdentifier::H),
            0xED => self.set_register(5, &RegisterIdentifier::L),
            0xEE => self.set_bi_register_ptr(5, &BiRegisterIdentifier::HL),
            0xEF => self.set_register(5, &RegisterIdentifier::A),
            0xF0 => self.set_register(6, &RegisterIdentifier::B),
            0xF1 => self.set_register(6, &RegisterIdentifier::C),
            0xF2 => self.set_register(6, &RegisterIdentifier::D),
            0xF3 => self.set_register(6, &RegisterIdentifier::E),
            0xF4 => self.set_register(6, &RegisterIdentifier::H),
            0xF5 => self.set_register(6, &RegisterIdentifier::L),
            0xF6 => self.set_bi_register_ptr(6, &BiRegisterIdentifier::HL),
            0xF7 => self.set_register(6, &RegisterIdentifier::A),
            0xF8 => self.set_register(7, &RegisterIdentifier::B),
            0xF9 => self.set_register(7, &RegisterIdentifier::C),
            0xFA => self.set_register(7, &RegisterIdentifier::D),
            0xFB => self.set_register(7, &RegisterIdentifier::E),
            0xFC => self.set_register(7, &RegisterIdentifier::H),
            0xFD => self.set_register(7, &RegisterIdentifier::L),
            0xFE => self.set_bi_register_ptr(7, &BiRegisterIdentifier::HL),
            0xFF => self.set_register(7, &RegisterIdentifier::A),
            _ => panic!("Unimplemented CB instruction")
        }
    }

    fn nop(&mut self) -> u32 {
        4
    }

    fn ei(&mut self) -> u32 {
        self.interrupt_enable_delay = 1;
        4
    }

    fn di(&mut self) -> u32 {
        self.interrupt_disable_delay = 1;
        4
    }

    fn reti(&mut self) -> u32 {
        let sp = self.stack_pointer.read();
        let value = self.memory_bus.borrow().read_16bit(sp as usize);
        self.stack_pointer.increment(2);
        self.program_counter.write(value);
        self.interrupt_master_enable = true;
        self.interrupt_enable_delay = 0;
        8
    }

    fn ld_bi_register_sppr8(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let sp = self.stack_pointer.read();
        let pc = self.program_counter.read();

        let value_to_add = self.memory_bus.borrow().read_8bit_signed(pc as usize);
        self.program_counter.increment(1);

        let sum = (sp as i32).wrapping_add(value_to_add as i32);

        self.set_flag(CPUFlag::H, ((sum & 0x0FFF) as u16) < (sp & 0x0FFF));
        self.set_flag(CPUFlag::C, sum > 0xFFFF);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::Z, false);

        self.write_bi_register(bi_register_identifier, sum as u16);
        12
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

    fn ld_register_d8(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.memory_bus.borrow().read_8bit(self.program_counter.read() as usize);
        self.program_counter.increment(1);

        self.write_register(register_identifier, value);
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
        self.program_counter.write((pc as i32 + value as i32 - 1) as u16);
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

    fn ld_bi_register_ptr_d8(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let value = self.memory_bus.borrow().read_8bit(self.program_counter.read() as usize);
        self.program_counter.increment(1);
        let address = self.read_bi_register(bi_register_identifier);
        self.memory_bus.borrow_mut().write_8bit(address as usize, value);
        12
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

    fn push_bi_register(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        self.stack_pointer.decrement(2);
        let sp = self.stack_pointer.read();
        let value = self.read_bi_register(bi_register_identifier);
        self.memory_bus.borrow_mut().write_16bit(sp as usize, value);
        16
    }

    fn pop_bi_register(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let sp = self.stack_pointer.read();
        let value = self.memory_bus.borrow().read_16bit(sp as usize);
        self.stack_pointer.increment(2);
        self.write_bi_register(bi_register_identifier, value);
        12
    }

    fn jp_a16(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let address = self.memory_bus.borrow().read_16bit(pc as usize);
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
        let value = self.memory_bus.borrow().read_16bit(sp as usize);
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
        let address = self.memory_bus.borrow().read_16bit(pc as usize);
        self.program_counter.increment(2);

        let sp = self.stack_pointer.read();
        let next_instruction_address = self.program_counter.read();
        self.memory_bus.borrow_mut().write_16bit(sp as usize, next_instruction_address);
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
        self.memory_bus.borrow_mut().write_16bit(sp as usize, pc);
        self.stack_pointer.decrement(2);

        self.program_counter.write(0x0000 + address as u16);
        32
    }

    fn ldh_a8_ptr_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);

        let pc = self.program_counter.read();
        let add_to_address = self.memory_bus.borrow().read_8bit(pc as usize);
        self.program_counter.increment(1);

        let resulting_address = 0xFF00 + add_to_address as usize;

        if resulting_address == 0xFFFF {
            self.interrupt_enable_register.write(value);
        } else {
            self.memory_bus.borrow_mut().write_8bit(resulting_address, value);
        }

        12
    }

    fn ld_register_ptr_register(&mut self,
                                first_register_identifier: &RegisterIdentifier,
                                second_register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(second_register_identifier);
        let add_to_address = self.read_register(first_register_identifier);

        self.memory_bus.borrow_mut().write_8bit(0xFF00 + add_to_address as usize, value);
        8
    }

    fn ld_register_register_ptr(&mut self,
                                first_register_identifier: &RegisterIdentifier,
                                second_register_identifier: &RegisterIdentifier) -> u32 {
        let add_to_address = self.read_register(second_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(0xFF00 + add_to_address as usize);
        self.write_register(first_register_identifier, value);
        8
    }

    fn ld_a16_ptr_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);

        let pc = self.program_counter.read();
        let address = self.memory_bus.borrow().read_16bit(pc as usize);
        self.program_counter.increment(2);

        self.memory_bus.borrow_mut().write_8bit(address as usize, value);
        16
    }

    fn ldh_register_a8_ptr(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let pc = self.program_counter.read();
        let address_to_add = self.memory_bus.borrow().read_8bit(pc as usize);
        self.program_counter.increment(1);
        let value = self.memory_bus.borrow().read_8bit(0xFF00 + address_to_add as usize);
        self.write_register(register_identifier, value);
        16
    }

    fn ld_sp_bi_register(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let value = self.read_bi_register(bi_register_identifier);
        self.stack_pointer.write(value);
        8
    }

    fn ld_register_a16_ptr(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let pc = self.program_counter.read();
        let address = self.memory_bus.borrow().read_16bit(pc as usize);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        self.program_counter.increment(2);
        self.write_register(register_identifier, value);
        16
    }

    // 8-bit rotations/shifts and bit instructions

    fn rlc(&mut self, value: u8) -> u8 {
        let result = (value << 1) | ((value & 0x80) >> 7);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x80 != 0);

        result
    }

    fn rl(&mut self, value: u8) -> u8 {
        let result = (value << 1) | (self.get_flag(CPUFlag::C) as u8);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x80 != 0);

        result
    }

    fn sla(&mut self, value: u8) -> u8 {
        let result = value << 1;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x80 != 0);

        result
    }

    fn sra(&mut self, value: u8) -> u8 {
        let result = (value & 0x80) | (value >> 1);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x01 != 0);

        result
    }

    fn srl(&mut self, value: u8) -> u8 {
        let result = value >> 1;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 0x01 != 0);

        result
    }

    fn rrc(&mut self, value: u8) -> u8 {
        let result = (value >> 1) | ((value & 1) << 7);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 1 != 0);

        result
    }

    fn rr(&mut self, value: u8) -> u8 {
        let result = (value >> 1) | ((self.get_flag(CPUFlag::C) as u8) << 7);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, value & 1 != 0);

        result
    }

    fn swap(&mut self, value: u8) -> u8 {
        let result = (value << 4) | (value >> 4);

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, false);

        result
    }

    fn bit(&mut self, bit: u8, value: u8) {
        self.set_flag(CPUFlag::Z, (value & (1 << bit)) == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, true);
    }

    fn res(&mut self, bit: u8, value: u8) -> u8 {
        return value & !(1 << bit)
    }

    fn set(&mut self, bit: u8, value: u8) -> u8 {
        return value | (1 << bit)
    }

    fn rlc_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.rlc(value);

        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn rlc_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.rlc(value);

        self.write_register(register_identifier, result);
        8
    }

    fn rrc_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.rrc(value);

        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn rrc_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.rrc(value);

        self.write_register(register_identifier, result);
        8
    }

    fn rl_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.rl(value);

        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn rl_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.rl(value);

        self.write_register(register_identifier, result);
        8
    }

    fn rr_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.rr(value);

        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn rr_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.rr(value);

        self.write_register(register_identifier, result);
        8
    }

    fn sla_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.sla(value);

        self.write_register(register_identifier, result);
        8
    }

    fn sla_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.sla(value);

        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn sra_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.sra(value);

        self.write_register(register_identifier, result);
        8
    }

    fn sra_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.sra(value);
        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn srl_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.srl(value);

        self.write_register(register_identifier, result);
        8
    }

    fn srl_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.srl(value);
        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn swap_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.swap(value);

        self.write_register(register_identifier, result);
        8
    }

    fn swap_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.swap(value);
        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn bit_register(&mut self, bit: u8, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        self.bit(bit, value);
        8
    }

    fn bit_bi_register_ptr(&mut self, bit: u8, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        self.bit(bit, value);
        16
    }

    fn res_register(&mut self, bit: u8, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.res(bit, value);

        self.write_register(register_identifier, result);
        8
    }

    fn res_bi_register_ptr(&mut self, bit: u8, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.res(bit, value);

        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    fn set_register(&mut self, bit: u8, register_identifier: &RegisterIdentifier) -> u32 {
        let value = self.read_register(register_identifier);
        let result = self.set(bit, value);

        self.write_register(register_identifier, result);
        8
    }

    fn set_bi_register_ptr(&mut self, bit: u8, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let value = self.memory_bus.borrow().read_8bit(address as usize);
        let result = self.set(bit, value);

        self.memory_bus.borrow_mut().write_8bit(address as usize, result);
        16
    }

    // 8-bit ALU
    fn add(&mut self, value: u8) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = value;

        let sum = (lhs as u16).wrapping_add(rhs as u16);

        self.set_flag(CPUFlag::Z, sum as u8 == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, (sum as u8 & 0x0F) < (lhs & 0x0F));
        self.set_flag(CPUFlag::C, sum > 0xFF);

        self.write_register(&RegisterIdentifier::A, sum as u8);
    }

    fn sub(&mut self, value: u8) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = value;
        let difference = (lhs as u16).wrapping_sub(rhs as u16);

        self.set_flag(CPUFlag::Z, difference == 0);
        self.set_flag(CPUFlag::N, true);
        self.set_flag(CPUFlag::H, (rhs & 0x0F) > (lhs & 0x0F));
        self.set_flag(CPUFlag::C, rhs > lhs);

        self.write_register(&RegisterIdentifier::A, difference as u8);
    }

    fn and(&mut self, value: u8) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = value;
        let result = lhs & rhs;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, true);
        self.set_flag(CPUFlag::C, false);

        self.write_register(&RegisterIdentifier::A, result);
    }

    fn xor(&mut self, value: u8) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = value;
        let result = lhs ^ rhs;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, false);

        self.write_register(&RegisterIdentifier::A, result);
    }

    fn or(&mut self, value: u8) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = value;
        let result = lhs | rhs;

        self.set_flag(CPUFlag::Z, result == 0);
        self.set_flag(CPUFlag::N, false);
        self.set_flag(CPUFlag::H, false);
        self.set_flag(CPUFlag::C, false);

        self.write_register(&RegisterIdentifier::A, result);
    }

    fn cp(&mut self, value: u8) {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let rhs = value;
        let difference = (lhs as u16).wrapping_sub(rhs as u16);

        self.set_flag(CPUFlag::Z, difference == 0);
        self.set_flag(CPUFlag::N, true);
        self.set_flag(CPUFlag::H, (rhs & 0x0F) > (lhs & 0x0F));
        self.set_flag(CPUFlag::C, rhs > lhs);
    }

    fn inc_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let lhs = self.read_register(register_identifier);
        let sum = lhs.wrapping_add(1);

        self.set_flag(CPUFlag::Z, sum == 0);
        self.set_flag(CPUFlag::H, (sum & 0xF) < (lhs & 0xF));
        self.set_flag(CPUFlag::N, false);

        self.write_register(register_identifier, sum);
        4
    }

    fn dec_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let lhs = self.read_register(register_identifier);
        let difference = lhs.wrapping_sub(1);

        self.set_flag(CPUFlag::Z, difference == 0);
        self.set_flag(CPUFlag::H, (difference & 0xF) <= (lhs & 0xF));
        self.set_flag(CPUFlag::N, true);

        self.write_register(register_identifier, difference);
        4
    }

    fn inc_sp(&mut self) -> u32 {
        self.stack_pointer.increment(1);
        8
    }

    fn dec_sp(&mut self) -> u32 {
        self.stack_pointer.decrement(1);
        8
    }

    fn inc_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let lhs = self.memory_bus.borrow().read_8bit(address as usize);
        let rhs = 1;
        let sum = lhs.wrapping_add(rhs);

        self.set_flag(CPUFlag::Z, sum == 0);
        self.set_flag(CPUFlag::H, (sum & 0x0F) < (lhs & 0x0F));
        self.set_flag(CPUFlag::N, false);

        self.memory_bus.borrow_mut().write_8bit(address as usize, sum);
        12
    }

    fn dec_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let lhs = self.memory_bus.borrow().read_8bit(address as usize);
        let rhs = 1;
        let difference = lhs.wrapping_sub(rhs);

        self.set_flag(CPUFlag::Z, difference == 0);
        self.set_flag(CPUFlag::H, (difference & 0x0F) <= (lhs & 0x0F));
        self.set_flag(CPUFlag::N, true);

        self.memory_bus.borrow_mut().write_8bit(address as usize, difference);
        12
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

    fn add_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let rhs = self.read_register(register_identifier);
        self.add(rhs);
        4
    }

    fn add_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let rhs = self.memory_bus.borrow().read_8bit(address as usize);
        self.add(rhs);
        8
    }

    fn add_d8(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let rhs = self.memory_bus.borrow().read_8bit(pc as usize);
        self.program_counter.increment(1);
        self.add(rhs);
        8
    }

    fn sub_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let rhs = self.read_register(register_identifier);
        self.sub(rhs);
        4
    }

    fn sub_d8(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let rhs = self.memory_bus.borrow().read_8bit(pc as usize);
        self.program_counter.increment(1);
        self.sub(rhs);
        8
    }

    fn sub_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let rhs = self.memory_bus.borrow().read_8bit(address as usize);
        self.sub(rhs);
        8
    }

    fn adc_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let mut rhs = self.read_register(register_identifier);
        rhs = rhs.wrapping_add(if self.get_flag(CPUFlag::C) {1} else {0});
        self.add(rhs);
        4
    }

    fn adc_d8(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let mut rhs = self.memory_bus.borrow().read_8bit(pc as usize);
        rhs = rhs.wrapping_add(if self.get_flag(CPUFlag::C) {1} else {0});
        self.program_counter.increment(1);
        self.add(rhs);
        8
    }

    fn adc_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let mut rhs = self.memory_bus.borrow().read_8bit(address as usize);
        rhs = rhs.wrapping_add(if self.get_flag(CPUFlag::C) {1} else {0});
        self.add(rhs);
        8
    }

    fn sbc_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let lhs = self.read_register(&RegisterIdentifier::A);
        let mut rhs = self.read_register(register_identifier);
        rhs = rhs.wrapping_add(if self.get_flag(CPUFlag::C) {1} else {0});
        self.sub(rhs);
        4
    }

    fn sbc_d8(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let mut rhs = self.memory_bus.borrow().read_8bit(pc as usize);
        rhs = rhs.wrapping_add(if self.get_flag(CPUFlag::C) {1} else {0});
        self.program_counter.increment(1);
        self.sub(rhs);
        8
    }

    fn sbc_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let mut rhs = self.memory_bus.borrow().read_8bit(address as usize);
        rhs = rhs.wrapping_add(if self.get_flag(CPUFlag::C) {1} else {0});
        self.sub(rhs);
        8
    }

    fn and_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let rhs = self.read_register(register_identifier);
        self.and(rhs);
        4
    }

    fn and_d8(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let rhs = self.memory_bus.borrow().read_8bit(pc as usize);
        self.program_counter.increment(1);
        self.and(rhs);
        8
    }

    fn and_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let rhs = self.memory_bus.borrow().read_8bit(address as usize);
        self.and(rhs);
        8
    }

    fn xor_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let rhs = self.read_register(register_identifier);
        self.xor(rhs);
        4
    }

    fn xor_d8(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let rhs = self.memory_bus.borrow().read_8bit(pc as usize);
        self.program_counter.increment(1);
        self.xor(rhs);
        8
    }

    fn xor_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let rhs = self.memory_bus.borrow().read_8bit(address as usize);
        self.xor(rhs);
        8
    }

    fn or_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let rhs = self.read_register(register_identifier);
        self.or(rhs);
        4
    }

    fn or_d8(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let rhs = self.memory_bus.borrow().read_8bit(pc as usize);
        self.program_counter.increment(1);
        self.or(rhs);
        8
    }

    fn or_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let rhs = self.memory_bus.borrow().read_8bit(address as usize);
        self.or(rhs);
        8
    }

    fn cp_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 {
        let rhs = self.read_register(register_identifier);
        self.cp(rhs);
        4
    }

    fn cp_d8(&mut self) -> u32 {
        let pc = self.program_counter.read();
        let rhs = self.memory_bus.borrow().read_8bit(pc as usize);
        self.program_counter.increment(1);
        self.cp(rhs);
        8
    }

    fn cp_bi_register_ptr(&mut self, bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let address = self.read_bi_register(bi_register_identifier);
        let rhs = self.memory_bus.borrow().read_8bit(address as usize);
        self.cp(rhs);
        8
    }

    // 16-bit ALU
    fn add_bi_register_bi_register(&mut self,
                                   first_register_identifier: &BiRegisterIdentifier,
                                   second_register_identifier: &BiRegisterIdentifier) -> u32 {
        let lhs = self.read_bi_register(first_register_identifier);
        let rhs = self.read_bi_register(second_register_identifier);
        let sum = (lhs as u32).wrapping_add(rhs as u32);

        self.set_flag(CPUFlag::H, ((sum & 0x0FFF) as u16) < (lhs & 0x0FFF));
        self.set_flag(CPUFlag::C, sum > 0xFFFF);
        self.set_flag(CPUFlag::N, false);

        self.write_bi_register(first_register_identifier, sum as u16);
        8
    }



    fn add_bi_register_sp(&mut self,
                          bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        use std::u32;

        let lhs = self.read_bi_register(bi_register_identifier);
        let rhs = self.stack_pointer.read();
        let sum = (lhs as u32).wrapping_add(rhs as u32);

        self.set_flag(CPUFlag::H, ((sum & 0x0FFF) as u16) < (lhs & 0x0FFF));
        self.set_flag(CPUFlag::C, sum > 0xFFFF);
        self.set_flag(CPUFlag::N, false);

        self.write_bi_register(bi_register_identifier, sum as u16);

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

    fn add_sp_r8(&mut self) -> u32 {
        let lhs = self.stack_pointer.read();
        let pc = self.program_counter.read();
        let rhs = self.memory_bus.borrow().read_8bit_signed(pc as usize);
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

#[cfg(test)]
mod test {

    use super::*;
    use gb::memory::cartridge::*;
    use gb::memory::ram::*;
    use gb::memory::oam::*;
    use gb::memory::high_ram::*;
    use gb::memory::io::*;

    fn create_cpu() -> CPU {
        let cartridge = Rc::new(RefCell::new(Cartridge::from_bytes([0; 0x8000])));
        let ram = Rc::new(RefCell::new(Ram::new()));
        let oam = Rc::new(RefCell::new(Oam::new()));
        let high_ram = Rc::new(RefCell::new(HighRam::new()));
        let io = Rc::new(RefCell::new(IO::new()));
        let memory_bus = Rc::new(RefCell::new(MemoryBus::new(cartridge.clone(), ram.clone(), oam.clone(), high_ram.clone(), io.clone())));
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
        cpu.set_flag(CPUFlag::Z, true);
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
    fn instruction_add_bi_register_sp() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::N, true);
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
        cpu.set_flag(CPUFlag::C, true);

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

        cpu.set_flag(CPUFlag::C, true);

        cpu.adc_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00010001);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_sub_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_register(&RegisterIdentifier::B, 0b00000011);

        cpu.sub_register(&RegisterIdentifier::B);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00001000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_sub_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC035);
        cpu.memory_bus.borrow_mut().write_8bit(0xC035, 0b00000011);

        cpu.sub_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00001000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_sbc_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_register(&RegisterIdentifier::B, 0b00000011);
        cpu.set_flag(CPUFlag::C, true);

        cpu.sbc_register(&RegisterIdentifier::B);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000111);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_sbc_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC035);
        cpu.memory_bus.borrow_mut().write_8bit(0xC035, 0b00000011);
        cpu.set_flag(CPUFlag::C, true);

        cpu.sbc_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000111);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_and_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_register(&RegisterIdentifier::B, 0b01111000);

        cpu.and_register(&RegisterIdentifier::B);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00100000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_and_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01111000);

        cpu.and_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00100000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_xor_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_register(&RegisterIdentifier::B, 0b01111000);

        cpu.xor_register(&RegisterIdentifier::B);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01011010);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_xor_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC025);
        cpu.memory_bus.borrow_mut().write_8bit(0xC025, 0b01111000);

        cpu.xor_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01011010);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_or_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_register(&RegisterIdentifier::B, 0b01111000);

        cpu.or_register(&RegisterIdentifier::B);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01111010);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_or_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC025);
        cpu.memory_bus.borrow_mut().write_8bit(0xC025, 0b01111000);

        cpu.or_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01111010);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_cp_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_register(&RegisterIdentifier::B, 0b00000011);

        cpu.cp_register(&RegisterIdentifier::B);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_cp_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC035);
        cpu.memory_bus.borrow_mut().write_8bit(0xC035, 0b00000011);

        cpu.cp_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_push_bi_register() {
        let mut cpu = create_cpu();

        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0xC090);


        cpu.push_bi_register(&BiRegisterIdentifier::BC);

        assert_eq!(cpu.stack_pointer.read(), 0xFFFC);
        assert_eq!(cpu.memory_bus.borrow().read_16bit(0xFFFC), 0xC090);
    }

    #[test]
    fn instruction_pop_bi_register() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xFFFC);
        cpu.memory_bus.borrow_mut().write_16bit(0xFFFC, 0xC090);

        cpu.pop_bi_register(&BiRegisterIdentifier::BC);

        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::BC), 0xC090);
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
    fn instruction_add_d8() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);

        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000101);

        cpu.add_d8();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00010000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_adc_d8() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);

        cpu.set_flag(CPUFlag::C, true);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000101);

        cpu.adc_d8();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00010001);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_sub_d8() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000011);

        cpu.sub_d8();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00001000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_sbc_d8() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000011);

        cpu.set_flag(CPUFlag::C, true);

        cpu.sbc_d8();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000111);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_and_d8() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01111000);

        cpu.and_d8();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00100000);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_xor_d8() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01111000);

        cpu.xor_d8();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01011010);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_or_d8() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00100010);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01111000);

        cpu.or_d8();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01111010);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_cp_d8() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00001011);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000011);

        cpu.cp_d8();

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), true);
        assert_eq!(cpu.get_flag(CPUFlag::H), false);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
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
    fn instruction_ldh_a8_ptr_register() {
        let mut cpu = create_cpu();

        cpu.write_register(&RegisterIdentifier::A, 0xA5);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0x03);

        cpu.ldh_a8_ptr_register(&RegisterIdentifier::A);

        assert_eq!(cpu.program_counter.read(), 0xC024);
        assert_eq!(cpu.memory_bus.borrow().read_16bit(0xFF03), 0xA5);
    }

    #[test]
    fn instruction_ld_register_ptr_register() {
        let mut cpu = create_cpu();

        cpu.write_register(&RegisterIdentifier::A, 0xA5);
        cpu.write_register(&RegisterIdentifier::C, 0x03);

        cpu.ld_register_ptr_register(&RegisterIdentifier::C, &RegisterIdentifier::A);

        assert_eq!(cpu.memory_bus.borrow().read_16bit(0xFF03), 0xA5);
    }

    #[test]
    fn instruction_ld_a16_ptr_register() {
        let mut cpu = create_cpu();

        cpu.write_register(&RegisterIdentifier::A, 0xA5);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_16bit(0xC023, 0xC256);

        cpu.ld_a16_ptr_register(&RegisterIdentifier::A);

        assert_eq!(cpu.program_counter.read(), 0xC025);
        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC256), 0xA5);
    }

    #[test]
    fn instruction_ldh_register_a8_ptr() {
        let mut cpu = create_cpu();

        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0x44);
        cpu.memory_bus.borrow_mut().write_8bit(0xFF44, 0x78);

        cpu.ldh_register_a8_ptr(&RegisterIdentifier::A);

        assert_eq!(cpu.program_counter.read(), 0xC024);
        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x78);
    }

    #[test]
    fn instruction_ld_register_register_ptr() {
        let mut cpu = create_cpu();

        cpu.write_register(&RegisterIdentifier::C, 0x3);
        cpu.memory_bus.borrow_mut().write_8bit(0xFF03, 0x56);

        cpu.ld_register_register_ptr(&RegisterIdentifier::A, &RegisterIdentifier::C);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x56);
    }

    #[test]
    fn instruction_ld_sp_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC356);

        cpu.ld_sp_bi_register(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.stack_pointer.read(), 0xC356);
    }

    #[test]
    fn instruction_jp_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC825);

        cpu.jp_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.program_counter.read(), 0xC825);
    }

    #[test]
    fn instruction_ld_register_a16_ptr() {
        let mut cpu = create_cpu();
        cpu.program_counter.write(0xC825);
        cpu.memory_bus.borrow_mut().write_16bit(0xC825, 0xC600);
        cpu.memory_bus.borrow_mut().write_8bit(0xC600, 0x35);

        cpu.ld_register_a16_ptr(&RegisterIdentifier::A);

        assert_eq!(cpu.program_counter.read(), 0xC827);
        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x35);
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
    fn instruction_add_sp_r8() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xFFFC);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 2);

        cpu.add_sp_r8();

        assert_eq!(cpu.program_counter.read(), 0xC024);
        assert_eq!(cpu.stack_pointer.read(), 0xFFFE);
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

    #[test]
    fn instruction_ld_bi_register_sppr8() {
        let mut cpu = create_cpu();
        cpu.stack_pointer.write(0xFFFC);
        cpu.program_counter.write(0xC023);
        cpu.memory_bus.borrow_mut().write_8bit_signed(0xC023, -2);

        cpu.ld_bi_register_sppr8(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.read_bi_register(&BiRegisterIdentifier::HL), 0xFFFA);
    }

    #[test]
    fn instruction_rlc_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b01000000);

        cpu.rlc_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rlc_register_2() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b10000000);

        cpu.rlc_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000001);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rlc_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01000000);

        cpu.rlc_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b10000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rlc_bi_register_ptr_2() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b10000000);

        cpu.rlc_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000001);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rrc_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00000010);

        cpu.rrc_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000001);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rrc_register_2() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00000001);

        cpu.rrc_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rrc_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000010);

        cpu.rrc_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000001);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rrc_bi_register_ptr_2() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000001);

        cpu.rrc_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b10000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rl_register() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b01000000);

        cpu.rl_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rl_register_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b10000000);

        cpu.rl_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rl_register_3() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00000000);
        cpu.set_flag(CPUFlag::C, true);

        cpu.rl_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000001);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rl_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01000000);

        cpu.rl_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b10000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rl_bi_register_ptr_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b10000000);

        cpu.rl_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rl_bi_register_ptr_3() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000000);
        cpu.set_flag(CPUFlag::C, true);

        cpu.rl_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000001);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }


    #[test]
    fn instruction_rr_register() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b00000010);

        cpu.rr_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000001);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rr_register_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_register(&RegisterIdentifier::A, 0b00000001);

        cpu.rr_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rr_register_3() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00000000);
        cpu.set_flag(CPUFlag::C, true);

        cpu.rr_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rr_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000010);

        cpu.rr_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000001);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_rr_bi_register_ptr_2() {
        let mut cpu = create_cpu();
        cpu.set_flag(CPUFlag::Z, false);
        cpu.set_flag(CPUFlag::N, false);
        cpu.set_flag(CPUFlag::H, false);
        cpu.set_flag(CPUFlag::C, false);

        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000001);

        cpu.rr_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
    }

    #[test]
    fn instruction_rr_bi_register_ptr_3() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000000);
        cpu.set_flag(CPUFlag::C, true);

        cpu.rr_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b10000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
    }

    #[test]
    fn instruction_sla_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00000001);

        cpu.sla_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000010);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_sla_register_2() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b10000001);

        cpu.sla_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000010);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_sla_register_3() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b10000000);

        cpu.sla_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
    }

    #[test]
    fn instruction_sla_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000001);

        cpu.sla_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000010);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_sla_bi_register_ptr_2() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b10000001);

        cpu.sla_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000010);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_sla_bi_register_ptr_3() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b10000000);

        cpu.sla_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
    }

    #[test]
    fn instruction_sra_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b10000000);

        cpu.sra_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b11000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_sra_register_2() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b10000001);

        cpu.sra_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b11000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_sra_register_3() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00000001);

        cpu.sra_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
    }

    #[test]
    fn instruction_sra_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b10000000);

        cpu.sra_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b11000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_sra_bi_register_ptr_2() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b10000001);

        cpu.sra_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b11000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_sra_bi_register_ptr_3() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000001);

        cpu.sra_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
    }

    #[test]
    fn instruction_srl_register_2() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b10000001);

        cpu.srl_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_srl_register_3() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b00000001);

        cpu.srl_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
    }

    #[test]
    fn instruction_srl_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b10000000);

        cpu.srl_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b01000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), false);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_srl_bi_register_ptr_2() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b10000001);

        cpu.srl_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b01000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
    }

    #[test]
    fn instruction_srl_bi_register_ptr_3() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b00000001);

        cpu.srl_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b00000000);
        assert_eq!(cpu.get_flag(CPUFlag::C), true);
        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
    }

    #[test]
    fn instruction_swap_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0xAB);

        cpu.swap_register(&RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0xBA);
    }

    #[test]
    fn instruction_swap_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0xAB);

        cpu.swap_bi_register_ptr(&BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0xBA);
    }

    #[test]
    fn instruction_bit_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b01001010);

        cpu.bit_register(0, &RegisterIdentifier::A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_register(1, &RegisterIdentifier::A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_register(2, &RegisterIdentifier::A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_register(3, &RegisterIdentifier::A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_register(4, &RegisterIdentifier::A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_register(5, &RegisterIdentifier::A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_register(6, &RegisterIdentifier::A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_register(7, &RegisterIdentifier::A);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
    }

    #[test]
    fn instruction_bit_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01001010);

        cpu.bit_bi_register_ptr(0, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_bi_register_ptr(1, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_bi_register_ptr(2, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_bi_register_ptr(3, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_bi_register_ptr(4, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_bi_register_ptr(5, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_bi_register_ptr(6, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), false);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);

        cpu.bit_bi_register_ptr(7, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.get_flag(CPUFlag::Z), true);
        assert_eq!(cpu.get_flag(CPUFlag::N), false);
        assert_eq!(cpu.get_flag(CPUFlag::H), true);
    }

    #[test]
    fn instruction_res_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b01000000);

        cpu.res_register(6, &RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0);
    }

    #[test]
    fn instruction_res_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01000000);

        cpu.res_bi_register_ptr(6, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0);
    }

    #[test]
    fn instruction_set_register() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b01000000);

        cpu.set_register(0, &RegisterIdentifier::A);

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b01000001);
    }

    #[test]
    fn instruction_set_bi_register_ptr() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::HL, 0xC023);
        cpu.memory_bus.borrow_mut().write_8bit(0xC023, 0b01000000);

        cpu.set_bi_register_ptr(0, &BiRegisterIdentifier::HL);

        assert_eq!(cpu.memory_bus.borrow().read_8bit(0xC023), 0b01000001);
    }
}

