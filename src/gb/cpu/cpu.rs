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
            stopped: false
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
            //0x2A => cycles += self.ldi_register_bi_register_ptr(&RegisterIdentifier::A, &BiRegisterIdentifier::HL),
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
            // LD(HL-),A
            //0x32 => cycles += self.ldd_bi_register_ptr_register(&BiRegisterIdentifier::HL, &RegisterIdentifier::A),
            // INC SP
            //0x33 => cycles += self.inc_sp(),
            // INC (HL)
            //0x34 => cycles += self.inc_bi_register_ptr(&BiRegisterIdentifier::HL),
            // DEC (HL)
            //0x35 => cycles += self.dec_bi_register_ptr(&BiRegisterIdentifier::HL),
            // LD (HL),d8
            //0x36 => cycles += self.ld_bi_register_ptr_d8(&BiRegisterIdentifier::HL),
            // SCF
            //0x37 => cycles += self.scf(),
            // JR C,r8
            0x38 => cycles += self.jr_flag_r8(CPUFlag::C, true),
            // ADD HL,SP
            //0x39 => cycles += self.add_bi_register_sp(&BiRegisterIdentifier::HL),
            // LD A,(HL-)
            //0x3A => cycles += self.ldd_register_bi_register_ptr(&BiRegisterIdentifier::HL),
            // DEC SP
            //0x3B => cycles += self.dec_sp(),
            // INC A
            0x3C => cycles += self.inc_register(&RegisterIdentifier::A),
            // DEC A
            0x3D => cycles += self.dec_register(&RegisterIdentifier::A),
            // LD A,d8
            0x3E => cycles += self.ld_register_d8(&RegisterIdentifier::A),
            // CCF
            //0x3F => cycles += self.ccf(),

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

    fn inc_bi_register(&mut self, register_identifier: &BiRegisterIdentifier) -> u32 { // TODO update cf spec
        self.bi_registers.get_mut(&register_identifier).unwrap().increment(1);
        8
    }

    fn dec_bi_register(&mut self, register_identifier: &BiRegisterIdentifier) -> u32 { // TODO update cf spec
        self.bi_registers.get_mut(&register_identifier).unwrap().decrement(1);
        8
    }

    fn inc_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 { // TODO update cf spec
        self.registers[&register_identifier].borrow_mut().increment(1);
        4
    }

    fn dec_register(&mut self, register_identifier: &RegisterIdentifier) -> u32 { // TODO update cf spec
        self.registers[&register_identifier].borrow_mut().decrement(1);
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
        let lhs;
        let rhs;

        {
            let lhs_register = self.bi_registers.get(&first_register_identifier).unwrap();
            lhs = lhs_register.read();
            rhs = self.bi_registers[&second_register_identifier].read();
        }

        {
            if (lhs & rhs) & (1 << 15) != 0 {
                self.set_flag(CPUFlag::C);
            }

            if (lhs & rhs) & (1 << 11) != 0 {
                self.set_flag(CPUFlag::H);
            }
        }

        {
            let lhs_register = self.bi_registers.get_mut(&first_register_identifier).unwrap();
            lhs_register.write(lhs + rhs);
        }

        self.unset_flag(CPUFlag::N);

        8
    }

    fn ld_register_bi_register(&mut self,
                               register_identifier: &RegisterIdentifier,
                               bi_register_identifier: &BiRegisterIdentifier) -> u32 {
        let mut lhs_register = self.registers[&register_identifier].borrow_mut();
        let rhs_value = self.bi_registers[&bi_register_identifier].read();
        lhs_register.write((rhs_value >> 8) as u8); // TODO ??

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
        let a = self.read_register(&RegisterIdentifier::A);
        self.write_register(&RegisterIdentifier::A, !a);
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
    fn instruction_ld_register_bi_register() {
        let mut cpu = create_cpu();
        cpu.write_bi_register(&BiRegisterIdentifier::BC, 0x05FB);

        cpu.ld_register_bi_register(&RegisterIdentifier::D, &BiRegisterIdentifier::BC);
        assert_eq!(cpu.read_register(&RegisterIdentifier::D), 0x05);
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

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0x180);
        assert_eq!(cpu.get_flag(CPUFlag::C), true)
    }

    #[test]
    fn instruction_cpl() {
        let mut cpu = create_cpu();
        cpu.write_register(&RegisterIdentifier::A, 0b01010101);

        cpu.cpl();

        assert_eq!(cpu.read_register(&RegisterIdentifier::A), 0b10101010);
    }
}

