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

///
/// Variant type for the CPU instructions callback functions
///
enum CPUInstructionCallback
{
    VoidCallback(fn(&mut CPU)),
    U8Callback(fn(&mut CPU, u8)),
    I8Callback(fn(&mut CPU, i8)),
    U16Callback(fn(&mut CPU, u16)),
    I16Callback(fn(&mut CPU, i16)),
}

pub enum CPUInstruction {
    NoInstruction,
    Instruction(CPUInstructionDefinition)
}

///
/// Represents a CPU instruction
///
pub struct CPUInstructionDefinition {
    ///
    /// The disassembly name of the instruction
    ///
    disassembly: String,
    ///
    /// The length of the operand in bytes
    ///
    operand_length: u8,
    ///
    /// The callback function of the instruction
    ///
    function: CPUInstructionCallback
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
        self.cpu_instructions = Some(self.create_instruction_array());
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


    fn instruction_nop(&mut self) {}
    fn instruction_ld_bc_d16(&mut self, arg: u16) {}
    fn instruction_ld_bcp_a(&mut self) {}
    fn instruction_inc_bc(&mut self) {}
    fn instruction_inc_b(&mut self) {}
    fn instruction_dec_b(&mut self) {}
    fn instruction_ld_b_d8(&mut self, arg: u8) {}
    fn instruction_rlca(&mut self) {}
    fn instruction_ld_a16p_sp(&mut self, arg: u16) {}
    fn instruction_add_hl_bc(&mut self) {}
    fn instruction_ld_a_bcp(&mut self) {}
    fn instruction_dec_bc(&mut self) {}
    fn instruction_inc_c(&mut self) {}
    fn instruction_dec_c(&mut self) {}
    fn instruction_ld_c_d8(&mut self, arg: u8) {}
    fn instruction_rrca(&mut self) {}
    fn instruction_stop_0(&mut self) {}
    fn instruction_ld_de_d16(&mut self, arg: u16) {}
    fn instruction_ld_dep_a(&mut self) {}
    fn instruction_inc_de(&mut self) {}
    fn instruction_inc_d(&mut self) {}
    fn instruction_dec_d(&mut self) {}
    fn instruction_ld_d_d8(&mut self, arg: u8) {}
    fn instruction_rla(&mut self) {}
    fn instruction_jr_r8(&mut self, arg: i8) {}
    fn instruction_add_hl_de(&mut self) {}
    fn instruction_ld_a_dep(&mut self) {}
    fn instruction_dec_de(&mut self) {}
    fn instruction_inc_e(&mut self) {}
    fn instruction_dec_e(&mut self) {}
    fn instruction_ld_e_d8(&mut self, arg: u8) {}
    fn instruction_rra(&mut self) {}
    fn instruction_jr_nz_r8(&mut self, arg: i8) {}
    fn instruction_ld_hl_d16(&mut self, arg: u16) {}
    fn instruction_ld_hlpp_a(&mut self) {}
    fn instruction_inc_hl(&mut self) {}
    fn instruction_inc_h(&mut self) {}
    fn instruction_dec_h(&mut self) {}
    fn instruction_ld_h_d8(&mut self, arg: u8) {}
    fn instruction_daa(&mut self) {}
    fn instruction_jr_z_r8(&mut self, arg: i8) {}
    fn instruction_add_hl_hl(&mut self) {}
    fn instruction_ld_a_hlpp(&mut self) {}
    fn instruction_dec_hl(&mut self) {}
    fn instruction_inc_l(&mut self) {}
    fn instruction_dec_l(&mut self) {}
    fn instruction_ld_l_d8(&mut self, arg: u8) {}
    fn instruction_cpl(&mut self) {}
    fn instruction_jr_nc_r8(&mut self, arg: i8) {}
    fn instruction_ld_sp_d16(&mut self, arg: u16) {}
    fn instruction_ld_hlmp_a(&mut self) {}
    fn instruction_inc_sp(&mut self) {}
    fn instruction_inc_hlp(&mut self) {}
    fn instruction_dec_hlp(&mut self) {}
    fn instruction_ld_hlp_d8(&mut self, arg: u8) {}
    fn instruction_scf(&mut self) {}
    fn instruction_jr_c_r8(&mut self, arg: i8) {}
    fn instruction_add_hl_sp(&mut self) {}
    fn instruction_ld_a_hlmp(&mut self) {}
    fn instruction_dec_sp(&mut self) {}
    fn instruction_inc_a(&mut self) {}
    fn instruction_dec_a(&mut self) {}
    fn instruction_ld_a_d8(&mut self, arg: u8) {}
    fn instruction_ccf(&mut self) {}
    fn instruction_ld_b_b(&mut self) {}
    fn instruction_ld_b_c(&mut self) {}
    fn instruction_ld_b_d(&mut self) {}
    fn instruction_ld_b_e(&mut self) {}
    fn instruction_ld_b_h(&mut self) {}
    fn instruction_ld_b_l(&mut self) {}
    fn instruction_ld_b_hlp(&mut self) {}
    fn instruction_ld_b_a(&mut self) {}
    fn instruction_ld_c_b(&mut self) {}
    fn instruction_ld_c_c(&mut self) {}
    fn instruction_ld_c_d(&mut self) {}
    fn instruction_ld_c_e(&mut self) {}
    fn instruction_ld_c_h(&mut self) {}
    fn instruction_ld_c_l(&mut self) {}
    fn instruction_ld_c_hlp(&mut self) {}
    fn instruction_ld_c_a(&mut self) {}
    fn instruction_ld_d_b(&mut self) {}
    fn instruction_ld_d_c(&mut self) {}
    fn instruction_ld_d_d(&mut self) {}
    fn instruction_ld_d_e(&mut self) {}
    fn instruction_ld_d_h(&mut self) {}
    fn instruction_ld_d_l(&mut self) {}
    fn instruction_ld_d_hlp(&mut self) {}
    fn instruction_ld_d_a(&mut self) {}
    fn instruction_ld_e_b(&mut self) {}
    fn instruction_ld_e_c(&mut self) {}
    fn instruction_ld_e_d(&mut self) {}
    fn instruction_ld_e_e(&mut self) {}
    fn instruction_ld_e_h(&mut self) {}
    fn instruction_ld_e_l(&mut self) {}
    fn instruction_ld_e_hlp(&mut self) {}
    fn instruction_ld_e_a(&mut self) {}
    fn instruction_ld_h_b(&mut self) {}
    fn instruction_ld_h_c(&mut self) {}
    fn instruction_ld_h_d(&mut self) {}
    fn instruction_ld_h_e(&mut self) {}
    fn instruction_ld_h_h(&mut self) {}
    fn instruction_ld_h_l(&mut self) {}
    fn instruction_ld_h_hlp(&mut self) {}
    fn instruction_ld_h_a(&mut self) {}
    fn instruction_ld_l_b(&mut self) {}
    fn instruction_ld_l_c(&mut self) {}
    fn instruction_ld_l_d(&mut self) {}
    fn instruction_ld_l_e(&mut self) {}
    fn instruction_ld_l_h(&mut self) {}
    fn instruction_ld_l_l(&mut self) {}
    fn instruction_ld_l_hlp(&mut self) {}
    fn instruction_ld_l_a(&mut self) {}
    fn instruction_ld_hlp_b(&mut self) {}
    fn instruction_ld_hlp_c(&mut self) {}
    fn instruction_ld_hlp_d(&mut self) {}
    fn instruction_ld_hlp_e(&mut self) {}
    fn instruction_ld_hlp_h(&mut self) {}
    fn instruction_ld_hlp_l(&mut self) {}
    fn instruction_halt(&mut self) {}
    fn instruction_ld_hlp_a(&mut self) {}
    fn instruction_ld_a_b(&mut self) {}
    fn instruction_ld_a_c(&mut self) {}
    fn instruction_ld_a_d(&mut self) {}
    fn instruction_ld_a_e(&mut self) {}
    fn instruction_ld_a_h(&mut self) {}
    fn instruction_ld_a_l(&mut self) {}
    fn instruction_ld_a_hlp(&mut self) {}
    fn instruction_ld_a_a(&mut self) {}
    fn instruction_add_a_b(&mut self) {}
    fn instruction_add_a_c(&mut self) {}
    fn instruction_add_a_d(&mut self) {}
    fn instruction_add_a_e(&mut self) {}
    fn instruction_add_a_h(&mut self) {}
    fn instruction_add_a_l(&mut self) {}
    fn instruction_add_a_hlp(&mut self) {}
    fn instruction_add_a_a(&mut self) {}
    fn instruction_adc_a_b(&mut self) {}
    fn instruction_adc_a_c(&mut self) {}
    fn instruction_adc_a_d(&mut self) {}
    fn instruction_adc_a_e(&mut self) {}
    fn instruction_adc_a_h(&mut self) {}
    fn instruction_adc_a_l(&mut self) {}
    fn instruction_adc_a_hlp(&mut self) {}
    fn instruction_adc_a_a(&mut self) {}
    fn instruction_sub_b(&mut self) {}
    fn instruction_sub_c(&mut self) {}
    fn instruction_sub_d(&mut self) {}
    fn instruction_sub_e(&mut self) {}
    fn instruction_sub_h(&mut self) {}
    fn instruction_sub_l(&mut self) {}
    fn instruction_sub_hlp(&mut self) {}
    fn instruction_sub_a(&mut self) {}
    fn instruction_sbc_a_b(&mut self) {}
    fn instruction_sbc_a_c(&mut self) {}
    fn instruction_sbc_a_d(&mut self) {}
    fn instruction_sbc_a_e(&mut self) {}
    fn instruction_sbc_a_h(&mut self) {}
    fn instruction_sbc_a_l(&mut self) {}
    fn instruction_sbc_a_hlp(&mut self) {}
    fn instruction_sbc_a_a(&mut self) {}
    fn instruction_and_b(&mut self) {}
    fn instruction_and_c(&mut self) {}
    fn instruction_and_d(&mut self) {}
    fn instruction_and_e(&mut self) {}
    fn instruction_and_h(&mut self) {}
    fn instruction_and_l(&mut self) {}
    fn instruction_and_hlp(&mut self) {}
    fn instruction_and_a(&mut self) {}
    fn instruction_xor_b(&mut self) {}
    fn instruction_xor_c(&mut self) {}
    fn instruction_xor_d(&mut self) {}
    fn instruction_xor_e(&mut self) {}
    fn instruction_xor_h(&mut self) {}
    fn instruction_xor_l(&mut self) {}
    fn instruction_xor_hlp(&mut self) {}
    fn instruction_xor_a(&mut self) {}
    fn instruction_or_b(&mut self) {}
    fn instruction_or_c(&mut self) {}
    fn instruction_or_d(&mut self) {}
    fn instruction_or_e(&mut self) {}
    fn instruction_or_h(&mut self) {}
    fn instruction_or_l(&mut self) {}
    fn instruction_or_hlp(&mut self) {}
    fn instruction_or_a(&mut self) {}
    fn instruction_cp_b(&mut self) {}
    fn instruction_cp_c(&mut self) {}
    fn instruction_cp_d(&mut self) {}
    fn instruction_cp_e(&mut self) {}
    fn instruction_cp_h(&mut self) {}
    fn instruction_cp_l(&mut self) {}
    fn instruction_cp_hlp(&mut self) {}
    fn instruction_cp_a(&mut self) {}
    fn instruction_ret_nz(&mut self) {}
    fn instruction_pop_bc(&mut self) {}
    fn instruction_jp_nz_a16(&mut self, arg: u16) {}
    fn instruction_jp_a16(&mut self, arg: u16) {}
    fn instruction_call_nz_a16(&mut self, arg: u16) {}
    fn instruction_push_bc(&mut self) {}
    fn instruction_add_a_d8(&mut self, arg: u8) {}
    fn instruction_rst_00h(&mut self) {}
    fn instruction_ret_z(&mut self) {}
    fn instruction_ret(&mut self) {}
    fn instruction_jp_z_a16(&mut self, arg: u16) {}
    fn instruction_prefix_cb(&mut self) {}
    fn instruction_call_z_a16(&mut self, arg: u16) {}
    fn instruction_call_a16(&mut self, arg: u16) {}
    fn instruction_adc_a_d8(&mut self, arg: u8) {}
    fn instruction_rst_08h(&mut self) {}
    fn instruction_ret_nc(&mut self) {}
    fn instruction_pop_de(&mut self) {}
    fn instruction_jp_nc_a16(&mut self, arg: u16) {}
    fn instruction_call_nc_a16(&mut self, arg: u16) {}
    fn instruction_push_de(&mut self) {}
    fn instruction_sub_d8(&mut self, arg: u8) {}
    fn instruction_rst_10h(&mut self) {}
    fn instruction_ret_c(&mut self) {}
    fn instruction_reti(&mut self) {}
    fn instruction_jp_c_a16(&mut self, arg: u16) {}
    fn instruction_call_c_a16(&mut self, arg: u16) {}
    fn instruction_sbc_a_d8(&mut self, arg: u8) {}
    fn instruction_rst_18h(&mut self) {}
    fn instruction_ldh_a8p_a(&mut self, arg: u8) {}
    fn instruction_pop_hl(&mut self) {}
    fn instruction_ld_cp_a(&mut self) {}
    fn instruction_push_hl(&mut self) {}
    fn instruction_and_d8(&mut self, arg: u8) {}
    fn instruction_rst_20h(&mut self) {}
    fn instruction_add_sp_r8(&mut self, arg: i8) {}
    fn instruction_jp_hlp(&mut self) {}
    fn instruction_ld_a16p_a(&mut self, arg: u16) {}
    fn instruction_xor_d8(&mut self, arg: u8) {}
    fn instruction_rst_28h(&mut self) {}
    fn instruction_ldh_a_a8p(&mut self, arg: u8) {}
    fn instruction_pop_af(&mut self) {}
    fn instruction_ld_a_cp(&mut self) {}
    fn instruction_di(&mut self) {}
    fn instruction_push_af(&mut self) {}
    fn instruction_or_d8(&mut self, arg: u8) {}
    fn instruction_rst_30h(&mut self) {}
    fn instruction_ld_hl_sppr8(&mut self, arg: i8) {}
    fn instruction_ld_sp_hl(&mut self) {}
    fn instruction_ld_a_a16p(&mut self, arg: u16) {}
    fn instruction_ei(&mut self) {}
    fn instruction_cp_d8(&mut self, arg: u8) {}
    fn instruction_rst_38h(&mut self) {}

    fn create_instruction_array() -> [CPUInstruction;256] {
        [
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("NOP"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_nop),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD BC, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_ld_bc_d16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (BC), A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_bcp_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC BC"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_bc),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ld_b_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RLCA"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rlca),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD ({:#X}), SP"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_ld_a16p_sp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD HL, BC"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_hl_bc),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, (BC)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_bcp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC BC"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_bc),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ld_c_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RRCA"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rrca),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("STOP 0"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_stop_0),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD DE, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_ld_de_d16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (DE), A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_dep_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC DE"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_de),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ld_d_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RLA"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rla),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JR {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::I8Callback(CPU::instruction_jr_r8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD HL, DE"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_hl_de),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, (DE)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_dep),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC DE"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_de),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ld_e_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RRA"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rra),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JR NZ, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::I8Callback(CPU::instruction_jr_nz_r8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD HL, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_ld_hl_d16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL+), A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlpp_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC HL"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_hl),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ld_h_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DAA"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_daa),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JR Z, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::I8Callback(CPU::instruction_jr_z_r8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD HL, HL"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_hl_hl),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, (HL+)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_hlpp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC HL"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_hl),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ld_l_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CPL"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cpl),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JR NC, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::I8Callback(CPU::instruction_jr_nc_r8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD SP, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_ld_sp_d16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL-), A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlmp_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC SP"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_sp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL), {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ld_hlp_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SCF"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_scf),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JR C, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::I8Callback(CPU::instruction_jr_c_r8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD HL, SP"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_hl_sp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, (HL-)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_hlmp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC SP"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_sp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("INC A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_inc_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DEC A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_dec_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ld_a_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CCF"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ccf),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_b_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_b_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_b_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_b_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_b_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_b_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_b_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD B, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_b_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_c_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_c_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_c_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_c_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_c_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_c_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_c_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD C, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_c_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_d_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_d_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_d_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_d_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_d_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_d_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_d_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD D, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_d_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_e_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_e_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_e_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_e_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_e_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_e_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_e_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD E, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_e_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_h_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_h_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_h_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_h_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_h_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_h_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_h_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD H, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_h_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_l_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_l_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_l_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_l_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_l_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_l_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_l_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD L, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_l_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL), B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlp_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL), C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlp_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL), D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlp_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL), E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlp_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL), H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlp_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL), L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlp_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("HALT"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_halt),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (HL), A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_hlp_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_a_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_a_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_a_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_a_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_a_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_a_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_a_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_add_a_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_adc_a_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_adc_a_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_adc_a_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_adc_a_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_adc_a_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_adc_a_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_adc_a_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_adc_a_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sub_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sub_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sub_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sub_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sub_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sub_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sub_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sub_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sbc_a_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sbc_a_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sbc_a_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sbc_a_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sbc_a_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sbc_a_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sbc_a_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_sbc_a_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_and_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_and_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_and_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_and_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_and_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_and_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_and_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_and_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_xor_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_xor_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_xor_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_xor_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_xor_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_xor_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_xor_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_xor_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_or_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_or_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_or_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_or_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_or_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_or_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_or_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_or_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP B"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cp_b),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cp_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP D"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cp_d),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP E"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cp_e),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cp_h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP L"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cp_l),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cp_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_cp_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RET NZ"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ret_nz),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("POP BC"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_pop_bc),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JP NZ, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_jp_nz_a16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JP {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_jp_a16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CALL NZ, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_call_nz_a16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("PUSH BC"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_push_bc),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD A, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_add_a_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RST 00H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rst_00h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RET Z"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ret_z),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RET"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ret),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JP Z, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_jp_z_a16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("PREFIX CB"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_prefix_cb),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CALL Z, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_call_z_a16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CALL {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_call_a16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADC A, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_adc_a_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RST 08H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rst_08h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RET NC"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ret_nc),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("POP DE"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_pop_de),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JP NC, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_jp_nc_a16),
            }),
            CPUInstruction::NoInstruction,
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CALL NC, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_call_nc_a16),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("PUSH DE"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_push_de),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SUB {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_sub_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RST 10H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rst_10h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RET C"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ret_c),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RETI"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_reti),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JP C, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_jp_c_a16),
            }),
            CPUInstruction::NoInstruction,
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CALL C, {:#X}"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_call_c_a16),
            }),
            CPUInstruction::NoInstruction,
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("SBC A, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_sbc_a_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RST 18H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rst_18h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LDH ({:#X}), A"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ldh_a8p_a),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("POP HL"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_pop_hl),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD (C), A"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_cp_a),
            }),
            CPUInstruction::NoInstruction,
            CPUInstruction::NoInstruction,
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("PUSH HL"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_push_hl),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("AND {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_and_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RST 20H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rst_20h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("ADD SP, {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::I8Callback(CPU::instruction_add_sp_r8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("JP (HL)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_jp_hlp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD ({:#X}), A"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_ld_a16p_a),
            }),
            CPUInstruction::NoInstruction,
            CPUInstruction::NoInstruction,
            CPUInstruction::NoInstruction,
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("XOR {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_xor_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RST 28H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rst_28h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LDH A, ({:#X})"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_ldh_a_a8p),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("POP AF"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_pop_af),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, (C)"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_a_cp),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("DI"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_di),
            }),
            CPUInstruction::NoInstruction,
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("PUSH AF"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_push_af),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("OR {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_or_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RST 30H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rst_30h),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD HL, SP+{:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::I8Callback(CPU::instruction_ld_hl_sppr8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD SP, HL"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ld_sp_hl),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("LD A, ({:#X})"),
                operand_length: 2,
                function: CPUInstructionCallback::U16Callback(CPU::instruction_ld_a_a16p),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("EI"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_ei),
            }),
            CPUInstruction::NoInstruction,
            CPUInstruction::NoInstruction,
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("CP {:#X}"),
                operand_length: 1,
                function: CPUInstructionCallback::U8Callback(CPU::instruction_cp_d8),
            }),
            CPUInstruction::Instruction(CPUInstructionDefinition {
                disassembly: String::from("RST 38H"),
                operand_length: 0,
                function: CPUInstructionCallback::VoidCallback(CPU::instruction_rst_38h),
            }),
        ]
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

