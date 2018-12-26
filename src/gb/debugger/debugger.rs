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

use gb::emulator::Emulator;
use gb::debugger::error::Error;
use gb::debugger::opcode_disassembly;


use std::collections::HashSet;
use std::u16;

pub struct Debugger {
    breakpoints: HashSet<u16>
}

impl Debugger
{
    pub fn new() -> Debugger {
        Debugger {
            breakpoints: HashSet::new(),
        }
    }

    pub fn run_command(&mut self, command: String, emulator: &mut Emulator) -> Result<(), Error> {
        if command.len() <= 1 {
            return Ok(())
        }

        let split_command: Vec<&str> = command.split_whitespace().collect();
        let identifier: &str = split_command.get(0).unwrap();
        let arguments: Vec<&str> = split_command[1..split_command.len()].to_vec();

        match identifier {
            "print" => self.command_print(arguments),
            "step" => self.command_step(arguments, emulator),
            "breakpoint" => self.command_breakpoint(arguments),
            "run" => self.command_run(emulator),
            "print_cpu_state" => self.command_print_cpu_state(emulator),
            "print_io_register" => self.command_print_io_register(arguments, emulator),
            _ => return Err(Error::UnknownCommand)
        }

        Ok(())
    }

    fn command_print(&self, arguments: Vec<&str>) {
        let output = arguments.join(" ");
        println!("{}", output);
    }

    fn command_step(&self, arguments: Vec<&str>, emulator: &mut Emulator) {
        assert!(arguments.len() <= 1);

        let step_count = if arguments.len() == 0 {
            1
        } else {
            arguments[0].parse().unwrap()
        };

        emulator.step(step_count);

        let cpu_state = emulator.dump_cpu_state();
        println!("PC: 0x{:X}  (Opcode: 0x{:X} = {})", cpu_state.program_counter, cpu_state.opcode, opcode_disassembly::disassemble(cpu_state.opcode));
    }

    fn command_breakpoint(&mut self, arguments: Vec<&str>) {
        assert!(arguments.len() == 1);
        let line = arguments.get(0).unwrap();
        let line = line.trim_left_matches("0x");
        let line_number = u16::from_str_radix(line, 16).unwrap();

        if !self.breakpoints.contains(&line_number) {
            println!("Breakpoint set on line 0x{:X}", &line_number);
            self.breakpoints.insert(line_number);
        } else {
            println!("Breakpoint unset on line 0x{:X}", &line_number);
            self.breakpoints.remove(&line_number);
        }
    }

    fn command_run(&mut self, emulator: &mut Emulator) {
        loop {
            emulator.step(1);

            let cpu_state = emulator.dump_cpu_state();
            println!("PC = 0x{:X}", cpu_state.program_counter);
            if self.breakpoints.contains(&cpu_state.program_counter) {
                println!("Breakpoint encounted on line 0x{:X}", cpu_state.program_counter);
                break;
            }
        }
    }

    fn command_print_cpu_state(&self, emulator: &Emulator) {
        let cpu_state = emulator.dump_cpu_state();
        println!();
        println!("CPU State:");
        println!("A: 0x{:X}", cpu_state.a);
        println!("B: 0x{:X}", cpu_state.b);
        println!("C: 0x{:X}", cpu_state.c);
        println!("D: 0x{:X}", cpu_state.d);
        println!("E: 0x{:X}", cpu_state.e);
        println!("F: 0x{:X}", cpu_state.f);
        println!("H: 0x{:X}", cpu_state.h);
        println!("L: 0x{:X}", cpu_state.l);
        println!("PC: 0x{:X}  (Opcode: 0x{:X} = {})", cpu_state.program_counter, cpu_state.opcode, opcode_disassembly::disassemble(cpu_state.opcode));
        println!("SP: 0x{:X}", cpu_state.stack_pointer);
        println!();
    }

    fn command_print_io_register(&self, arguments: Vec<&str>, emulator: &Emulator) {
        assert!(arguments.len() == 1);
        let arg = arguments.get(0).unwrap();
        let arg = arg.trim_left_matches("0x");
        let address = u16::from_str_radix(arg, 16).unwrap();
        let data = emulator.dump_io_register(address);
        println!("0x{:X} = 0x{:X}", address, data);
    }


}