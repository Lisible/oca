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

use gb::cpu::cpu::*;
use gb::memory::memory_bus::MemoryBus;
use gb::debugger::debugger::Debugger;
use gb::debugger::error::Error;

use std::rc::Rc;
use std::cell::RefCell;
use std::io;


pub struct Emulator {
    cpu: CPU,
    memory_bus: Rc<RefCell<MemoryBus>>
}

impl Emulator {
    pub fn new() -> Emulator {
        let memory_bus = Rc::new(RefCell::new(MemoryBus::new()));
        let cpu = CPU::new(memory_bus.clone());

        Emulator {
            cpu,
            memory_bus,
        }
    }

    pub fn start(&mut self) {
        self.cpu.initialize();
        loop {
            self.cpu.step();
        }
    }

    pub fn start_debug(&mut self) {
        self.cpu.initialize();
        let mut debugger = Debugger::new();

        loop {
            let mut input = String::new();
            io::stdin().read_line(&mut input)
                .expect("Failed to read line");

             if let Err(Error::UnknownCommand) = debugger.run_command(input, self) {
                 println!("Unknown command")
             }
            println!();
        }
    }

    pub fn step(&mut self, step_count: i32) {
        for i in 0..step_count {
            self.cpu.step();
        }
    }

    pub fn dump_cpu_state(&self) -> CPUState {
        self.cpu.dump_state()
    }
}