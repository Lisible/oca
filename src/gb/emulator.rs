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
use gb::gpu::gpu::*;

use gb::event::event::Event;
use gb::memory::memory_bus::MemoryBus;
use gb::display::display::Display;

use gb::debugger::debugger::Debugger;
use gb::debugger::error::Error;

use std::rc::Rc;
use std::cell::RefCell;
use std::io;



pub struct Emulator {
    cpu: CPU,
    gpu: GPU,
    memory_bus: Rc<RefCell<MemoryBus>>
}

impl Emulator {
    pub fn new(display: Box<Display>) -> Emulator {
        let memory_bus = Rc::new(RefCell::new(MemoryBus::new()));
        let cpu = CPU::new(memory_bus.clone());
        let gpu = GPU::new(memory_bus.clone(), display);

        Emulator {
            cpu,
            gpu,
            memory_bus,
        }
    }

    pub fn start(&mut self, rom_file: String) {
        self.initialize(&rom_file);
    }

    pub fn update(&mut self) {
        const MAX_CYCLES: u32 = 69905;
        let mut cycles_this_update = 0;

        while cycles_this_update < MAX_CYCLES {
            let cycles = self.cpu.step();
            cycles_this_update += cycles;
            self.gpu.step(cycles);
            self.cpu.run_interrupts();
        }
    }

    pub fn render(&mut self) {
        self.gpu.render();
    }

    pub fn handle_event(&mut self, event: Event) {
        println!("Emulator received event: {}", event);
    }

    pub fn start_debug(&mut self, rom_file: String) {
        self.initialize(&rom_file);
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

    fn initialize(&mut self, rom_file: &str) {
        let rom_data = self.load_rom_file(rom_file);
        self.memory_bus.borrow_mut().load_rom(rom_data);
        self.cpu.initialize();
    }

    fn load_rom_file(&self, rom_file: &str) -> [u8; 0x8000] {
        use std::fs;

        let contents = fs::read(rom_file).unwrap();
        let mut buffer = [0; 0x8000];
        let contents = &contents[..buffer.len()];
        buffer.copy_from_slice(contents);

        buffer
    }

    pub fn step(&mut self, step_count: i32) {
        for _ in 0..step_count {
            let cycles = self.cpu.step();
            self.gpu.step(cycles);
            self.cpu.run_interrupts();
        }
    }

    pub fn dump_cpu_state(&self) -> CPUState {
        self.cpu.dump_state()
    }

    pub fn dump_io_register(&self, address: u16) ->  u8 {
        self.memory_bus.borrow().dump_io_register(address)
    }
}