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

extern crate sdl2;

use gb::cpu::cpu::CPU;
use gb::memory::memory_bus::MemoryBus;
use gb::memory::cartridge::Cartridge;
use gb::memory::ram::Ram;
use gb::memory::oam::Oam;
use gb::memory::high_ram::HighRam;
use gb::memory::io::IO;

use std::rc::Rc;
use std::cell::RefCell;



pub struct Console {
    cpu: CPU,
    ram: Rc<RefCell<Ram>>,
    oam: Rc<RefCell<Oam>>,
    high_ram: Rc<RefCell<HighRam>>,
    io: Rc<RefCell<IO>>,
    cartridge: Rc<RefCell<Cartridge>>,
    memory_bus: Rc<RefCell<MemoryBus>>
}

impl Console {
    pub fn new() -> Console {
        let cartridge = Rc::new(RefCell::new(Cartridge::from_bytes([0;0x8000])));
        let ram = Rc::new(RefCell::new(Ram::new()));
        let oam = Rc::new(RefCell::new(Oam::new()));
        let high_ram = Rc::new(RefCell::new(HighRam::new()));
        let io = Rc::new(RefCell::new(IO::new()));
        let memory_bus = Rc::new(RefCell::new(MemoryBus::new(cartridge.clone(), ram.clone(), oam.clone(), high_ram.clone(), io.clone())));
        let cpu = CPU::new(memory_bus.clone());

        Console {
            cpu,
            ram,
            oam,
            high_ram,
            io,
            cartridge,
            memory_bus,
        }
    }

    pub fn start(&mut self) {
        use std::io;
        use std::thread::*;
        use std::time::*;

        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem.window("oca GameBoy Emulator", 800, 600).position_centered().build().unwrap();

        self.load_rom("/home/clements/Documents/t.gb");
        self.cpu.initialize();

        loop {
            self.cpu.step();

            //let mut l = String::new();
            //io::stdin().read_line(&mut l);
        }
    }

    pub fn load_rom(&mut self, file: &'static str) {
        use std::fs;

        let contents = fs::read(file)
            .expect("Error reading rom file");

        let mut array = [0; 0x8000];
        let contents = &contents[..array.len()];
        array.copy_from_slice(contents);
        self.cartridge.replace(Cartridge::from_bytes(array));
    }
}