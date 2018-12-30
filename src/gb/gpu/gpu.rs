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

use std::rc::Rc;
use std::cell::RefCell;

use gb::memory::memory::*;
use gb::memory::memory_bus::MemoryBus;

use gb::display::display::Display;
use gb::display::color::Color;
use gb::gpu::nes_color::NESColor;

use gb::utils::binary::Binary;

const ADDRESS_LCD_CONTROL: u16 = 0xFF40;
const ADDRESS_LCD_STAT: u16 = 0xFF41;
const ADDRESS_SCROLL_X: u16 = 0xFF43;
const ADDRESS_SCROLL_Y: u16 = 0xFF42;
const ADDRESS_WINDOW_X: u16 = 0xFF4B;
const ADDRESS_WINDOW_Y: u16 = 0xFF4A;
const ADDRESS_LY: u16 = 0xFF44;
const ADDRESS_LYC: u16 = 0xFF45;
const ADDRESS_PALETTE: u16 = 0xFF47;

pub struct GPU {
    memory_bus: Rc<RefCell<MemoryBus>>,
    display: Box<Display>,
}

impl GPU {
    pub fn new(memory_bus: Rc<RefCell<MemoryBus>>, display: Box<Display>) -> GPU {
        GPU {
            memory_bus,
            display,
        }
    }

    pub fn step(&mut self, cycles: u32) {
        println!("GPU Step");
    }

    pub fn render(&mut self) {
        println!("GPU Rendering");
        self.display.render();
    }
}