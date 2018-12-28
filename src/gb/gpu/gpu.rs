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
    scanline_counter: i32
}

impl GPU {
    pub fn new(memory_bus: Rc<RefCell<MemoryBus>>, display: Box<Display>) -> GPU {
        GPU {
            memory_bus,
            display,
            scanline_counter: 0
        }
    }

    pub fn step(&mut self, cycles: u32) {
        self.update_lcd_status();

        if self.is_lcd_enabled() {
            self.scanline_counter = self.scanline_counter.wrapping_sub(cycles as i32);
        } else {;
            return;
        }

        if self.scanline_counter <= 0 {
            let line = self.memory_bus.borrow().read_8bit(ADDRESS_LY);
            let current_line = line.wrapping_add(1);
            self.memory_bus.borrow_mut().write_8bit(ADDRESS_LY, current_line);

            self.scanline_counter = 456;

            if current_line == 144 {
                const ADDRESS_INTERRUPT_FLAGS: u16 = 0xFF0F;
                let interrupt_flags = self.memory_bus.borrow().read_8bit(ADDRESS_INTERRUPT_FLAGS);
                let interrupt_flags = interrupt_flags | 1;
                self.memory_bus.borrow_mut().write_8bit(ADDRESS_INTERRUPT_FLAGS, interrupt_flags);
            }
            else if current_line > 153 {
                self.memory_bus.borrow_mut().write_8bit(ADDRESS_LY, 0);
            }
            else if current_line < 144 {
                self.render_scanline();
            }
        }
    }

    pub fn render(&mut self) {
        self.display.render();
    }


    fn update_lcd_status(&mut self) {
        let mut lcd_status = self.memory_bus.borrow().read_8bit(ADDRESS_LCD_STAT);
        if !self.is_lcd_enabled() {
            self.scanline_counter = 456;
            self.memory_bus.borrow_mut().write_8bit(ADDRESS_LY, 0);
            lcd_status &= 252;
            lcd_status |= 1;
            self.memory_bus.borrow_mut().write_8bit(ADDRESS_LCD_STAT, lcd_status);
            return;
        }

        let current_line = self.memory_bus.borrow().read_8bit(ADDRESS_LY);
        let current_mode = lcd_status & 0x3;

        let mut mode = 0;
        let mut request_interrupt = false;

        if current_line >= 144 {
            mode = 1;
            lcd_status = lcd_status | 1;
            lcd_status = lcd_status & !(1 << 1);
            request_interrupt = Binary::is_bit_set(&lcd_status, 4);
        } else {
            let mode2bounds = 456-80;
            let mode3bounds = mode2bounds - 172;

            if self.scanline_counter >= mode2bounds {
                mode = 2;
                lcd_status = lcd_status | (1 << 1);
                lcd_status = lcd_status & !1;
                request_interrupt = Binary::is_bit_set(&lcd_status, 5);
            } else if self.scanline_counter >= mode3bounds {
                mode = 3;
                lcd_status = lcd_status | (1 << 1);
                lcd_status = lcd_status | 1;
            } else {
                mode = 0;
                lcd_status = lcd_status & !(1 << 1);
                lcd_status = lcd_status & !1;
                request_interrupt = Binary::is_bit_set(&lcd_status, 3);
            }
        }

        if request_interrupt && (mode != current_mode) {
            const ADDRESS_INTERRUPT_FLAGS: u16 = 0xFF0F;
            let interrupt_flags = self.memory_bus.borrow().read_8bit(ADDRESS_INTERRUPT_FLAGS);
            let interrupt_flags = interrupt_flags | (1 << 1);
            self.memory_bus.borrow_mut().write_8bit(ADDRESS_INTERRUPT_FLAGS, interrupt_flags);
        }

        let ly = self.memory_bus.borrow().read_8bit(ADDRESS_LY);
        if ly == self.memory_bus.borrow().read_8bit(ADDRESS_LYC) {
            lcd_status = lcd_status | (1 << 2);
            if Binary::is_bit_set(&lcd_status, 6) {
                const ADDRESS_INTERRUPT_FLAGS: u16 = 0xFF0F;
                let interrupt_flags = self.memory_bus.borrow().read_8bit(ADDRESS_INTERRUPT_FLAGS);
                let interrupt_flags = interrupt_flags | (1 << 1);
                self.memory_bus.borrow_mut().write_8bit(ADDRESS_INTERRUPT_FLAGS, interrupt_flags);
            }
        }
        else {
            lcd_status = lcd_status & !(1 << 2);
        }

        self.memory_bus.borrow_mut().write_8bit(ADDRESS_LCD_STAT, lcd_status);
    }

    fn is_lcd_enabled(&mut self) -> bool {
        Binary::is_bit_set(&self.memory_bus.borrow().read_8bit(ADDRESS_LCD_CONTROL), 7)
    }

    pub fn render_scanline(&mut self) {
        const BIT_BG_AND_WINDOW_DISPLAY: u8 = 0;
        const BIT_OBJ_DISPLAY: u8 = 1;
        const BIT_LCD_DISPLAY: u8 = 7;

        let lcd_control = self.memory_bus.borrow().read_8bit(ADDRESS_LCD_CONTROL);

        // If the LCD isn't enable we don't draw
        if !Binary::is_bit_set(&lcd_control, BIT_LCD_DISPLAY) {
            return;
        }

        if Binary::is_bit_set(&lcd_control, BIT_BG_AND_WINDOW_DISPLAY) {
            self.render_tiles(&lcd_control);
        }

        if Binary::is_bit_set(&lcd_control, BIT_OBJ_DISPLAY) {
            self.render_objects();
        }
    }

    fn render_tiles(&mut self, lcd_control: &u8) {
        const BIT_BG_TILE_MAP_DISPLAY_SELECT: u8 = 3;
        const BIT_BG_AND_WINDOW_TILE_DATA_SELECT: u8 = 4;
        const BIT_WINDOW_DISPLAY: u8 = 5;
        const BIT_WINDOW_TILE_MAP_DISPLAY_SELECT: u8 = 6;

        let scroll_x = self.memory_bus.borrow().read_8bit(ADDRESS_SCROLL_X);
        let scroll_y = self.memory_bus.borrow().read_8bit(ADDRESS_SCROLL_Y);
        let window_x = self.memory_bus.borrow().read_8bit(ADDRESS_WINDOW_X).wrapping_sub(7);
        let window_y = self.memory_bus.borrow().read_8bit(ADDRESS_WINDOW_Y);
        let window_used = Binary::is_bit_set(lcd_control, BIT_WINDOW_DISPLAY) &&
                                window_y <= self.memory_bus.borrow().read_8bit(ADDRESS_LY);

        // Tile data
        let tile_data_memory_address =
            if Binary::is_bit_set(lcd_control, BIT_BG_AND_WINDOW_TILE_DATA_SELECT) {
                0x8000
            } else {
                0x8800
            };

        let background_memory_address= if !window_used {
            if Binary::is_bit_set(lcd_control, BIT_BG_TILE_MAP_DISPLAY_SELECT) {
                0x9C00
            } else {
                0x9800
            }
        } else {
            if Binary::is_bit_set(lcd_control, BIT_WINDOW_TILE_MAP_DISPLAY_SELECT) {
                0x9C00
            } else {
                0x9800
            }
        };

        let y_pos = if !window_used {
            scroll_y + self.memory_bus.borrow().read_8bit(ADDRESS_LY)
        } else {
            self.memory_bus.borrow().read_8bit(ADDRESS_LY) - window_y
        };

        let tile_row = ((y_pos/8) as u16) *32u16;

        for pixel in 0..160 {
            let mut x_pos = pixel + scroll_x;
            if window_used {
                if pixel >= window_x {
                    x_pos = pixel - window_x;
                }
            }

            let tile_col = (x_pos / 8) as u16;

            let tile_address = background_memory_address + tile_row + tile_col;
            let tile_number: i16 = if tile_data_memory_address == 0x8000 {
                self.memory_bus.borrow().read_8bit(tile_address) as i16
            } else {
                self.memory_bus.borrow().read_8bit_signed(tile_address) as i16
            };

            let mut tile_location = tile_data_memory_address as u16;
            if tile_data_memory_address == 0x8000 {
                tile_location = (tile_location as i32 + (tile_number * 16) as i32) as u16;
            } else {
                tile_location = (tile_location as i32 + ((tile_number + 128) * 16) as i32) as u16;
            }

            let line = (y_pos % 8)*2;
            let pixel_data = tile_location + line as u16;
            let data_1 = self.memory_bus.borrow().read_8bit(pixel_data);
            let data_2 = self.memory_bus.borrow().read_8bit(pixel_data + 1u16);


            let color_bit = -((x_pos % 8) as i32 - 7);
            let color_number = (Binary::read_bit(&data_2, color_bit as u8) << 1) |
                                    (Binary::read_bit(&data_1, color_bit as u8));

            let color = self.get_color(color_number, ADDRESS_PALETTE);
            let mut red = 0;
            let mut green = 0;
            let mut blue = 0;

            match color {
                NESColor::White => {red = 255; green = 255; blue = 255;},
                NESColor::LightGray => {red = 0xCC; green = 0xCC; blue = 0xCC;},
                NESColor::DarkGray => {red = 0x77; green = 0x77; blue = 0x77;},
                NESColor::Black => {red = 0; green = 0; blue = 0;},
            }

            let final_y = self.memory_bus.borrow().read_8bit(ADDRESS_LY);
            println!("{}", final_y);

            self.display.draw_pixel(pixel, final_y, Color::from([red, green, blue]));
        }
    }

    fn render_objects(&mut self) {
    }

    fn get_color(&mut self, color_number: u8, palette_address: u16) -> NESColor {
        let palette = self.memory_bus.borrow().read_8bit(palette_address);

        let (hi, lo) = match color_number {
            0 => (1u8, 0u8),
            1 => (3u8, 2u8),
            2 => (5u8, 4u8),
            3 => (7u8, 6u8),
            _ => panic!("Unknown color bit")
        };

        let color = (Binary::read_bit(&palette, hi) << 1) | Binary::read_bit(&palette, lo);
        let res = match color {
            0 => NESColor::White,
            1 => NESColor::LightGray,
            2 => NESColor::DarkGray,
            3 => NESColor::Black,
            _ => panic!("Unknown color")
        };

        res
    }
}