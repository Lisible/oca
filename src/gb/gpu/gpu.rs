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

use gb::utils::bit_manipulation::BitManipulation;

use gb::memory::memory::*;
use gb::memory::memory_bus::MemoryBus;

use gb::display::display::Display;
use gb::display::color::Color;
use gb::gpu::nes_color::NESColor;


const ADDRESS_LCD_CONTROL: u16 = 0xFF40;
const ADDRESS_LCD_STAT: u16 = 0xFF41;
const ADDRESS_SCROLL_X: u16 = 0xFF43;
const ADDRESS_SCROLL_Y: u16 = 0xFF42;
const ADDRESS_WINDOW_X: u16 = 0xFF4B;
const ADDRESS_WINDOW_Y: u16 = 0xFF4A;
const ADDRESS_LY: u16 = 0xFF44;
const ADDRESS_LYC: u16 = 0xFF45;
const ADDRESS_PALETTE: u16 = 0xFF47;
const ADDRESS_BG_TILE_MAP_2: u16 = 0x9800;
const ADDRESS_BG_TILE_MAP_1: u16 = 0x9C00;
const ADDRESS_WINDOW_TILE_MAP_2: u16 = 0x9800;
const ADDRESS_WINDOW_TILE_MAP_1: u16 = 0x9C00;
const ADDRESS_TILE_DATA_2: u16 = 0x8800;
const ADDRESS_TILE_DATA_1: u16 = 0x8000;
const ADDRESS_SPRITE_DATA: u16 = 0x8000;
const ADDRESS_SPRITE_ATTRIBUTE_TABLE: u16 = 0xFE00;


pub struct GPU {
    memory_bus: Rc<RefCell<MemoryBus>>,
    display: Box<Display>,
    scanline_counter: i32,
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
            self.scanline_counter -= cycles as i32;
        } else {
            return;
        }
        if self.scanline_counter <= 0 {
            let line = self.memory_bus.borrow().read_8bit(ADDRESS_LY);
            let current_line = line.wrapping_add(1);
            self.memory_bus.borrow_mut().write_8bit(ADDRESS_LY, current_line);

            self.scanline_counter = 456;
            if current_line == 144 {
                self.memory_bus.borrow_mut().request_interrupt(0);
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

    fn render_scanline(&mut self) {
        let lcd_control = self.memory_bus.borrow().read_8bit(ADDRESS_LCD_CONTROL);

        if lcd_control.is_bit_set(0) {
            self.render_tiles();
        }

        if lcd_control.is_bit_set(1) {
            self.render_objects();
        }
    }

    fn render_tiles(&mut self) {
        let scroll_x = self.memory_bus.borrow().read_8bit(ADDRESS_SCROLL_X);
        let scroll_y = self.memory_bus.borrow().read_8bit(ADDRESS_SCROLL_Y);
        let window_x = self.memory_bus.borrow().read_8bit(ADDRESS_WINDOW_X).wrapping_sub(7);
        let window_y = self.memory_bus.borrow().read_8bit(ADDRESS_WINDOW_Y);
        let lcd_control = self.memory_bus.borrow().read_8bit(ADDRESS_LCD_CONTROL);



        let mut using_window = false;
        if lcd_control.is_bit_set(5) &&
            (window_y <= self.memory_bus.borrow().read_8bit(ADDRESS_LY)) {
            using_window = true;
        }

        let mut tile_data_address: u16;
        let mut unsigned_tile_identifiers = true;
        if lcd_control.is_bit_set(4) {
            tile_data_address = ADDRESS_TILE_DATA_1;
        } else {
            tile_data_address = ADDRESS_TILE_DATA_2;
            unsigned_tile_identifiers = false;
        }

        let mut background_memory_address: u16;
        if !using_window {
            if lcd_control.is_bit_set(3) {
                background_memory_address = ADDRESS_BG_TILE_MAP_1;
            } else {
                background_memory_address = ADDRESS_BG_TILE_MAP_2;
            }
        } else {
            if lcd_control.is_bit_set(6) {
                background_memory_address = ADDRESS_BG_TILE_MAP_1;
            } else {
                background_memory_address = ADDRESS_BG_TILE_MAP_2;
            }
        }

        let mut y_pos =
            if !using_window {

                scroll_y + self.memory_bus.borrow().read_8bit(ADDRESS_LY)
            } else {
                self.memory_bus.borrow().read_8bit(ADDRESS_LY) - window_y
            };

        let tile_row = ((y_pos/8) as u16)*32;

        for pixel in 0..160 {
            let mut x_pos = pixel + scroll_x;

            if using_window && (pixel >= window_x){
                x_pos = pixel - window_x;
            }

            let tile_column = (x_pos/8) as u16;

            let tile_address = background_memory_address + tile_row + tile_column;
            let tile_identifier =
                if unsigned_tile_identifiers {
                    self.memory_bus.borrow().read_8bit(tile_address) as i16
                } else {
                    self.memory_bus.borrow().read_8bit_signed(tile_address) as i16
                };

            let mut tile_location = tile_data_address;
            tile_location +=
                if unsigned_tile_identifiers {
                    (tile_identifier as u16 * 16)
                } else {
                    ((tile_identifier as u16 + 128) * 16)
                };

            let line = (y_pos % 8)*2;
            let data_1 = self.memory_bus.borrow().read_8bit(tile_location + line as u16);
            let data_2 = self.memory_bus.borrow().read_8bit(tile_location + line  as u16 + 1u16);

            let color_bit = -1*((x_pos % 8) as i32 - 7) as i32;
            let color_identifier = (data_2.get_bit(color_bit as u8) << 1) | data_2.get_bit(color_bit as u8);
            let color = self.get_color(color_identifier, ADDRESS_PALETTE);

            let final_y = self.memory_bus.borrow().read_8bit(ADDRESS_LY);
            self.display.draw_pixel(pixel as u32, final_y as u32, Color::from(color));
        }
    }

    fn get_color(&self,  color_identifier: u8, address: u16) -> NESColor {
        let palette = self.memory_bus.borrow().read_8bit(address);

        let (hi, lo) = match color_identifier {
            0 => (1, 0),
            1 => (3, 2),
            2 => (5, 4),
            3 => (7, 6),
            _ => panic!("Unknown color")
        };

        let color = (palette.get_bit(hi) << 1) | palette.get_bit(0);
        match color {
            0 => NESColor::White,
            1 => NESColor::LightGray,
            2 => NESColor::DarkGray,
            3 => NESColor::Black,
            _ => panic!("Unknown color")
        }
    }

    fn render_objects(&mut self) {
        let lcd_control = self.memory_bus.borrow().read_8bit(ADDRESS_LCD_CONTROL);
        let use_8x16_sprites = lcd_control.is_bit_set(2);

        for sprite in 0..40 {
            let index = sprite * 4;
            let y_position = self.memory_bus.borrow().read_8bit(ADDRESS_SPRITE_ATTRIBUTE_TABLE + index).wrapping_sub(16);
            let x_position = self.memory_bus.borrow().read_8bit(ADDRESS_SPRITE_ATTRIBUTE_TABLE + index + 1).wrapping_sub(8);
            let tile_location = self.memory_bus.borrow().read_8bit(ADDRESS_SPRITE_ATTRIBUTE_TABLE + index + 2);
            let attributes = self.memory_bus.borrow().read_8bit(ADDRESS_SPRITE_ATTRIBUTE_TABLE + index + 3);

            let flip_y = attributes.is_bit_set(6);
            let flip_x = attributes.is_bit_set(5);

            let scanline = self.memory_bus.borrow().read_8bit(ADDRESS_LY);

            let y_size = if use_8x16_sprites {16} else {8};

            if (scanline >= y_position) && (scanline < (y_position + y_size)) {
                let mut line: i32 = scanline as i32 - y_position as i32;

                if flip_y {
                    line -= y_size as i32;
                    line *= -4;
                }

                line *= 2;
                let data_address = (ADDRESS_SPRITE_DATA + (tile_location * 16) as u16) + line as u16;
                let data_1 = self.memory_bus.borrow().read_8bit(data_address);
                let data_2 = self.memory_bus.borrow().read_8bit(data_address + 1);

                for tile_pixel in 7..0 {
                    let mut color_bit: i8 = tile_pixel;

                    if flip_x {
                        color_bit -= 7;
                        color_bit *= -1;
                    }

                    let color_identifier = (data_2.get_bit(color_bit as u8) << 1) | data_1.get_bit(color_bit as u8);
                    let color_address = if attributes.is_bit_set(4)  {0xFF49} else {0xFF48};
                    let color = self.get_color(color_identifier, color_address);

                    if color == NESColor::White {
                        continue;
                    }

                    let color = Color::from(color);
                    let x_pixel = -tile_pixel as i32 + 7;

                    let pixel = x_position as i32 + x_pixel;
                    self.display.draw_pixel(pixel as u32, scanline as u32, color);
                }
            }
        }
    }

    fn update_lcd_status(&mut self) {
        let mut status = self.memory_bus.borrow().read_8bit(ADDRESS_LCD_STAT);
        if !self.is_lcd_enabled() {
            // If the LCD is disabled, reset the mode and the scnaline
            self.scanline_counter = 456;
            self.memory_bus.borrow_mut().write_8bit(ADDRESS_LCD_CONTROL, 0);
            status &= 252;
            status = status.set_bit(0);
            self.memory_bus.borrow_mut().write_8bit(ADDRESS_LCD_STAT, status);
            return;
        }

        let current_line = self.memory_bus.borrow().read_8bit(ADDRESS_LY);
        let current_mode = status & 0x3;

        let mut mode = 0;
        let mut request_interrupt = false;

        // 144 => VBlank
        if current_line >= 144 {
            mode = 1;
            status = status.set_bit(0);
            status = status.reset_bit(1);
            request_interrupt = status.is_bit_set(4);
        }
        else {
            let mode_2_bounds = 456-80;
            let mode_3_bounds = mode_2_bounds - 172;

            if self.scanline_counter >= mode_2_bounds {
                mode = 2;
                status = status.set_bit(1);
                status = status.reset_bit(0);
                request_interrupt = status.is_bit_set(5);
            }
            else if self.scanline_counter >= mode_3_bounds {
                mode = 3;
                status = status.set_bit(0);
                status = status.set_bit(1);
            }
            else {
                mode = 0;
                status = status.reset_bit(0);
                status = status.reset_bit(1);
                request_interrupt = status.is_bit_set(3);
            }
        }

        if request_interrupt && (mode != current_mode) {
            const BIT_LCD_STAT_INTERRUPT: u8 = 1;
            self.memory_bus.borrow_mut().request_interrupt(BIT_LCD_STAT_INTERRUPT);
        }

        if current_line == self.memory_bus.borrow().read_8bit(ADDRESS_LYC) {
            status = status.set_bit(2);
            if status.is_bit_set(6) {
                self.memory_bus.borrow_mut().request_interrupt(1);
            }
        }
        else {
            status = status.reset_bit(2);
        }

        self.memory_bus.borrow_mut().write_8bit(ADDRESS_LCD_STAT, status);
    }

    /// Returns true if the LCD is enabled according to the LCD Control register
    fn is_lcd_enabled(&self) -> bool {
        const BIT_LCD_DISPLAY_ENABLE: u8 = 7;
        self.memory_bus.borrow().read_8bit(ADDRESS_LCD_CONTROL).is_bit_set(BIT_LCD_DISPLAY_ENABLE)
    }
}