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

use std::convert::From;
use gb::gpu::nes_color::NESColor;

pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8
}

impl From<[u8; 3]> for Color {
    fn from(color: [u8; 3]) -> Self {
        Color {r: color[0], g: color[1],  b: color[2]}
    }
}

impl From<(u8, u8, u8)> for Color {
    fn from(color: (u8, u8, u8)) -> Self {
        Color {r: color.0, g: color.1,  b: color.2}
    }
}

impl From<NESColor> for Color {
    fn from(color: NESColor) -> Self {
        match color {
            NESColor::Black => Color{r: 0, g: 0, b: 0},
            NESColor::DarkGray => Color{r: 0xCC, g: 0xCC, b: 0xCC},
            NESColor::LightGray => Color{r: 0x77, g: 0x77, b: 0x77},
            NESColor::White => Color{r: 255, g: 255, b: 255},
        }
    }
}