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

use gb::display::display::Display;
use gb::display::color::Color;

use sdl2::video::Window;
use sdl2::render::Canvas;
use sdl2::pixels::Color as SDLColor;
use sdl2::rect::Point;
use sdl2::rect::Rect;

pub struct SDL2Display {
    canvas: Canvas<Window>
}

impl SDL2Display {
    pub fn new(window: Window) -> SDL2Display {
        let mut canvas = window.into_canvas().accelerated().build().unwrap();
        canvas.set_scale(800f32/160f32, 600f32/144f32);

        SDL2Display {
            canvas
        }
    }
}

impl Display for SDL2Display {
    fn draw_pixel(&mut self, x: u32, y: u32, color: Color) {
        self.canvas.set_draw_color(SDLColor::from((color.r, color.g, color.b)));
        self.canvas.draw_point(Point::new(x as i32, y as i32));
    }

    fn render(&mut self) {
        self.canvas.present();
    }

    fn clear(&mut self) {
        self.canvas.set_draw_color(SDLColor::RGB(0,0,0));

        self.canvas.fill_rect(Rect::new(0, 0, 800, 600));
    }
}