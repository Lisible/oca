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
extern crate oca;

use std::env;

use sdl2::render::Canvas;
use sdl2::video::Window;
use sdl2::pixels::Color;
use sdl2::rect::*;
use sdl2::event::Event as SDLEvent;
use sdl2::keyboard::*;

use oca::gb::display::display::Display;
use oca::gb::event::event::Event;

struct SDL2Display {
    canvas: Canvas<Window>,
}

impl SDL2Display {
    pub fn new(window: Window) -> SDL2Display {
        let mut canvas = window.into_canvas()
            .present_vsync()
            .build()
            .unwrap();
        canvas.set_scale(800f32/160f32, 600f32/144f32);

        SDL2Display {
            canvas
        }
    }
}

impl Display for SDL2Display {
    fn draw_pixel(&mut self, x: u8, y: u8, color: u8) {
        self.canvas.set_draw_color(Color::RGB(color, color, color));
        self.canvas.draw_point(Point::new(x as i32, y as i32));
    }

    fn render(&mut self) {
        self.canvas.present();
    }

    fn clear(&mut self) {
        self.canvas.set_draw_color(Color::RGB(0,0,0));
        self.canvas.clear();
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    assert_eq!(args.len(), 2);

    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let window = video_subsystem.window("rust", 800, 600)
        .position_centered()
        .build()
        .unwrap();

    let mut display = SDL2Display::new(window);
    let mut event_pump = sdl_context.event_pump().unwrap();


    let mut console = oca::gb::emulator::Emulator::new();
    console.start(args.get(1).unwrap().to_string());

    'main_loop: loop {
        for event in event_pump.poll_iter() {
            match event {
                SDLEvent::Quit {..} |
                SDLEvent::KeyDown { keycode: Some(Keycode::Escape), ..} => break 'main_loop,
                SDLEvent::KeyDown { keycode: Some(Keycode::Up), ..} => console.handle_event(Event::ControllerUp),
                SDLEvent::KeyDown { keycode: Some(Keycode::Down), ..} => console.handle_event(Event::ControllerDown),
                SDLEvent::KeyDown { keycode: Some(Keycode::Left), ..} => console.handle_event(Event::ControllerLeft),
                SDLEvent::KeyDown { keycode: Some(Keycode::Right), ..} => console.handle_event(Event::ControllerRight),
                SDLEvent::KeyDown { keycode: Some(Keycode::W), ..} => console.handle_event(Event::ControllerA),
                SDLEvent::KeyDown { keycode: Some(Keycode::X), ..} => console.handle_event(Event::ControllerB),
                SDLEvent::KeyDown { keycode: Some(Keycode::Asterisk), ..} => console.handle_event(Event::ControllerSelect),
                SDLEvent::KeyDown { keycode: Some(Keycode::Return), ..} => console.handle_event(Event::ControllerStart),
                _ => {}
            }
        }

        console.update();
    }
}
