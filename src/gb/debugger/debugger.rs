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

use std::collections::HashMap;
use std::error::Error;

use gb::debugger::debugger_command::DebuggerCommand;

pub struct Debugger<F>
    where F : FnMut(Option<Vec<String>>)
{
    command_handlers: HashMap<&'static str, F>,
}

impl<F> Debugger<F>
    where F : FnMut(Option<Vec<String>>)
{
    pub fn new() -> Debugger<F> {
        Debugger {
            command_handlers: HashMap::new()
        }
    }

    fn register_command_handler(&mut self, command_identifier: &'static str, command_handler: F) {
        self.command_handlers.insert(command_identifier, command_handler);
    }

    fn run_command(&mut self, command: DebuggerCommand) {
        let identifier = command.get_identifier();
        let arguments = command.get_arguments();
        self.command_handlers.get_mut(identifier).unwrap()(arguments);
    }
}

#[cfg(test)]
mod tests {
    use gb::debugger::debugger::*;
    use super::*;

    static mut I : i32 = 5;

    #[test]
    fn debugger_register_handler() {
        let mut debugger = Debugger::new();

        debugger.register_command_handler("print", |arguments| {
            println!("{}", arguments.unwrap().get(0).unwrap());
        });

        assert_eq!(debugger.command_handlers.len(), 1);
    }

    #[test]
    fn debugger_run_command() {
        let mut debugger = Debugger::new();



        debugger.register_command_handler("add1", |arguments| {
            unsafe {
                I += 1;
            }
        });

        debugger.run_command(DebuggerCommand::new("add1", None));

        unsafe {
            assert_eq!(I, 6);
        }
    }
}