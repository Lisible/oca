pub struct Console {

}

impl Console {
    pub fn new() -> Console {
        Console {}
    }

    pub fn start(&mut self) {
        println!("Hello !");
    }
}