use std::fs::File;
use ld06::ld06_driver::LD06;

use std::thread::sleep;
use std::time::Duration;

fn main() {
    println!("Connecting to a port automatically...");
    let mut ld = LD06::new_auto_port().unwrap();
    //let mut ld = LD06::from_reader(File::open("test_assets/example2.0.txt").unwrap());
    ld.listen();

    println!("Connected!");
    
    loop {
        if let Some(scan) = ld.next_scan() {
            println!("{:?}", scan);
        }
    }
}
