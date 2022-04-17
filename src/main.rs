use ld06::ld06_driver::LD06;

use std::thread::sleep;
use std::time::Duration;

fn main() {
    println!("Connecting to a port automatically...");
    let mut ld = LD06::new_auto_port().unwrap();
    ld.listen();

    println!("Connected!");
    sleep(Duration::new(1, 0));

    loop {
        if let Some(scan) = ld.next_scan() {
            println!("{:?}", scan);
        }
    }
}
