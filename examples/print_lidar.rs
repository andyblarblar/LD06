use std::thread::sleep;
use std::time::Duration;
use ld06::ld06_driver::LD06;

fn main() {
    simple_log::console("debug").unwrap();

    println!("Connecting to a port automatically...");
    let mut ld = LD06::from_reader(std::fs::File::open("test_assets/example3.0.txt").unwrap());//LD06::new_auto_port().unwrap();
    ld.listen();

    println!("Connected!");

    loop {
        if let Some(scan) = ld.next_scan() {
            println!("{:?}", scan);
        } else {
            sleep(Duration::from_millis(15)); //To avoid burning the CPU
        }
    }
}
