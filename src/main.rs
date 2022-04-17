use ld06::ld06_driver::LD06;
use std::fs::File;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    let file = File::open("test_assets/data.txt").unwrap();
    let mut ld = LD06::from_reader(file);
    ld.listen();

    sleep(Duration::new(1, 0));
    let begin = std::time::Instant::now();

    while let Some(scan) = ld.next_scan() {
        println!("{:?}", scan);
    }
    let end = std::time::Instant::now();

    println!("Done in: {:?}", end - begin);
}
