# LD06

This crate is a Rust driver for the tiny LD06 LiDAR sold under a few brands, namely Innomaker.
The driver is cross-platform, but not designed for embedded use.

```no_run
use std::thread::sleep;
use std::time::Duration;
use ld06::ld06_driver::LD06;

fn main() {
    println!("Connecting to a port automatically...");
    let mut ld = LD06::new_auto_port().unwrap();
    ld.listen(); //Port is released in Drop

    println!("Connected!");

    loop {
        if let Some(scan) = ld.next_scan() {
            println!("{:?}", scan);
        } else {
            sleep(Duration::from_millis(15)); //To avoid burning the CPU
        }
    }
}
```

## Info
The driver spawns a background thread that it will listen to a LiDAR from. Once spawned, the thread will read data from the 
LiDAR into a buffer, which can be queried using the struct on the main thread. This buffer will not loop, so packets will
be dropped if the buffer is full.

The driver aims to have a minimal memory footprint, and only contains dynamic allocation for the buffer the port reads into
(and even then, this buffer should not grow under normal conditions).

If you wish to use an alternative source of LiDAR data, you can use `LD06::from_reader()` to provide your own source. 
The driver implementation makes no assumptions on the use of a serial port, however whatever reader is provided must 
maintain the same properties as RS232 (i.e. Little Endian). This is useful for testing from `cat`ed files.
