use std::io::BufRead;
use std::{io::BufReader, io::Read, time::Duration};

use crate::crc::crc8;
use crate::scan_types::{PartialScan, Scan, ScanBuilder};
use anyhow::Result;
use byteorder::ByteOrder;
use cancellation::CancellationTokenSource;
use ringbuf::{Consumer, Producer, RingBuffer};
use serialport::SerialPortType::UsbPort;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

//Alias serial to prevent leaking implementation details.
pub type Serial = Box<dyn SerialPort>;

/// Struct providing access to the output data of an LD06 LiDAR.
pub struct LD06<R: Read + Send> {
    port: Option<BufReader<R>>,
    cons: Consumer<Scan>,
    prod: Option<Producer<Scan>>,
    cts: CancellationTokenSource,
}

impl LD06<Serial> {
    /// Attempts to automatically find and open the port the LiDAR is attached to.
    pub fn new_auto_port() -> Result<Self> {
        // Attempt to find a port with the same product ID as my LD06
        let mut port_name = None;
        for port in serialport::available_ports()? {
            if let UsbPort(info) = port.port_type {
                if info.pid == 0xea60 {
                    port_name.replace(port.port_name);
                }
            }
        }

        // Configure port
        let port = serialport::new(
            port_name.ok_or_else(|| std::io::Error::from(std::io::ErrorKind::NotFound))?,
            230400,
        );
        let serial = port
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .timeout(Duration::new(2, 0))
            .open()?;

        let spsc = RingBuffer::new(8);
        let (p, c) = spsc.split();

        Ok(LD06 {
            port: Some(BufReader::new(serial)),
            prod: Some(p),
            cons: c,
            cts: CancellationTokenSource::new(),
        })
    }

    /// Attempts to open the port at path.
    pub fn new(path: &str) -> Result<Self> {
        // Configure port
        let port = serialport::new(path, 230400);
        let serial = port
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .open()?;

        let spsc = RingBuffer::new(8);
        let (p, c) = spsc.split();

        Ok(LD06 {
            port: Some(BufReader::new(serial)),
            prod: Some(p),
            cons: c,
            cts: CancellationTokenSource::new(),
        })
    }
}

impl<R: Read + Send + 'static> LD06<R> {
    /// Creates a new driver instance where a Read impl is used in place of the serial port.
    pub fn from_reader(data: R) -> Self {
        let spsc = RingBuffer::new(8);
        let (p, c) = spsc.split();

        LD06 {
            port: Some(BufReader::new(data)),
            prod: Some(p),
            cons: c,
            cts: CancellationTokenSource::new(),
        }
    }

    /// Creates a new background thread which begins to buffer data from the LiDAR.
    ///
    /// If listen is called while already listening, nothing will happen.
    pub fn listen(&mut self) {
        let ring = self.prod.take();

        // Just early return if already listening.
        if ring.is_none() {
            return;
        }
        let mut ring = ring.unwrap();

        /*
        Because the struct is only designed to be listened to once, we move all members into the thread
        Instead of using Arcs with mutexes. This considerably improves performance.
        */
        let ct = self.cts.token().clone();
        let mut reader = self.port.take().unwrap();

        //Spawn detached thread, which is controlled by the cancellation token.
        std::thread::spawn(move || {
            let mut buf: Vec<u8> = Vec::with_capacity(47); //Size of a packet
            let mut builder = ScanBuilder::default();

            while !ct.is_canceled() {
                let mut packet = PartialScan::default();
                //Read until start of next packet. This means the header of the next packet is at the end of the buffer now.
                reader.read_until(0x54, &mut buf).unwrap(); //Panic is fine here, as it just kills background thread

                // On the first read, we may not get a full packet.
                if buf.len() < 47 {
                    if buf.is_empty() {
                        continue;
                    }

                    buf.clear();
                    continue;
                }

                //Run CRC checksum
                if crc8(&buf[0..=45]) != 0 {
                    log::error!("Checksum Failed!");
                    buf.clear();
                    continue;
                }

                //See docs/refrence.pdf for packet format
                packet.radar_speed = byteorder::LE::read_u16(&buf[1..=2]);
                packet.start_angle = byteorder::LE::read_u16(&buf[3..=4]) as f32 / 100.0;

                for (i, range) in buf[5..12 * 3 + 5 /*5-40*/].chunks(3).enumerate() {
                    packet.data[i].dist = byteorder::LE::read_u16(&range[0..=1]);
                    packet.data[i].confidence = range[2];
                } //Read up to 40 here

                packet.end_angle = byteorder::LE::read_u16(&buf[41..=42]) as f32 / 100.0;
                packet.stamp = byteorder::LE::read_u16(&buf[43..=44]);
                packet.crc = buf[45];

                let full_scan = builder.add_partial_scan(packet);

                //Scans are only sent to buffer when built 360
                if let Some(scan) = full_scan {
                    //Avoid pushing packet to flushed buffer if cancelled
                    if !ct.is_canceled() && ring.push(scan).is_err() {
                        log::error!("Dropping packet due to full buffer");
                    }
                }

                buf.clear();
            }
        });
    }

    /// Stops listening to new data.
    pub fn stop(self) {
        self.cts.cancel();
    }

    /// Stops listening to new data, without moving self.
    ///
    /// # Safety
    /// This function is unsafe as it allows for continued use of a stopped listener. After this
    /// function is called, all future calls to listen will instantly exit. Unless you really need
    /// to keep the object around, use [stop] or [flush_stop] instead.
    pub unsafe fn stop_borrow(&self) {
        self.cts.cancel()
    }

    /// Stops listening to new data, and returns the remaining scans in the buffer as a vec.
    pub fn flush_stop(self) -> Vec<Scan> {
        self.cts.cancel();
        let mut v = Vec::new();
        self.cons.iter().for_each(|s| v.push(s.clone()));
        v
    }

    /// Reads the next scan from the buffer, if any.
    pub fn next_scan(&mut self) -> Option<Scan> {
        self.cons.pop()
    }

    /// True if there is a scan ready in the buffer.
    pub fn has_scan(&self) -> bool {
        !self.cons.is_empty()
    }
}

impl<R: Read + Send> Drop for LD06<R> {
    fn drop(&mut self) {
        self.cts.cancel();
    }
}

#[cfg(test)]
mod test {
    use crate::ld06_driver::LD06;
    use std::thread::sleep;
    use std::time::Duration;

    const TEST_BYTES: &[u8] = include_bytes!("../test_assets/example3.0.txt");

    #[test]
    fn test_read() {
        let mut driver = LD06::from_reader(TEST_BYTES);
        driver.listen();

        sleep(Duration::new(1, 0));

        while driver.has_scan() {
            let scan = driver.next_scan().unwrap();

            println!("{:?}", scan);
        }
    }

    /*#[test]
    fn test_refrence() {
        //Example from the manual
        const REF_BYTES: &[u8] = &[
            0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E, 0xE0, 0x00, 0xE4, 0xDC, 0x00, 0xE2, 0xD9, 0x00,
            0xE5, 0xD5, 0x00, 0xE3, 0xD3, 0x00, 0xE4, 0xD0, 0x00, 0xE9, 0xCD, 0x00, 0xE4, 0xCA,
            0x00, 0xE2, 0xC7, 0x00, 0xE9, 0xC5, 0x00, 0xE5, 0xC2, 0x00, 0xE5, 0xC0, 0x00, 0xE5,
            0xBE, 0x82, 0x3A, 0x1A, 0x50, 0x54,
        ];

        let mut driver = LD06::from_reader(REF_BYTES);
        driver.listen();

        sleep(Duration::new(1, 0));

        let scan = driver.next_scan().unwrap(); //Will panic if CRC fails
        println!("{:?}", scan);
    }*/
}
