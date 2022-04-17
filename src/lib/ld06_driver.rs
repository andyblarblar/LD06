use std::io::{BufRead, BufReader, Read};
use std::sync::Arc;

use anyhow::Result;
use byteorder::ByteOrder;
use cancellation::CancellationTokenSource;
use parking_lot::Mutex;
use ringbuffer::{ConstGenericRingBuffer, RingBufferWrite};
use serialport::SerialPortType::UsbPort;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Range {
    pub dist: u16,
    pub confidence: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Scan {
    pub radar_speed: u16,
    pub start_angle: f32,
    /// The measured ranges.
    ///
    /// The first range angle is at [start_angle].
    pub data: [Range; 12],
    pub end_angle: f32,
    pub stamp: u16,
    pub crc: u8,
}

impl Scan {
    /// Gets the angular step per range reading.
    pub fn get_step(&self) -> f32 {
        (self.end_angle - self.start_angle) / (12. - 1.)
    }

    /// Calculates the angle the nth reading was at in this packet.
    /// The reading number in this case is 1 indexed.
    pub fn get_angle_of_reading(&self, reading_num: u8) -> f32 {
        self.start_angle + self.get_step() * (reading_num - 1) as f32
    }
}

pub struct LD06<R: Read> {
    port: Arc<Mutex<BufReader<R>>>,
    buff: Arc<Mutex<ConstGenericRingBuffer<Scan, 100>>>,
    cts: CancellationTokenSource,
}

impl LD06<Box<dyn SerialPort>> {
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
            23040,
        );
        let serial = port
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .open()?;

        Ok(LD06 {
            port: Arc::from(Mutex::new(BufReader::new(serial))),
            buff: Default::default(),
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

        Ok(LD06 {
            port: Arc::from(Mutex::new(BufReader::new(serial))),
            buff: Default::default(),
            cts: CancellationTokenSource::new(),
        })
    }
}

impl<R: Read + Send + Sync + 'static> LD06<R> {
    pub fn from_reader(data: R) -> Self {
        LD06 {
            port: Arc::from(Mutex::new(BufReader::new(data))),
            buff: Default::default(),
            cts: CancellationTokenSource::new(),
        }
    }

    pub fn listen(&mut self) {
        let ct = self.cts.token().clone();
        let reader = self.port.clone();
        let ring = self.buff.clone();

        std::thread::spawn(move || {
            let mut buf: Vec<u8> = Vec::with_capacity(47); //Size of a packet
            let mut reader = reader.lock(); //Locking forever is fine cause we never read outside of here

            while !ct.is_canceled() {
                let mut packet = Scan::default();
                //Read until start of next packet. This means the header of the next packet is at the end of the buffer now.
                reader.read_until(0x54, &mut buf).unwrap(); //Panic is fine here, as it just kills background thread

                //See docs/refrence.pdf for packet format
                packet.radar_speed = byteorder::BE::read_u16(&buf[1..=2]);
                packet.start_angle = byteorder::BE::read_u16(&buf[3..=4]) as f32 / 10.0;

                for (i, range) in buf[5..12 * 3 + 5 /*5-40*/].chunks(3).enumerate() {
                    packet.data[i].dist = byteorder::BE::read_u16(&range[0..=1]);
                    packet.data[i].confidence = range[2];
                } //Read up to 40 here

                packet.end_angle = byteorder::BE::read_u16(&buf[41..=42]) as f32 / 10.0;
                packet.stamp = byteorder::BE::read_u16(&buf[43..=44]);
                packet.crc = buf[45]; //TODO Add crc checking before building this struct

                let mut lck = ring.lock();
                lck.push(packet);

                buf.clear();
            }
        });
    }
}
