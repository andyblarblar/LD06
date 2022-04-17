use std::io::{BufRead, BufReader, Read};
use std::sync::Arc;

use anyhow::Result;
use byteorder::ByteOrder;
use cancellation::CancellationTokenSource;
use parking_lot::Mutex;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer, RingBufferExt, RingBufferRead, RingBufferWrite};
use serialport::SerialPortType::UsbPort;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

/// A range reading from the sensor.
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Range {
    /// The distance from the unit, in mm.
    pub dist: u16,
    /// The intensity of the scan. 200 is 'average'.
    pub confidence: u8,
}

/// A single scan packet from the Lidar.
///
/// All angles are with clockwise respect to the arrow on the top of the unit.
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Scan {
    /// The rotational speed of the unit, in degrees per second.
    pub radar_speed: u16,
    /// The starting angle of this scan.
    pub start_angle: f32,
    /// The measured ranges.
    ///
    /// The first range angle is at [start_angle].
    pub data: [Range; 12],
    /// The ending angle of this scan.
    pub end_angle: f32,
    /// The timestamp of this scan. This will roll over at 30000.
    pub stamp: u16,
    /// The CRC check from the lidar. This is currently not implimented.
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

/// Struct providing access to the output data of an LD06 LiDAR.
pub struct LD06<R: Read + Send> {
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

impl<R: Read + Send + 'static> LD06<R> {
    pub fn from_reader(data: R) -> Self {
        LD06 {
            port: Arc::from(Mutex::new(BufReader::new(data))),
            buff: Default::default(),
            cts: CancellationTokenSource::new(),
        }
    }

    /// Creates a new background thread which begins to buffer data from the LiDAR.
    ///
    /// Buffered data is flushed before reads begin.
    ///
    /// If there was already a thread spawned by another call to listen, it is cancelled.
    pub fn listen(&mut self) {
        self.cts.cancel(); //Cancel any previously running listeners.
        self.cts = CancellationTokenSource::new(); //Create a new cts in case we were stopped before.
        self.buff.lock().clear(); //Flush old buffer to avoid stale data reemerging.

        let ct = self.cts.token().clone();
        let reader = self.port.clone();
        let ring = self.buff.clone();

        //Spawn detached thread, which is controlled by the cancellation token.
        std::thread::spawn(move || {
            let mut buf: Vec<u8> = Vec::with_capacity(47); //Size of a packet
            let mut reader = reader.lock(); //Locking forever is fine cause we never read outside of here

            while !ct.is_canceled() {
                let mut packet = Scan::default();
                //Read until start of next packet. This means the header of the next packet is at the end of the buffer now.
                reader.read_until(0x54, &mut buf).unwrap(); //Panic is fine here, as it just kills background thread

                // On the first read, we may not get a full packet.
                if buf.len() < 47 {
                    buf.clear();
                    continue;
                }

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

    /// Stops listening to new data. Buffered data is still readable.
    ///
    /// It is possible to continue listening by calling [listen] again.
    pub fn stop(&self) {
        self.cts.cancel();
    }

    /// Reads the next scan from the buffer, if any.
    pub fn next_scan(&mut self) -> Option<Scan> {
        let mut lck = self.buff.lock();
        lck.dequeue()
    }

    pub fn has_scan(&self) -> bool {
        let lck = self.buff.lock();
        !lck.is_empty()
    }
}

impl<R: Read + Send> Drop for LD06<R> {
    fn drop(&mut self) {
        self.cts.cancel();
    }
}
