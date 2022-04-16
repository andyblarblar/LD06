use anyhow::Result;
use cancellation::CancellationTokenSource;
use parking_lot::Mutex;
use ringbuffer::ConstGenericRingBuffer;
use serialport::SerialPortType::UsbPort;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{BufRead, BufReader, Read};
use std::mem::size_of;
use std::sync::Arc;

#[repr(C, packed)]
pub struct Range {
    pub dist_lsb: u8,
    pub dist_msb: u8,
    pub confidence: u8,
}

#[repr(C, packed)]
struct RawScan {
    data_len: u8, //Should always be 12
    /// In degrees per second.
    radar_speed_lsb: u8,
    radar_speed_msb: u8,
    /// The start angle of the scan, in 0.01 degrees each.
    start_angle_lsb: u8,
    start_angle_msb: u8,
    /// The ranges measured.
    data: [Range; 12],
    /// The end angle of the scan.
    end_angle_lsb: u8,
    end_angle_msb: u8,
    /// In ms, rolls over at 30000.
    stamp_lsb: u8,
    stamp_msb: u8,
    crc: u8, //TODO
}

#[repr(C)]
pub struct Scan {
    pub radar_speed: u16,
    pub start_angle: f32,
    pub data: [Range; 12],
    pub end_angle: f32,
    pub stamp: u16,
    pub crc: u8,
}

impl From<RawScan> for Scan {
    fn from(raw: RawScan) -> Self {
        Scan {
            radar_speed: raw.radar_speed,
            start_angle: raw.start_angle as f32 / 10.,
            data: raw.data,
            end_angle: raw.end_angle as f32 / 10.,
            stamp: raw.stamp,
            crc: raw.crc,
        }
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
        let port = serialport::new(path, 23040);
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
        let mut reader = self.port.clone();
        let mut buf = self.buff.clone();

        std::thread::spawn(move || {
            let mut buf: Vec<u8> = Vec::with_capacity(size_of::<RawScan>());
            let mut reader = reader.lock(); //Locking forever is fine cause we never read outside of here

            while !ct.is_canceled() {
                reader.read_until(0x54, &mut buf).unwrap();
                let (_, body, _) = unsafe { buf.align_to::<RawScan>() };
            }
        });
    }
}
