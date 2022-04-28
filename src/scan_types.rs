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
#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct PartialScan {
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
    /// The timestamp of this scan, in ms. This will roll over at 30000.
    pub stamp: u16,
    /// The CRC check from the lidar.
    pub crc: u8,
}

/// A full scan around the LiDAR.
///
/// All angles are with clockwise respect to the arrow on the top of the unit.
#[repr(C)]
#[derive(Clone, Debug, Default)]
pub struct Scan {
    /// The average rotational speed of the unit, in degrees per second.
    pub radar_speed: u16,
    /// The starting angle of this scan.
    pub start_angle: f32,
    /// The measured ranges.
    ///
    /// The first range angle is at [start_angle].
    pub data: Vec<Range>,
    /// The ending angle of this scan.
    pub end_angle: f32,
    /// The timestamp of this scan, in ms. This will roll over at 30000. This is the stamp on the
    /// first scan from this iteration.
    pub stamp: u16,
}

impl Scan {
    /// Gets the angular step per range reading.
    pub fn get_step(&self) -> f32 {
        (self.end_angle - self.start_angle + 360.0) / (self.data.len() - 1) as f32
        //TODO test this works
    }

    /// Calculates the angle the nth reading was at in this packet.
    /// The reading number in this case is 1 indexed.
    pub fn get_angle_of_reading(&self, reading_num: u8) -> f32 {
        let angle = self.start_angle + self.get_step() * (reading_num - 1) as f32;
        if angle > 360.0 {
            angle - 360.0
        } else {
            angle
        }
    }
}

/// Aids in building 360 scans from the partial readings the LiDAR emits.
pub(crate) struct ScanBuilder {
    initial_angle: f32,
    buffer: Scan,
    was_over_90: bool,
    working_radar_speed: u64,
}

impl Default for ScanBuilder {
    fn default() -> Self {
        ScanBuilder {
            initial_angle: f32::INFINITY,
            buffer: Scan::default(),
            was_over_90: false,
            working_radar_speed: 0,
        }
    }
}

impl ScanBuilder {
    /// Adds the partial scan to the full scan. If the full scan was completed, it will
    /// be returned.
    pub fn add_partial_scan(&mut self, partial: PartialScan) -> Option<Scan> {
        // First scan in a series is marked by inf
        if self.initial_angle == f32::INFINITY {
            self.initial_angle = partial.start_angle;
            self.buffer.start_angle = partial.start_angle;
            self.buffer.stamp = partial.stamp;
        }

        // Common work
        self.buffer.data.extend_from_slice(&partial.data);
        self.working_radar_speed += partial.radar_speed as u64; //Use u64 to avoid overflow

        // End conditions. We track a full scan by if the scan goes above 90, then under it again.
        if !self.was_over_90 && (partial.end_angle - self.initial_angle) > 90.0 {
            self.was_over_90 = true; //Was under, now over 90 from initial
            None
        } else if (partial.end_angle - self.initial_angle) < 90.0 && self.was_over_90 {
            // Scan has completed over a full rotation, so we can build the final scan

            // Average radar speed
            self.buffer.radar_speed =
                (self.working_radar_speed / self.num_of_scans() as u64) as u16;
            self.buffer.end_angle = partial.end_angle;

            //Reset state
            self.was_over_90 = false;
            self.initial_angle = f32::INFINITY;

            let mut new = Scan::default();
            std::mem::swap(&mut new, &mut self.buffer);

            Some(new)
        } else {
            None
        }
    }

    #[inline(always)]
    /// Gets the number of scans collected via the ranges.
    fn num_of_scans(&self) -> u8 {
        (self.buffer.data.len() / 12) as u8
    }
}
