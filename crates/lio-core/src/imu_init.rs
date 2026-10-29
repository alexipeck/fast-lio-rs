//! IMU initialisation: `ImuProcess::IMU_init` together with the init branch of
//! `ImuProcess::Process`, from `src/IMU_Processing.hpp`.
//!
//! While the sensor sits still, gyro and accelerometer readings are averaged
//! over the first frames. The averages seed gravity's direction and the gyro
//! bias, and the covariance is reset to fixed values. Initialisation ends once
//! more than [`MAX_INI_COUNT`] samples have been seen; that counts samples, not
//! frames.

use crate::esekf::{Cov, Esekf};
use crate::mat::{Mat3, Vec3};
use crate::s2::S2;
use crate::so3::Quat;
use crate::state::idx;

pub const MAX_INI_COUNT: usize = 10;
/// FAST-LIO's gravity magnitude for scaling accelerometer readings.
pub const G_M_S2: f64 = 9.81;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuMeasurement {
    /// Seconds.
    pub t: f64,
    /// Specific force, in any unit; only its direction and norm ratio are used.
    pub acc: Vec3,
    /// rad/s.
    pub gyro: Vec3,
}

#[derive(Clone, Debug)]
pub struct ImuInit {
    mean_acc: Vec3,
    mean_gyr: Vec3,
    n: usize,
    first_frame: bool,
    first_lidar_time: f64,
}

impl Default for ImuInit {
    fn default() -> Self {
        Self {
            mean_acc: Vec3::new(0.0, 0.0, -1.0),
            mean_gyr: Vec3::zeros(),
            n: 1,
            first_frame: true,
            first_lidar_time: 0.0,
        }
    }
}

impl ImuInit {
    /// Mean accelerometer reading. The IMU front end scales every later reading by
    /// `G_M_S2 / |mean_acc|`.
    pub fn mean_acc(&self) -> Vec3 {
        self.mean_acc
    }

    pub fn first_lidar_time(&self) -> f64 {
        self.first_lidar_time
    }

    /// Feeds one frame's IMU samples and re-seeds the filter. Returns true once
    /// initialisation is complete.
    ///
    /// FAST-LIO also accumulates accelerometer and gyro variances here, but
    /// overwrites them with the configured values when initialisation ends, so
    /// they are not computed.
    pub fn feed(
        &mut self,
        imu: &[ImuMeasurement],
        lidar_beg_time: f64,
        kf: &mut Esekf,
        lidar_t_wrt_imu: &Vec3,
        lidar_r_wrt_imu: &Mat3,
    ) -> bool {
        let Some(first) = imu.first() else {
            return false;
        };
        if self.first_frame {
            self.n = 1;
            self.first_frame = false;
            self.mean_acc = first.acc;
            self.mean_gyr = first.gyro;
            self.first_lidar_time = lidar_beg_time;
        }
        for m in imu {
            let n = self.n as f64;
            self.mean_acc += (m.acc - self.mean_acc) / n;
            self.mean_gyr += (m.gyro - self.mean_gyr) / n;
            self.n += 1;
        }

        let mut x = kf.x;
        x.grav = S2::new(&(-self.mean_acc / self.mean_acc.norm() * G_M_S2));
        x.bg = self.mean_gyr;
        x.offset_t_l_i = *lidar_t_wrt_imu;
        x.offset_r_l_i = Quat::from_matrix(lidar_r_wrt_imu);
        kf.x = x;

        let mut p = Cov::identity();
        for i in 0..3 {
            p[(idx::OFFSET_R + i, idx::OFFSET_R + i)] = 0.00001;
            p[(idx::OFFSET_T + i, idx::OFFSET_T + i)] = 0.00001;
            p[(idx::BG + i, idx::BG + i)] = 0.0001;
            p[(idx::BA + i, idx::BA + i)] = 0.001;
        }
        p[(idx::GRAV, idx::GRAV)] = 0.00001;
        p[(idx::GRAV + 1, idx::GRAV + 1)] = 0.00001;
        kf.p = p;

        self.n > MAX_INI_COUNT
    }
}
