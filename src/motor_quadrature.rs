use core::{
    f32::consts::{self, PI},
    ops::Sub,
};
use embassy_rp::adc;
use fixed::traits::ToFixed;

use fixed::types::I32F32;

#[derive(Copy, Clone, PartialEq)]
pub enum Sector {
    S0,
    S1, // ^
    S2, // | clockwise (CW)
    S3, // |
    S4,
    S5,
}

impl Sector {
    pub fn from_hall(a: bool, b: bool, c: bool) -> Result<Self, &'static str> {
        match (a, b, c) {
            (true, true, false) => Ok(Sector::S0),
            (true, false, false) => Ok(Sector::S1),
            (true, false, true) => Ok(Sector::S2),
            (false, false, true) => Ok(Sector::S3),
            (false, true, true) => Ok(Sector::S4),
            (false, true, false) => Ok(Sector::S5),
            (_, _, _) => Err(
                "Hall sensor readings are inconsistent. Make sure hall sensor is powered and connected.",
            ),
        }
    }

    pub fn get_number(&self) -> u8 {
        match self {
            Self::S0 => 0,
            Self::S1 => 1,
            Self::S2 => 2,
            Self::S3 => 3,
            Self::S4 => 4,
            Self::S5 => 5,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub struct SectorAngle(pub i64);

impl SectorAngle {
    pub fn rotate_by(self, angle: i64) -> SectorAngle {
        SectorAngle(self.0 + angle)
    }

    pub fn to_sector(self) -> Sector {
        match self.0.rem_euclid(6) {
            0 => Sector::S0,
            1 => Sector::S1,
            2 => Sector::S2,
            3 => Sector::S3,
            4 => Sector::S4,
            5 => Sector::S5,
            _ => Sector::S0,
        }
    }

    pub fn angle_rel_to_zero(self) -> f32 {
        return 2.0 * PI * (self.0.div_euclid(6)) as f32;
        // test cases
        // 0 -> 0
        // 1 -> 0
        // 2 -> 0
        // 5 -> 0
        // 6 -> 2pi
        // 7 -> 2pi
        // 11 -> 2pi
        // 12 -> 4pi
        // -1 -> -2pi
        // -2 -> -2pi
        // -5 -> -2pi
        // -6 -> -2pi
        // -7 -> -4pi
    }

    pub fn update_angle(self, new_sector: Sector) -> Result<SectorAngle, &'static str> {
        let old_sector = self.to_sector();

        return match (old_sector, new_sector) {
            // no rotation
            (Sector::S0, Sector::S0) => Ok(self),
            (Sector::S1, Sector::S1) => Ok(self),
            (Sector::S2, Sector::S2) => Ok(self),
            (Sector::S3, Sector::S3) => Ok(self),
            (Sector::S4, Sector::S4) => Ok(self),
            (Sector::S5, Sector::S5) => Ok(self),

            // counter-clockwise rotations
            (Sector::S0, Sector::S1) => Ok(self.rotate_by(1)),
            (Sector::S1, Sector::S2) => Ok(self.rotate_by(1)),
            (Sector::S2, Sector::S3) => Ok(self.rotate_by(1)),
            (Sector::S3, Sector::S4) => Ok(self.rotate_by(1)),
            (Sector::S4, Sector::S5) => Ok(self.rotate_by(1)),
            (Sector::S5, Sector::S0) => Ok(self.rotate_by(1)),

            // clockwise rotations
            (Sector::S0, Sector::S5) => Ok(self.rotate_by(-1)),
            (Sector::S1, Sector::S0) => Ok(self.rotate_by(-1)),
            (Sector::S2, Sector::S1) => Ok(self.rotate_by(-1)),
            (Sector::S3, Sector::S2) => Ok(self.rotate_by(-1)),
            (Sector::S4, Sector::S3) => Ok(self.rotate_by(-1)),
            (Sector::S5, Sector::S4) => Ok(self.rotate_by(-1)),

            // error
            (_, _) => Err("Failed to track hall sensor commutation"),
        };
    }
}

fn wrapped_angle_sub(lhs: f32, rhs: f32) -> f32 {
    let rem_euclid = |lhs: f32, rhs: f32| -> f32 {
        let r = lhs % rhs;
        if r < 0.0 { r + rhs } else { r }
    };
    let raw_diff = lhs - rhs;
    rem_euclid(raw_diff + consts::PI, 2. * consts::PI) - consts::PI
}

pub struct HallAngleTracker {
    cum_rotations: i32,
    offset: fixed::types::I32F32,
    prev_wrapped_angle: Option<f32>,
}

impl HallAngleTracker {
    pub fn new() -> Self {
        Self {
            cum_rotations: 0,
            offset: 0.to_fixed::<I32F32>(),
            prev_wrapped_angle: None,
        }
    }

    pub fn update(
        &mut self,
        ha_norm: f32,
        hb_norm: f32,
        hc_norm: f32,
    ) -> Result<I32F32, &'static str> {
        // This may fail if the hall sensor isn't connected to the ADC correctly
        // let new_sector = Sector::from_hall(ha_norm > 0., hb_norm > 0., hc_norm > 0.)?;

        // This may fail if the motor is spinning faster than 12,000 RPM
        // (which is faster than its free speed) or the motor quadrature tracker
        // loop isn't running at the required 3 kHz
        // self.sector_angle = self.sector_angle.update_angle(new_sector)?;

        use crate::constants::SQRT_3;
        use consts::{PI, TAU};

        // Apply Clarke transformation
        fn clarke_trans(a: f32, b: f32, c: f32) -> f32 {
            let alpha = a;
            let beta = (b - c) / SQRT_3;
            let theta_clarke = libm::atan2f(beta, alpha);
            return theta_clarke;
        }

        // negate to make CCW motor rotation positive
        let new_wrapped_angle = -clarke_trans(ha_norm, hb_norm, hc_norm);

        // handle case of very first reading
        let Some(prev_wrapped_angle) = self.prev_wrapped_angle else {
            self.prev_wrapped_angle = Some(new_wrapped_angle);
            // set the offest so it starts at 0
            self.offset = -new_wrapped_angle.to_fixed::<I32F32>();
            return Ok(0.to_fixed());
        };

        // check if previous angle is within 90 degrees of current angle
        // if not, we can't safely continue tracking. The loop must be tighter
        let wrapped_diff = wrapped_angle_sub(new_wrapped_angle, prev_wrapped_angle);

        if wrapped_diff.abs() > consts::FRAC_PI_2 {
            return Err("Failed to track hall sensor commutation");
        }

        // now we can safely assume the previous angle is within 90 degrees of
        // the current one
        let delta_angle = new_wrapped_angle - prev_wrapped_angle;

        if delta_angle > PI {
            // must have gone from something like -170 to 170, which is
            // decreasing in angle
            self.cum_rotations -= 1;
        } else if delta_angle < -PI {
            // must have gone from something like 170 to -170, which is
            // increasing in angle
            self.cum_rotations += 1;
        }
        self.prev_wrapped_angle = Some(new_wrapped_angle);

        let tau_fixed: I32F32 = TAU.to_fixed::<I32F32>();
        let new_cum_angle = (self.cum_rotations.to_fixed::<I32F32>() * tau_fixed)
            + new_wrapped_angle.to_fixed::<I32F32>();

        Ok(new_cum_angle + self.offset)
    }

    pub fn reset(&mut self, init_angle: I32F32) {
        let Some(prev_wrapped_angle) = self.prev_wrapped_angle else {
            return;
        };
        let tau_fixed: I32F32 = consts::TAU.to_fixed::<I32F32>();
        let cum_angle = (self.cum_rotations.to_fixed::<I32F32>() * tau_fixed)
            + prev_wrapped_angle.to_fixed::<I32F32>();
        self.offset = init_angle - cum_angle;
    }

    pub fn zero(&mut self) {
        self.reset(0.to_fixed());
    }
}
