//! Basin approximation for the determinstic Hénon extended boundary map 
//! 
//! The independent state is `(x, y, theta)`, where `n = (cos(theta), sin(theta))` 
//! is the unit boundary normal.
//! 
//! The implementation constructs conservative box-image enclosures and a reverse 
//! transtion graph. It returns:
//! 
//! - an inner approximation: every enclosed successor reaches the target;
//! - an outer approximation: at least one enclosed successor can reach the target;
//! - an unresolved band between the two approximations.
//! 
//! 
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};
use wasm_bindgen::prelude::*;


const UNREACHED: i32 = -1;
const MAX_GRID_CELLS: usize = 2_000_000;
const MAX_GRAPH_EDGES: usize = 40_000_000;


#[derive(Debug, Clone, Serialize, Deserialize, Copy)]
pub struct BasinBounds {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

impl BasinBounds {
    fn validate(self) -> Result<(), String>{
        if ![
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
        ].iter()
        .all(|value| value.is_finite())
        {
            return Err("Basin bounds must be finite".to_string());
        }

        if self.x_min >= self.x_max || self.y_min >= self.y_max {
            return Err("Basin bounds must have positive width and height".to_string());
        }

        Ok(())
    }

}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BasinApproximationConfig {
    pub a: f64,
    pub b: f64,
    pub epsilon: f64,
    pub bounds: BasinBounds,

    pub grid_x: usize,
    pub grid_y: usize,
    pub grid_theta: usize,
    pub target_position_radius: f64,
    pub target_angle_radius: f64,
    pub max_levels: usize,
}

impl BasinApproximationConfig {
    fn validate(&self) -> Result<(), String> {
        self.bounds.validate()?;

        if !self.a.is_finite() {
            return Err("Hénon parameter a must be finite".to_string());
        }

        if !self.b.is_finite() || self.b.abs() < 1e-12 {
            return Err("Hénon parameter b must be finite and nonzero".to_string());
        }

        if !self.epsilon.is_finite() || self.epsilon < 0.0 {
            return Err("Boundary-map epsilon must be finite and nonnegative".to_string());
        }

        if self.grid_x < 4 || self.grid_y < 4 || self.grid_theta < 8 {
            return Err(
                "The basin grid must be at least 4 × 4 × 8 in (x, y, theta)"
                    .to_string(),
            );
        }

        if self.grid_x > 512 || self.grid_y > 512 || self.grid_theta > 256 {
            return Err("Requested basin grid exceeds the supported per-axis limit".to_string());
        }

        let cells = self
            .grid_x
            .checked_mul(self.grid_y)
            .and_then(|value| value.checked_mul(self.grid_theta))
            .ok_or("Basin grid size overflow")?;

        if cells > MAX_GRID_CELLS {
            return Err(format!(
                "Basin grid contains {cells} cells; maximum is {MAX_GRID_CELLS}"
            ));
        }

        if !self.target_position_radius.is_finite() 
            || self.target_angle_radius <= 0.0 {
                return Err("Target position radius must be positive and finite".to_string());
        }

        if !self.target_angle_radius.is_finite()
            || self.target_angle_radius <= 0.0
            || self.target_angle_radius > PI
        {
            return Err(
                "Target angle radius must lie in the interval (0, π]".to_string(),
            );
        }

        if self.max_levels == 0 {
            return Err("At least one basin expansion level is required".to_string());
        }

        Ok(())
    }
}


#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BasinTargetPoint {
    pub x: f64,
    pub y: f64,
    pub nx: f64,
    pub ny: f64,
}

impl BasinTargetPoint {
    fn normalized(self) -> Result<Self, String> {
        if ![self.x, self.y, self.nx, self.ny].iter() 
            .all(|value| value.is_finite()) {
                return Err("Lifted MIS target points must be finite".to_string());
        }

        let normal_length = self.nx.hypot(self.ny);
        if normal_length < 1e-12 {
            return Err("Lifted MIS target contains a zero normal".to_string());
        }

        Ok(Self {
            x: self.x, 
            y: self.y,
            nx: self.nx / normal_length,
            ny: self.ny / normal_length,
        })
    } 
    

    fn theta(self) -> f64 {
        normalize_angle(self.ny.atan2(self.nx))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BasinStopReason {
    FixedPointReached,
    ResolutionLimited,
    DomainTruncated,
    IterationLimit,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BasinProjectionCell {
    pub ix: usize,
    pub iy: usize,
    pub x: f64,
    pub y: f64,


    pub inner_coverage: f64,
    pub outer_coverage: f64,
    pub min_inner_level: Option<usize>,
    pub max_inner_level: Option<usize>,

}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BasinApproximationResult {
    pub grid_x: usize,
    pub grid_y: usize,
    pub grid_theta: usize,


    /// Row-major levels indexed by `(theta, y, x)`.
    ///
    /// `-1` means the cell is not in the certified inner approximation.
    /// `0` means it belongs to the trapping seed.
    pub inner_levels: Vec<i32>,

    /// Conservative possible capture levels.
    ///
    /// `-1` means that no path through the conservative transition graph
    /// reaches the target within the completed expansion.
    pub outer_levels: Vec<i32>,

    pub projection: Vec<BasinProjectionCell>,
    pub target_cell_count: usize,
    pub inner_cell_count: usize, 
    pub outer_cell_count: usize,
    pub unresolved_cell_count: usize,
    pub domain_exit_cell_count: usize,
    pub boundary_contact_cell_count: usize,

    pub completed_inner_levels: usize,
    pub completed_outer_levels: usize,

    pub graph_edge_count: usize,
    pub trapping_verified: bool,
    pub stop_reason: BasinStopReason,

}


#[derive(Debug, Copy, Clone)]
struct State3 {
    x: f64,
    y: f64,
    theta: f64,
}

#[derive(Debug, Clone)]
struct Grid3 {
    bounds: BasinBounds,
    nx: usize,
    ny: usize,
    nt: usize,
    dx: f64,
    dy: f64,
    dt: f64
}

impl Grid3 {
    fn new(config: &BasinApproximationConfig) -> Self {
        Self {
            bounds: config.bounds,
            nx: config.grid_x,
            ny: config.grid_y,
            nt: config.grid_theta,
            dx: (config.bounds.x_max - config.bounds.x_min) / config.grid_x as f64,
            dy: (config.bounds.y_max - config.bounds.y_min) / config.grid_y as f64,
            dt: TAU / config.grid_theta as f64,
        }
    }

    fn cell_count(&self) -> usize {
        self.nx * self.ny * self.nt
    }

    fn id(&self, ix: usize, iy: usize, it: usize) -> usize {
        (it * self.ny + iy) * self.nx + ix
    } 

    fn decode(&self, id: usize) -> (usize, usize, usize) {
        let ix = id % self.nx;
        let remainder = id / self.nx;
        let iy = remainder % self.ny;
        let it = remainder / self.ny;
        (ix, iy, it)
    }

    fn center(&self, id: usize) -> State3 {
        let (ix, iy, it) = self.decode(id);
        State3 {
            x: self.bounds.x_min + (ix as f64 + 0.5) * self.dx,
            y: self.bounds.y_min + (iy as f64 + 0.5) * self.dy,
            theta: (it as f64 + 0.5) * self.dt
        }

    }

    fn intervals(&self, id: usize) -> [Interval; 3] {
        let (ix, iy, it) = self.decode(id);

        [
            Interval::new(
                self.bounds.x_min + ix as f64 * self.dx,
                self.bounds.x_min + (ix + 1) as f64 * self.dx,
            ),
            Internval::new(
                self.bounds.y_min + iy as f64 * self.dy,
                self.bounds.y_min + (iy + 1) as f64 * self.dy,
            ),
            Interval::new(it as f64 * self.dt, (it + 1) as f64 * self.dt),
        ]
    }

    fn half_width(&self) -> [f64;3] {
        [0.5 * self.dx, 0.5 * self.dy, 0.5 * self.dt]
    }


}
#[derive(Debug, Clone, Copy)]
struct Interval {
    lo: f64,
    hi: f64
}

impl Interval {
    fn new(lo: f64, hi: f64) -> Self {
        debug_assert!(lo <= hi);
        Self { lo, hi }
    }

    fn point(value: f64) -> Self {
        Self {
            lo: value, 
            hi: value,
        }
    }

    fn from_center_radius(center: f64, radius: f64) -> Self {
        Self {
            lo: next_down(center - radius),
            hi: next_up(center + radius),
        }
    }

    fn add(self, other: Self) -> Self {
        Self {
            lo: next_down(self.lo + other.lo), 
            hi: next_up(self.hi + other.hi),
        }
    }

    fn sub(self, other: Self) -> Self {
        Self {
            lo: next_down(self.lo - other.lo),
            hi: next_up(self.hi - other.hi),
        }
    }

    fn mul(self, other: Self) -> Self {
        let products = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi
        ];

        let lo = products.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = products
            .iter() 
            .copied() 
            .fold(f64::NEG_INFINITY, f64::max);

        Self {
            lo: next_down(lo),
            hi: next_up(hi),
        }
    }

    fn scale(self, scalar: f64) -> Self { 
        self.mul(Self::point(scalar))
    }

    fn square(self) -> Self {
        if self.lo <= 0.0 && self.hi >= 0.0 {
            let hi = (self.lo * self.lo).max(self.hi * self.hi);
            Self {
                lo: 0.0,
                hi: next_up(hi),
            }
        } else {
            let left = self.lo * self.lo;
            let right = self.hi * self.hi;
            Self {
                lo: next_down(left.min(right)).max(0.0),
                hi: next_up(left.max(right)),
            }
        }
    }


    fn div(self, denominator: f64) -> Result<Self, String> {
        if denominator.lo <= 0.0 && denominator.hi >= 0.0 {
            return Err("Interval division encountered a zero-containing denominator".to_string());
        }

        let reciprocal = Self {
            lo: next_down(1.0 / denominator.hi),
            hi: next_up(1.0 / denominator.lo),
        };

        Ok(self.mul(reciprocal))
    }

    fn max_abs(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }

    fn sin(self) -> Self {
        if self.hi - self.lo >= TAU {
            return Self::new(-1.0, 1.0);
        }

        let mut lo = self.lo.sin().min(self.hi.sin());
        let mut hi = self.lo.sin().max(self.hi.sin());

        if contains_periodic_point(self.lo, self.hi, 0.5 * PI, TAU) {
            hi = 1.0;
        }
        if contains_periodic_point(self.lo, self.hi, 1.5 * PI, TAU) {
            lo = -1.0;
        }

        Self {
            lo: next_down(lo).max(-1.0),
            hi: next_up(hi).min(1.0),
        }
    }

    fn cos(self) -> Self {
        Interval::new(self.lo + 0.5 * PI, self.hi + 0.5 * PI).sin()
    }
}

fn contains_periodic_point(lo: f64, hi: f64, offset: f64, period: f64) -> bool {
    let first = ((lo - offset) / period).ceil();
    let last = ((hi - offset) / period).floor();
    first <= last
}

#[derive(Debug, Copy, Clone)]
struct DualInterval{ 
    value: Interval,
    derivative: [Interval; 3],
}

impl DualInterval {
    fn constant(value: f64) -> Self {
        Self {
            value: Interval::point(value),
            derivative: [Interval::point(0.0);3],
        }
    }

    fn variable(value: Interval, dimension: usize) -> Self {
        let mut derivative = [Interval::point(0.0); 3];
        derivative[dimension] = Interval::point(1.0);
        Self { value, derivative }
    }

    fn add(self, other: Self) -> Self {
        Self {
            value: self.value.add(other.value),
            derivative: std::array::from_fn(|index| {
                self.derivative[index].add(other.derivative[index])
            }),
        }
    }

    fn sub(self, other: Self) -> Self {
        Self {
            value: self.value.sub(other.value),
            derivative: std::array::from_fn(|index| {
                self.derivative[index].sub(other.derivative[index])
            }),
        }
    }

    fn mul(self, other: Self) -> Self {
        Self {
            value: self.value.mul(other.value),
            derivative: std::array::from_fn(|index| {
                self.derivative[index]
                    .mul(other.value)
                    .add(self.value.mul(other.derivative[index]))
            }),
        }
    }

    fn scale(self, scalar: f64) -> Self {
        self.mul(Self::constant(scalar))
    }

    fn square(self) -> Self {
        self.mul(self)
    }

    fn div(self, other: Self) -> Result<Self, String> {
        let value = self.value.div(other.value)?;
        let denominator = other.value.square();

        let mut derivative = [Interval::point(0.0); 3];
        for (index, output) in derivative.iter_mut().enumerate() {
            let numerator = self.derivative[index]
                .mul(other.value)
                .sub(self.value.mul(other.derivative[index]));
            *output = numerator.div(denominator)?;
        }

        Ok(Self {
            value,
            derivative,
        })
    }

    fn sin(self) -> Self {
        let cosine = self.value.cos();
        Self {
            value: self.value.sin(),
            derivative: std::array::from_fn(|index| {
                cosine.mul(self.derivative[index])
            }),
        }
    }

    fn cos(self) -> Self {
        let negative_sine = self.value.sin().scale(-1.0);
        Self {
            value: self.value.cos(),
            derivative: std::array::from_fn(|index| {
                negative_sine.mul(self.derivative[index])
            }),
        }
    }


}
