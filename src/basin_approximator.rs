//! Basin approximation for the deterministic Hénon extended
//! boundary map.
//!
//! The independent state is `(x, y, theta)`, where
//! `n = (cos(theta), sin(theta))` is the unit boundary normal.
//!
//! 1. validate parameters and MIS boundary states (position plus normal direction);
//! 2. verify the target tube using outward-rounded forward enclosures;
//! 3. discover predecessor candidates through inverse frontier enclosures;
//! 4. cache each reached forward row as exact merged successor runs;
//! 5. extract and verify a forward-invariant trapping core inside the requested
//!    MIS tube;
//! 6. propagate certified-inner and possible-outer predecessor levels, or an
//!    outer-only result when no trapping core can be verified;
//! 7. project angular coverage to the position plane.

use serde::{Deserialize, Serialize};
#[cfg(test)]
use std::collections::VecDeque;
use std::collections::{HashMap, HashSet};
use std::f64::consts::{PI, TAU};
use wasm_bindgen::prelude::*;

const UNREACHED: i32 = -1;
const MAX_GRID_CELLS: usize = 2_000_000;
/// Fail-safe for pathological finite-grid searches. Production searches stop
/// earlier as soon as neither the inner nor outer predecessor set grows.
const MAX_AUTOMATIC_EXPANSION_LEVELS: usize = 512;
const NORMAL_EPSILON: f64 = 1e-14;
// Split every source cell before interval evaluation.  The union still covers
// the complete source cell, but avoids the severe dependency inflation of one
// mean-value enclosure over a coarse (x, y, theta) box.
const ENCLOSURE_SUBDIVISIONS: usize = 3;

#[derive(Debug, Clone, Serialize, Deserialize, Copy, PartialEq)]
pub struct BasinBounds {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

impl BasinBounds {
    fn validate(self) -> Result<(), String> {
        if ![self.x_min, self.x_max, self.y_min, self.y_max]
            .iter()
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
            return Err("The basin grid must be at least 4 × 4 × 8 in (x, y, theta)".to_string());
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
        if !self.target_position_radius.is_finite() || self.target_position_radius <= 0.0 {
            return Err("Target position radius must be positive and finite".to_string());
        }
        if !self.target_angle_radius.is_finite()
            || self.target_angle_radius <= 0.0
            || self.target_angle_radius > PI
        {
            return Err("Target angle radius must lie in the interval (0, π]".to_string());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct BasinTargetPoint {
    pub x: f64,
    pub y: f64,
    pub nx: f64,
    pub ny: f64,
}

impl BasinTargetPoint {
    fn normalized(self) -> Result<Self, String> {
        if ![self.x, self.y, self.nx, self.ny]
            .iter()
            .all(|value| value.is_finite())
        {
            return Err("MIS boundary samples and normal directions must be finite".to_string());
        }
        let normal_length = self.nx.hypot(self.ny);
        if normal_length < 1e-12 {
            return Err("An MIS boundary sample contains a zero normal direction".to_string());
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
    ResourceLimit,
    NoTrappingCore,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
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
    pub dx: f64,
    pub dy: f64,

    /// Row-major levels indexed by `(theta, y, x)`.
    /// `-1` is unreached and `0` belongs to the trapping seed.
    pub inner_levels: Vec<i32>,
    /// Conservative possible-capture levels with the same indexing.
    pub outer_levels: Vec<i32>,

    pub projection: Vec<BasinProjectionCell>,
    pub candidate_target_cell_count: usize,
    pub target_cell_count: usize,
    pub inner_cell_count: usize,
    pub outer_cell_count: usize,
    pub unresolved_cell_count: usize,
    pub domain_exit_cell_count: usize,
    pub boundary_contact_cell_count: usize,
    pub completed_inner_levels: usize,
    pub completed_outer_levels: usize,
    pub graph_edge_count: usize,
    /// Number of disjoint successor runs used to store those logical edges.
    pub transition_run_count: usize,
    /// Number of source boxes whose forward row was actually evaluated.
    pub evaluated_cell_count: usize,
    /// Number of captured destination boxes expanded through the inverse map.
    pub inverse_frontier_cell_count: usize,
    pub trapping_verified: bool,
    /// True only when another predecessor expansion cannot add a new box.
    pub converged: bool,
    /// Private fail-safe used by the automatic fixed-point search.
    pub expansion_limit: usize,
    pub stop_reason: BasinStopReason,

    /// Angularly averaged position-plane areas.  These are not probabilities.
    pub inner_area: f64,
    pub outer_area: f64,
    pub unresolved_area: f64,
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
    dt: f64,
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

    #[cfg(test)]
    fn center(&self, id: usize) -> State3 {
        let (ix, iy, it) = self.decode(id);
        State3 {
            x: self.bounds.x_min + (ix as f64 + 0.5) * self.dx,
            y: self.bounds.y_min + (iy as f64 + 0.5) * self.dy,
            theta: (it as f64 + 0.5) * self.dt,
        }
    }

    fn intervals(&self, id: usize) -> [Interval; 3] {
        let (ix, iy, it) = self.decode(id);
        [
            Interval::new(
                self.bounds.x_min + ix as f64 * self.dx,
                self.bounds.x_min + (ix + 1) as f64 * self.dx,
            ),
            Interval::new(
                self.bounds.y_min + iy as f64 * self.dy,
                self.bounds.y_min + (iy + 1) as f64 * self.dy,
            ),
            Interval::new(it as f64 * self.dt, (it + 1) as f64 * self.dt),
        ]
    }

    #[cfg(test)]
    fn half_width(&self) -> [f64; 3] {
        [0.5 * self.dx, 0.5 * self.dy, 0.5 * self.dt]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Interval {
    lo: f64,
    hi: f64,
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
            lo: next_down(self.lo - other.hi),
            hi: next_up(self.hi - other.lo),
        }
    }

    fn mul(self, other: Self) -> Self {
        let products = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi,
        ];
        let lo = products.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = products.iter().copied().fold(f64::NEG_INFINITY, f64::max);
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

    fn div(self, denominator: Self) -> Result<Self, String> {
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
    ((lo - offset) / period).ceil() <= ((hi - offset) / period).floor()
}

#[derive(Debug, Copy, Clone)]
struct DualInterval {
    value: Interval,
    derivative: [Interval; 3],
}

impl DualInterval {
    fn constant(value: f64) -> Self {
        Self {
            value: Interval::point(value),
            derivative: [Interval::point(0.0); 3],
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
            derivative: std::array::from_fn(|i| self.derivative[i].add(other.derivative[i])),
        }
    }

    fn sub(self, other: Self) -> Self {
        Self {
            value: self.value.sub(other.value),
            derivative: std::array::from_fn(|i| self.derivative[i].sub(other.derivative[i])),
        }
    }

    fn mul(self, other: Self) -> Self {
        Self {
            value: self.value.mul(other.value),
            derivative: std::array::from_fn(|i| {
                self.derivative[i]
                    .mul(other.value)
                    .add(self.value.mul(other.derivative[i]))
            }),
        }
    }

    fn scale(self, scalar: f64) -> Self {
        self.mul(Self::constant(scalar))
    }

    fn square(self) -> Self {
        self.mul(self)
    }

    fn sin(self) -> Self {
        let cosine = self.value.cos();
        Self {
            value: self.value.sin(),
            derivative: std::array::from_fn(|i| cosine.mul(self.derivative[i])),
        }
    }

    fn cos(self) -> Self {
        let negative_sine = self.value.sin().scale(-1.0);
        Self {
            value: self.value.cos(),
            derivative: std::array::from_fn(|i| negative_sine.mul(self.derivative[i])),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct ImageEnclosure {
    x: Interval,
    y: Interval,
    angle_center: f64,
    angle_radius: f64,
}

fn normalize_angle(angle: f64) -> f64 {
    angle.rem_euclid(TAU)
}

fn circular_distance(left: f64, right: f64) -> f64 {
    let difference = (normalize_angle(left) - normalize_angle(right)).abs();
    difference.min(TAU - difference)
}

fn next_up(value: f64) -> f64 {
    if value.is_nan() || value == f64::INFINITY {
        return value;
    }
    if value == 0.0 {
        return f64::from_bits(1);
    }
    let bits = value.to_bits();
    f64::from_bits(if value > 0.0 { bits + 1 } else { bits - 1 })
}

fn next_down(value: f64) -> f64 {
    if value.is_nan() || value == f64::NEG_INFINITY {
        return value;
    }
    if value == 0.0 {
        return -f64::from_bits(1);
    }
    let bits = value.to_bits();
    f64::from_bits(if value > 0.0 { bits - 1 } else { bits + 1 })
}

fn map_extended_point(state: State3, config: &BasinApproximationConfig) -> Result<State3, String> {
    let nx = state.theta.cos();
    let ny = state.theta.sin();
    let raw_nx = ny;
    let raw_ny = (nx + 2.0 * config.a * state.x * ny) / config.b;
    let normal_length = raw_nx.hypot(raw_ny);
    if !normal_length.is_finite() || normal_length < NORMAL_EPSILON {
        return Err("Extended boundary map produced a degenerate normal".to_string());
    }
    let next_nx = raw_nx / normal_length;
    let next_ny = raw_ny / normal_length;
    let next_x = 1.0 - config.a * state.x * state.x + state.y + config.epsilon * next_nx;
    let next_y = config.b * state.x + config.epsilon * next_ny;
    let next_theta = normalize_angle(next_ny.atan2(next_nx));
    if ![next_x, next_y, next_theta]
        .iter()
        .all(|value| value.is_finite())
    {
        return Err("Extended boundary map produced a non-finite state".to_string());
    }
    Ok(State3 {
        x: next_x,
        y: next_y,
        theta: next_theta,
    })
}

/// Exact inverse of the deterministic extended Hénon boundary map.
///
/// For output (X, Y, m), first remove the boundary displacement,
/// invert the ordinary Hénon position map, and then pull the output normal
/// back with the transposed Jacobian.
fn map_extended_inverse_point(
    state: State3,
    config: &BasinApproximationConfig,
) -> Result<State3, String> {
    let mx = state.theta.cos();
    let my = state.theta.sin();
    let x = (state.y - config.epsilon * my) / config.b;
    let y = state.x - config.epsilon * mx - 1.0 + config.a * x * x;
    let raw_nx = -2.0 * config.a * x * mx + config.b * my;
    let raw_ny = mx;
    let normal_length = raw_nx.hypot(raw_ny);
    if !normal_length.is_finite() || normal_length < NORMAL_EPSILON {
        return Err("Inverse extended boundary map produced a degenerate normal".to_string());
    }
    let theta = normalize_angle(raw_ny.atan2(raw_nx));
    if ![x, y, theta].iter().all(|value| value.is_finite()) {
        return Err("Inverse extended boundary map produced a non-finite state".to_string());
    }
    Ok(State3 { x, y, theta })
}

fn enclose_box_image(
    intervals: [Interval; 3],
    center: State3,
    half_width: [f64; 3],
    config: &BasinApproximationConfig,
) -> Result<ImageEnclosure, String> {
    let x = DualInterval::variable(intervals[0], 0);
    let y = DualInterval::variable(intervals[1], 1);
    let theta = DualInterval::variable(intervals[2], 2);

    let u = theta.sin();
    let v = theta
        .cos()
        .add(x.mul(theta.sin()).scale(2.0 * config.a))
        .scale(1.0 / config.b);

    // ||Df(x)^(-T)n|| based on sigma_min(A) >= |det A| / ||A||_F.
    let max_abs_x = intervals[0].max_abs();
    let inv_b = 1.0 / config.b;
    let frobenius_upper =
        (1.0 + inv_b * inv_b + (2.0 * config.a * max_abs_x * inv_b).powi(2)).sqrt();
    let r_min = next_down(inv_b.abs() / frobenius_upper).max(f64::MIN_POSITIVE);
    let r_squared = u.value.square().add(v.value.square());
    let r_max = next_up(r_squared.hi.sqrt());
    if !r_min.is_finite() || !r_max.is_finite() || r_min <= 0.0 || r_min > r_max {
        return Err("Could not bound transformed-normal length".to_string());
    }

    let r = Interval::new(r_min, r_max);
    let r2 = Interval::new(
        next_down(r_min * r_min).max(f64::MIN_POSITIVE),
        next_up(r_max * r_max),
    );
    let r3 = Interval::new(
        next_down(r_min * r_min * r_min).max(f64::MIN_POSITIVE),
        next_up(r_max * r_max * r_max),
    );
    let uv = u.value.mul(v.value);
    let u2 = u.value.square();
    let v2 = v.value.square();

    let mut nx_derivative = [Interval::point(0.0); 3];
    let mut ny_derivative = [Interval::point(0.0); 3];
    let mut angle_derivative = [Interval::point(0.0); 3];
    for dimension in 0..3 {
        nx_derivative[dimension] = v2
            .mul(u.derivative[dimension])
            .sub(uv.mul(v.derivative[dimension]))
            .div(r3)?;
        ny_derivative[dimension] = u2
            .mul(v.derivative[dimension])
            .sub(uv.mul(u.derivative[dimension]))
            .div(r3)?;
        angle_derivative[dimension] = u
            .value
            .mul(v.derivative[dimension])
            .sub(v.value.mul(u.derivative[dimension]))
            .div(r2)?;
    }

    let nx = DualInterval {
        value: u.value.div(r)?,
        derivative: nx_derivative,
    };
    let ny = DualInterval {
        value: v.value.div(r)?,
        derivative: ny_derivative,
    };
    let image_x = DualInterval::constant(1.0)
        .sub(x.square().scale(config.a))
        .add(y)
        .add(nx.scale(config.epsilon));
    let image_y = x.scale(config.b).add(ny.scale(config.epsilon));

    let center_image = map_extended_point(center, config)?;
    let component_radius = |derivatives: &[Interval; 3]| {
        next_up(
            derivatives
                .iter()
                .zip(half_width)
                .map(|(derivative, width)| derivative.max_abs() * width)
                .sum::<f64>(),
        )
    };
    let x_radius = component_radius(&image_x.derivative);
    let y_radius = component_radius(&image_y.derivative);
    let angle_radius = component_radius(&angle_derivative).min(PI);

    Ok(ImageEnclosure {
        x: Interval::from_center_radius(center_image.x, x_radius),
        y: Interval::from_center_radius(center_image.y, y_radius),
        angle_center: center_image.theta,
        angle_radius,
    })
}

/// one-step inverse enclosure. This is used only to discover
/// candidate predecessor boxes; every discovered source box is subsequently
/// verified by a forward enclosure before it can be accepted.
fn enclose_box_inverse_image(
    intervals: [Interval; 3],
    center: State3,
    half_width: [f64; 3],
    config: &BasinApproximationConfig,
) -> Result<ImageEnclosure, String> {
    let output_x = DualInterval::variable(intervals[0], 0);
    let output_y = DualInterval::variable(intervals[1], 1);
    let theta = DualInterval::variable(intervals[2], 2);
    let mx = theta.cos();
    let my = theta.sin();

    let x = output_y.sub(my.scale(config.epsilon)).scale(1.0 / config.b);
    let y = output_x
        .sub(mx.scale(config.epsilon))
        .sub(DualInterval::constant(1.0))
        .add(x.square().scale(config.a));
    let raw_nx = x.mul(mx).scale(-2.0 * config.a).add(my.scale(config.b));
    let raw_ny = mx;

    // The transposed Jacobian is invertible. Bound its smallest singular
    // value by the absolute determinant divided by the Frobenius norm.
    let max_abs_x = x.value.max_abs();
    let frobenius_upper = ((2.0 * config.a * max_abs_x).powi(2) + 1.0 + config.b * config.b).sqrt();
    let r_min = next_down(config.b.abs() / frobenius_upper).max(f64::MIN_POSITIVE);
    let r_squared = raw_nx.value.square().add(raw_ny.value.square());
    let r_max = next_up(r_squared.hi.sqrt());
    if !r_min.is_finite() || !r_max.is_finite() || r_min <= 0.0 || r_min > r_max {
        return Err("Could not bound inverse transformed-normal length".to_string());
    }
    let r2 = Interval::new(
        next_down(r_min * r_min).max(f64::MIN_POSITIVE),
        next_up(r_max * r_max),
    );
    let mut angle_derivative = [Interval::point(0.0); 3];
    for dimension in 0..3 {
        angle_derivative[dimension] = raw_nx
            .value
            .mul(raw_ny.derivative[dimension])
            .sub(raw_ny.value.mul(raw_nx.derivative[dimension]))
            .div(r2)?;
    }

    let center_image = map_extended_inverse_point(center, config)?;
    let component_radius = |derivatives: &[Interval; 3]| {
        next_up(
            derivatives
                .iter()
                .zip(half_width)
                .map(|(derivative, width)| derivative.max_abs() * width)
                .sum::<f64>(),
        )
    };
    Ok(ImageEnclosure {
        x: Interval::from_center_radius(center_image.x, component_radius(&x.derivative)),
        y: Interval::from_center_radius(center_image.y, component_radius(&y.derivative)),
        angle_center: center_image.theta,
        angle_radius: component_radius(&angle_derivative).min(PI),
    })
}

fn subbox_geometry(
    grid: &Grid3,
    cell_id: usize,
    sx: usize,
    sy: usize,
    st: usize,
    subdivisions: usize,
) -> ([Interval; 3], State3, [f64; 3]) {
    debug_assert!(subdivisions > 0);
    debug_assert!(sx < subdivisions && sy < subdivisions && st < subdivisions);
    let parent = grid.intervals(cell_id);
    let widths = [
        grid.dx / subdivisions as f64,
        grid.dy / subdivisions as f64,
        grid.dt / subdivisions as f64,
    ];
    let indices = [sx, sy, st];
    let intervals = std::array::from_fn(|dimension| {
        let lo = parent[dimension].lo + indices[dimension] as f64 * widths[dimension];
        let hi = if indices[dimension] + 1 == subdivisions {
            parent[dimension].hi
        } else {
            parent[dimension].lo + (indices[dimension] + 1) as f64 * widths[dimension]
        };
        Interval::new(lo, hi)
    });
    let center = State3 {
        x: 0.5 * (intervals[0].lo + intervals[0].hi),
        y: 0.5 * (intervals[1].lo + intervals[1].hi),
        theta: 0.5 * (intervals[2].lo + intervals[2].hi),
    };
    let half_width = [0.5 * widths[0], 0.5 * widths[1], 0.5 * widths[2]];
    (intervals, center, half_width)
}

#[cfg(test)]
fn enclose_cell_image(
    grid: &Grid3,
    cell_id: usize,
    config: &BasinApproximationConfig,
) -> Result<ImageEnclosure, String> {
    enclose_box_image(
        grid.intervals(cell_id),
        grid.center(cell_id),
        grid.half_width(),
        config,
    )
}

#[cfg(test)]
fn enclose_cell_inverse_image(
    grid: &Grid3,
    cell_id: usize,
    config: &BasinApproximationConfig,
) -> Result<ImageEnclosure, String> {
    enclose_box_inverse_image(
        grid.intervals(cell_id),
        grid.center(cell_id),
        grid.half_width(),
        config,
    )
}

#[cfg(test)]
#[derive(Debug, Clone)]
struct CsrGraph {
    offsets: Vec<usize>,
    edges: Vec<usize>,
}

#[cfg(test)]
impl CsrGraph {
    fn neighbors(&self, node: usize) -> &[usize] {
        &self.edges[self.offsets[node]..self.offsets[node + 1]]
    }

    #[cfg(test)]
    fn from_rows(rows: &[Vec<usize>]) -> Self {
        let mut offsets = Vec::with_capacity(rows.len() + 1);
        let mut edges = Vec::new();
        offsets.push(0);
        for row in rows {
            edges.extend(row.iter().copied());
            offsets.push(edges.len());
        }
        Self { offsets, edges }
    }
}

#[cfg(test)]
#[derive(Debug)]
struct TransitionGraph {
    forward: CsrGraph,
    reverse: CsrGraph,
    domain_exit: Vec<bool>,
}

fn intersecting_linear_indices(
    interval: Interval,
    minimum: f64,
    step: f64,
    count: usize,
) -> Option<(usize, usize)> {
    let maximum = minimum + step * count as f64;
    if interval.hi < minimum || interval.lo > maximum {
        return None;
    }
    let start = (((interval.lo.max(minimum) - minimum) / step).floor() as isize)
        .clamp(0, count as isize - 1) as usize;
    let end = (((interval.hi.min(maximum) - minimum) / step).floor() as isize)
        .clamp(0, count as isize - 1) as usize;
    Some((start, end))
}

fn intersecting_angle_indices(center: f64, radius: f64, grid: &Grid3) -> Vec<usize> {
    if radius >= PI {
        return (0..grid.nt).collect();
    }
    let center = normalize_angle(center);
    let lo = center - radius;
    let hi = center + radius;
    let mut ranges = Vec::with_capacity(2);
    if lo < 0.0 {
        ranges.push((0.0, hi));
        ranges.push((lo + TAU, TAU));
    } else if hi >= TAU {
        ranges.push((lo, TAU));
        ranges.push((0.0, hi - TAU));
    } else {
        ranges.push((lo, hi));
    }
    let mut indices = Vec::new();
    for (range_lo, range_hi) in ranges {
        let start = (range_lo / grid.dt)
            .floor()
            .clamp(0.0, grid.nt as f64 - 1.0) as usize;
        let end = (range_hi / grid.dt)
            .floor()
            .clamp(0.0, grid.nt as f64 - 1.0) as usize;
        for index in start..=end {
            if indices.last().copied() != Some(index) && !indices.contains(&index) {
                indices.push(index);
            }
        }
    }
    indices
}

fn domain_tolerance(grid: &Grid3) -> f64 {
    64.0 * f64::EPSILON
        * grid
            .bounds
            .x_min
            .abs()
            .max(grid.bounds.x_max.abs())
            .max(grid.bounds.y_min.abs())
            .max(grid.bounds.y_max.abs())
            .max(1.0)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SuccessorRun {
    start: usize,
    end: usize,
}

#[derive(Debug, Clone)]
struct CachedTransitionRow {
    /// Sorted, disjoint half-open ranges of row-major successor identifiers.
    /// A nonlinear enclosure can cover thousands of cells; runs keep that
    /// coverage exact without storing one integer per edge.
    successor_runs: Vec<SuccessorRun>,
    domain_exit: bool,
}

impl CachedTransitionRow {
    fn is_empty(&self) -> bool {
        self.successor_runs.is_empty()
    }

    fn successor_count(&self) -> usize {
        self.successor_runs
            .iter()
            .map(|run| run.end - run.start)
            .sum()
    }

    fn any_successor_in_prefix(&self, membership_prefix: &[usize]) -> bool {
        self.successor_runs
            .iter()
            .any(|run| membership_prefix[run.end] > membership_prefix[run.start])
    }

    fn all_successors_in_prefix(&self, membership_prefix: &[usize]) -> bool {
        self.successor_runs.iter().all(|run| {
            membership_prefix[run.end] - membership_prefix[run.start] == run.end - run.start
        })
    }

    #[cfg(test)]
    fn successor_ids(&self) -> Vec<usize> {
        self.successor_runs
            .iter()
            .flat_map(|run| run.start..run.end)
            .collect()
    }
}

fn membership_prefix(
    cell_count: usize,
    mut contains: impl FnMut(usize) -> bool,
) -> Result<Vec<usize>, String> {
    let mut prefix = Vec::new();
    prefix
        .try_reserve_exact(cell_count + 1)
        .map_err(|error| format!("Unable to reserve a basin membership prefix: {error}"))?;
    prefix.push(0);
    for cell in 0..cell_count {
        prefix.push(prefix[cell] + usize::from(contains(cell)));
    }
    Ok(prefix)
}

#[derive(Debug, Default)]
struct SparseTransitionGraph {
    rows: HashMap<usize, CachedTransitionRow>,
    edge_count: usize,
    run_count: usize,
}

impl SparseTransitionGraph {
    fn row(&self, source: usize) -> Option<&CachedTransitionRow> {
        self.rows.get(&source)
    }

    fn insert(&mut self, source: usize, row: CachedTransitionRow) -> Result<(), String> {
        if self.rows.contains_key(&source) {
            return Ok(());
        }
        let successor_count = row.successor_count();
        self.edge_count = self
            .edge_count
            .checked_add(successor_count)
            .ok_or("Sparse transition edge count overflow")?;
        self.run_count = self
            .run_count
            .checked_add(row.successor_runs.len())
            .ok_or("Sparse transition run count overflow")?;
        self.rows
            .try_reserve(1)
            .map_err(|error| format!("Unable to reserve a sparse transition row: {error}"))?;
        self.rows.insert(source, row);
        Ok(())
    }
}

fn compute_forward_row(
    grid: &Grid3,
    source: usize,
    config: &BasinApproximationConfig,
) -> Result<CachedTransitionRow, String> {
    let mut successor_runs: Vec<SuccessorRun> = Vec::new();
    let mut domain_exit = false;
    let tolerance = domain_tolerance(grid);
    for st in 0..ENCLOSURE_SUBDIVISIONS {
        for sy in 0..ENCLOSURE_SUBDIVISIONS {
            for sx in 0..ENCLOSURE_SUBDIVISIONS {
                let (intervals, center, half_width) =
                    subbox_geometry(grid, source, sx, sy, st, ENCLOSURE_SUBDIVISIONS);
                let enclosure = enclose_box_image(intervals, center, half_width, config)?;
                domain_exit |= enclosure.x.lo < grid.bounds.x_min - tolerance
                    || enclosure.x.hi > grid.bounds.x_max + tolerance
                    || enclosure.y.lo < grid.bounds.y_min - tolerance
                    || enclosure.y.hi > grid.bounds.y_max + tolerance;

                if let (Some((ix_start, ix_end)), Some((iy_start, iy_end))) = (
                    intersecting_linear_indices(enclosure.x, grid.bounds.x_min, grid.dx, grid.nx),
                    intersecting_linear_indices(enclosure.y, grid.bounds.y_min, grid.dy, grid.ny),
                ) {
                    for it in intersecting_angle_indices(
                        enclosure.angle_center,
                        enclosure.angle_radius,
                        grid,
                    ) {
                        for iy in iy_start..=iy_end {
                            successor_runs.try_reserve(1).map_err(|error| {
                                format!("Unable to reserve a transition run: {error}")
                            })?;
                            successor_runs.push(SuccessorRun {
                                start: grid.id(ix_start, iy, it),
                                end: grid.id(ix_end, iy, it) + 1,
                            });
                        }
                    }
                }
            }
        }
    }
    successor_runs.sort_unstable_by_key(|run| run.start);
    let mut merged_runs: Vec<SuccessorRun> = Vec::new();
    merged_runs
        .try_reserve(successor_runs.len())
        .map_err(|error| {
            format!(
                "Unable to reserve {} merged transition runs: {error}",
                successor_runs.len()
            )
        })?;
    for run in successor_runs {
        if let Some(previous) = merged_runs.last_mut() {
            if run.start <= previous.end {
                previous.end = previous.end.max(run.end);
                continue;
            }
        }
        merged_runs.push(run);
    }
    Ok(CachedTransitionRow {
        successor_runs: merged_runs,
        domain_exit,
    })
}

fn ensure_forward_row(
    graph: &mut SparseTransitionGraph,
    grid: &Grid3,
    source: usize,
    config: &BasinApproximationConfig,
) -> Result<bool, String> {
    if graph.rows.contains_key(&source) {
        return Ok(false);
    }
    let row = compute_forward_row(grid, source, config)?;
    graph.insert(source, row)?;
    Ok(true)
}

/// Discover every grid cell that may contain a predecessor of one destination
/// box. The result is only a candidate set; forward rows provide the final
/// possible/certified acceptance tests.
fn inverse_predecessor_candidates(
    grid: &Grid3,
    destination: usize,
    config: &BasinApproximationConfig,
) -> Result<(Vec<usize>, bool), String> {
    // Mark candidates directly by cell id. A raw vector of ids can contain up
    // to 27 copies of every grid cell before deduplication when an inverse
    // enclosure is broad, which is precisely the high-nonlinearity case this
    // lazy algorithm must handle without exhausting WebAssembly memory.
    let mut candidate_mask = vec![false; grid.cell_count()];
    let mut leaves_domain = false;
    let tolerance = domain_tolerance(grid);
    for st in 0..ENCLOSURE_SUBDIVISIONS {
        for sy in 0..ENCLOSURE_SUBDIVISIONS {
            for sx in 0..ENCLOSURE_SUBDIVISIONS {
                let (intervals, center, half_width) =
                    subbox_geometry(grid, destination, sx, sy, st, ENCLOSURE_SUBDIVISIONS);
                let enclosure = enclose_box_inverse_image(intervals, center, half_width, config)?;
                leaves_domain |= enclosure.x.lo < grid.bounds.x_min - tolerance
                    || enclosure.x.hi > grid.bounds.x_max + tolerance
                    || enclosure.y.lo < grid.bounds.y_min - tolerance
                    || enclosure.y.hi > grid.bounds.y_max + tolerance;
                if let (Some((ix_start, ix_end)), Some((iy_start, iy_end))) = (
                    intersecting_linear_indices(enclosure.x, grid.bounds.x_min, grid.dx, grid.nx),
                    intersecting_linear_indices(enclosure.y, grid.bounds.y_min, grid.dy, grid.ny),
                ) {
                    for it in intersecting_angle_indices(
                        enclosure.angle_center,
                        enclosure.angle_radius,
                        grid,
                    ) {
                        for iy in iy_start..=iy_end {
                            for ix in ix_start..=ix_end {
                                candidate_mask[grid.id(ix, iy, it)] = true;
                            }
                        }
                    }
                }
            }
        }
    }
    let candidate_count = candidate_mask
        .iter()
        .filter(|&&candidate| candidate)
        .count();
    let mut candidates = Vec::new();
    candidates
        .try_reserve_exact(candidate_count)
        .map_err(|error| {
            format!("Unable to reserve {candidate_count} inverse predecessor candidates: {error}")
        })?;
    candidates.extend(
        candidate_mask
            .into_iter()
            .enumerate()
            .filter_map(|(cell, candidate)| candidate.then_some(cell)),
    );
    Ok((candidates, leaves_domain))
}

#[cfg(test)]
fn build_reverse(forward: &CsrGraph, node_count: usize) -> Result<CsrGraph, String> {
    let mut counts = vec![0usize; node_count];
    for &destination in &forward.edges {
        counts[destination] = counts[destination]
            .checked_add(1)
            .ok_or("Reverse transition degree overflow")?;
    }
    let mut offsets = vec![0usize; node_count + 1];
    for node in 0..node_count {
        offsets[node + 1] = offsets[node]
            .checked_add(counts[node])
            .ok_or("Reverse transition offset overflow")?;
    }
    let mut cursors = offsets[..node_count].to_vec();
    let mut edges = Vec::new();
    edges
        .try_reserve_exact(forward.edges.len())
        .map_err(|error| {
            format!(
                "Unable to allocate the reverse transition graph with {} edges: {error}",
                forward.edges.len()
            )
        })?;
    edges.resize(forward.edges.len(), 0usize);
    for source in 0..node_count {
        for &destination in forward.neighbors(source) {
            edges[cursors[destination]] = source;
            cursors[destination] += 1;
        }
    }
    Ok(CsrGraph { offsets, edges })
}

#[cfg(test)]
#[allow(dead_code)]
fn build_transition_graph(
    grid: &Grid3,
    config: &BasinApproximationConfig,
) -> Result<TransitionGraph, String> {
    let node_count = grid.cell_count();
    let mut offsets = Vec::with_capacity(node_count + 1);
    let mut edges = Vec::new();
    let mut domain_exit = vec![false; node_count];
    offsets.push(0);
    let domain_tolerance = 64.0
        * f64::EPSILON
        * grid
            .bounds
            .x_min
            .abs()
            .max(grid.bounds.x_max.abs())
            .max(grid.bounds.y_min.abs())
            .max(grid.bounds.y_max.abs())
            .max(1.0);

    for source in 0..node_count {
        let mut row_edges = Vec::new();
        for st in 0..ENCLOSURE_SUBDIVISIONS {
            for sy in 0..ENCLOSURE_SUBDIVISIONS {
                for sx in 0..ENCLOSURE_SUBDIVISIONS {
                    let (intervals, center, half_width) =
                        subbox_geometry(grid, source, sx, sy, st, ENCLOSURE_SUBDIVISIONS);
                    let enclosure = enclose_box_image(intervals, center, half_width, config)?;
                    domain_exit[source] |= enclosure.x.lo < grid.bounds.x_min - domain_tolerance
                        || enclosure.x.hi > grid.bounds.x_max + domain_tolerance
                        || enclosure.y.lo < grid.bounds.y_min - domain_tolerance
                        || enclosure.y.hi > grid.bounds.y_max + domain_tolerance;

                    if let (Some((ix_start, ix_end)), Some((iy_start, iy_end))) = (
                        intersecting_linear_indices(
                            enclosure.x,
                            grid.bounds.x_min,
                            grid.dx,
                            grid.nx,
                        ),
                        intersecting_linear_indices(
                            enclosure.y,
                            grid.bounds.y_min,
                            grid.dy,
                            grid.ny,
                        ),
                    ) {
                        let theta_indices = intersecting_angle_indices(
                            enclosure.angle_center,
                            enclosure.angle_radius,
                            grid,
                        );
                        for it in theta_indices {
                            for iy in iy_start..=iy_end {
                                for ix in ix_start..=ix_end {
                                    row_edges.push(grid.id(ix, iy, it));
                                }
                            }
                        }
                    }
                }
            }
        }
        row_edges.sort_unstable();
        row_edges.dedup();
        edges.try_reserve(row_edges.len()).map_err(|error| {
            format!(
                "Unable to grow the forward transition graph beyond {} edges: {error}",
                edges.len()
            )
        })?;
        edges.extend(row_edges);
        offsets.push(edges.len());
    }

    let forward = CsrGraph { offsets, edges };
    let reverse = build_reverse(&forward, node_count)?;
    Ok(TransitionGraph {
        forward,
        reverse,
        domain_exit,
    })
}

fn target_candidate_cells(
    grid: &Grid3,
    targets: &[BasinTargetPoint],
    config: &BasinApproximationConfig,
) -> Vec<bool> {
    let mut candidate = vec![false; grid.cell_count()];
    let position_radius = config.target_position_radius;
    let angle_radius = config.target_angle_radius;

    for target in targets {
        let x_interval = Interval::new(target.x - position_radius, target.x + position_radius);
        let y_interval = Interval::new(target.y - position_radius, target.y + position_radius);
        let (Some((ix_start, ix_end)), Some((iy_start, iy_end))) = (
            intersecting_linear_indices(x_interval, grid.bounds.x_min, grid.dx, grid.nx),
            intersecting_linear_indices(y_interval, grid.bounds.y_min, grid.dy, grid.ny),
        ) else {
            continue;
        };
        let target_theta = target.theta();
        for it in 0..grid.nt {
            let theta_center = (it as f64 + 0.5) * grid.dt;
            if angle_radius < PI
                && circular_distance(theta_center, target_theta) + 0.5 * grid.dt > angle_radius
            {
                continue;
            }
            for iy in iy_start..=iy_end {
                let y0 = grid.bounds.y_min + iy as f64 * grid.dy;
                let y1 = y0 + grid.dy;
                for ix in ix_start..=ix_end {
                    let x0 = grid.bounds.x_min + ix as f64 * grid.dx;
                    let x1 = x0 + grid.dx;
                    let max_distance = [x0, x1]
                        .into_iter()
                        .flat_map(|x| {
                            [y0, y1]
                                .into_iter()
                                .map(move |y| (x - target.x).hypot(y - target.y))
                        })
                        .fold(0.0, f64::max);
                    if max_distance <= position_radius {
                        candidate[grid.id(ix, iy, it)] = true;
                    }
                }
            }
        }
    }
    candidate
}

#[cfg(test)]
#[allow(dead_code)]
fn trapping_core(candidate: &[bool], graph: &TransitionGraph) -> Vec<bool> {
    let mut core = candidate.to_vec();
    let mut queue = VecDeque::new();
    for source in 0..core.len() {
        if core[source]
            && (graph.domain_exit[source]
                || graph.forward.neighbors(source).is_empty()
                || graph
                    .forward
                    .neighbors(source)
                    .iter()
                    .any(|&destination| !core[destination]))
        {
            core[source] = false;
            queue.push_back(source);
        }
    }
    while let Some(removed) = queue.pop_front() {
        for &predecessor in graph.reverse.neighbors(removed) {
            if core[predecessor] {
                core[predecessor] = false;
                queue.push_back(predecessor);
            }
        }
    }
    core
}

fn trapping_core_sparse(
    candidate: &[bool],
    graph: &SparseTransitionGraph,
) -> Result<Vec<bool>, String> {
    let mut core = candidate.to_vec();
    loop {
        let core_prefix = membership_prefix(core.len(), |cell| core[cell])?;
        let mut changed = false;
        for source in 0..core.len() {
            if !core[source] {
                continue;
            }
            let row = graph
                .row(source)
                .ok_or_else(|| format!("Missing cached target transition row for cell {source}"))?;
            if row.domain_exit || row.is_empty() || !row.all_successors_in_prefix(&core_prefix) {
                core[source] = false;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    Ok(core)
}

#[cfg(test)]
fn expand_inner(
    graph: &TransitionGraph,
    seed: &[bool],
    expansion_limit: usize,
) -> (Vec<i32>, usize, bool) {
    let node_count = seed.len();
    let mut levels = vec![UNREACHED; node_count];
    let mut remaining = vec![0usize; node_count];
    let mut frontier = Vec::new();
    for node in 0..node_count {
        if seed[node] {
            levels[node] = 0;
            frontier.push(node);
        }
        remaining[node] =
            graph.forward.neighbors(node).len() + usize::from(graph.domain_exit[node]);
    }

    let mut completed = 0;
    for level in 1..=expansion_limit {
        if frontier.is_empty() {
            break;
        }
        for &accepted in &frontier {
            for &predecessor in graph.reverse.neighbors(accepted) {
                if levels[predecessor] == UNREACHED && remaining[predecessor] > 0 {
                    remaining[predecessor] -= 1;
                }
            }
        }
        let mut next = Vec::new();
        for node in 0..node_count {
            if levels[node] == UNREACHED
                && remaining[node] == 0
                && !graph.forward.neighbors(node).is_empty()
            {
                levels[node] = level as i32;
                next.push(node);
            }
        }
        if next.is_empty() {
            frontier.clear();
            break;
        }
        completed = level;
        frontier = next;
    }
    let can_grow = (0..node_count).any(|node| {
        levels[node] == UNREACHED
            && !graph.forward.neighbors(node).is_empty()
            && !graph.domain_exit[node]
            && graph
                .forward
                .neighbors(node)
                .iter()
                .all(|&destination| levels[destination] != UNREACHED)
    });
    (levels, completed, can_grow)
}

#[cfg(test)]
fn expand_outer(
    reverse: &CsrGraph,
    seed: &[bool],
    expansion_limit: usize,
) -> (Vec<i32>, usize, bool) {
    let mut levels = vec![UNREACHED; seed.len()];
    let mut frontier = Vec::new();
    for (node, &is_seed) in seed.iter().enumerate() {
        if is_seed {
            levels[node] = 0;
            frontier.push(node);
        }
    }
    let mut completed = 0;
    for level in 1..=expansion_limit {
        if frontier.is_empty() {
            break;
        }
        let mut next = Vec::new();
        for &accepted in &frontier {
            for &predecessor in reverse.neighbors(accepted) {
                if levels[predecessor] == UNREACHED {
                    levels[predecessor] = level as i32;
                    next.push(predecessor);
                }
            }
        }
        if next.is_empty() {
            frontier.clear();
            break;
        }
        completed = level;
        frontier = next;
    }
    let can_grow = frontier.iter().any(|&accepted| {
        reverse
            .neighbors(accepted)
            .iter()
            .any(|&predecessor| levels[predecessor] == UNREACHED)
    });
    (levels, completed, can_grow)
}

struct LazyExpansionResult {
    inner_levels: Vec<i32>,
    outer_levels: Vec<i32>,
    completed_inner_levels: usize,
    completed_outer_levels: usize,
    inner_can_grow: bool,
    outer_can_grow: bool,
    inverse_frontier_cell_count: usize,
    inverse_left_domain: bool,
}

fn expand_lazy_predecessors(
    grid: &Grid3,
    config: &BasinApproximationConfig,
    graph: &mut SparseTransitionGraph,
    inner_seed: &[bool],
    outer_seed: &[bool],
    expansion_limit: usize,
) -> Result<LazyExpansionResult, String> {
    if expansion_limit == 0 {
        return Err("The internal basin expansion limit must be positive".to_string());
    }
    let node_count = grid.cell_count();
    let mut inner_levels = vec![UNREACHED; node_count];
    let mut outer_levels = vec![UNREACHED; node_count];
    let mut outer_frontier = Vec::new();
    let inner_enabled = inner_seed.iter().any(|&is_seed| is_seed);
    for node in 0..node_count {
        if inner_seed[node] {
            inner_levels[node] = 0;
        }
        if outer_seed[node] {
            outer_levels[node] = 0;
            outer_frontier.push(node);
        }
    }

    let mut inverse_expanded = vec![false; node_count];
    let mut inverse_frontier_cell_count = 0usize;
    let mut inverse_left_domain = false;
    let mut completed_inner_levels = 0usize;
    let mut completed_outer_levels = 0usize;

    for level in 1..=expansion_limit {
        let mut possible_sources = HashSet::new();
        for &destination in &outer_frontier {
            if !inverse_expanded[destination] {
                let (candidates, leaves_domain) =
                    inverse_predecessor_candidates(grid, destination, config)?;
                inverse_expanded[destination] = true;
                inverse_frontier_cell_count += 1;
                inverse_left_domain |= leaves_domain;
                possible_sources.extend(candidates);
            }
        }

        for &source in &possible_sources {
            ensure_forward_row(graph, grid, source, config)?;
        }
        let previous_outer_prefix = membership_prefix(node_count, |cell| {
            outer_levels[cell] >= 0 && outer_levels[cell] < level as i32
        })?;

        let mut next_outer = Vec::new();
        for source in possible_sources {
            if outer_levels[source] != UNREACHED {
                continue;
            }
            let row = graph
                .row(source)
                .ok_or_else(|| format!("Missing cached transition row for cell {source}"))?;
            if row.any_successor_in_prefix(&previous_outer_prefix) {
                outer_levels[source] = level as i32;
                next_outer.push(source);
            }
        }
        next_outer.sort_unstable();
        next_outer.dedup();
        if !next_outer.is_empty() {
            completed_outer_levels = level;
        }

        let mut next_inner = Vec::new();
        if inner_enabled {
            let previous_inner_prefix =
                membership_prefix(node_count, |cell| inner_levels[cell] != UNREACHED)?;
            for (&source, row) in &graph.rows {
                if inner_levels[source] == UNREACHED
                    && !row.domain_exit
                    && !row.is_empty()
                    && row.all_successors_in_prefix(&previous_inner_prefix)
                {
                    next_inner.push(source);
                }
            }
        }
        for source in &next_inner {
            inner_levels[*source] = level as i32;
        }
        if !next_inner.is_empty() {
            completed_inner_levels = level;
        }

        outer_frontier = next_outer;
        if outer_frontier.is_empty() && next_inner.is_empty() {
            break;
        }
    }

    let mut beyond_limit_sources = HashSet::new();
    for &destination in &outer_frontier {
        if !inverse_expanded[destination] {
            let (candidates, leaves_domain) =
                inverse_predecessor_candidates(grid, destination, config)?;
            inverse_expanded[destination] = true;
            inverse_frontier_cell_count += 1;
            inverse_left_domain |= leaves_domain;
            beyond_limit_sources.extend(candidates);
        }
    }
    for &source in &beyond_limit_sources {
        ensure_forward_row(graph, grid, source, config)?;
    }
    let final_outer_prefix = membership_prefix(node_count, |cell| outer_levels[cell] != UNREACHED)?;
    let outer_can_grow = beyond_limit_sources.into_iter().any(|source| {
        outer_levels[source] == UNREACHED
            && graph
                .row(source)
                .is_some_and(|row| row.any_successor_in_prefix(&final_outer_prefix))
    });
    let final_inner_prefix = if inner_enabled {
        Some(membership_prefix(node_count, |cell| {
            inner_levels[cell] != UNREACHED
        })?)
    } else {
        None
    };
    let inner_can_grow = final_inner_prefix.as_deref().is_some_and(|prefix| {
        graph.rows.iter().any(|(&source, row)| {
            inner_levels[source] == UNREACHED
                && !row.domain_exit
                && !row.is_empty()
                && row.all_successors_in_prefix(prefix)
        })
    });

    Ok(LazyExpansionResult {
        inner_levels,
        outer_levels,
        completed_inner_levels,
        completed_outer_levels,
        inner_can_grow,
        outer_can_grow,
        inverse_frontier_cell_count,
        inverse_left_domain,
    })
}

fn project_levels(
    grid: &Grid3,
    inner_levels: &[i32],
    outer_levels: &[i32],
) -> Vec<BasinProjectionCell> {
    let mut projection = Vec::with_capacity(grid.nx * grid.ny);
    for iy in 0..grid.ny {
        for ix in 0..grid.nx {
            let mut inner_count = 0usize;
            let mut outer_count = 0usize;
            let mut min_level: Option<usize> = None;
            let mut max_level: Option<usize> = None;
            for it in 0..grid.nt {
                let id = grid.id(ix, iy, it);
                if inner_levels[id] >= 0 {
                    let level = inner_levels[id] as usize;
                    inner_count += 1;
                    min_level = Some(min_level.map_or(level, |current| current.min(level)));
                    max_level = Some(max_level.map_or(level, |current| current.max(level)));
                }
                if outer_levels[id] >= 0 {
                    outer_count += 1;
                }
            }
            projection.push(BasinProjectionCell {
                ix,
                iy,
                x: grid.bounds.x_min + (ix as f64 + 0.5) * grid.dx,
                y: grid.bounds.y_min + (iy as f64 + 0.5) * grid.dy,
                inner_coverage: inner_count as f64 / grid.nt as f64,
                outer_coverage: outer_count as f64 / grid.nt as f64,
                min_inner_level: min_level,
                max_inner_level: max_level,
            });
        }
    }
    projection
}

fn classify_stop_reason(
    converged: bool,
    trapping_verified: bool,
    boundary_contact: bool,
    reachable_domain_exit: bool,
    inverse_left_domain: bool,
    has_unresolved_cells: bool,
) -> BasinStopReason {
    if !converged {
        BasinStopReason::ResourceLimit
    } else if !trapping_verified {
        BasinStopReason::NoTrappingCore
    } else if boundary_contact || reachable_domain_exit || inverse_left_domain {
        BasinStopReason::DomainTruncated
    } else if has_unresolved_cells {
        BasinStopReason::ResolutionLimited
    } else {
        BasinStopReason::FixedPointReached
    }
}

pub fn compute_henon_extended_basin(
    target_points: &[BasinTargetPoint],
    config: &BasinApproximationConfig,
) -> Result<BasinApproximationResult, String> {
    config.validate()?;
    if target_points.is_empty() {
        return Err(
            "At least one MIS boundary sample with a normal direction is required".to_string(),
        );
    }
    let targets = target_points
        .iter()
        .copied()
        .map(BasinTargetPoint::normalized)
        .collect::<Result<Vec<_>, _>>()?;
    let grid = Grid3::new(config);
    let candidate = target_candidate_cells(&grid, &targets, config);
    let candidate_target_cell_count = candidate.iter().filter(|&&value| value).count();
    if candidate_target_cell_count == 0 {
        return Err("The MIS target tube contains no complete grid boxes; enlarge the tube or refine the grid".to_string());
    }
    let mut graph = SparseTransitionGraph::default();
    for (source, &is_candidate) in candidate.iter().enumerate() {
        if is_candidate {
            ensure_forward_row(&mut graph, &grid, source, config)?;
        }
    }
    let seed = trapping_core_sparse(&candidate, &graph)?;
    let target_cell_count = seed.iter().filter(|&&value| value).count();
    let trapping_verified = target_cell_count > 0;

    // A failed trapping-core proof invalidates the certified inner basin, but
    // it does not invalidate possible reachability.  In that case use the
    // requested MIS tube itself as the outer seed and return an honest
    // amber-only approximation instead of aborting the whole computation.
    let outer_seed = if trapping_verified { &seed } else { &candidate };
    let expansion = expand_lazy_predecessors(
        &grid,
        config,
        &mut graph,
        &seed,
        outer_seed,
        MAX_AUTOMATIC_EXPANSION_LEVELS,
    )?;
    let LazyExpansionResult {
        inner_levels,
        outer_levels,
        completed_inner_levels,
        completed_outer_levels,
        inner_can_grow,
        outer_can_grow,
        inverse_frontier_cell_count,
        inverse_left_domain,
    } = expansion;
    let projection = project_levels(&grid, &inner_levels, &outer_levels);

    let inner_cell_count = inner_levels.iter().filter(|&&level| level >= 0).count();
    let outer_cell_count = outer_levels.iter().filter(|&&level| level >= 0).count();
    let unresolved_cell_count = inner_levels
        .iter()
        .zip(&outer_levels)
        .filter(|(inner, outer)| **inner < 0 && **outer >= 0)
        .count();
    let domain_exit_cell_count = graph.rows.values().filter(|row| row.domain_exit).count();
    let boundary_contact_cell_count = outer_levels
        .iter()
        .enumerate()
        .filter(|(id, level)| {
            if **level < 0 {
                return false;
            }
            let (ix, iy, _) = grid.decode(*id);
            ix == 0 || iy == 0 || ix + 1 == grid.nx || iy + 1 == grid.ny
        })
        .count();
    let reachable_domain_exit = outer_levels
        .iter()
        .enumerate()
        .any(|(id, &level)| level >= 0 && graph.row(id).is_some_and(|row| row.domain_exit));

    let cell_area = grid.dx * grid.dy;
    let inner_area: f64 = projection
        .iter()
        .map(|cell| cell.inner_coverage * cell_area)
        .sum();
    let outer_area: f64 = projection
        .iter()
        .map(|cell| cell.outer_coverage * cell_area)
        .sum();
    let unresolved_area = (outer_area - inner_area).max(0.0);
    let converged = !(inner_can_grow || outer_can_grow);
    let stop_reason = classify_stop_reason(
        converged,
        trapping_verified,
        boundary_contact_cell_count > 0,
        reachable_domain_exit,
        inverse_left_domain,
        unresolved_cell_count > 0,
    );

    Ok(BasinApproximationResult {
        grid_x: grid.nx,
        grid_y: grid.ny,
        grid_theta: grid.nt,
        dx: grid.dx,
        dy: grid.dy,
        inner_levels,
        outer_levels,
        projection,
        candidate_target_cell_count,
        target_cell_count,
        inner_cell_count,
        outer_cell_count,
        unresolved_cell_count,
        domain_exit_cell_count,
        boundary_contact_cell_count,
        completed_inner_levels,
        completed_outer_levels,
        graph_edge_count: graph.edge_count,
        transition_run_count: graph.run_count,
        evaluated_cell_count: graph.rows.len(),
        inverse_frontier_cell_count,
        trapping_verified,
        converged,
        expansion_limit: MAX_AUTOMATIC_EXPANSION_LEVELS,
        stop_reason,
        inner_area,
        outer_area,
        unresolved_area,
    })
}

#[wasm_bindgen(js_name = "computeHenonExtendedBasin")]
pub fn compute_henon_extended_basin_js(
    target_points: JsValue,
    config: JsValue,
) -> Result<JsValue, JsValue> {
    let target_points: Vec<BasinTargetPoint> = serde_wasm_bindgen::from_value(target_points)
        .map_err(|error| JsValue::from_str(&format!("Invalid MIS boundary data: {error}")))?;
    let config: BasinApproximationConfig = serde_wasm_bindgen::from_value(config)
        .map_err(|error| JsValue::from_str(&format!("Invalid basin configuration: {error}")))?;
    let result = compute_henon_extended_basin(&target_points, &config)
        .map_err(|error| JsValue::from_str(&error))?;
    serde_wasm_bindgen::to_value(&result)
        .map_err(|error| JsValue::from_str(&format!("Failed to serialize basin result: {error}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> BasinApproximationConfig {
        BasinApproximationConfig {
            a: 0.4,
            b: 0.3,
            epsilon: 0.1,
            bounds: BasinBounds {
                x_min: -2.0,
                x_max: 2.0,
                y_min: -1.5,
                y_max: 1.5,
            },
            grid_x: 8,
            grid_y: 8,
            grid_theta: 16,
            target_position_radius: 0.25,
            target_angle_radius: 0.5,
        }
    }

    #[test]
    fn validation_rejects_invalid_inputs() {
        let mut invalid = config();
        invalid.b = 0.0;
        assert!(invalid.validate().unwrap_err().contains("nonzero"));
        invalid = config();
        invalid.target_position_radius = 0.0;
        assert!(invalid.validate().unwrap_err().contains("position radius"));
        invalid = config();
        invalid.target_angle_radius = PI + 0.1;
        assert!(invalid.validate().unwrap_err().contains("(0, π]"));
        invalid = config();
        invalid.grid_theta = 4;
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn unfinished_automatic_expansion_reports_the_internal_resource_limit() {
        assert_eq!(
            classify_stop_reason(false, false, false, false, false, false),
            BasinStopReason::ResourceLimit
        );
        assert_eq!(
            classify_stop_reason(true, false, false, false, false, true),
            BasinStopReason::NoTrappingCore
        );
    }

    #[test]
    fn point_map_matches_known_henon_boundary_step() {
        let mapped = map_extended_point(
            State3 {
                x: 1.0,
                y: 0.0,
                theta: 0.0,
            },
            &config(),
        )
        .unwrap();
        assert!((mapped.x - 0.6).abs() < 1e-12);
        assert!((mapped.y - 0.4).abs() < 1e-12);
        assert!((mapped.theta - 0.5 * PI).abs() < 1e-12);
    }

    #[test]
    fn extended_inverse_roundtrip_recovers_position_and_normal() {
        let configuration = config();
        for state in [
            State3 {
                x: -0.7,
                y: 0.2,
                theta: 0.15,
            },
            State3 {
                x: 0.4,
                y: -0.3,
                theta: 2.4,
            },
            State3 {
                x: 1.1,
                y: 0.5,
                theta: TAU - 0.08,
            },
        ] {
            let image = map_extended_point(state, &configuration).unwrap();
            let recovered = map_extended_inverse_point(image, &configuration).unwrap();
            assert!((recovered.x - state.x).abs() < 1e-11);
            assert!((recovered.y - state.y).abs() < 1e-11);
            assert!(circular_distance(recovered.theta, state.theta) < 1e-11);
        }
    }

    #[test]
    fn interval_operations_are_outward_and_subtraction_is_correct() {
        let left = Interval::new(1.0, 2.0);
        let right = Interval::new(3.0, 5.0);
        let difference = left.sub(right);
        assert!(difference.lo <= -4.0 && difference.hi >= -1.0);
        let quotient = Interval::new(2.0, 4.0)
            .div(Interval::new(2.0, 3.0))
            .unwrap();
        assert!(quotient.lo <= 2.0 / 3.0 && quotient.hi >= 2.0);
        assert!(Interval::new(-0.1, 0.1).sin().lo <= -0.1_f64.sin());
        assert!(Interval::new(-0.1, 0.1).sin().hi >= 0.1_f64.sin());
    }

    #[test]
    fn cell_enclosure_contains_dense_samples_including_angle_wrap() {
        let configuration = config();
        let grid = Grid3::new(&configuration);
        for &cell_id in &[0, grid.id(3, 4, grid.nt - 1), grid.id(7, 7, 7)] {
            let enclosure = enclose_cell_image(&grid, cell_id, &configuration).unwrap();
            let intervals = grid.intervals(cell_id);
            for sx in 0..=4 {
                for sy in 0..=4 {
                    for st in 0..=4 {
                        let state = State3 {
                            x: intervals[0].lo
                                + (intervals[0].hi - intervals[0].lo) * sx as f64 / 4.0,
                            y: intervals[1].lo
                                + (intervals[1].hi - intervals[1].lo) * sy as f64 / 4.0,
                            theta: intervals[2].lo
                                + (intervals[2].hi - intervals[2].lo) * st as f64 / 4.0,
                        };
                        let image = map_extended_point(state, &configuration).unwrap();
                        assert!(image.x >= enclosure.x.lo && image.x <= enclosure.x.hi);
                        assert!(image.y >= enclosure.y.lo && image.y <= enclosure.y.hi);
                        assert!(
                            circular_distance(image.theta, enclosure.angle_center)
                                <= enclosure.angle_radius + 1e-12
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn inverse_cell_enclosure_contains_dense_preimage_samples() {
        let configuration = config();
        let grid = Grid3::new(&configuration);
        for &cell_id in &[
            grid.id(2, 3, 0),
            grid.id(5, 4, grid.nt - 1),
            grid.id(7, 7, 7),
        ] {
            let enclosure = enclose_cell_inverse_image(&grid, cell_id, &configuration).unwrap();
            let intervals = grid.intervals(cell_id);
            for sx in 0..=4 {
                for sy in 0..=4 {
                    for st in 0..=4 {
                        let state = State3 {
                            x: intervals[0].lo
                                + (intervals[0].hi - intervals[0].lo) * sx as f64 / 4.0,
                            y: intervals[1].lo
                                + (intervals[1].hi - intervals[1].lo) * sy as f64 / 4.0,
                            theta: intervals[2].lo
                                + (intervals[2].hi - intervals[2].lo) * st as f64 / 4.0,
                        };
                        let preimage = map_extended_inverse_point(state, &configuration).unwrap();
                        assert!(preimage.x >= enclosure.x.lo && preimage.x <= enclosure.x.hi);
                        assert!(preimage.y >= enclosure.y.lo && preimage.y <= enclosure.y.hi);
                        assert!(
                            circular_distance(preimage.theta, enclosure.angle_center)
                                <= enclosure.angle_radius + 1e-12
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn lazy_inverse_frontier_evaluates_only_discovered_sources() {
        let configuration = config();
        let grid = Grid3::new(&configuration);
        let seed_id = grid.id(4, 4, 8);
        let mut inner_seed = vec![false; grid.cell_count()];
        let mut outer_seed = vec![false; grid.cell_count()];
        inner_seed[seed_id] = true;
        outer_seed[seed_id] = true;
        let mut graph = SparseTransitionGraph::default();
        ensure_forward_row(&mut graph, &grid, seed_id, &configuration).unwrap();

        let result = expand_lazy_predecessors(
            &grid,
            &configuration,
            &mut graph,
            &inner_seed,
            &outer_seed,
            1,
        )
        .unwrap();

        assert!(result.inverse_frontier_cell_count >= 1);
        assert!(graph.rows.len() < grid.cell_count());
        assert!(graph.rows.contains_key(&seed_id));
    }

    #[test]
    fn lazy_inverse_frontier_respects_eager_level_semantics() {
        let configuration = config();
        let expansion_limit = 2;
        let grid = Grid3::new(&configuration);
        let seed_id = grid.id(4, 4, 8);
        let mut seed = vec![false; grid.cell_count()];
        seed[seed_id] = true;

        let eager_graph = build_transition_graph(&grid, &configuration).unwrap();
        let (eager_inner, _, _) = expand_inner(&eager_graph, &seed, expansion_limit);
        let (eager_outer, _, _) = expand_outer(&eager_graph.reverse, &seed, expansion_limit);

        let mut lazy_graph = SparseTransitionGraph::default();
        ensure_forward_row(&mut lazy_graph, &grid, seed_id, &configuration).unwrap();
        let lazy = expand_lazy_predecessors(
            &grid,
            &configuration,
            &mut lazy_graph,
            &seed,
            &seed,
            expansion_limit,
        )
        .unwrap();

        assert_eq!(lazy.inner_levels, eager_inner);
        for (cell, &lazy_level) in lazy.outer_levels.iter().enumerate() {
            if lazy_level >= 0 {
                assert!(
                    eager_outer[cell] >= 0 && eager_outer[cell] <= lazy_level,
                    "lazy outer cell {cell} was accepted before its eager graph level"
                );
            }
        }
    }

    #[test]
    fn compact_successor_runs_match_expanded_graph_rows() {
        let mut configuration = config();
        configuration.a = 1.4;
        let grid = Grid3::new(&configuration);
        let eager = build_transition_graph(&grid, &configuration).unwrap();

        for source in [0, grid.id(3, 4, 7), grid.cell_count() - 1] {
            let compact = compute_forward_row(&grid, source, &configuration).unwrap();
            assert_eq!(compact.successor_ids(), eager.forward.neighbors(source));
            assert!(compact.successor_runs.len() <= compact.successor_count());
        }
    }

    #[test]
    fn high_nonlinearity_basin_completes_with_compact_transition_storage() {
        let mut configuration = config();
        configuration.a = 1.4;
        configuration.grid_x = 16;
        configuration.grid_y = 16;
        configuration.grid_theta = 8;
        configuration.target_position_radius = 0.6;
        configuration.target_angle_radius = 1.5;
        let targets = [
            BasinTargetPoint {
                x: 0.676,
                y: 0.302,
                nx: 1.0,
                ny: 0.0,
            },
            BasinTargetPoint {
                x: 0.583,
                y: 0.076,
                nx: 0.0,
                ny: 1.0,
            },
            BasinTargetPoint {
                x: -0.298,
                y: 0.248,
                nx: -1.0,
                ny: 0.0,
            },
        ];

        let result = compute_henon_extended_basin(&targets, &configuration).unwrap();
        assert!(result.evaluated_cell_count > 0);
        assert!(result.graph_edge_count > result.transition_run_count);
        assert!(
            result.transition_run_count * 4 < result.graph_edge_count,
            "high-nonlinearity transitions should remain materially compressed"
        );
    }

    #[test]
    fn reverse_graph_is_exact_transpose() {
        let forward = CsrGraph::from_rows(&[vec![1, 2], vec![2], vec![0]]);
        let reverse = build_reverse(&forward, 3).unwrap();
        assert_eq!(reverse.neighbors(0), &[2]);
        assert_eq!(reverse.neighbors(1), &[0]);
        assert_eq!(reverse.neighbors(2), &[0, 1]);
    }

    #[test]
    fn inner_and_outer_expansion_assign_chain_levels() {
        // 0 -> 1 -> 2, with 2 the trapping seed.
        let forward = CsrGraph::from_rows(&[vec![1], vec![2], vec![2]]);
        let reverse = build_reverse(&forward, 3).unwrap();
        let graph = TransitionGraph {
            forward,
            reverse,
            domain_exit: vec![false; 3],
        };
        let seed = vec![false, false, true];
        let (inner, inner_completed, _) = expand_inner(&graph, &seed, 4);
        let (outer, outer_completed, _) = expand_outer(&graph.reverse, &seed, 4);
        assert_eq!(inner, vec![2, 1, 0]);
        assert_eq!(outer, vec![2, 1, 0]);
        assert_eq!(inner_completed, 2);
        assert_eq!(outer_completed, 2);
    }

    #[test]
    fn domain_exit_prevents_false_inner_capture_but_not_outer_reachability() {
        let forward = CsrGraph::from_rows(&[vec![1], vec![1]]);
        let reverse = build_reverse(&forward, 2).unwrap();
        let graph = TransitionGraph {
            forward,
            reverse,
            domain_exit: vec![true, false],
        };
        let seed = vec![false, true];
        let (inner, _, _) = expand_inner(&graph, &seed, 3);
        let (outer, _, _) = expand_outer(&graph.reverse, &seed, 3);
        assert_eq!(inner, vec![UNREACHED, 0]);
        assert_eq!(outer, vec![1, 0]);
    }

    #[test]
    fn projection_reports_angular_coverage_and_level_range() {
        let configuration = BasinApproximationConfig {
            grid_x: 4,
            grid_y: 4,
            grid_theta: 8,
            ..config()
        };
        let grid = Grid3::new(&configuration);
        let mut inner = vec![UNREACHED; grid.cell_count()];
        let mut outer = vec![UNREACHED; grid.cell_count()];
        inner[grid.id(1, 2, 0)] = 3;
        inner[grid.id(1, 2, 1)] = 1;
        for it in 0..4 {
            outer[grid.id(1, 2, it)] = it as i32;
        }
        let projection = project_levels(&grid, &inner, &outer);
        let cell = &projection[2 * grid.nx + 1];
        assert_eq!(cell.inner_coverage, 0.25);
        assert_eq!(cell.outer_coverage, 0.5);
        assert_eq!(cell.min_inner_level, Some(1));
        assert_eq!(cell.max_inner_level, Some(3));
    }

    #[test]
    fn end_to_end_full_trapping_domain_is_computable() {
        let configuration = BasinApproximationConfig {
            a: 0.0,
            b: 0.5,
            epsilon: 0.0,
            bounds: BasinBounds {
                x_min: 0.0,
                x_max: 2.0,
                y_min: -1.0,
                y_max: 1.0,
            },
            grid_x: 4,
            grid_y: 4,
            grid_theta: 8,
            target_position_radius: 100.0,
            target_angle_radius: PI,
        };
        let result = compute_henon_extended_basin(
            &[BasinTargetPoint {
                x: 1.0,
                y: 0.0,
                nx: 1.0,
                ny: 0.0,
            }],
            &configuration,
        )
        .unwrap();
        assert!(result.trapping_verified);
        assert!(result.converged);
        assert_eq!(result.expansion_limit, MAX_AUTOMATIC_EXPANSION_LEVELS);
        assert_eq!(
            result.target_cell_count,
            configuration.grid_x * configuration.grid_y * configuration.grid_theta
        );
        assert_eq!(result.inner_cell_count, result.outer_cell_count);
        assert_eq!(result.unresolved_cell_count, 0);
        assert_eq!(result.inner_levels.len(), 128);
    }

    #[test]
    fn missing_trapping_core_returns_possible_outer_basin_instead_of_error() {
        let mut configuration = config();
        configuration.target_position_radius = 0.8;
        configuration.target_angle_radius = 1.0;
        let result = compute_henon_extended_basin(
            &[BasinTargetPoint {
                x: 0.0,
                y: 0.0,
                nx: 1.0,
                ny: 0.0,
            }],
            &configuration,
        )
        .unwrap();

        assert!(result.candidate_target_cell_count > 0);
        assert!(!result.trapping_verified);
        assert_eq!(result.target_cell_count, 0);
        assert_eq!(result.inner_cell_count, 0);
        assert!(result.outer_cell_count >= result.candidate_target_cell_count);
        assert!(result.converged);
        assert_eq!(result.stop_reason, BasinStopReason::NoTrappingCore);
        assert!(result.inverse_frontier_cell_count > 0);
        assert!(
            result.evaluated_cell_count
                <= configuration.grid_x * configuration.grid_y * configuration.grid_theta
        );
    }
}
