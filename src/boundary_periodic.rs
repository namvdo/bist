use crate::dynamical_systems::{
    DynamicalSystem, ExtendedState, HenonSystem, UserDefinedDynamicalSystem,
};
use crate::parameters::parameter_set_from_js;
use crate::range::{clamp_pair, RANGE_LIMIT};
use crate::ulam::{Grid, UlamComputer};
use core::f64;
use nalgebra::{Matrix5, Vector2, Vector4, Vector5};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use wasm_bindgen::prelude::*;
#[cfg(target_arch = "wasm32")]
use web_sys::console;

#[wasm_bindgen]
#[derive(Debug, Clone, PartialEq, Copy)]
pub enum StabilityType {
    Stable,
    Unstable,
    Saddle,
}

fn log_message(s: &str) {
    #[cfg(target_arch = "wasm32")]
    console::log_1(&s.into());
    #[cfg(not(target_arch = "wasm32"))]
    println!("{}", s);
}

pub const DEFAULT_PERIODIC_GRID_SIZE: usize = 10;
pub const DEFAULT_THETA_GRID_SIZE: usize = 10;
pub const DEFAULT_PERIODIC_RESIDUAL_THRESHOLD: f64 = 1e-10;
const MIN_PERIODIC_GRID_SIZE: usize = 2;
const MAX_PERIODIC_GRID_SIZE: usize = 256;

fn sanitize_grid_size(value: usize, fallback: usize) -> usize {
    if value == 0 {
        return fallback;
    }
    value.clamp(MIN_PERIODIC_GRID_SIZE, MAX_PERIODIC_GRID_SIZE)
}

fn sanitize_residual_threshold(value: f64) -> f64 {
    if !value.is_finite() || value <= 0.0 {
        return DEFAULT_PERIODIC_RESIDUAL_THRESHOLD;
    }
    value.clamp(1e-14, 1e-2)
}

#[derive(Debug, Clone, Copy)]
pub struct BoundaryPoint {
    pub x: f64,
    pub y: f64,
}

// Extended point in the boundary space (x,y,n_x,n_y)
#[wasm_bindgen]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ExtendedPoint {
    pub x: f64,
    pub y: f64,
    pub nx: f64,
    pub ny: f64,
}

impl ExtendedPoint {
    pub fn new(x: f64, y: f64, n_x: f64, n_y: f64) -> Self {
        Self {
            x,
            y,
            nx: n_x,
            ny: n_y,
        }
    }

    pub fn from_angle(x: f64, y: f64, theta: f64) -> Self {
        Self {
            x: x,
            y: y,
            nx: theta.cos(),
            ny: theta.sin(),
        }
    }

    pub fn normalize(&mut self) {
        let norm = (self.nx * self.nx + self.ny * self.ny).sqrt();
        if norm > 1e-12 {
            self.nx /= norm;
            self.ny /= norm;
        }
    }

    pub fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.nx.is_finite() && self.ny.is_finite()
    }

    pub fn is_bounded(&self, max_val: f64) -> bool {
        self.x.abs() < max_val && self.y.abs() < max_val
    }
}

#[derive(Debug, Clone)]
pub struct PeriodicOrbit {
    pub points: Vec<BoundaryPoint>,
    pub extended_points: Vec<ExtendedPoint>,
    pub period: usize,
    pub stability: StabilityType,
    pub eigenvalues: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct PeriodicOrbitDatabase {
    pub orbits: Vec<PeriodicOrbit>,
}

impl PeriodicOrbitDatabase {
    pub fn new() -> Self {
        Self { orbits: Vec::new() }
    }

    pub fn add_orbit(&mut self, orbit: PeriodicOrbit) {
        self.orbits.push(orbit);
    }

    pub fn contains_point(&self, x: f64, y: f64, tol: f64) -> bool {
        self.orbits.iter().any(|orbit| {
            orbit
                .points
                .iter()
                .any(|p| (p.x - x).abs() < tol && (p.y - y).abs() < tol)
        })
    }

    pub fn contains_extended_point(&self, p: &ExtendedPoint, tol: f64) -> bool {
        self.orbits.iter().any(|orbit| {
            orbit.extended_points.iter().any(|ep| {
                let dist = ((ep.x - p.x).powi(2)
                    + (ep.y - p.y).powi(2)
                    + (ep.nx - p.nx).powi(2)
                    + (ep.ny - p.ny).powi(2))
                .sqrt();
                dist < tol
            })
        })
    }

    /// Check if any existing orbit has a point spatially close (x,y only).
    /// This prevents the same physical orbit found with different normal vectors
    /// from being added twice.
    pub fn contains_spatial_point(&self, p: &ExtendedPoint, tol: f64) -> bool {
        self.orbits.iter().any(|orbit| {
            orbit.extended_points.iter().any(|ep| {
                let dist = ((ep.x - p.x).powi(2) + (ep.y - p.y).powi(2)).sqrt();
                dist < tol
            })
        })
    }

    fn find_matching_orbit(&self, x: f64, y: f64, tol: f64) -> Option<(usize, StabilityType, f64)> {
        for orbit in &self.orbits {
            for point in &orbit.points {
                let dist = ((point.x - x).powi(2) + (point.y - y).powi(2)).sqrt();
                if dist < tol {
                    return Some((orbit.period, orbit.stability.clone(), dist));
                }
            }
        }
        None
    }

    pub fn classify_point(&self, x: f64, y: f64, tol: f64) -> PointClassification {
        if let Some((period, stability, distance)) = self.find_matching_orbit(x, y, tol) {
            PointClassification::NearPeriodicOrbit {
                period: period,
                stability: stability,
                distance: distance,
            }
        } else {
            PointClassification::Regular
        }
    }

    pub fn total_count(&self) -> usize {
        self.orbits.len()
    }

    pub fn get_points_of_period(&self, period: usize) -> Vec<BoundaryPoint> {
        self.orbits
            .iter()
            .filter(|orbit| orbit.period == period)
            .flat_map(|o| o.points.clone())
            .collect()
    }

    pub fn get_extended_points_of_period(&self, period: usize) -> Vec<ExtendedPoint> {
        self.orbits
            .iter()
            .filter(|orbit| orbit.period == period)
            .flat_map(|o| o.extended_points.clone())
            .collect()
    }
}

#[derive(Debug, Clone, Copy)]
#[wasm_bindgen]
pub enum PeriodicType {
    Stable,
    Unstable,
    Saddle,
}

pub enum PointClassification {
    Regular,
    NearPeriodicOrbit {
        period: usize,
        stability: StabilityType,
        distance: f64,
    },
}

// 2x2 Jacobian matrix for standard Henon map
#[derive(Debug, Clone, Copy)]
pub struct Jacobian {
    pub j11: f64,
    pub j12: f64,
    pub j21: f64,
    pub j22: f64,
}

impl Jacobian {
    pub fn new(j11: f64, j12: f64, j21: f64, j22: f64) -> Self {
        Self {
            j11: j11,
            j12: j12,
            j21: j21,
            j22: j22,
        }
    }

    pub fn identity() -> Self {
        Self {
            j11: 1.0,
            j12: 0.0,
            j21: 0.0,
            j22: 1.0,
        }
    }

    pub fn multiply(&self, other: &Jacobian) -> Jacobian {
        Jacobian {
            j11: self.j11 * other.j11 + self.j12 * other.j21,
            j12: self.j11 * other.j12 + self.j12 * other.j22,
            j21: self.j21 * other.j11 + self.j22 * other.j21,
            j22: self.j21 * other.j12 + self.j22 * other.j22,
        }
    }

    pub fn eigenvalues(&self) -> (f64, f64, bool) {
        let trace = self.j11 + self.j22;
        let det = self.j11 * self.j22 - self.j12 * self.j21;
        let discriminant = trace * trace - 4.0 * det;

        if discriminant >= 0.0 {
            let sqrt_disc = discriminant.sqrt();
            ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0, false)
        } else {
            let modulus = det.sqrt();
            (modulus, modulus, true)
        }
    }
}

// 4x4 matrix for extended boundary map
#[derive(Copy, Clone)]
pub struct Jacobian4x4 {
    pub data: [[f64; 4]; 4],
}

impl Jacobian4x4 {
    pub fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    pub fn multiply(&self, other: &Jacobian4x4) -> Jacobian4x4 {
        let mut result = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    result[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        Jacobian4x4 { data: result }
    }

    pub fn subtract_identity(&self) -> Jacobian4x4 {
        let mut result = self.data.clone();
        for i in 0..4 {
            result[i][i] -= 1.0;
        }
        return Jacobian4x4 { data: result };
    }

    // Compute eigenvalues of 4x4 matrix using companion matrix approach
    // Returns up to 4 eigenvalue magnitudes
    pub fn eigenvalue_magnitudes(&self) -> Vec<f64> {
        // Compute eigenvalues using QR algorithm approximation
        // For simplicity, we compute characteristic polynomial and find roots

        let a = &self.data;
        // Characteristic polynomial coefficient for 4x4 matrix
        // det(A - lamda * I) = lambda^4 - p1*lambda^3 + p2*lambda^2 - p3*lambda + p4 = 0

        // use trace and other invariants
        let trace = a[0][0] + a[1][1] + a[2][2] + a[3][3];

        let sum_2x2_minors = (a[0][0] * a[1][1] - a[0][1] * a[1][0])
            + (a[0][0] * a[2][2] - a[0][2] * a[2][0])
            + (a[0][0] * a[3][3] - a[0][3] * a[3][0])
            + (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            + (a[1][1] * a[3][3] - a[1][3] * a[3][1])
            + (a[2][2] * a[3][3] - a[2][3] * a[3][2]);

        // sum 3x3 principle minor
        let det_3x3_012 = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

        let det_3x3_013 = a[0][0] * (a[1][1] * a[3][3] - a[1][3] * a[3][1])
            - a[0][1] * (a[1][0] * a[3][3] - a[1][3] * a[3][0])
            + a[0][3] * (a[1][0] * a[3][1] - a[1][1] * a[3][0]);

        let det_3x3_023 = a[0][0] * (a[2][2] * a[3][3] - a[2][3] * a[3][2])
            - a[0][2] * (a[2][0] * a[3][3] - a[2][3] * a[3][0])
            + a[0][3] * (a[2][0] * a[3][2] - a[2][2] * a[3][0]);

        let det_3x3_123 = a[1][1] * (a[2][2] * a[3][3] - a[2][3] * a[3][2])
            - a[1][2] * (a[2][1] * a[3][3] - a[2][3] * a[3][1])
            + a[1][3] * (a[2][1] * a[3][2] - a[2][2] * a[3][1]);
        let sum_3x3_minors = det_3x3_012 + det_3x3_013 + det_3x3_023 + det_3x3_123;

        // 4x4 determinant
        let determinant = self.determinant();

        // characteristic polynomial: lambda^4 - p1*lambda^3 + p2*lambda^2 - p3*lambda + p4 = 0

        let p1 = trace;
        let p2 = sum_2x2_minors;
        let p3 = sum_3x3_minors;
        let p4 = determinant;

        // Find root numerically using companion matrix eigenvalues
        self.find_polynomial_root_quartic(p1, p2, p3, p4)
    }

    pub fn determinant(&self) -> f64 {
        let a = &self.data;
        // Laplace expansion along first row
        let minor00 = a[1][1] * (a[2][2] * a[3][3] - a[2][3] * a[3][2])
            - a[1][2] * (a[2][1] * a[3][3] - a[2][3] * a[3][1])
            + a[1][3] * (a[2][1] * a[3][2] - a[2][2] * a[3][1]);

        let minor01 = a[1][0] * (a[2][2] * a[3][3] - a[2][3] * a[3][2])
            - a[1][2] * (a[2][0] * a[3][3] - a[2][3] * a[3][0])
            + a[1][3] * (a[2][0] * a[3][2] - a[2][2] * a[3][0]);

        let minor02 = a[1][0] * (a[2][1] * a[3][3] - a[2][3] * a[3][1])
            - a[1][1] * (a[2][0] * a[3][3] - a[2][3] * a[3][0])
            + a[1][3] * (a[2][0] * a[3][1] - a[2][1] * a[3][0]);

        let minor03 = a[1][0] * (a[2][1] * a[3][2] - a[2][2] * a[3][1])
            - a[1][1] * (a[2][0] * a[3][2] - a[2][2] * a[3][0])
            + a[1][2] * (a[2][0] * a[3][1] - a[2][1] * a[3][0]);

        a[0][0] * minor00 - a[0][1] * minor01 + a[0][2] * minor02 - a[0][3] * minor03
    }

    pub fn find_polynomial_root_quartic(&self, p1: f64, p2: f64, p3: f64, p4: f64) -> Vec<f64> {
        // Finding roots of x^4 - p1*x^3 + p2*x^2 - p3*x + p4 = 0
        // Using Newton's method with multiple starting points

        let f = |x: f64| x.powi(4) - p1 * x.powi(3) + p2 * x.powi(2) - p3 * x + p4;
        let df = |x: f64| 4.0 * x.powi(3) - 3.0 * p1 * x.powi(2) + 2.0 * p2 * x - p3;

        let mut roots = Vec::new();
        let starts = [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0, 3.0];

        for start in starts {
            let mut x = start;
            for _ in 0..50 {
                let fx = f(x);
                let dfx = df(x);
                if dfx.abs() < 1e-12 {
                    break;
                }
                let x_new = x - fx / dfx;
                if (x_new - x).abs() < 1e-10 {
                    x = x_new;
                    break;
                }
                x = x_new;
            }

            if f(x).abs() < 1e-6 {
                // check if this is a new root
                let is_new = roots.iter().all(|&r: &f64| (r - x).abs() > 0.01);
                if is_new {
                    roots.push(x);
                }
            }
        }
        roots.iter().map(|r| r.abs()).collect()
    }

    // invert 4x4 matrix using Gaussian elimination
    pub fn inverse(&self) -> Option<Jacobian4x4> {
        let mut a = self.data.clone();
        let mut inv = [[0.0; 4]; 4];
        for i in 0..4 {
            inv[i][i] = 1.0;
        }

        for col in 0..4 {
            let mut max_row = col;
            for row in (col + 1)..4 {
                if a[row][col].abs() > a[max_row][col].abs() {
                    max_row = row;
                }
            }

            a.swap(col, max_row);
            inv.swap(col, max_row);

            // check for singular matrix
            if a[col][col].abs() < 1e-12 {
                return None;
            }

            let pivot = a[col][col];
            for j in 0..4 {
                a[col][j] /= pivot;
                inv[col][j] /= pivot;
            }

            for row in 0..4 {
                if row != col {
                    let factor = a[row][col];
                    for j in 0..4 {
                        a[row][j] -= factor * a[col][j];
                        inv[row][j] -= factor * inv[col][j];
                    }
                }
            }
        }
        Some(Jacobian4x4 { data: inv })
    }
}

pub struct TrajectoryPoint {
    pub x: f64,
    pub y: f64,
    pub nx: f64,
    pub ny: f64,
    pub classification: PointClassification,
}

/// Classify stability based on 4D Jacobian eigenvalues
/// For boundary map, one eigenvalue is always 0 (constraint ||n|| = 1)
/// We ignore this eigenvalue and classify based on the remaining 3
pub fn classify_stability_4d(jac: &Jacobian4x4) -> (StabilityType, Vec<f64>) {
    let eigenvalues = jac.eigenvalue_magnitudes();

    // filter out non-zero eigenvalues
    let nonzero_eigenvalues: Vec<f64> = eigenvalues.into_iter().filter(|&e| e > 1e-5).collect();
    if nonzero_eigenvalues.is_empty() {
        return (StabilityType::Stable, vec![]);
    }

    let all_stable = nonzero_eigenvalues.iter().all(|&e| e < 0.999);
    let all_unstable = nonzero_eigenvalues.iter().all(|&e| e > 1.001);

    let stability = if all_stable {
        StabilityType::Stable
    } else if all_unstable {
        StabilityType::Unstable
    } else {
        StabilityType::Saddle
    };

    (stability, nonzero_eigenvalues)
}

/// WASM-exposed boundary map for Hénon, routing through the generic pipeline.
#[wasm_bindgen(js_name = "boundary_map")]
pub fn boundary_map_henon(
    x: f64,
    y: f64,
    nx: f64,
    ny: f64,
    a: f64,
    b: f64,
    ep: f64,
) -> ExtendedPoint {
    let system = HenonSystem::new(a, b, ep);
    boundary_map_generic(&system, x, y, nx, ny)
}

pub struct BoundaryHenonSystemAnalysis {
    pub a: f64,
    pub b: f64,
    pub epsilon: f64,
    pub orbit_database: PeriodicOrbitDatabase,
    pub trajectory: Vec<TrajectoryPoint>,
}

impl BoundaryHenonSystemAnalysis {
    pub fn new(
        a: f64,
        b: f64,
        epsilon: f64,
        max_period: usize,
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
    ) -> Self {
        Self::new_with_search_settings(
            a,
            b,
            epsilon,
            max_period,
            x_min,
            x_max,
            y_min,
            y_max,
            DEFAULT_PERIODIC_GRID_SIZE,
            DEFAULT_THETA_GRID_SIZE,
            DEFAULT_PERIODIC_RESIDUAL_THRESHOLD,
        )
    }

    pub fn new_with_search_settings(
        a: f64,
        b: f64,
        epsilon: f64,
        max_period: usize,
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
        grid_size: usize,
        theta_grid_size: usize,
        residual_threshold: f64,
    ) -> Self {
        let grid_size = sanitize_grid_size(grid_size, DEFAULT_PERIODIC_GRID_SIZE);
        let theta_grid_size = sanitize_grid_size(theta_grid_size, DEFAULT_THETA_GRID_SIZE);
        let residual_threshold = sanitize_residual_threshold(residual_threshold);

        let system = HenonSystem::new(a, b, epsilon);
        let orbit_database = find_all_boundary_periodic_orbits_generic_with_threshold(
            &system,
            max_period,
            grid_size,
            theta_grid_size,
            x_min,
            x_max,
            y_min,
            y_max,
            residual_threshold,
        );
        log_message(&format!(
            "Total orbits found (boundary map): {}",
            orbit_database.total_count()
        ));

        Self {
            a,
            b,
            epsilon,
            orbit_database,
            trajectory: Vec::new(),
        }
    }

    pub fn track_trajectory(
        &mut self,
        initial_x: f64,
        initial_y: f64,
        initial_nx: f64,
        initial_ny: f64,
        max_iterations: usize,
    ) {
        self.trajectory.clear();

        let mut x = initial_x;
        let mut y = initial_y;
        let mut nx = initial_nx;
        let mut ny = initial_ny;

        // normalize initial normal
        let norm = (nx * nx + ny * ny).sqrt();
        if norm > 1e-12 {
            nx /= norm;
            ny /= norm;
        }

        let classification = self.orbit_database.classify_point(x, y, 0.005);
        self.trajectory.push(TrajectoryPoint {
            x,
            y,
            nx,
            ny,
            classification,
        });

        let system = HenonSystem::new(self.a, self.b, self.epsilon);
        for iter in 1..=max_iterations {
            let next_point = boundary_map_generic(&system, x, y, nx, ny);

            if !next_point.is_finite() || !next_point.is_bounded(100.0) {
                log_message(&format!("Point diverged at iteration {}", iter));
                break;
            }
            let classification =
                self.orbit_database
                    .classify_point(next_point.x, next_point.y, 1e-4);
            self.trajectory.push(TrajectoryPoint {
                x: next_point.x,
                y: next_point.y,
                nx: next_point.nx,
                ny: next_point.ny,
                classification,
            });

            x = next_point.x;
            y = next_point.y;
            nx = next_point.nx;
            ny = next_point.ny;
        }

        log_message(&format!(
            "Trajectory complete. Total points: {}",
            self.trajectory.len()
        ));
    }
}

#[derive(Serialize, Deserialize)]
pub struct TrajectoryPointJS {
    pub x: f64,
    pub y: f64,
    pub nx: f64,
    pub ny: f64,
    pub classification: String,
    pub period: Option<usize>,
    pub stability: Option<String>,
}

#[derive(Serialize, Deserialize)]
pub struct PeriodicOrbitJS {
    pub points: Vec<(f64, f64)>,
    pub extended_points: Vec<(f64, f64, f64, f64)>,
    pub period: usize,
    pub stability: String,
    pub eigenvalues: Vec<f64>,
}

impl From<&StabilityType> for String {
    fn from(stability: &StabilityType) -> Self {
        match stability {
            StabilityType::Stable => "stable".to_string(),
            StabilityType::Unstable => "unstable".to_string(),
            StabilityType::Saddle => "saddle".to_string(),
        }
    }
}

impl From<&TrajectoryPoint> for TrajectoryPointJS {
    fn from(point: &TrajectoryPoint) -> Self {
        match &point.classification {
            PointClassification::Regular => TrajectoryPointJS {
                x: point.x,
                y: point.y,
                nx: point.nx,
                ny: point.ny,
                classification: "regular".to_string(),
                period: None,
                stability: None,
            },
            PointClassification::NearPeriodicOrbit {
                period,
                stability,
                distance: _,
            } => TrajectoryPointJS {
                x: point.x,
                y: point.y,
                nx: point.nx,
                ny: point.ny,
                classification: "periodic".to_string(),
                period: Some(*period),
                stability: Some(String::from(stability)),
            },
        }
    }
}

#[wasm_bindgen]
pub struct BoundaryHenonSystemWasm {
    system: BoundaryHenonSystemAnalysis,
    current_iteration: usize,
}

#[wasm_bindgen]
impl BoundaryHenonSystemWasm {
    #[wasm_bindgen(constructor)]
    pub fn new(
        a: f64,
        b: f64,
        epsilon: f64,
        max_period: usize,
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
        grid_size: Option<usize>,
        theta_grid_size: Option<usize>,
        residual_threshold: Option<f64>,
    ) -> Result<BoundaryHenonSystemWasm, JsValue> {
        console_error_panic_hook::set_once();

        let system = BoundaryHenonSystemAnalysis::new_with_search_settings(
            a,
            b,
            epsilon,
            max_period,
            x_min,
            x_max,
            y_min,
            y_max,
            grid_size.unwrap_or(DEFAULT_PERIODIC_GRID_SIZE),
            theta_grid_size.unwrap_or(DEFAULT_THETA_GRID_SIZE),
            residual_threshold.unwrap_or(DEFAULT_PERIODIC_RESIDUAL_THRESHOLD),
        );
        Ok(Self {
            system,
            current_iteration: 0,
        })
    }

    #[wasm_bindgen(js_name = getPeriodicOrbits)]
    pub fn get_periodic_orbits(&self) -> Result<JsValue, JsValue> {
        let orbits: Vec<PeriodicOrbitJS> = self
            .system
            .orbit_database
            .orbits
            .iter()
            .map(|orbit| PeriodicOrbitJS {
                points: orbit.points.iter().map(|p| (p.x, p.y)).collect(),
                extended_points: orbit
                    .extended_points
                    .iter()
                    .map(|p| (p.x, p.y, p.nx, p.ny))
                    .collect(),
                period: orbit.period,
                stability: String::from(&orbit.stability),
                eigenvalues: orbit.eigenvalues.clone(),
            })
            .collect();
        serde_wasm_bindgen::to_value(&orbits)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
    }
    #[wasm_bindgen(js_name = "trackTrajectory")]
    pub fn track_trajectory(
        &mut self,
        initial_x: f64,
        initial_y: f64,
        initial_nx: f64,
        initial_ny: f64,
        max_iterations: usize,
    ) {
        self.system
            .track_trajectory(initial_x, initial_y, initial_nx, initial_ny, max_iterations);
        self.current_iteration = 0;
    }

    #[wasm_bindgen(js_name = "getCurrentPoint")]
    pub fn get_current_point(&self) -> Result<JsValue, JsValue> {
        if self.current_iteration < self.system.trajectory.len() {
            let point = &self.system.trajectory[self.current_iteration];
            let point_js = TrajectoryPointJS::from(point);

            serde_wasm_bindgen::to_value(&point_js)
                .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
        } else {
            Ok(JsValue::NULL)
        }
    }

    #[wasm_bindgen(js_name = "getTrajectory")]
    pub fn get_trajectory(&self, start: usize, end: usize) -> Result<JsValue, JsValue> {
        let end = end.min(self.system.trajectory.len());
        let points: Vec<TrajectoryPointJS> = self.system.trajectory[start..end]
            .iter()
            .map(TrajectoryPointJS::from)
            .collect();

        serde_wasm_bindgen::to_value(&points)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
    }

    #[wasm_bindgen()]
    pub fn step(&mut self) -> bool {
        if self.current_iteration + 1 < self.system.trajectory.len() {
            self.current_iteration += 1;
            true
        } else {
            false
        }
    }

    #[wasm_bindgen]
    pub fn reset(&mut self) {
        self.current_iteration = 0;
    }

    #[wasm_bindgen(js_name = "getTotalIterations")]
    pub fn get_total_iterations(&self) -> usize {
        self.system.trajectory.len()
    }

    #[wasm_bindgen(js_name = "getCurrentIteration")]
    pub fn get_current_iteration(&self) -> usize {
        self.current_iteration
    }

    #[wasm_bindgen(js_name = "getOrbitCount")]
    pub fn get_orbit_count(&self) -> usize {
        self.system.orbit_database.total_count()
    }

    #[wasm_bindgen(js_name = "getEpsilon")]
    pub fn get_epsilon(&self) -> f64 {
        self.system.epsilon
    }
}

fn boundary_map_generic(
    system: &dyn DynamicalSystem,
    x: f64,
    y: f64,
    nx: f64,
    ny: f64,
) -> ExtendedPoint {
    let state = ExtendedState {
        pos: Vector2::new(x, y),
        normal: Vector2::new(nx, ny),
    };
    match system.extended_map(state, 1) {
        Ok(next) => ExtendedPoint {
            x: next.pos.x,
            y: next.pos.y,
            nx: next.normal.x,
            ny: next.normal.y,
        },
        Err(_) => ExtendedPoint {
            x: f64::NAN,
            y: f64::NAN,
            nx: f64::NAN,
            ny: f64::NAN,
        },
    }
}

fn boundary_map_jacobian_generic(
    system: &dyn DynamicalSystem,
    x: f64,
    y: f64,
    nx: f64,
    ny: f64,
) -> Jacobian4x4 {
    let h = 1e-6;
    let vars = [x, y, nx, ny];
    let mut data = [[0.0; 4]; 4];

    for j in 0..4 {
        let mut vars_plus = vars;
        let mut vars_minus = vars;
        vars_plus[j] += h;
        vars_minus[j] -= h;

        let f_plus = boundary_map_generic(
            system,
            vars_plus[0],
            vars_plus[1],
            vars_plus[2],
            vars_plus[3],
        );
        let f_minus = boundary_map_generic(
            system,
            vars_minus[0],
            vars_minus[1],
            vars_minus[2],
            vars_minus[3],
        );

        let f_plus_arr = [f_plus.x, f_plus.y, f_plus.nx, f_plus.ny];
        let f_minus_arr = [f_minus.x, f_minus.y, f_minus.nx, f_minus.ny];

        for i in 0..4 {
            data[i][j] = (f_plus_arr[i] - f_minus_arr[i]) / (2.0 * h);
        }
    }

    Jacobian4x4 { data }
}

fn compose_boundary_map_n_times_generic(
    system: &dyn DynamicalSystem,
    p: ExtendedPoint,
    n: usize,
) -> (ExtendedPoint, Jacobian4x4) {
    if n == 0 {
        return (p, Jacobian4x4::identity());
    }

    let mut accumulated_jacobian = boundary_map_jacobian_generic(system, p.x, p.y, p.nx, p.ny);
    let mut current = boundary_map_generic(system, p.x, p.y, p.nx, p.ny);

    for _ in 1..n {
        if !current.is_finite() || !current.is_bounded(1e10) {
            return (
                ExtendedPoint::new(f64::NAN, f64::NAN, f64::NAN, f64::NAN),
                Jacobian4x4::identity(),
            );
        }

        let jac_current =
            boundary_map_jacobian_generic(system, current.x, current.y, current.nx, current.ny);
        accumulated_jacobian = jac_current.multiply(&accumulated_jacobian);
        current = boundary_map_generic(system, current.x, current.y, current.nx, current.ny);
    }

    (current, accumulated_jacobian)
}

fn find_boundary_periodic_point_davidchack_lai_generic(
    system: &dyn DynamicalSystem,
    x0: f64,
    y0: f64,
    nx_0: f64,
    ny_0: f64,
    period: usize,
    beta: Option<f64>,
    max_iter: usize,
    tol: f64, // tolerance for how small the correct step can be 
    residual_threshold: f64,
) -> Option<ExtendedPoint> {
    let mut x = x0;
    let mut y = y0;
    let mut nx = nx_0;
    let mut ny = ny_0;

    let beta_val = beta.unwrap_or_else(|| 15.0 * 1.3_f64.powi(period as i32));

    for _ in 0..max_iter {
        if !x.is_finite()
            || !y.is_finite()
            || !nx.is_finite()
            || !ny.is_finite()
            || x.abs() > 100.0
            || y.abs() > 100.0
        {
            return None;
        }

        let current = ExtendedPoint::new(x, y, nx, ny);
        let (mapped, jac_fn) = compose_boundary_map_n_times_generic(system, current, period);

        if !mapped.is_finite() {
            return None;
        }

        let gx = mapped.x - x;
        let gy = mapped.y - y;
        let gnx = mapped.nx - nx;
        let gny = mapped.ny - ny;

        let g_norm = (gx * gx + gy * gy + gnx * gnx + gny * gny).sqrt();

        if g_norm < residual_threshold {
            return Some(current);
        }

        let jac_g = jac_fn.subtract_identity();
        let scaled_beta = beta_val * g_norm;

        let mut modified_jac = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                modified_jac[i][j] = -jac_g.data[i][j];
            }
            modified_jac[i][i] += scaled_beta;
        }
        let modified_jac = Jacobian4x4 { data: modified_jac };
        let jac_inv = match modified_jac.inverse() {
            Some(inv) => inv,
            None => return None,
        };

        let dx = jac_inv.data[0][0] * gx
            + jac_inv.data[0][1] * gy
            + jac_inv.data[0][2] * gnx
            + jac_inv.data[0][3] * gny;
        let dy = jac_inv.data[1][0] * gx
            + jac_inv.data[1][1] * gy
            + jac_inv.data[1][2] * gnx
            + jac_inv.data[1][3] * gny;
        let dnx = jac_inv.data[2][0] * gx
            + jac_inv.data[2][1] * gy
            + jac_inv.data[2][2] * gnx
            + jac_inv.data[2][3] * gny;
        let dny = jac_inv.data[3][0] * gx
            + jac_inv.data[3][1] * gy
            + jac_inv.data[3][2] * gnx
            + jac_inv.data[3][3] * gny;

        if !dx.is_finite() || !dy.is_finite() || !dnx.is_finite() || !dny.is_finite() {
            return None;
        }

        x += dx;
        y += dy;
        nx += dnx;
        ny += dny;

        let norm = (nx * nx + ny * ny).sqrt();
        if norm <= 1e-12 || !norm.is_finite() {
            return None;
        }
        nx /= norm;
        ny /= norm;

        let delta_norm = (dx * dx + dy * dy + dnx * dnx + dny * dny).sqrt();
        if delta_norm < tol {
            break;
        }
    }

    let final_point = ExtendedPoint::new(x, y, nx, ny);
    let (mapped, _) = compose_boundary_map_n_times_generic(system, final_point, period);

    let dist_sq = (mapped.x - x).powi(2)
        + (mapped.y - y).powi(2)
        + (mapped.nx - nx).powi(2)
        + (mapped.ny - ny).powi(2);

    if dist_sq <= residual_threshold * residual_threshold {
        Some(final_point)
    } else {
        None
    }
}

fn davidchack_lai_boundary_map_generic(
    system: &dyn DynamicalSystem,
    max_period: usize,
    grid_size: usize,
    theta_grid_size: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    residual_threshold: f64,
) -> PeriodicOrbitDatabase {
    let mut database = PeriodicOrbitDatabase::new();

    let (x_min, x_max) = clamp_pair(x_min, x_max, RANGE_LIMIT);
    let (y_min, y_max) = clamp_pair(y_min, y_max, RANGE_LIMIT);
    let x_range = (x_min, x_max);
    let y_range = (y_min, y_max);
    let theta_range = (0.0, 2.0 * PI);

    for period in 1..=max_period {
        log_message(&format!("Searching for period {} orbits...", period));

        let mut found_count = 0;

        for i in 0..grid_size {
            for j in 0..grid_size {
                for k in 0..theta_grid_size {
                    let x0 =
                        x_range.0 + (x_range.1 - x_range.0) * (i as f64 + 0.5) / (grid_size as f64);
                    let y0 =
                        y_range.0 + (y_range.1 - y_range.0) * (j as f64 + 0.5) / (grid_size as f64);
                    let theta = theta_range.0
                        + (theta_range.1 - theta_range.0) * (k as f64 + 0.5)
                            / (theta_grid_size as f64);
                    let nx0 = theta.cos();
                    let ny0 = theta.sin();

                    if let Some(fixed_point) = find_boundary_periodic_point_davidchack_lai_generic(
                        system,
                        x0,
                        y0,
                        nx0,
                        ny0,
                        period,
                        None,
                        100,
                        1e-10,
                        residual_threshold,
                    ) {
                        if !fixed_point.is_bounded(100.0) {
                            continue;
                        }

                        if database.contains_spatial_point(&fixed_point, 0.01) {
                            continue;
                        }

                        let mut orbit_points = vec![BoundaryPoint {
                            x: fixed_point.x,
                            y: fixed_point.y,
                        }];
                        let mut extended_orbit_points = vec![fixed_point];
                        let mut current = fixed_point;

                        for _ in 1..period {
                            current = boundary_map_generic(
                                system, current.x, current.y, current.nx, current.ny,
                            );
                            orbit_points.push(BoundaryPoint {
                                x: current.x,
                                y: current.y,
                            });
                            extended_orbit_points.push(current);
                        }

                        let (_, jac_fn) =
                            compose_boundary_map_n_times_generic(system, fixed_point, period);
                        let (stability, eigenvalues) = classify_stability_4d(&jac_fn);

                        database.add_orbit(PeriodicOrbit {
                            points: orbit_points,
                            extended_points: extended_orbit_points,
                            period,
                            stability,
                            eigenvalues,
                        });
                        found_count += 1;
                    }
                }
            }
        }
        log_message(&format!(
            "Found {} orbits of period: {}",
            found_count, period
        ));
    }
    database
}

fn verify_minimal_period_generic(
    system: &dyn DynamicalSystem,
    point: &ExtendedPoint,
    claimed_period: usize,
    residual_threshold: f64,
) -> bool {
    for divisor in 1..claimed_period {
        if claimed_period % divisor == 0 {
            let (mapped, _) = compose_boundary_map_n_times_generic(system, *point, divisor);
            let dist = (mapped.x - point.x).powi(2)
                + (mapped.y - point.y).powi(2)
                + (mapped.nx - point.nx).powi(2)
                + (mapped.ny - point.ny).powi(2);
            if dist <= residual_threshold * residual_threshold {
                return false;
            }
        }
    }
    true
}

fn try_add_orbit_generic(
    system: &dyn DynamicalSystem,
    database: &mut PeriodicOrbitDatabase,
    fp: ExtendedPoint,
    period: usize,
    residual_threshold: f64,
) -> bool {
    if !fp.is_finite() {
        return false;
    }
    if !fp.is_bounded(100.0) {
        return false;
    }
    // Deduplicate using spatial (x,y) distance only — the same orbit can be
    // found from different initial normal vectors, yielding different (nx,ny)
    // but the same physical periodic point.
    if database.contains_spatial_point(&fp, 0.01) {
        return false;
    }
    if !verify_minimal_period_generic(system, &fp, period, residual_threshold) {
        return false;
    }

    let mut orbit_points = vec![BoundaryPoint { x: fp.x, y: fp.y }];
    let mut extended_orbit_points = vec![fp];
    let mut current = fp;

    for _ in 1..period {
        current = boundary_map_generic(system, current.x, current.y, current.nx, current.ny);
        orbit_points.push(BoundaryPoint {
            x: current.x,
            y: current.y,
        });
        extended_orbit_points.push(current);
    }

    let (_, jac_fn) = compose_boundary_map_n_times_generic(system, fp, period);
    let (stability, eigenvalues) = classify_stability_4d(&jac_fn);

    database.add_orbit(PeriodicOrbit {
        points: orbit_points,
        extended_points: extended_orbit_points,
        period,
        stability,
        eigenvalues,
    });
    true
}

pub fn find_all_boundary_periodic_orbits_generic(
    system: &dyn DynamicalSystem,
    max_period: usize,
    grid_size: usize,
    theta_grid_size: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> PeriodicOrbitDatabase {
    find_all_boundary_periodic_orbits_generic_with_threshold(
        system,
        max_period,
        grid_size,
        theta_grid_size,
        x_min,
        x_max,
        y_min,
        y_max,
        DEFAULT_PERIODIC_RESIDUAL_THRESHOLD,
    )
}

pub fn find_all_boundary_periodic_orbits_generic_with_threshold(
    system: &dyn DynamicalSystem,
    max_period: usize,
    grid_size: usize,
    theta_grid_size: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    residual_threshold: f64,
) -> PeriodicOrbitDatabase {
    let mut database = PeriodicOrbitDatabase::new();
    let residual_threshold = sanitize_residual_threshold(residual_threshold);

    let (x_min, x_max) = clamp_pair(x_min, x_max, RANGE_LIMIT);
    let (y_min, y_max) = clamp_pair(y_min, y_max, RANGE_LIMIT);
    let x_range = (x_min, x_max);
    let y_range = (y_min, y_max);
    let theta_range = (0.0, 2.0 * PI);

    for period in 1..=max_period {
        let gs = grid_size;
        let ts = theta_grid_size;

        log_message(&format!(
            "Searching period {} orbits (grid {}x{}x{})...",
            period, gs, gs, ts
        ));

        let mut found_count = 0;

        // stage 1: uniform grid search
        for i in 0..gs {
            for j in 0..gs {
                for k in 0..ts {
                    let x0 = x_range.0 + (x_range.1 - x_range.0) * (i as f64 + 0.5) / (gs as f64);
                    let y0 = y_range.0 + (y_range.1 - y_range.0) * (j as f64 + 0.5) / (gs as f64);
                    let theta = theta_range.0
                        + (theta_range.1 - theta_range.0) * (k as f64 + 0.5) / (ts as f64);
                    let nx0 = theta.cos();
                    let ny0 = theta.sin();

                    if let Some(fp) = find_boundary_periodic_point_davidchack_lai_generic(
                        system,
                        x0,
                        y0,
                        nx0,
                        ny0,
                        period,
                        None,
                        150,
                        1e-12,
                        residual_threshold,
                    ) {
                        if try_add_orbit_generic(
                            system,
                            &mut database,
                            fp,
                            period,
                            residual_threshold,
                        ) {
                            found_count += 1;
                        }
                    }
                }
            }
        }

        // stage 2: continuation — perturb known orbit points as seeds
        let known: Vec<ExtendedPoint> = database
            .orbits
            .iter()
            .flat_map(|o| o.extended_points.iter().copied())
            .collect();
        let perturbations = [0.01, -0.01, 0.005, -0.005];
        for ep in &known {
            for &dp in &perturbations {
                for &(dx, dy) in &[(dp, 0.0), (0.0, dp)] {
                    if let Some(fp) = find_boundary_periodic_point_davidchack_lai_generic(
                        system,
                        ep.x + dx,
                        ep.y + dy,
                        ep.nx,
                        ep.ny,
                        period,
                        None,
                        150,
                        1e-12,
                        residual_threshold,
                    ) {
                        if try_add_orbit_generic(
                            system,
                            &mut database,
                            fp,
                            period,
                            residual_threshold,
                        ) {
                            found_count += 1;
                        }
                    }
                }
            }
        }

        log_message(&format!(
            "Found {} orbits of period {}",
            found_count, period
        ));
    }

    log_message(&format!(
        "Total boundary map orbits: {}",
        database.total_count()
    ));
    database
}

/// convert PeriodicOrbitDatabase to Vec<FoundPeriodicOrbit>.
fn database_to_found_orbits_generic(db: &PeriodicOrbitDatabase) -> Vec<FoundPeriodicOrbit> {
    db.orbits
        .iter()
        .map(|orbit| FoundPeriodicOrbit {
            points: orbit.points.iter().map(|p| (p.x, p.y)).collect(),
            extended_points: orbit
                .extended_points
                .iter()
                .map(|p| (p.x, p.y, p.nx, p.ny))
                .collect(),
            period: orbit.period,
            stability: String::from(&orbit.stability),
            eigenvalues: orbit.eigenvalues.clone(),
        })
        .collect()
}

pub fn parameter_sweep_henon_fast(
    base_params: &[(String, f64)],
    sweep_param_name: &str,
    sweep_min: f64,
    sweep_max: f64,
    num_samples: usize,
    epsilon: f64,
    max_period: usize,
    grid_size: usize,
    theta_grid_size: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> ParameterSweepResult {
    let mut results = Vec::with_capacity(num_samples);

    let get_param = |name: &str, override_name: &str, override_val: f64| -> f64 {
        if name == override_name {
            override_val
        } else {
            base_params
                .iter()
                .find(|(n, _)| n == name)
                .map(|(_, v)| *v)
                .unwrap_or(0.0)
        }
    };

    for i in 0..num_samples {
        let sweep_val = if num_samples <= 1 {
            sweep_min
        } else {
            sweep_min + (sweep_max - sweep_min) * (i as f64) / ((num_samples - 1) as f64)
        };

        log_message(&format!(
            "Sweep {}={:.4} ({}/{})",
            sweep_param_name,
            sweep_val,
            i + 1,
            num_samples
        ));

        let a = get_param("a", sweep_param_name, sweep_val);
        let b = get_param("b", sweep_param_name, sweep_val);
        let epsilon_at_sample = if sweep_param_name == "epsilon" {
            sweep_val
        } else {
            epsilon
        };

        let system = HenonSystem::new(a, b, epsilon_at_sample);
        let db = find_all_boundary_periodic_orbits_generic(
            &system,
            max_period,
            grid_size,
            theta_grid_size,
            x_min,
            x_max,
            y_min,
            y_max,
        );

        let orbits = database_to_found_orbits_generic(&db);
        let stable_count = orbits.iter().filter(|o| o.stability == "stable").count();
        let unstable_count = orbits.iter().filter(|o| o.stability == "unstable").count();
        let saddle_count = orbits.iter().filter(|o| o.stability == "saddle").count();

        results.push(SweepResult {
            param_value: sweep_val,
            total_orbits: orbits.len(),
            stable_count,
            unstable_count,
            saddle_count,
            orbits,
        });
    }

    ParameterSweepResult {
        param_name: sweep_param_name.to_string(),
        param_min: sweep_min,
        param_max: sweep_max,
        num_samples,
        b: base_params
            .iter()
            .find(|(n, _)| n == "b")
            .map(|(_, v)| *v)
            .unwrap_or(0.0),
        epsilon,
        max_period,
        results,
    }
}

/// run parameter sweep over a named parameter using the generic pipeline.
pub fn parameter_sweep_generic(
    x_eq: &str,
    y_eq: &str,
    base_params: &[(String, f64)],
    sweep_param_name: &str,
    sweep_min: f64,
    sweep_max: f64,
    num_samples: usize,
    epsilon: f64,
    max_period: usize,
    grid_size: usize,
    theta_grid_size: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> ParameterSweepResult {
    let mut results = Vec::with_capacity(num_samples);

    for i in 0..num_samples {
        let sweep_val = if num_samples <= 1 {
            sweep_min
        } else {
            sweep_min + (sweep_max - sweep_min) * (i as f64) / ((num_samples - 1) as f64)
        };

        log_message(&format!(
            "Sweep {}={:.4} ({}/{})",
            sweep_param_name,
            sweep_val,
            i + 1,
            num_samples
        ));

        // Build parameter set with the swept value
        let mut entries = Vec::new();
        for (name, value) in base_params {
            if name == sweep_param_name {
                entries.push(crate::parameters::ParameterEntry {
                    name: name.clone(),
                    value: sweep_val,
                });
            } else {
                entries.push(crate::parameters::ParameterEntry {
                    name: name.clone(),
                    value: *value,
                });
            }
        }

        let param_set = match crate::parameters::ParameterSet::new(entries) {
            Ok(ps) => ps,
            Err(e) => {
                log_message(&format!("Parameter error at sample {}: {}", i, e));
                continue;
            }
        };

        let system = match UserDefinedDynamicalSystem::new(x_eq, y_eq, epsilon, param_set) {
            Ok(s) => s,
            Err(e) => {
                log_message(&format!("System error at sample {}: {}", i, e));
                continue;
            }
        };

        let db = find_all_boundary_periodic_orbits_generic(
            &system,
            max_period,
            grid_size,
            theta_grid_size,
            x_min,
            x_max,
            y_min,
            y_max,
        );

        let orbits = database_to_found_orbits_generic(&db);
        let stable_count = orbits.iter().filter(|o| o.stability == "stable").count();
        let unstable_count = orbits.iter().filter(|o| o.stability == "unstable").count();
        let saddle_count = orbits.iter().filter(|o| o.stability == "saddle").count();

        results.push(SweepResult {
            param_value: sweep_val,
            total_orbits: orbits.len(),
            stable_count,
            unstable_count,
            saddle_count,
            orbits,
        });
    }

    ParameterSweepResult {
        param_name: sweep_param_name.to_string(),
        param_min: sweep_min,
        param_max: sweep_max,
        num_samples,
        b: base_params
            .iter()
            .find(|(n, _)| n == "b")
            .map(|(_, v)| *v)
            .unwrap_or(0.0),
        epsilon,
        max_period,
        results,
    }
}

pub fn filter_orbits_by_ulam_support(
    database: PeriodicOrbitDatabase,
    invariant_measure: &[f64],
    subdivisions: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    threshold: f64,
) -> PeriodicOrbitDatabase {
    if subdivisions == 0 || invariant_measure.is_empty() {
        return database;
    }

    let dx = (x_max - x_min) / (subdivisions as f64);
    let dy = (y_max - y_min) / (subdivisions as f64);
    if !dx.is_finite() || !dy.is_finite() || dx <= 0.0 || dy <= 0.0 {
        return database;
    }

    let on_support = |x: f64, y: f64| -> bool {
        if x < x_min || x > x_max || y < y_min || y > y_max {
            return false;
        }
        let mut ix = ((x - x_min) / dx).floor() as isize;
        let mut iy = ((y - y_min) / dy).floor() as isize;

        if ix == subdivisions as isize {
            ix -= 1;
        }
        if iy == subdivisions as isize {
            iy -= 1;
        }

        if ix < 0 || iy < 0 {
            return false;
        }

        let idx = (iy as usize) * subdivisions + (ix as usize);
        invariant_measure.get(idx).copied().unwrap_or(0.0) > threshold
    };

    let mut filtered = PeriodicOrbitDatabase::new();
    for orbit in database.orbits {
        if orbit
            .points
            .iter()
            .all(|point| on_support(point.x, point.y))
        {
            filtered.add_orbit(orbit);
        }
    }
    return filtered;
}

#[wasm_bindgen]
pub struct BoundaryUserDefinedSystemWasm {
    orbit_database: PeriodicOrbitDatabase,
    system: UserDefinedDynamicalSystem,
    current_iteration: usize,
    trajectory: Vec<TrajectoryPoint>,
}

#[wasm_bindgen]
impl BoundaryUserDefinedSystemWasm {
    #[wasm_bindgen(constructor)]
    pub fn new(
        x_eq: &str,
        y_eq: &str,
        params: JsValue,
        epsilon: f64,
        max_period: usize,
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
        grid_size: Option<usize>,
        theta_grid_size: Option<usize>,
        residual_threshold: Option<f64>,
    ) -> Result<BoundaryUserDefinedSystemWasm, JsValue> {
        console_error_panic_hook::set_once();

        let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
        let system = UserDefinedDynamicalSystem::new(x_eq, y_eq, epsilon, param_set)
            .map_err(|e| JsValue::from_str(&format!("Error parsing equations: {}", e)))?;

        let grid_size = sanitize_grid_size(
            grid_size.unwrap_or(DEFAULT_PERIODIC_GRID_SIZE),
            DEFAULT_PERIODIC_GRID_SIZE,
        );
        let theta_grid_size = sanitize_grid_size(
            theta_grid_size.unwrap_or(DEFAULT_THETA_GRID_SIZE),
            DEFAULT_THETA_GRID_SIZE,
        );
        let residual_threshold =
            sanitize_residual_threshold(residual_threshold.unwrap_or(DEFAULT_PERIODIC_RESIDUAL_THRESHOLD));
        let orbit_database = davidchack_lai_boundary_map_generic(
            &system,
            max_period,
            grid_size,
            theta_grid_size,
            x_min,
            x_max,
            y_min,
            y_max,
            residual_threshold,
        );

        log_message(&format!(
            "Total orbits found (user-defined boundary map): {}",
            orbit_database.total_count()
        ));

        Ok(Self {
            orbit_database,
            system,
            current_iteration: 0,
            trajectory: Vec::new(),
        })
    }

    #[wasm_bindgen(js_name = getPeriodicOrbits)]
    pub fn get_periodic_orbits(&self) -> Result<JsValue, JsValue> {
        let orbits: Vec<PeriodicOrbitJS> = self
            .orbit_database
            .orbits
            .iter()
            .map(|orbit| PeriodicOrbitJS {
                points: orbit.points.iter().map(|p| (p.x, p.y)).collect(),
                extended_points: orbit
                    .extended_points
                    .iter()
                    .map(|p| (p.x, p.y, p.nx, p.ny))
                    .collect(),
                period: orbit.period,
                stability: String::from(&orbit.stability),
                eigenvalues: orbit.eigenvalues.clone(),
            })
            .collect();
        serde_wasm_bindgen::to_value(&orbits)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
    }

    #[wasm_bindgen(js_name = "getOrbitCount")]
    pub fn get_orbit_count(&self) -> usize {
        self.orbit_database.total_count()
    }

    #[wasm_bindgen(js_name = "getEpsilon")]
    pub fn get_epsilon(&self) -> f64 {
        self.system.get_epsilon()
    }
}

#[wasm_bindgen]
pub fn boundary_map_user_defined(
    x: f64,
    y: f64,
    nx: f64,
    ny: f64,
    x_eq: &str,
    y_eq: &str,
    params: JsValue,
    epsilon: f64,
) -> Result<JsValue, JsValue> {
    use crate::dynamical_systems::{DynamicalSystem, ExtendedState, UserDefinedDynamicalSystem};
    use nalgebra::Vector2;

    let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
    let system = UserDefinedDynamicalSystem::new(x_eq, y_eq, epsilon, param_set)
        .map_err(|e| JsValue::from_str(&format!("Error parsing equations: {}", e)))?;

    let pos = Vector2::new(x, y);
    let normal = Vector2::new(nx, ny);
    let state = ExtendedState { pos, normal };

    let next_state = system
        .extended_map(state, 1)
        .map_err(|e| JsValue::from_str(&format!("Error evaluating extended map: {}", e)))?;

    let result = ExtendedPoint {
        x: next_state.pos.x,
        y: next_state.pos.y,
        nx: next_state.normal.x,
        ny: next_state.normal.y,
    };

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    result.serialize(&serializer).map_err(|e| {
        web_sys::console::error_1(&JsValue::from_str(&format!("Serialization error: {:?}", e)));
        JsValue::from_str("Failed to serialize result")
    })
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FoundPeriodicOrbit {
    pub points: Vec<(f64, f64)>,
    pub extended_points: Vec<(f64, f64, f64, f64)>,
    pub period: usize,
    pub stability: String,
    pub eigenvalues: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SweepResult {
    pub param_value: f64,
    pub orbits: Vec<FoundPeriodicOrbit>,
    pub total_orbits: usize,
    pub stable_count: usize,
    pub unstable_count: usize,
    pub saddle_count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterSweepResult {
    pub param_name: String,
    pub param_min: f64,
    pub param_max: f64,
    pub num_samples: usize,
    pub b: f64,
    pub epsilon: f64,
    pub max_period: usize,
    pub results: Vec<SweepResult>,
}

impl ParameterSweepResult {
    pub fn to_csv(&self) -> String {
        let mut csv = String::new();
        csv.push_str("parameter_a,period,stability,x,y,nx,ny\n");
        for result in &self.results {
            for orbit in &result.orbits {
                for &(x, y, nx, ny) in &orbit.extended_points {
                    csv.push_str(&format!(
                        "{},{},{},{},{},{},{}\n",
                        result.param_value, orbit.period, orbit.stability, x, y, nx, ny
                    ));
                }
            }
        }
        csv
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_default()
    }
}

const DEFAULT_HITTING_SUPPORT_MASS: f64 = 0.995;
const DEFAULT_HITTING_DISTANCE_TOLERANCE: f64 = 1e-2;
const MIN_HITTING_DISTANCE_TOLERANCE: f64 = 1e-12;
const MAX_HITTING_DISTANCE_TOLERANCE: f64 = 1e-1;
const MAX_HITTING_PERIOD: usize = 10;
const MAX_HITTING_LEVEL: usize = 10;
const MAX_HITTING_GRID_SIZE: usize = 180;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HittingLevelHit {
    pub level: usize,
    pub target_index: usize,
    pub orbit_index: usize,
    pub point_index: usize,
    pub period: usize,
    pub stability: String,
    pub distance: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HittingLevelCell {
    pub index: usize,
    pub x: f64,
    pub y: f64,
    pub best_level: usize,
    pub hits: Vec<HittingLevelHit>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HittingLevelTarget {
    pub target_index: usize,
    pub orbit_index: usize,
    pub point_index: usize,
    pub x: f64,
    pub y: f64,
    pub nx: f64,
    pub ny: f64,
    pub period: usize,
    pub stability: String,
    pub eigenvalues: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HittingLevelSettings {
    pub max_period: usize,
    pub max_level: usize,
    pub ulam_subdivisions: usize,
    pub ulam_points_per_box: usize,
    pub ulam_iterations: usize,
    pub support_mass: f64,
    pub support_threshold: f64,
    pub theta_grid_size: usize,
    pub sample_grid_size: usize,
    pub hit_radius: f64,
    pub residual_threshold: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HittingLevelSummary {
    pub active_boxes: usize,
    pub total_boxes: usize,
    pub orbit_count: usize,
    pub target_count: usize,
    pub hit_cell_count: usize,
    pub total_hits: usize,
    pub level_counts: Vec<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HittingLevelResult {
    pub cells: Vec<HittingLevelCell>,
    pub targets: Vec<HittingLevelTarget>,
    pub levels_present: Vec<usize>,
    pub settings: HittingLevelSettings,
    pub summary: HittingLevelSummary,
}

#[derive(Debug, Clone)]
struct HittingTargetInternal {
    target_index: usize,
    orbit_index: usize,
    point_index: usize,
    point: ExtendedPoint,
    period: usize,
    stability: StabilityType,
    eigenvalues: Vec<f64>,
}

fn sanitize_hitting_grid_size(value: usize, fallback: usize) -> usize {
    if value == 0 {
        return fallback;
    }
    value.clamp(2, MAX_HITTING_GRID_SIZE)
}

fn sanitize_hitting_period(value: usize) -> usize {
    if value == 0 {
        return MAX_HITTING_PERIOD;
    }
    value.clamp(1, MAX_HITTING_PERIOD)
}

fn sanitize_hitting_level(value: usize) -> usize {
    if value == 0 {
        return MAX_HITTING_LEVEL;
    }
    value.clamp(1, MAX_HITTING_LEVEL)
}

fn sanitize_support_mass(value: f64) -> f64 {
    if !value.is_finite() || value <= 0.0 {
        return DEFAULT_HITTING_SUPPORT_MASS;
    }
    value.clamp(0.80, 0.9999)
}

fn sanitize_hit_tolerance(value: f64) -> f64 {
    if !value.is_finite() || value <= 0.0 {
        return DEFAULT_HITTING_DISTANCE_TOLERANCE;
    }
    value.clamp(MIN_HITTING_DISTANCE_TOLERANCE, MAX_HITTING_DISTANCE_TOLERANCE)
}

fn build_henon_ulam_grid_and_measure(
    system: &HenonSystem,
    subdivisions: usize,
    points_per_box: usize,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    iterations: usize,
) -> Result<(Grid, Vec<f64>), String> {
    let (x_min, x_max) = clamp_pair(x_min, x_max, RANGE_LIMIT);
    let (y_min, y_max) = clamp_pair(y_min, y_max, RANGE_LIMIT);
    let grid = Grid::new(Vector2::new(x_min, y_min), Vector2::new(x_max, y_max), subdivisions);
    let n_boxes = grid.boxes.len();
    let samples_per_dim = (points_per_box as f64).sqrt().ceil().max(1.0) as usize;
    let mut transitions: HashMap<usize, Vec<(usize, f64)>> = HashMap::new();

    for source in 0..n_boxes {
        let rect = &grid.boxes[source];
        let center = Vector2::new(rect.center.0, rect.center.1);
        let radius = Vector2::new(rect.radius.0, rect.radius.1);
        let mut counts: HashMap<usize, usize> = HashMap::new();

        for sy in 0..samples_per_dim {
            for sx in 0..samples_per_dim {
                let tx = if samples_per_dim > 1 {
                    -1.0 + 2.0 * (sx as f64) / ((samples_per_dim - 1) as f64)
                } else {
                    0.0
                };
                let ty = if samples_per_dim > 1 {
                    -1.0 + 2.0 * (sy as f64) / ((samples_per_dim - 1) as f64)
                } else {
                    0.0
                };
                let sample = Vector2::new(center.x + tx * radius.x, center.y + ty * radius.y);
                let mapped = system.map(sample)?;
                let intersecting = grid.find_intersecting_boxes(&mapped, epsilon);
                for target in intersecting {
                    *counts.entry(target).or_insert(0) += 1;
                }
            }
        }

        if !counts.is_empty() {
            let total = counts.values().sum::<usize>() as f64;
            transitions.insert(
                source,
                counts
                    .into_iter()
                    .map(|(target, count)| (target, count as f64 / total))
                    .collect(),
            );
        }
    }

    let measure = UlamComputer::compute_right_eigenvector(
        &transitions,
        n_boxes,
        iterations.max(1),
    );
    Ok((grid, measure))
}

fn dilate_grid_indices(indices: &HashSet<usize>, subdivisions: usize) -> HashSet<usize> {
    let mut dilated = indices.clone();
    let n = subdivisions as isize;
    for &idx in indices {
        let ix = (idx % subdivisions) as isize;
        let iy = (idx / subdivisions) as isize;
        for dy in -1..=1 {
            for dx in -1..=1 {
                let nx = ix + dx;
                let ny = iy + dy;
                if nx >= 0 && nx < n && ny >= 0 && ny < n {
                    dilated.insert((ny as usize) * subdivisions + (nx as usize));
                }
            }
        }
    }
    dilated
}

fn select_active_ulam_boxes(
    measure: &[f64],
    subdivisions: usize,
    support_mass: f64,
) -> (HashSet<usize>, f64) {
    let n_boxes = measure.len();
    if n_boxes == 0 || subdivisions == 0 {
        return (HashSet::new(), 0.0);
    }

    let total_mass = measure.iter().sum::<f64>();
    if total_mass <= 1e-15 {
        return ((0..n_boxes).collect(), 0.0);
    }

    let target_mass = sanitize_support_mass(support_mass) * total_mass;
    let mut ranked: Vec<(usize, f64)> = measure.iter().copied().enumerate().collect();
    ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    let mut active = HashSet::new();
    let mut accumulated = 0.0;
    let mut threshold = 0.0;

    for (idx, mass) in ranked {
        if mass <= 0.0 && !active.is_empty() {
            break;
        }
        active.insert(idx);
        accumulated += mass;
        threshold = mass;
        if accumulated >= target_mass {
            break;
        }
    }

    if active.is_empty() {
        if let Some((idx, mass)) = measure
            .iter()
            .copied()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        {
            active.insert(idx);
            threshold = mass;
        }
    }

    (dilate_grid_indices(&active, subdivisions), threshold)
}

fn find_boundary_periodic_orbits_from_active_boxes(
    system: &HenonSystem,
    grid: &Grid,
    active_boxes: &HashSet<usize>,
    max_period: usize,
    theta_grid_size: usize,
    residual_threshold: f64,
) -> PeriodicOrbitDatabase {
    let mut database = PeriodicOrbitDatabase::new();
    let mut active: Vec<usize> = active_boxes.iter().copied().collect();
    active.sort_unstable();

    for period in 1..=max_period {
        for idx in &active {
            let Some(rect) = grid.boxes.get(*idx) else {
                continue;
            };
            let x0 = rect.center.0;
            let y0 = rect.center.1;
            for k in 0..theta_grid_size {
                let theta = 2.0 * PI * (k as f64 + 0.5) / (theta_grid_size as f64);
                if let Some(fp) = find_boundary_periodic_point_davidchack_lai_generic(
                    system,
                    x0,
                    y0,
                    theta.cos(),
                    theta.sin(),
                    period,
                    None,
                    150,
                    1e-12,
                    residual_threshold,
                ) {
                    try_add_orbit_generic(system, &mut database, fp, period, residual_threshold);
                }
            }
        }
    }

    database
}

fn flatten_hitting_targets(database: &PeriodicOrbitDatabase) -> Vec<HittingTargetInternal> {
    let mut targets = Vec::new();
    for (orbit_index, orbit) in database.orbits.iter().enumerate() {
        for (point_index, point) in orbit.extended_points.iter().copied().enumerate() {
            targets.push(HittingTargetInternal {
                target_index: targets.len(),
                orbit_index,
                point_index,
                point,
                period: orbit.period,
                stability: orbit.stability,
                eigenvalues: orbit.eigenvalues.clone(),
            });
        }
    }
    targets
}

fn target_to_js(target: &HittingTargetInternal) -> HittingLevelTarget {
    HittingLevelTarget {
        target_index: target.target_index,
        orbit_index: target.orbit_index,
        point_index: target.point_index,
        x: target.point.x,
        y: target.point.y,
        nx: target.point.nx,
        ny: target.point.ny,
        period: target.period,
        stability: String::from(&target.stability),
        eigenvalues: target.eigenvalues.clone(),
    }
}

fn compute_hitting_cells_for_targets(
    system: &HenonSystem,
    targets: &[HittingTargetInternal],
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    sample_grid_size: usize,
    theta_grid_size: usize,
    max_level: usize,
    hit_radius: f64,
) -> (Vec<HittingLevelCell>, Vec<usize>, Vec<usize>) {
    let dx = (x_max - x_min) / (sample_grid_size as f64);
    let dy = (y_max - y_min) / (sample_grid_size as f64);
    let hit_radius_sq = hit_radius * hit_radius;
    let mut cells = Vec::new();
    let mut level_counts = vec![0usize; max_level + 1];
    let mut levels_present = HashSet::new();

    if targets.is_empty() || dx <= 0.0 || dy <= 0.0 {
        return (cells, Vec::new(), level_counts);
    }

    for iy in 0..sample_grid_size {
        for ix in 0..sample_grid_size {
            let x = x_min + dx * (ix as f64 + 0.5);
            let y = y_min + dy * (iy as f64 + 0.5);
            let mut hits = Vec::new();
            let mut seen = HashSet::new();

            for k in 0..theta_grid_size {
                let theta = 2.0 * PI * (k as f64 + 0.5) / (theta_grid_size as f64);
                let mut current = ExtendedPoint::from_angle(x, y, theta);

                for level in 1..=max_level {
                    current = boundary_map_generic(
                        system,
                        current.x,
                        current.y,
                        current.nx,
                        current.ny,
                    );
                    if !current.is_finite() || !current.is_bounded(100.0) {
                        break;
                    }

                    for target in targets {
                        let dist_sq = (current.x - target.point.x).powi(2)
                            + (current.y - target.point.y).powi(2);
                        if dist_sq <= hit_radius_sq {
                            let key = (level, target.target_index);
                            if seen.insert(key) {
                                hits.push(HittingLevelHit {
                                    level,
                                    target_index: target.target_index,
                                    orbit_index: target.orbit_index,
                                    point_index: target.point_index,
                                    period: target.period,
                                    stability: String::from(&target.stability),
                                    distance: dist_sq.sqrt(),
                                });
                                level_counts[level] += 1;
                                levels_present.insert(level);
                            }
                        }
                    }
                }
            }

            if !hits.is_empty() {
                hits.sort_by(|a, b| {
                    a.level
                        .cmp(&b.level)
                        .then(a.orbit_index.cmp(&b.orbit_index))
                        .then(a.point_index.cmp(&b.point_index))
                });
                let best_level = hits.first().map(|hit| hit.level).unwrap_or(max_level);
                cells.push(HittingLevelCell {
                    index: iy * sample_grid_size + ix,
                    x,
                    y,
                    best_level,
                    hits,
                });
            }
        }
    }

    let mut sorted_levels: Vec<usize> = levels_present.into_iter().collect();
    sorted_levels.sort_unstable();
    (cells, sorted_levels, level_counts)
}

pub fn compute_henon_hitting_level_sets(
    a: f64,
    b: f64,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    max_period: usize,
    ulam_subdivisions: usize,
    ulam_points_per_box: usize,
    ulam_iterations: usize,
    support_mass: f64,
    theta_grid_size: usize,
    sample_grid_size: usize,
    max_level: usize,
    hit_tolerance: f64,
    residual_threshold: f64,
) -> Result<HittingLevelResult, String> {
    let max_period = sanitize_hitting_period(max_period);
    let max_level = sanitize_hitting_level(max_level);
    let ulam_subdivisions = sanitize_hitting_grid_size(ulam_subdivisions, 40);
    let sample_grid_size = sanitize_hitting_grid_size(sample_grid_size, 60);
    let theta_grid_size = sanitize_grid_size(theta_grid_size, DEFAULT_THETA_GRID_SIZE);
    let ulam_points_per_box = ulam_points_per_box.clamp(4, 256);
    let ulam_iterations = if ulam_iterations == 0 { 20 } else { ulam_iterations.clamp(1, 100) };
    let support_mass = sanitize_support_mass(support_mass);
    let hit_radius = sanitize_hit_tolerance(hit_tolerance);
    let residual_threshold = sanitize_residual_threshold(residual_threshold);
    let (x_min, x_max) = clamp_pair(x_min, x_max, RANGE_LIMIT);
    let (y_min, y_max) = clamp_pair(y_min, y_max, RANGE_LIMIT);
    let system = HenonSystem::new(a, b, epsilon);

    let (ulam_grid, measure) = build_henon_ulam_grid_and_measure(
        &system,
        ulam_subdivisions,
        ulam_points_per_box,
        epsilon,
        x_min,
        x_max,
        y_min,
        y_max,
        ulam_iterations,
    )?;
    let (active_boxes, support_threshold) =
        select_active_ulam_boxes(&measure, ulam_subdivisions, support_mass);

    let orbit_database = find_boundary_periodic_orbits_from_active_boxes(
        &system,
        &ulam_grid,
        &active_boxes,
        max_period,
        theta_grid_size,
        residual_threshold,
    );
    let targets_internal = flatten_hitting_targets(&orbit_database);
    let targets: Vec<HittingLevelTarget> = targets_internal.iter().map(target_to_js).collect();

    let (cells, levels_present, level_counts) = compute_hitting_cells_for_targets(
        &system,
        &targets_internal,
        x_min,
        x_max,
        y_min,
        y_max,
        sample_grid_size,
        theta_grid_size,
        max_level,
        hit_radius,
    );
    let total_hits = cells.iter().map(|cell| cell.hits.len()).sum();
    let hit_cell_count = cells.len();

    Ok(HittingLevelResult {
        cells,
        targets,
        levels_present,
        settings: HittingLevelSettings {
            max_period,
            max_level,
            ulam_subdivisions,
            ulam_points_per_box,
            ulam_iterations,
            support_mass,
            support_threshold,
            theta_grid_size,
            sample_grid_size,
            hit_radius,
            residual_threshold,
        },
        summary: HittingLevelSummary {
            active_boxes: active_boxes.len(),
            total_boxes: measure.len(),
            orbit_count: orbit_database.total_count(),
            target_count: targets_internal.len(),
            hit_cell_count,
            total_hits,
            level_counts,
        },
    })
}

#[wasm_bindgen(js_name = "computeHenonHittingLevelSets")]
pub fn compute_henon_hitting_level_sets_wasm(
    a: f64,
    b: f64,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    max_period: usize,
    ulam_subdivisions: usize,
    ulam_points_per_box: usize,
    ulam_iterations: usize,
    support_mass: f64,
    theta_grid_size: usize,
    sample_grid_size: usize,
    max_level: usize,
    hit_tolerance: f64,
    residual_threshold: f64,
) -> Result<JsValue, JsValue> {
    console_error_panic_hook::set_once();
    let result = compute_henon_hitting_level_sets(
        a,
        b,
        epsilon,
        x_min,
        x_max,
        y_min,
        y_max,
        max_period,
        ulam_subdivisions,
        ulam_points_per_box,
        ulam_iterations,
        support_mass,
        theta_grid_size,
        sample_grid_size,
        max_level,
        hit_tolerance,
        residual_threshold,
    )
    .map_err(|e| JsValue::from_str(&e))?;

    serde_wasm_bindgen::to_value(&result)
        .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
}

#[derive(Debug, Clone, Copy)]
struct HenonContinuationParams {
    a: f64,
    b: f64,
    epsilon: f64,
}

fn lerp_henon_params(
    old_params: HenonContinuationParams,
    new_params: HenonContinuationParams,
    lambda: f64,
) -> HenonSystem {
    HenonSystem::new(
        old_params.a + (new_params.a - old_params.a) * lambda,
        old_params.b + (new_params.b - old_params.b) * lambda,
        old_params.epsilon + (new_params.epsilon - old_params.epsilon) * lambda,
    )
}

fn normalize_vector4_normal(z: &mut Vector4<f64>) -> bool {
    let norm = (z[2] * z[2] + z[3] * z[3]).sqrt();
    if norm <= 1e-12 || !norm.is_finite() {
        return false;
    }
    z[2] /= norm;
    z[3] /= norm;
    true
}

fn found_orbit_seed(orbit: &FoundPeriodicOrbit) -> Option<ExtendedPoint> {
    orbit
        .extended_points
        .first()
        .map(|&(x, y, nx, ny)| ExtendedPoint::new(x, y, nx, ny))
        .filter(ExtendedPoint::is_finite)
}

fn correct_henon_seed_at_params(
    seed: ExtendedPoint,
    period: usize,
    params: HenonContinuationParams,
    residual_threshold: f64,
) -> Option<ExtendedPoint> {
    let system = HenonSystem::new(params.a, params.b, params.epsilon);
    find_boundary_periodic_point_davidchack_lai_generic(
        &system,
        seed.x,
        seed.y,
        seed.nx,
        seed.ny,
        period,
        None,
        80,
        1e-12,
        residual_threshold,
    )
}

fn continue_henon_seed_to_target_arclength(
    seed: ExtendedPoint,
    period: usize,
    old_params: HenonContinuationParams,
    new_params: HenonContinuationParams,
    residual_threshold: f64,
) -> Option<ExtendedPoint> {
    let build = |lambda: f64| lerp_henon_params(old_params, new_params, lambda);
    let mut cont = PseudoArclengthContinuation::new::<HenonSystem, _>(
        &seed,
        0.0,
        0.04,
        period,
        true,
        &build,
    )?;
    cont.lambda_min = -0.05;
    cont.lambda_max = 1.05;
    cont.residual_threshold = residual_threshold;
    cont.newton_max_iter = 40;
    cont.newton_tol = 1e-11;
    cont.min_ds = 1e-4;
    cont.max_ds = 0.08;
    cont.max_fails = 10;

    let mut previous_lambda = cont.lambda;
    let mut previous_z = cont.z;
    let mut closest_lambda = cont.lambda;
    let mut closest_z = cont.z;

    for _ in 0..120 {
        match cont.step::<HenonSystem, _>(&build) {
            StepOutcome::Converged => {
                let current_lambda = cont.lambda;
                let current_z = cont.z;
                if (current_lambda - 1.0).abs() < (closest_lambda - 1.0).abs() {
                    closest_lambda = current_lambda;
                    closest_z = current_z;
                }

                let crossed_target = (previous_lambda - 1.0) * (current_lambda - 1.0) <= 0.0
                    && (current_lambda - previous_lambda).abs() > 1e-12;
                if crossed_target {
                    let alpha = (1.0 - previous_lambda) / (current_lambda - previous_lambda);
                    let mut z_target = previous_z + (current_z - previous_z) * alpha;
                    if !normalize_vector4_normal(&mut z_target) {
                        return None;
                    }
                    let target_seed =
                        ExtendedPoint::new(z_target[0], z_target[1], z_target[2], z_target[3]);
                    return correct_henon_seed_at_params(
                        target_seed,
                        period,
                        new_params,
                        residual_threshold,
                    );
                }

                if current_lambda > 1.05 {
                    break;
                }
                previous_lambda = current_lambda;
                previous_z = current_z;
            }
            StepOutcome::Retry => continue,
            StepOutcome::Stalled | StepOutcome::OutOfRange => break,
        }
    }

    if (closest_lambda - 1.0).abs() <= 0.05 {
        if normalize_vector4_normal(&mut closest_z) {
            let target_seed =
                ExtendedPoint::new(closest_z[0], closest_z[1], closest_z[2], closest_z[3]);
            return correct_henon_seed_at_params(
                target_seed,
                period,
                new_params,
                residual_threshold,
            );
        }
    }

    None
}


// simple continuation for Henon boundary map

pub fn continue_henon_orbits_from_previous(
    previous_orbits: &[FoundPeriodicOrbit],
    old_a: f64,
    old_b: f64,
    old_epsilon: f64,
    new_a: f64,
    new_b: f64,
    new_epsilon: f64,
    max_period: usize,
    residual_threshold: f64,
) -> PeriodicOrbitDatabase {
    let residual_threshold = sanitize_residual_threshold(residual_threshold);
    let old_params = HenonContinuationParams {
        a: old_a,
        b: old_b,
        epsilon: old_epsilon,
    };
    let new_params = HenonContinuationParams {
        a: new_a,
        b: new_b,
        epsilon: new_epsilon,
    };
    let new_system = HenonSystem::new(new_a, new_b, new_epsilon);
    let mut database = PeriodicOrbitDatabase::new();

    for orbit in previous_orbits {
        if orbit.period == 0 || orbit.period > max_period {
            continue;
        }
        let Some(seed) = found_orbit_seed(orbit) else {
            continue;
        };

        let corrected = correct_henon_seed_at_params(
            seed,
            orbit.period,
            new_params,
            residual_threshold,
        )
        .or_else(|| {
            continue_henon_seed_to_target_arclength(
                seed,
                orbit.period,
                old_params,
                new_params,
                residual_threshold,
            )
        });

        if let Some(fp) = corrected {
            try_add_orbit_generic(
                &new_system,
                &mut database,
                fp,
                orbit.period,
                residual_threshold,
            );
        }
    }

    database
}

// WASM

#[wasm_bindgen(js_name = "continueBoundaryHenonOrbits")]
pub fn continue_boundary_henon_orbits_wasm(
    previous_orbits_js: JsValue,
    old_a: f64,
    old_b: f64,
    old_epsilon: f64,
    new_a: f64,
    new_b: f64,
    new_epsilon: f64,
    max_period: usize,
    residual_threshold: f64,
) -> Result<JsValue, JsValue> {
    console_error_panic_hook::set_once();

    let previous_orbits: Vec<FoundPeriodicOrbit> =
        serde_wasm_bindgen::from_value(previous_orbits_js)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse previous orbits: {}", e)))?;

    let db = continue_henon_orbits_from_previous(
        &previous_orbits,
        old_a,
        old_b,
        old_epsilon,
        new_a,
        new_b,
        new_epsilon,
        max_period,
        residual_threshold,
    );
    let orbits = database_to_found_orbits_generic(&db);
    serde_wasm_bindgen::to_value(&orbits)
        .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
}

#[wasm_bindgen(js_name = "parameterSweep")]
pub fn parameter_sweep_wasm(
    b: f64,
    epsilon: f64,
    a_min: f64,
    a_max: f64,
    num_samples: usize,
    max_period: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> Result<JsValue, JsValue> {
    console_error_panic_hook::set_once();
    let base_params = vec![
        ("a".to_string(), a_min), // placeholder, will be swept
        ("b".to_string(), b),
    ];
    let result = parameter_sweep_henon_fast(
        &base_params,
        "a",
        a_min,
        a_max,
        num_samples,
        epsilon,
        max_period,
        15,
        12,
        x_min,
        x_max,
        y_min,
        y_max,
    );
    serde_wasm_bindgen::to_value(&result)
        .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
}

/// Unified parameter sweep: works for any system type + any parameter.
#[wasm_bindgen(js_name = "parameterSweepGeneric")]
pub fn parameter_sweep_generic_wasm(
    system_type: &str,
    x_eq: &str,
    y_eq: &str,
    params_js: JsValue,
    sweep_param_name: &str,
    sweep_min: f64,
    sweep_max: f64,
    num_samples: usize,
    epsilon: f64,
    max_period: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> Result<JsValue, JsValue> {
    console_error_panic_hook::set_once();

    let param_set =
        crate::parameters::parameter_set_from_js(params_js).map_err(|e| JsValue::from_str(&e))?;

    let base_params: Vec<(String, f64)> = param_set
        .entries()
        .iter()
        .map(|e| (e.name.clone(), e.value))
        .collect();

    let grid_size = 15;
    let theta_grid_size = 12;

    let result = if system_type == "henon" || system_type == "discrete_henon" {
        parameter_sweep_henon_fast(
            &base_params,
            sweep_param_name,
            sweep_min,
            sweep_max,
            num_samples,
            epsilon,
            max_period,
            grid_size,
            theta_grid_size,
            x_min,
            x_max,
            y_min,
            y_max,
        )
    } else {
        let (actual_x_eq, actual_y_eq) = match system_type {
            "duffing" | "discrete_duffing" => ("y", "-b * x + a * y - y^3"),
            _ => (x_eq, y_eq),
        };
        parameter_sweep_generic(
            actual_x_eq,
            actual_y_eq,
            &base_params,
            sweep_param_name,
            sweep_min,
            sweep_max,
            num_samples,
            epsilon,
            max_period,
            grid_size,
            theta_grid_size,
            x_min,
            x_max,
            y_min,
            y_max,
        )
    };

    serde_wasm_bindgen::to_value(&result)
        .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
}

#[wasm_bindgen(js_name = "parameterSweepCsv")]
pub fn parameter_sweep_csv_wasm(
    b: f64,
    epsilon: f64,
    a_min: f64,
    a_max: f64,
    num_samples: usize,
    max_period: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> String {
    console_error_panic_hook::set_once();
    let base_params = vec![("a".to_string(), a_min), ("b".to_string(), b)];
    let result = parameter_sweep_henon_fast(
        &base_params,
        "a",
        a_min,
        a_max,
        num_samples,
        epsilon,
        max_period,
        15,
        12,
        x_min,
        x_max,
        y_min,
        y_max,
    );
    result.to_csv()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepOutcome {
    /// Corrector converged to a point on the same branch; lambda advanced
    Converged,
    /// Corrector failed (or jumped orbits), step shrunk, lambda not advanced
    Retry, 
    /// Repeated failures even at minimum step 
    Stalled,
    /// Stepped past the requested [lambda_min, lambda_max] range
    OutOfRange
}



/// Iterate boundary map p times and returns the final point 
fn iterate_boundary_p(
    system: &dyn DynamicalSystem,
    z: ExtendedPoint,
    p: usize
) -> Option<ExtendedPoint> {
    let mut cur = z; 

    for _ in 0..p { 
        cur = boundary_map_generic(system, cur.x, cur.y, cur.nx, cur.ny);
        if !cur.is_finite() || !cur.is_bounded(1e10) {
            return None 
        }
    }
    Some(cur)
}


/// At (z, lambda), return the period-p residual H = E^p(z) - z in R^4
/// the state Jacobian D_zH = D_zE^p - I (4x4), and the parameter derivative
/// D_λH = ∂E^p/∂λ in R^4 by central finite difference 

fn state_residual_jacobian<S, F>(
    z: &Vector4<f64>,
    lambda: f64, 
    period: usize,
    build_system: &F,
    fd_h: f64
) -> Option<(Vector4<f64>, Jacobian4x4, Vector4<f64>)> 
where 
    S: DynamicalSystem,
    F: Fn(f64) -> S, 
{
    let system = build_system(lambda);
    let zp = ExtendedPoint::new(z[0], z[1], z[2], z[3]);

    let (mapped, dz_ep) = compose_boundary_map_n_times_generic(&system, zp, period);
    if !mapped.is_finite() {
        return None;
    }

    let h_res = Vector4::new(
        mapped.x - zp.x, 
        mapped.y - zp.y,
        mapped.nx - zp.nx,
        mapped.ny - zp.ny
    );

    let dz_h = dz_ep.subtract_identity();

    let sys_p = build_system(lambda + fd_h);
    let sys_m = build_system(lambda - fd_h);
    let mp = iterate_boundary_p(&sys_p, zp, period)?;
    let mm = iterate_boundary_p(&sys_m, zp, period)?;

    let da_h = Vector4::new(
        (mp.x - mm.x) / (2.0 * fd_h),
        (mp.y - mm.y) / (2.0 * fd_h),
        (mp.nx - mm.nx) / (2.0 * fd_h),
        (mp.ny - mm.ny) / (2.0 * fd_h)
    );

    Some((h_res, dz_h, da_h))

}


/// Initial branch tangent from the natural-continuation formula
fn initial_tangent(
    dz_h: &Jacobian4x4,
    da_h: &Vector4<f64>,
    lambda_increasing: bool,
) -> Option<Vector5<f64>> {
    let inv = dz_h.inverse()?;
    let mut v = [0.0f64;4];
    for i in 0..4 {
        let mut s = 0.0;
        for j in 0..4 {
            s += inv.data[i][j] * da_h[j];
        }
        v[i] = -s;
    }
    let mut t = Vector5::new(v[0], v[1], v[2], v[3], 1.0);
    t /= t.norm();
    if (t[4] > 0.0) != lambda_increasing {
        t = -t;
    }
    Some(t)
}

/// Next tangent via bordered system: solve [D_zH | D_lambdaH ; t_prev^T] t = e5,
/// then normalize and orient to continue in the same direction as t_prev
/// the border makes this solvable through a fold, where D_zH alone is singular
fn compute_tangent(
    dz_h: &Jacobian4x4,
    da_h: &Vector4<f64>,
    prev_tangent: &Vector5<f64>,
) -> Option<Vector5<f64>> {
    let mut m = Matrix5::<f64>::zeros();
    for i in 0..4 {
        for j in 0..4 {
            m[(i, j)] = dz_h.data[i][j];
        }
        m[(i, 4)] = da_h[i]
    }
    for j in 0..5 {
        m[(4, j)] = prev_tangent[j];
    }
    let rhs = Vector5::new(0.0, 0.0, 0.0, 0.0, 1.0);
    let t = m.lu().solve(&rhs)?;
    if t.iter().any(|v| !v.is_finite() || t.norm() < 1e-30) {
        return None;
    }
    let t = t / t.norm() ;
    Some(if t.dot(prev_tangent) < 0.0 { -t } else { t } )
}

/// Pseudo-arclength Keller continuation of the boundary map periodic orbits.
/// 
/// Parameterizes the branch by arclength s in the combined (z, λ) space, so λ is 
/// a free unknown that may rise, peak, fall. The augmented 5x5 Newton system
/// stays non-singular at a quadric fold - where natural continuation's 4x4 
/// Dz_H block goes singular -- so it rounds saddle-node turning points (e.g., the 
/// a ≈ 0.595 topological bifurcation) instead of stalling

pub struct PseudoArclengthContinuation {
    pub z: Vector4<f64>,
    pub lambda: f64, 
    /// Unit tangent (dz, dλ) in R^5 to to branch at the current point.
    pub tangent: Vector5<f64>,
    /// Arclength step magnitude (always > 0; direction lives in the tangent)
    pub ds: f64,
    pub period: usize,

    pub lambda_min: f64,
    pub lambda_max: f64,
    pub min_ds: f64,
    pub max_ds: f64,
    pub residual_threshold: f64,
    pub newton_max_iter: usize,
    pub newton_tol: f64,
    pub fd_h: f64,

    consecutive_fails: u32,
    max_fails: u32,
}

impl PseudoArclengthContinuation {
    /// Seed from already converged orbit point. `lambda_increasing` sets the initial 
    /// travel direction. `build_system(λ)` makes λ into a fresh system,
    /// e.g., `|a| HenonSystem::new(a, b, eps)`
    pub fn new<S, F>(
        seed: &ExtendedPoint,
        lambda0: f64,
        ds: f64,
        period: usize,
        lambda_increasing: bool, 
        build_system: &F
    ) -> Option<Self>
    where 
        S: DynamicalSystem,
        F: Fn(f64) -> S,
    {
        let z = Vector4::new(seed.x, seed.y, seed.nx, seed.ny);
        let (_, dz_h, da_h) = state_residual_jacobian(&z, lambda0, period, build_system, 1e-6)?;
        let tangent = initial_tangent(&dz_h, &da_h, lambda_increasing)?;

        Some(Self {
            z,
            lambda: lambda0,
            tangent, 
            ds: ds.abs().max(1e-6),
            period, 
            lambda_min: f64::NEG_INFINITY,
            lambda_max: f64::INFINITY,
            min_ds: 1e-4,
            max_ds: 0.05,
            residual_threshold: DEFAULT_PERIODIC_RESIDUAL_THRESHOLD,
            newton_max_iter: 50,
            newton_tol: 1e-12,
            fd_h: 1e-6,
            consecutive_fails: 0,
            max_fails: 8
        })
    }

    pub fn seed_point(&self) -> ExtendedPoint {
        ExtendedPoint::new(self.z[0], self.z[1], self.z[2], self.z[3])
    }


    pub fn d_lambda_ds(&self) -> f64 { 
        self.tangent[4]
    }

    pub fn classify<S: DynamicalSystem>(&self, system: &S) -> (StabilityType, Vec<f64>) {
        let (_, jac) = compose_boundary_map_n_times_generic(system, self.seed_point(), self.period);
        classify_stability_4d(&jac)
    }

    /// One predictor-corrector step along the branch 
    pub fn step<S, F>(&mut self, build_system: &F) -> StepOutcome 
    where 
        S: DynamicalSystem,
        F: Fn(f64) -> S,
    {
        let w0 = Vector5::new(self.z[0], self.z[1], self.z[2], self.z[3], self.lambda);


        // predictor: step ds along the unit tangent
        let mut w = w0 + self.ds * self.tangent;

        // corrector: Newton on [H_p(z, λ) = 0; t * (w - w0) - ds = 0]

        let mut converged = false;
        for _ in 0..self.newton_max_iter {
            let z_cur = Vector4::new(w[0], w[1], w[2], w[3]);
            let lam_cur = w[4];

            let (h_res, dz_h, da_h) = match state_residual_jacobian(
                &z_cur, 
                lam_cur,
                self.period,
                build_system,
                self.fd_h,
            ) {
                Some(v) => v,
                None => break,
            };

            let n_res = self.tangent.dot(&(w - w0)) - self.ds;
            if h_res.norm() < self.residual_threshold && n_res.abs() < self.newton_tol {
                converged = true;
                break;
            }

            let r = Vector5::new(h_res[0], h_res[1], h_res[2], h_res[3], n_res);

            // Augmented Jacobian: top-left D_zH, top-right D_λH, bottom row t^T
            let mut jm = Matrix5::<f64>::zeros();
            for i in 0..4 {
                for j in 0..4 {
                    jm[(i, j)] = dz_h.data[i][j];
                }
                jm[(i, 4)] = da_h[i];
            }
            for j in 0..5 {
                jm[(4, j)] = self.tangent[j];
            }

            let dw = match jm.lu().solve(&(-r)) {
                Some(d) => d,
                None => break,
            };

            if dw.iter().any(|v| !v.is_finite()) {
                break;
            }

            w += dw;

            let nn = (w[2] * w[2] + w[3] * w[3]).sqrt();
            if nn > 1e-12 {
                w[2] /= nn;
                w[3] /= nn;
            }

            if dw.norm() < self.newton_tol {
                let z_chk = Vector4::new(w[0], w[1], w[2], w[3]);

                if let Some((h2, _, _)) = state_residual_jacobian(
                    &z_chk,
                    w[4],
                    self.period,
                    build_system,
                    self.fd_h
                ) {
                    let n2 = self.tangent.dot(&(w - w0)) - self.ds;
                    converged = h2.norm() < self.residual_threshold && n2.abs() < self.newton_tol;
                }
                break;
            }
        }

        let z_new = ExtendedPoint::new(w[0], w[1], w[2], w[3]);
        if !converged || !z_new.is_finite() || !z_new.is_bounded(100.0) {
            self.consecutive_fails += 1;
            self.ds = (self.ds * 0.5).max(self.min_ds);
            return if self.consecutive_fails >= self.max_fails {
                StepOutcome::Stalled
            } else {
                StepOutcome::Retry
            };
        }

        if w[4] < self.lambda_min || w[4] > self.lambda_max {
            return StepOutcome::OutOfRange;
        }

        let z_acc = Vector4::new(w[0], w[1], w[2], w[3]);
        if let Some((_, dz_h, da_h))  = state_residual_jacobian(&z_acc, w[4], self.period, build_system, self.fd_h) {
            if let Some(t_new) = compute_tangent(&dz_h, &da_h, &self.tangent) {
                self.tangent = t_new;
            }
        }

        self.z = z_acc;
        self.lambda = w[4];
        self.consecutive_fails = 0;
        self.ds = (self.ds * 1.3).min(self.max_ds);
        StepOutcome::Converged

    }
    
    
}



#[derive(Debug, Clone)]
pub struct BranchPoint {
    pub lambda: f64,
    pub point: ExtendedPoint,
    pub period: usize,
    pub stability: StabilityType,
    pub eigenvalues: Vec<f64>,
    pub d_lambda_ds: f64,
}

fn record_branch_point<S, F> (
    cont: &PseudoArclengthContinuation,
    build_system: &F
) -> BranchPoint 
where 
    S: DynamicalSystem,
    F: Fn(f64) -> S, 
{
    let sys = build_system(cont.lambda);
    let (stability, eigenvalues) = cont.classify(&sys);
    BranchPoint {
        lambda: cont.lambda,
        point: cont.seed_point(),
        period: cont.period,
        stability,
        eigenvalues,
        d_lambda_ds: cont.d_lambda_ds(),
    }
}

/// Follow one branch from `seed` by arclength until it leaves [lambda_min,
/// lambda_max], stalls, or hits `max_points`. Rounds folds; a saddle-node shows
/// up as a sign flip of `d_lambda_ds` between consecutive returned points.
pub fn follow_branch_arclength<S, F>(
    seed: &ExtendedPoint,
    lambda0: f64,
    ds: f64,
    period: usize,
    lambda_increasing: bool,
    lambda_min: f64,
    lambda_max: f64,
    max_points: usize,
    build_system: &F,
) -> Vec<BranchPoint>
where
    S: DynamicalSystem,
    F: Fn(f64) -> S,
{
    let mut cont = match PseudoArclengthContinuation::new(
        seed, lambda0, ds, period, lambda_increasing, build_system,
    ) {
        Some(c) => c,
        None => return Vec::new(),
    };
    cont.lambda_min = lambda_min;
    cont.lambda_max = lambda_max;

    let mut branch = vec![record_branch_point(&cont, build_system)];
    for _ in 0..max_points {
        match cont.step(build_system) {
            StepOutcome::Converged => branch.push(record_branch_point(&cont, build_system)),
            StepOutcome::Retry => continue,
            StepOutcome::Stalled | StepOutcome::OutOfRange => break,
        }
    }
    branch
}



#[test]
fn test_natural_continuation_tracks_fixed_point() {
    let (b, eps) = (0.3, 0.01);
    let build = |a| HenonSystem::new(a, b, eps);

    let sys0 = build(1.4);
    let db = find_all_boundary_periodic_orbits_generic(&sys0, 1, 15, 12, -3.0, 3.0, -3.0, 3.0);
    let seed = db.orbits.iter().find(|o| o.period == 1)
        .expect("a period-1 orbit at a = 1.4").extended_points[0];

    let branch = follow_branch_arclength(&seed, 1.4, 0.02, 1, false, 0.5, 1.4, 200, &build);
    assert!(branch.len() > 1, "continuation should produce multiple points");

    for bp in &branch {
        let sys = build(bp.lambda);
        let m = boundary_map_generic(&sys, bp.point.x, bp.point.y, bp.point.nx, bp.point.ny);
        let d = ((m.x - bp.point.x).powi(2) + (m.y - bp.point.y).powi(2)).sqrt();
        assert!(d < 1e-6, "point at lambda={} not fixed (d={})", bp.lambda, d);
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jacobian_multiply_identity() {
        let a = Jacobian::new(1.0, 2.0, 3.0, 4.0);
        let id = Jacobian::identity();
        let result = a.multiply(&id);
        assert!((result.j11 - 1.0).abs() < 1e-12);
        assert!((result.j12 - 2.0).abs() < 1e-12);
        assert!((result.j21 - 3.0).abs() < 1e-12);
        assert!((result.j22 - 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_jacobian_multiply_known_product() {
        let a = Jacobian::new(1.0, 2.0, 3.0, 4.0);
        let b = Jacobian::new(5.0, 6.0, 7.0, 8.0);
        let result = a.multiply(&b);
        assert!((result.j11 - 19.0).abs() < 1e-12, "j11: got {}", result.j11);
        assert!((result.j12 - 22.0).abs() < 1e-12, "j12: got {}", result.j12);
        assert!((result.j21 - 43.0).abs() < 1e-12, "j21: got {}", result.j21);
        assert!((result.j22 - 50.0).abs() < 1e-12, "j22: got {}", result.j22);
    }

    #[test]
    fn test_jacobian_multiply_associative() {
        let a = Jacobian::new(1.0, 2.0, 3.0, 4.0);
        let b = Jacobian::new(5.0, 6.0, 7.0, 8.0);
        let c = Jacobian::new(-1.0, 0.5, 0.3, -2.0);
        let ab_c = a.multiply(&b).multiply(&c);
        let a_bc = a.multiply(&b.multiply(&c));
        assert!((ab_c.j11 - a_bc.j11).abs() < 1e-10);
        assert!((ab_c.j12 - a_bc.j12).abs() < 1e-10);
        assert!((ab_c.j21 - a_bc.j21).abs() < 1e-10);
        assert!((ab_c.j22 - a_bc.j22).abs() < 1e-10);
    }

    #[test]
    fn test_jacobian_eigenvalues_real() {
        let j = Jacobian::new(3.0, 0.0, 0.0, 1.0);
        let (l1, l2, complex) = j.eigenvalues();
        assert!(!complex);
        assert!((l1 - 3.0).abs() < 1e-10 || (l1 - 1.0).abs() < 1e-10);
        assert!((l2 - 3.0).abs() < 1e-10 || (l2 - 1.0).abs() < 1e-10);
        assert!((l1 - l2).abs() > 1.0); // they're different
    }

    #[test]
    fn test_jacobian_eigenvalues_complex() {
        let j = Jacobian::new(0.0, -1.0, 1.0, 0.0);
        let (l1, l2, complex) = j.eigenvalues();
        assert!(complex);
        assert!((l1 - 1.0).abs() < 1e-10);
        assert!((l2 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_jacobian_determinant_correct() {
        let j = Jacobian::new(2.0, 3.0, 1.0, 4.0);
        let (l1, l2, _) = j.eigenvalues();
        let det_from_eig = l1 * l2;
        assert!(
            (det_from_eig - 5.0).abs() < 1e-10,
            "det from eigenvalues: {}",
            det_from_eig
        );
    }

    #[test]
    fn test_henon_jacobian_values() {
        let a = 1.4;
        let b = 0.3;
        let x = 0.5;
        let sys = make_henon_system(a, b, 0.01);
        let j = sys.jacobian(Vector2::new(x, 0.0));
        assert!((j[(0, 0)] - (-2.0 * a * x)).abs() < 1e-12);
        assert!((j[(0, 1)] - 1.0).abs() < 1e-12);
        assert!((j[(1, 0)] - b).abs() < 1e-12);
        assert!((j[(1, 1)] - 0.0).abs() < 1e-12);
    }

    fn make_henon_system(a: f64, b: f64, ep: f64) -> HenonSystem {
        HenonSystem::new(a, b, ep)
    }

    #[test]
    fn test_generic_dl_finds_fixed_point() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let result = find_boundary_periodic_point_davidchack_lai_generic(
            &sys,
            0.6,
            0.2,
            1.0,
            0.0,
            1,
            None,
            200,
            1e-12,
            DEFAULT_PERIODIC_RESIDUAL_THRESHOLD,
        );
        if let Some(fp) = result {
            let mapped = boundary_map_generic(&sys, fp.x, fp.y, fp.nx, fp.ny);
            assert!((mapped.x - fp.x).abs() < 1e-8, "Not a fixed point in x");
            assert!((mapped.y - fp.y).abs() < 1e-8, "Not a fixed point in y");
            let norm = (fp.nx * fp.nx + fp.ny * fp.ny).sqrt();
            assert!((norm - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_generic_finds_fixed_points() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let db = find_all_boundary_periodic_orbits_generic(&sys, 1, 10, 8, -3.0, 3.0, -3.0, 3.0);

        let p1_orbits: Vec<_> = db.orbits.iter().filter(|o| o.period == 1).collect();
        assert!(
            !p1_orbits.is_empty(),
            "Should find at least one fixed point"
        );

        for orbit in &p1_orbits {
            let ep_pt = &orbit.extended_points[0];
            let mapped = boundary_map_generic(&sys, ep_pt.x, ep_pt.y, ep_pt.nx, ep_pt.ny);
            assert!((mapped.x - ep_pt.x).abs() < 1e-6, "Fixed point x mismatch");
            assert!((mapped.y - ep_pt.y).abs() < 1e-6, "Fixed point y mismatch");
        }
    }

    #[test]
    fn test_generic_finds_period2() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let db = find_all_boundary_periodic_orbits_generic(&sys, 2, 12, 10, -3.0, 3.0, -3.0, 3.0);

        let p2_orbits: Vec<_> = db.orbits.iter().filter(|o| o.period == 2).collect();
        for orbit in &p2_orbits {
            assert_eq!(orbit.extended_points.len(), 2);
            let p = &orbit.extended_points[0];
            let mapped1 = boundary_map_generic(&sys, p.x, p.y, p.nx, p.ny);
            let mapped2 = boundary_map_generic(&sys, mapped1.x, mapped1.y, mapped1.nx, mapped1.ny);
            assert!((mapped2.x - p.x).abs() < 1e-6, "Period-2 x doesn't return");
            assert!((mapped2.y - p.y).abs() < 1e-6, "Period-2 y doesn't return");
            let not_fp = (mapped1.x - p.x).abs() > 1e-4 || (mapped1.y - p.y).abs() > 1e-4;
            assert!(not_fp, "Period-2 orbit is actually a fixed point");
        }
    }

    #[test]
    fn test_generic_no_duplicates() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let db = find_all_boundary_periodic_orbits_generic(&sys, 2, 10, 8, -3.0, 3.0, -3.0, 3.0);

        for i in 0..db.orbits.len() {
            for j in (i + 1)..db.orbits.len() {
                for pi in &db.orbits[i].extended_points {
                    for pj in &db.orbits[j].extended_points {
                        let dist = ((pi.x - pj.x).powi(2) + (pi.y - pj.y).powi(2)).sqrt();
                        assert!(dist > 0.001,
                            "Duplicate points between orbits {} and {}: ({:.4},{:.4}) and ({:.4},{:.4})",
                            i, j, pi.x, pi.y, pj.x, pj.y);
                    }
                }
            }
        }
    }

    #[test]
    fn test_generic_stability_assigned() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let db = find_all_boundary_periodic_orbits_generic(&sys, 2, 10, 8, -3.0, 3.0, -3.0, 3.0);

        for orbit in &db.orbits {
            match orbit.stability {
                StabilityType::Stable | StabilityType::Unstable | StabilityType::Saddle => {}
            }
            for ev in &orbit.eigenvalues {
                assert!(ev.is_finite(), "Eigenvalue not finite: {}", ev);
            }
        }
    }

    #[test]
    fn test_generic_normal_unit_length() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let db = find_all_boundary_periodic_orbits_generic(&sys, 1, 10, 8, -3.0, 3.0, -3.0, 3.0);

        for orbit in &db.orbits {
            for p in &orbit.extended_points {
                let norm = (p.nx * p.nx + p.ny * p.ny).sqrt();
                assert!(
                    (norm - 1.0).abs() < 1e-8,
                    "Normal not unit length: {} at ({:.4},{:.4})",
                    norm,
                    p.x,
                    p.y
                );
            }
        }
    }

    #[test]
    fn test_generic_verify_minimal_period() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let db = find_all_boundary_periodic_orbits_generic(&sys, 1, 10, 8, -3.0, 3.0, -3.0, 3.0);

        for orbit in db.orbits.iter().filter(|o| o.period == 1) {
            let p = &orbit.extended_points[0];
            assert!(verify_minimal_period_generic(
                &sys,
                p,
                1,
                DEFAULT_PERIODIC_RESIDUAL_THRESHOLD
            ));
            assert!(
                !verify_minimal_period_generic(&sys, p, 2, DEFAULT_PERIODIC_RESIDUAL_THRESHOLD),
                "Fixed point should NOT be minimal period 2"
            );
        }
    }

    #[test]
    fn test_classify_stability_4d_diagonal() {
        let stable_jac = Jacobian4x4 {
            data: [
                [0.5, 0.0, 0.0, 0.0],
                [0.0, 0.3, 0.0, 0.0],
                [0.0, 0.0, 0.2, 0.0],
                [0.0, 0.0, 0.0, 0.1],
            ],
        };
        let (stab, _) = classify_stability_4d(&stable_jac);
        assert!(
            matches!(stab, StabilityType::Stable),
            "Expected stable, got {:?}",
            stab
        );

        let unstable_jac = Jacobian4x4 {
            data: [
                [2.0, 0.0, 0.0, 0.0],
                [0.0, 3.0, 0.0, 0.0],
                [0.0, 0.0, 1.5, 0.0],
                [0.0, 0.0, 0.0, 4.0],
            ],
        };
        let (stab, _) = classify_stability_4d(&unstable_jac);
        assert!(
            matches!(stab, StabilityType::Unstable),
            "Expected unstable, got {:?}",
            stab
        );
    }

    #[test]
    fn test_generic_database_to_found_orbits() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let db = find_all_boundary_periodic_orbits_generic(&sys, 1, 10, 8, -3.0, 3.0, -3.0, 3.0);
        let found = database_to_found_orbits_generic(&db);

        assert_eq!(found.len(), db.orbits.len());
        for (fo, orbit) in found.iter().zip(db.orbits.iter()) {
            assert_eq!(fo.period, orbit.period);
            assert_eq!(fo.points.len(), orbit.points.len());
            assert_eq!(fo.extended_points.len(), orbit.extended_points.len());
            for (ep_tuple, ep_orig) in fo.extended_points.iter().zip(orbit.extended_points.iter()) {
                assert!((ep_tuple.0 - ep_orig.x).abs() < 1e-14);
                assert!((ep_tuple.1 - ep_orig.y).abs() < 1e-14);
                assert!((ep_tuple.2 - ep_orig.nx).abs() < 1e-14);
                assert!((ep_tuple.3 - ep_orig.ny).abs() < 1e-14);
            }
        }
    }

    #[test]
    fn test_generic_sweep_basic() {
        let base_params = vec![("a".to_string(), 0.5), ("b".to_string(), 0.3)];
        let result = parameter_sweep_generic(
            "1 - a * x^2 + y",
            "b * x",
            &base_params,
            "a",
            0.5,
            1.0,
            3,
            0.01,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );
        assert_eq!(result.results.len(), 3);
        assert_eq!(result.param_name, "a");
        assert!((result.param_min - 0.5).abs() < 1e-12);
        assert!((result.param_max - 1.0).abs() < 1e-12);
        assert!((result.epsilon - 0.01).abs() < 1e-12);
    }

    #[test]
    fn test_generic_sweep_orbit_counts_consistent() {
        let base_params = vec![("a".to_string(), 0.5), ("b".to_string(), 0.3)];
        let result = parameter_sweep_generic(
            "1 - a * x^2 + y",
            "b * x",
            &base_params,
            "a",
            0.5,
            1.5,
            3,
            0.01,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );
        for sweep in &result.results {
            assert_eq!(
                sweep.total_orbits,
                sweep.stable_count + sweep.unstable_count + sweep.saddle_count,
                "Stability counts don't sum to total at a={}",
                sweep.param_value
            );
        }
    }

    #[test]
    fn test_generic_sweep_csv_export() {
        let base_params = vec![("a".to_string(), 1.0), ("b".to_string(), 0.3)];
        let result = parameter_sweep_generic(
            "1 - a * x^2 + y",
            "b * x",
            &base_params,
            "a",
            1.0,
            1.4,
            2,
            0.01,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );
        let csv = result.to_csv();
        assert!(
            csv.starts_with("parameter_a,period,stability,x,y,nx,ny\n"),
            "CSV header mismatch: {}",
            csv.lines().next().unwrap_or("")
        );
        let lines: Vec<&str> = csv.lines().collect();
        assert!(lines.len() > 1, "CSV should have data rows");
        for line in &lines[1..] {
            if !line.is_empty() {
                let cols: Vec<&str> = line.split(',').collect();
                assert_eq!(
                    cols.len(),
                    7,
                    "Expected 7 columns, got {}: {}",
                    cols.len(),
                    line
                );
            }
        }
    }

    #[test]
    fn test_generic_sweep_json_export() {
        let base_params = vec![("a".to_string(), 1.0), ("b".to_string(), 0.3)];
        let result = parameter_sweep_generic(
            "1 - a * x^2 + y",
            "b * x",
            &base_params,
            "a",
            1.0,
            1.2,
            2,
            0.01,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );
        let json = result.to_json();
        assert!(!json.is_empty());
        assert!(json.starts_with('{'));
        assert!(json.contains("\"param_name\""));
        assert!(json.contains("\"results\""));
        assert!(json.contains("\"epsilon\""));
    }

    #[test]
    fn test_generic_sweep_single_sample() {
        let base_params = vec![("a".to_string(), 1.0), ("b".to_string(), 0.3)];
        let result = parameter_sweep_generic(
            "1 - a * x^2 + y",
            "b * x",
            &base_params,
            "a",
            1.0,
            1.0,
            1,
            0.01,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );
        assert_eq!(result.results.len(), 1);
        assert!((result.results[0].param_value - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_generic_sweep_finds_orbits() {
        let base_params = vec![("a".to_string(), 0.5), ("b".to_string(), 0.3)];
        let result = parameter_sweep_generic(
            "1 - a * x^2 + y",
            "b * x",
            &base_params,
            "a",
            0.5,
            1.5,
            3,
            0.01,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );
        let total: usize = result.results.iter().map(|r| r.total_orbits).sum();
        assert!(total > 0, "Sweep should find at least some orbits");
    }

    #[test]
    fn test_user_defined_henon_matches_generic() {
        let sys_native = make_henon_system(1.4, 0.3, 0.01);
        let params = crate::parameters::ParameterSet::new(vec![
            crate::parameters::ParameterEntry {
                name: "a".to_string(),
                value: 1.4,
            },
            crate::parameters::ParameterEntry {
                name: "b".to_string(),
                value: 0.3,
            },
        ])
        .unwrap();
        let sys_user =
            UserDefinedDynamicalSystem::new("1 - a * x^2 + y", "b * x", 0.01, params).unwrap();

        let db_native =
            find_all_boundary_periodic_orbits_generic(&sys_native, 1, 10, 8, -3.0, 3.0, -3.0, 3.0);
        let db_user =
            find_all_boundary_periodic_orbits_generic(&sys_user, 1, 10, 8, -3.0, 3.0, -3.0, 3.0);

        assert!(
            !db_native.orbits.is_empty(),
            "Native HenonSystem found no orbits"
        );
        assert!(
            !db_user.orbits.is_empty(),
            "UserDefined Hénon found no orbits"
        );
    }

    #[test]
    fn test_generic_sweep_custom_param_name() {
        let base_params = vec![("alpha".to_string(), 1.0), ("beta".to_string(), 0.3)];
        let result = parameter_sweep_generic(
            "1 - alpha * x^2 + y",
            "beta * x",
            &base_params,
            "alpha",
            0.5,
            1.5,
            3,
            0.01,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );
        assert_eq!(result.results.len(), 3);
        assert_eq!(result.param_name, "alpha");
    }

    #[test]
    fn test_boundary_map_preserves_normal_unit_length() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let result = boundary_map_generic(&sys, 0.5, 0.15, 1.0, 0.0);
        let norm = (result.nx * result.nx + result.ny * result.ny).sqrt();
        assert!(
            (norm - 1.0).abs() < 1e-10,
            "Normal should be unit length, got {}",
            norm
        );
    }

    #[test]
    fn test_boundary_map_reduces_to_henon_at_zero_epsilon() {
        let a = 1.4;
        let b = 0.3;
        let x = 0.5;
        let y = 0.15;
        let sys = make_henon_system(a, b, 0.0);
        let result = boundary_map_generic(&sys, x, y, 1.0, 0.0);
        let mapped = sys.map(Vector2::new(x, y)).unwrap();
        assert!(
            (result.x - mapped.x).abs() < 1e-10,
            "At ep=0, boundary map should equal Henon map"
        );
        assert!((result.y - mapped.y).abs() < 1e-10);
    }

    #[test]
    fn test_4x4_jacobian_determinant() {
        let j = Jacobian4x4 {
            data: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 2.0, 0.0, 0.0],
                [0.0, 0.0, 3.0, 0.0],
                [0.0, 0.0, 0.0, 4.0],
            ],
        };
        assert!((j.determinant() - 24.0).abs() < 1e-10);
    }

    #[test]
    fn test_4x4_jacobian_inverse() {
        let j = Jacobian4x4 {
            data: [
                [2.0, 1.0, 0.0, 0.0],
                [0.0, 3.0, 1.0, 0.0],
                [0.0, 0.0, 2.0, 1.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        };
        let inv = j.inverse().expect("Should be invertible");
        let product = j.multiply(&inv);
        for i in 0..4 {
            for k in 0..4 {
                let expected = if i == k { 1.0 } else { 0.0 };
                assert!(
                    (product.data[i][k] - expected).abs() < 1e-10,
                    "J*J^-1 [{},{}] = {}, expected {}",
                    i,
                    k,
                    product.data[i][k],
                    expected
                );
            }
        }
    }

    #[test]
    fn test_henon_system_map_inverse_roundtrip() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let p = Vector2::new(0.6, 0.2);
        let mapped = sys.map(p).unwrap();
        let recovered = sys.map_inverse(mapped).unwrap();
        assert!((recovered.x - p.x).abs() < 1e-10, "x roundtrip failed");
        assert!((recovered.y - p.y).abs() < 1e-10, "y roundtrip failed");
    }

    #[test]
    fn test_henon_analytic_vs_numerical_jacobian() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let p = Vector2::new(0.6, 0.2);
        let analytic = sys.jacobian(p);

        let h = 1e-7;
        let fx_plus = sys.map(Vector2::new(p.x + h, p.y)).unwrap();
        let fx_minus = sys.map(Vector2::new(p.x - h, p.y)).unwrap();
        let fy_plus = sys.map(Vector2::new(p.x, p.y + h)).unwrap();
        let fy_minus = sys.map(Vector2::new(p.x, p.y - h)).unwrap();

        let num_00 = (fx_plus.x - fx_minus.x) / (2.0 * h);
        let num_01 = (fy_plus.x - fy_minus.x) / (2.0 * h);
        let num_10 = (fx_plus.y - fx_minus.y) / (2.0 * h);
        let num_11 = (fy_plus.y - fy_minus.y) / (2.0 * h);

        assert!((analytic[(0, 0)] - num_00).abs() < 1e-5, "J[0,0] mismatch");
        assert!((analytic[(0, 1)] - num_01).abs() < 1e-5, "J[0,1] mismatch");
        assert!((analytic[(1, 0)] - num_10).abs() < 1e-5, "J[1,0] mismatch");
        assert!((analytic[(1, 1)] - num_11).abs() < 1e-5, "J[1,1] mismatch");
    }

    #[test]
    fn test_boundary_map_generic_orbit_is_periodic() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let fp = find_boundary_periodic_point_davidchack_lai_generic(
            &sys,
            0.6,
            0.2,
            1.0,
            0.0,
            1,
            None,
            200,
            1e-12,
            DEFAULT_PERIODIC_RESIDUAL_THRESHOLD,
        );
        if let Some(fp) = fp {
            let mapped = boundary_map_generic(&sys, fp.x, fp.y, fp.nx, fp.ny);
            let dist = (mapped.x - fp.x).powi(2)
                + (mapped.y - fp.y).powi(2)
                + (mapped.nx - fp.nx).powi(2)
                + (mapped.ny - fp.ny).powi(2);
            assert!(
                dist < 1e-8,
                "Fixed point should map to itself, dist={}",
                dist
            );
        }
    }

    #[test]
    fn test_generic_4d_jacobian_numerical_vs_analytic() {
        let sys = make_henon_system(1.4, 0.3, 0.01);
        let x = 0.5;
        let y = 0.15;
        let nx = 1.0;
        let ny = 0.0;
        let jac = boundary_map_jacobian_generic(&sys, x, y, nx, ny);

        let h = 1e-6;
        let e_plus = boundary_map_generic(&sys, x + h, y, nx, ny);
        let e_minus = boundary_map_generic(&sys, x - h, y, nx, ny);
        let de_dx_0 = (e_plus.x - e_minus.x) / (2.0 * h);
        let de_dx_1 = (e_plus.y - e_minus.y) / (2.0 * h);

        assert!(
            (jac.data[0][0] - de_dx_0).abs() < 1e-4,
            "dE1/dx: analytic={}, numerical={}",
            jac.data[0][0],
            de_dx_0
        );
        assert!(
            (jac.data[1][0] - de_dx_1).abs() < 1e-4,
            "dE2/dx: analytic={}, numerical={}",
            jac.data[1][0],
            de_dx_1
        );
    }

    #[test]
    fn test_spatial_dedup_prevents_duplicate_orbits() {
        // Two extended points at the same (x,y) but different normals
        // should be treated as the same orbit by spatial dedup
        let mut db = PeriodicOrbitDatabase::new();
        let p1 = ExtendedPoint::new(0.5, 0.15, 1.0, 0.0);

        db.add_orbit(PeriodicOrbit {
            points: vec![BoundaryPoint { x: 0.5, y: 0.15 }],
            extended_points: vec![p1],
            period: 1,
            stability: StabilityType::Stable,
            eigenvalues: vec![0.3, 0.1],
        });

        // Same (x,y) with different normal: should be detected as duplicate
        let p2 = ExtendedPoint::new(0.5, 0.15, 0.0, 1.0);
        assert!(
            db.contains_spatial_point(&p2, 0.01),
            "Spatial dedup should catch same (x,y) with different normal"
        );
        // But 4D dedup would miss it (distance = sqrt(2) ≈ 1.414)
        assert!(
            !db.contains_extended_point(&p2, 0.01),
            "4D dedup misses same orbit with different normal"
        );
    }

    #[test]
    fn test_convergence_tolerance_rejects_loose_fixed_points() {
        // A point that maps to something at distance ~1e-8 should NOT be
        // accepted as a fixed point (tolerance is 1e-10)
        let system = HenonSystem::new(1.4, 0.3, 0.01);

        // Use a random point that's clearly not a fixed point
        let non_fp = ExtendedPoint::new(0.123, 0.456, 1.0, 0.0);
        let (mapped, _) = compose_boundary_map_n_times_generic(&system, non_fp, 1);
        let dist = (mapped.x - non_fp.x).powi(2)
            + (mapped.y - non_fp.y).powi(2)
            + (mapped.nx - non_fp.nx).powi(2)
            + (mapped.ny - non_fp.ny).powi(2);
        // A random point should have large residual, confirming our tolerance matters
        assert!(dist > 1e-10, "Random point should not be a fixed point");
    }

    #[test]
    fn test_hitting_level_hit_radius_is_configurable() {
        let result = compute_henon_hitting_level_sets(
            1.4,
            0.3,
            0.01,
            -1.5,
            1.5,
            -1.5,
            1.5,
            1,
            10,
            4,
            1,
            0.8,
            1,
            10,
            1,
            1e-4,
            DEFAULT_PERIODIC_RESIDUAL_THRESHOLD,
        )
        .expect("hitting-level computation should complete for minimal settings");

        assert_eq!(result.settings.hit_radius, 1e-4);
    }

    #[test]
    fn test_henon_unique_orbit_count_at_typical_params() {
        // At a=0.4, b=0.3, epsilon=0.1, the Hénon boundary map should find
        // a small number of distinct orbits (not duplicates)
        let system = HenonSystem::new(0.4, 0.3, 0.1);
        let db =
            find_all_boundary_periodic_orbits_generic(&system, 1, 15, 12, -3.0, 3.0, -3.0, 3.0);

        // Count stable orbits — should be at most 1 for period 1
        let stable_count = db
            .orbits
            .iter()
            .filter(|o| o.period == 1 && matches!(o.stability, StabilityType::Stable))
            .count();
        assert!(
            stable_count <= 1,
            "Expected at most 1 stable fixed point, got {}",
            stable_count
        );

        // All orbits should be spatially distinct
        for (i, orbit_a) in db.orbits.iter().enumerate() {
            for (j, orbit_b) in db.orbits.iter().enumerate() {
                if i >= j {
                    continue;
                }
                for pa in &orbit_a.points {
                    for pb in &orbit_b.points {
                        let dist = ((pa.x - pb.x).powi(2) + (pa.y - pb.y).powi(2)).sqrt();
                        assert!(
                            dist > 0.01,
                            "Orbits {} and {} have spatially overlapping points ({:.4},{:.4}) vs ({:.4},{:.4}), dist={:.6}",
                            i, j, pa.x, pa.y, pb.x, pb.y, dist
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_sweep_and_viz_grid_consistency() {
        // Verify that the sweep finds the same orbits as the visualization
        // at a given parameter value, since they now use the same grid size
        let system = HenonSystem::new(0.4, 0.3, 0.1);

        // "Visualization" path
        let viz_db =
            find_all_boundary_periodic_orbits_generic(&system, 1, 15, 12, -3.0, 3.0, -3.0, 3.0);

        // "Sweep" path (same grid now)
        let base_params = vec![("a".to_string(), 0.4), ("b".to_string(), 0.3)];
        let sweep_result = parameter_sweep_henon_fast(
            &base_params,
            "a",
            0.4,
            0.4,
            1,
            0.1,
            1,
            15,
            12,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );

        assert_eq!(sweep_result.results.len(), 1);
        let sweep_orbits = &sweep_result.results[0].orbits;

        assert_eq!(
            viz_db.total_count(),
            sweep_orbits.len(),
            "Sweep and visualization should find same number of orbits"
        );
    }

    #[test]
    fn test_continue_henon_orbits_tracks_parameter_shift() {
        let old_system = HenonSystem::new(0.4, 0.3, 0.1);
        let old_db =
            find_all_boundary_periodic_orbits_generic(&old_system, 1, 10, 8, -3.0, 3.0, -3.0, 3.0);
        let previous = database_to_found_orbits_generic(&old_db);

        let continued = continue_henon_orbits_from_previous(
            &previous,
            0.4,
            0.3,
            0.1,
            0.42,
            0.3,
            0.1,
            1,
            DEFAULT_PERIODIC_RESIDUAL_THRESHOLD,
        );

        assert!(
            continued.total_count() > 0,
            "Continuation should retain at least one fixed point after a small a-shift"
        );

        let new_system = HenonSystem::new(0.42, 0.3, 0.1);
        for orbit in &continued.orbits {
            let point = orbit.extended_points[0];
            let mapped = boundary_map_generic(&new_system, point.x, point.y, point.nx, point.ny);
            let residual = (mapped.x - point.x).powi(2)
                + (mapped.y - point.y).powi(2)
                + (mapped.nx - point.nx).powi(2)
                + (mapped.ny - point.ny).powi(2);
            assert!(
                residual < 1e-12,
                "Continued orbit is not periodic at the new parameter, residual={}",
                residual
            );
        }
    }

    #[test]
    fn test_continue_henon_orbits_tracks_epsilon_shift() {
        let old_system = HenonSystem::new(0.4, 0.3, 0.08);
        let old_db =
            find_all_boundary_periodic_orbits_generic(&old_system, 1, 10, 8, -3.0, 3.0, -3.0, 3.0);
        let previous = database_to_found_orbits_generic(&old_db);

        let continued = continue_henon_orbits_from_previous(
            &previous,
            0.4,
            0.3,
            0.08,
            0.4,
            0.3,
            0.1,
            1,
            DEFAULT_PERIODIC_RESIDUAL_THRESHOLD,
        );

        assert!(
            continued.total_count() > 0,
            "Continuation should retain at least one fixed point after a small epsilon-shift"
        );
    }

    #[test]
    fn test_henon_sweep_applies_epsilon_samples() {
        let base_params = vec![("a".to_string(), 0.4), ("b".to_string(), 0.3)];
        let low_eps = parameter_sweep_henon_fast(
            &base_params,
            "epsilon",
            0.05,
            0.05,
            1,
            0.1,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );
        let high_eps = parameter_sweep_henon_fast(
            &base_params,
            "epsilon",
            0.12,
            0.12,
            1,
            0.1,
            1,
            10,
            8,
            -3.0,
            3.0,
            -3.0,
            3.0,
        );

        assert_eq!(low_eps.results.len(), 1);
        assert_eq!(high_eps.results.len(), 1);
        assert!((low_eps.results[0].param_value - 0.05).abs() < 1e-12);
        assert!((high_eps.results[0].param_value - 0.12).abs() < 1e-12);

        let low_first = low_eps.results[0]
            .orbits
            .first()
            .and_then(|orbit| orbit.extended_points.first())
            .copied();
        let high_first = high_eps.results[0]
            .orbits
            .first()
            .and_then(|orbit| orbit.extended_points.first())
            .copied();

        if let (Some(low), Some(high)) = (low_first, high_first) {
            let position_delta = ((low.0 - high.0).powi(2) + (low.1 - high.1).powi(2)).sqrt();
            assert!(
                position_delta > 1e-5,
                "Sweeping epsilon should change the computed boundary orbit"
            );
        }
    }

    #[test]
    fn test_periodic_search_input_sanitization() {
        assert_eq!(
            sanitize_grid_size(0, DEFAULT_PERIODIC_GRID_SIZE),
            DEFAULT_PERIODIC_GRID_SIZE
        );
        assert_eq!(sanitize_grid_size(1, DEFAULT_PERIODIC_GRID_SIZE), 2);
        assert_eq!(sanitize_grid_size(9999, DEFAULT_PERIODIC_GRID_SIZE), 256);

        assert_eq!(
            sanitize_residual_threshold(f64::NAN),
            DEFAULT_PERIODIC_RESIDUAL_THRESHOLD
        );
        assert_eq!(
            sanitize_residual_threshold(-1.0),
            DEFAULT_PERIODIC_RESIDUAL_THRESHOLD
        );
    }

    #[test]
    fn test_looser_residual_threshold_finds_at_least_as_many_orbits() {
        let system = HenonSystem::new(0.4, 0.3, 0.1);

        let strict = find_all_boundary_periodic_orbits_generic_with_threshold(
            &system, 1, 10, 8, -3.0, 3.0, -3.0, 3.0, 1e-13,
        );
        let loose = find_all_boundary_periodic_orbits_generic_with_threshold(
            &system, 1, 10, 8, -3.0, 3.0, -3.0, 3.0, 1e-8,
        );

        assert!(
            loose.total_count() >= strict.total_count(),
            "Looser residual threshold should not reduce accepted orbit count"
        );
        assert!(
            loose.total_count() > 0,
            "Expected at least one orbit with loose threshold"
        );
    }

    #[test]
    fn test_hitting_support_selects_cumulative_mass_with_safety_dilation() {
        let measure = vec![
            0.0, 0.0, 0.0,
            0.0, 0.98, 0.0,
            0.0, 0.02, 0.0,
        ];

        let (active, threshold) = select_active_ulam_boxes(&measure, 3, 0.95);

        assert!((threshold - 0.98).abs() < 1e-12);
        assert!(active.contains(&4), "central high-mass box must be active");
        assert!(active.contains(&0), "one-cell safety dilation should include diagonal neighbors");
        assert_eq!(active.len(), 9, "central support dilation should cover the 3x3 grid");
    }

    #[test]
    fn test_hitting_cells_record_multiple_levels_for_same_sample() {
        let system = HenonSystem::new(0.4, 0.3, 0.05);
        let seed = ExtendedPoint::from_angle(0.5, 0.5, PI);
        let level_one = boundary_map_generic(&system, seed.x, seed.y, seed.nx, seed.ny);
        let level_two = boundary_map_generic(
            &system,
            level_one.x,
            level_one.y,
            level_one.nx,
            level_one.ny,
        );

        let targets = vec![
            HittingTargetInternal {
                target_index: 0,
                orbit_index: 0,
                point_index: 0,
                point: level_one,
                period: 1,
                stability: StabilityType::Saddle,
                eigenvalues: vec![],
            },
            HittingTargetInternal {
                target_index: 1,
                orbit_index: 1,
                point_index: 0,
                point: level_two,
                period: 2,
                stability: StabilityType::Stable,
                eigenvalues: vec![],
            },
        ];

        let (cells, levels_present, level_counts) = compute_hitting_cells_for_targets(
            &system,
            &targets,
            0.0,
            1.0,
            0.0,
            1.0,
            1,
            1,
            2,
            1e-10,
        );

        assert_eq!(cells.len(), 1);
        assert_eq!(levels_present, vec![1, 2]);
        assert_eq!(level_counts[1], 1);
        assert_eq!(level_counts[2], 1);
        assert_eq!(cells[0].hits.len(), 2);
        assert_eq!(cells[0].hits[0].level, 1);
        assert_eq!(cells[0].hits[1].level, 2);
    }
}
