use nalgebra::{Matrix2, Vector2};
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

use crate::dynamical_systems::{DynamicalSystem, ExtendedState};
use crate::parameters::{parameter_set_from_js, ParameterSet};
use crate::unstable_manifold::{
    ManifoldConfig, SaddlePoint, SaddleType, StopReason, Trajectory, UnstableManifoldComputer,
};
use crate::user_defined::ParsedEquations;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
    #[wasm_bindgen(js_namespace = console)]
    fn error(s: &str);
}

macro_rules! console_log {
    ($($t:tt)*) => {
        log(&format!($($t)*))
    }
}

macro_rules! console_error {
    ($($t:tt)*) => {
        error(&format!($($t)*))
    }
}

#[cfg(target_arch = "wasm32")]
fn get_time_secs() -> f64 {
    js_sys::Date::now() / 1000.0
}

#[cfg(not(target_arch = "wasm32"))]
fn get_time_secs() -> f64 {
    use std::time::Instant;
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_secs_f64()
}

#[derive(Clone, Debug)]
pub struct DuffingODE {
    pub delta: f64,
}

impl DuffingODE {
    pub fn new(delta: f64) -> Result<Self, String> {
        if !delta.is_finite() {
            return Err("Damping delta must be finite".to_string());
        }
        if delta < 0.0 {
            return Err("Damping delta should typically be non-negative".to_string());
        }
        Ok(Self { delta })
    }

    /// continuous vector field: f(x) = (x2, x1 - x1^3 - delta * x2)
    pub fn vector_field(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        if !pos.x.is_finite() || !pos.y.is_finite() {
            return Err("Invalid position: non finite coordinates".to_string());
        }
        let x_dot = pos.y;
        let y_dot = pos.x - pos.x.powi(3) - self.delta * pos.y;
        Ok(Vector2::new(x_dot, y_dot))
    }

    /// continous Jacobian: Df(x) = [[0, 1], [1 - 3*x1^2, -delta]]
    pub fn jacobian(&self, pos: Vector2<f64>) -> Matrix2<f64> {
        Matrix2::new(0.0, 1.0, 1.0 - 3.0 * pos.x.powi(2), -self.delta)
    }

    /// RK4 step: x_{n+1} = x_n + (h/6) * (k1 + 2k2 + 2k3 + k4)
    pub fn rk4_step(&self, pos: Vector2<f64>, h: f64) -> Result<Vector2<f64>, String> {
        let k1 = self.vector_field(pos)?;
        let k2 = self.vector_field(pos + k1 * (h / 2.0))?;
        let k3 = self.vector_field(pos + k2 * (h / 2.0))?;
        let k4 = self.vector_field(pos + k3 * h)?;
        let next_pos = pos + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (h / 6.0);
        if !next_pos.x.is_finite() || !next_pos.y.is_finite() {
            return Err("RK4 step produced non-finite values".to_string());
        }
        Ok(next_pos)
    }
}

pub trait OdeSystem {
    fn vector_field(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String>;
    fn jacobian(&self, pos: Vector2<f64>) -> Matrix2<f64>;

    fn rk4_step(&self, pos: Vector2<f64>, h: f64) -> Result<Vector2<f64>, String> {
        let k1 = self.vector_field(pos)?;
        let k2 = self.vector_field(pos + k1 * (h / 2.0))?;
        let k3 = self.vector_field(pos + k2 * (h / 2.0))?;
        let k4 = self.vector_field(pos + k3 * h)?;
        let next_pos = pos + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (h / 6.0);
        if !next_pos.x.is_finite() || !next_pos.y.is_finite() {
            return Err("RK4 step produced non-finite values".to_string());
        }
        Ok(next_pos)
    }
}

impl OdeSystem for DuffingODE {
    fn vector_field(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        DuffingODE::vector_field(self, pos)
    }

    fn jacobian(&self, pos: Vector2<f64>) -> Matrix2<f64> {
        DuffingODE::jacobian(self, pos)
    }

    fn rk4_step(&self, pos: Vector2<f64>, h: f64) -> Result<Vector2<f64>, String> {
        DuffingODE::rk4_step(self, pos, h)
    }
}

#[derive(Clone, Debug)]
pub struct UserDefinedOdeSystem {
    equations: ParsedEquations,
}

impl UserDefinedOdeSystem {
    pub fn new(x_str: &str, y_str: &str, params: ParameterSet) -> Result<Self, String> {
        let equations = ParsedEquations::new(x_str, y_str, params)?;
        Ok(Self { equations })
    }
}

impl OdeSystem for UserDefinedOdeSystem {
    fn vector_field(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        let (dx, dy) = self.equations.eval(pos.x, pos.y)?;
        Ok(Vector2::new(dx, dy))
    }

    fn jacobian(&self, pos: Vector2<f64>) -> Matrix2<f64> {
        let h = 1e-5;
        let eval =
            |x: f64, y: f64| -> (f64, f64) { self.equations.eval(x, y).unwrap_or((0.0, 0.0)) };

        let (fx1, fy1) = eval(pos.x + h, pos.y);
        let (fx2, fy2) = eval(pos.x - h, pos.y);
        let (fx3, fy3) = eval(pos.x, pos.y + h);
        let (fx4, fy4) = eval(pos.x, pos.y - h);

        let dfx_dx = (fx1 - fx2) / (2.0 * h);
        let dfx_dy = (fx3 - fx4) / (2.0 * h);
        let dfy_dx = (fy1 - fy2) / (2.0 * h);
        let dfy_dy = (fy3 - fy4) / (2.0 * h);

        Matrix2::new(dfx_dx, dfx_dy, dfy_dx, dfy_dy)
    }
}

#[derive(Clone, Debug)]
pub struct EulerMap {
    pub ode: DuffingODE,
    pub h: f64,
    pub epsilon: f64,
}

impl EulerMap {
    pub fn new(ode: DuffingODE, h: f64, epsilon: f64) -> Result<Self, String> {
        if !h.is_finite() || h <= 0.0 {
            return Err("Step size h must be finite and positive".to_string());
        }
        if !epsilon.is_finite() || epsilon < 0.0 {
            return Err("Epsilon must be non-negative and finite".to_string());
        }
        Ok(Self { ode, h, epsilon })
    }

    /// Euler map: F_h(x) = x + h * f(x)
    pub fn euler_step(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        let f_val = self.ode.vector_field(pos)?;
        let next_pos = pos + self.h * f_val;
        if !next_pos.x.is_finite() || !next_pos.y.is_finite() {
            return Err("Euler map produced non-finite values".to_string());
        }
        Ok(next_pos)
    }

    /// inverse Euler map map_inverse(y) using Newton iteration on y = x + h*f(x)
    pub fn euler_step_inverse(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        let mut x = pos;
        for _ in 0..100 {
            let f_val = self.ode.vector_field(x)?;
            let diff = x + self.h * f_val - pos;
            if diff.norm() < 1e-10 {
                return Ok(x);
            }
            let df = self.jacobian(x);
            let dx = df
                .try_inverse()
                .ok_or("Singular Jacobian in inverse Euler map")?
                * diff;
            x -= dx;
        }
        Err("Inverse Euler map failed to converge".to_string())
    }

    /// linearization of Euler map: DF_h(x) = I + h * Df(x)
    pub fn jacobian(&self, pos: Vector2<f64>) -> Matrix2<f64> {
        let df = self.ode.jacobian(pos);
        Matrix2::identity() + self.h * df
    }
}

impl DynamicalSystem for EulerMap {
    fn map(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        self.euler_step(pos)
    }

    fn map_inverse(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        self.euler_step_inverse(pos)
    }

    fn jacobian(&self, pos: Vector2<f64>) -> Matrix2<f64> {
        self.jacobian(pos)
    }

    fn get_epsilon(&self) -> f64 {
        self.h * self.epsilon
    }
}

pub fn bde_rhs<S: OdeSystem>(
    ode: &S,
    eps: f64,
    x: Vector2<f64>,
    nhat: Vector2<f64>,
) -> Result<(Vector2<f64>, Vector2<f64>), String> {
    let fx = ode.vector_field(x)?;
    let dx = Vector2::new(fx.x + eps * nhat.x, fx.y + eps * nhat.y);

    let jac = ode.jacobian(x);
    // Df(x)^T * n
    let jfn_x = jac.m11 * nhat.x + jac.m21 * nhat.y;
    let jfn_y = jac.m12 * nhat.x + jac.m22 * nhat.y;

    // v = -Df^T n
    let vx = -jfn_x;
    let vy = -jfn_y;

    // <n, v>
    let proj = vx * nhat.x + vy * nhat.y;

    // dn = v - proj * nhat
    let dn = Vector2::new(vx - proj * nhat.x, vy - proj * nhat.y);

    Ok((dx, dn))
}

// Approximate the next point using RK4 method
pub fn rk4_bde_step<S: OdeSystem>(
    ode: &S,
    eps: f64,
    h: f64,
    x: Vector2<f64>,
    nhat: Vector2<f64>,
) -> Result<(Vector2<f64>, Vector2<f64>), String> {
    let (k1x, k1n) = bde_rhs(ode, eps, x, nhat)?;

    let x2 = x + k1x * (h / 2.0);
    let n2 = (nhat + k1n * (h / 2.0)).normalize();
    let (k2x, k2n) = bde_rhs(ode, eps, x2, n2)?;

    let x3 = x + k2x * (h / 2.0);
    let n3 = (nhat + k2n * (h / 2.0)).normalize();
    let (k3x, k3n) = bde_rhs(ode, eps, x3, n3)?;

    let x4 = x + k3x * h;
    let n4 = (nhat + k3n * h).normalize();
    let (k4x, k4n) = bde_rhs(ode, eps, x4, n4)?;

    let xp = x + (k1x + k2x * 2.0 + k3x * 2.0 + k4x) * (h / 6.0);
    let np = (nhat + (k1n + k2n * 2.0 + k3n * 2.0 + k4n) * (h / 6.0)).normalize();

    Ok((xp, np))
}

/// find fixed points of the ODE (which are fixed points of the Euler map)
fn find_euler_map_fixed_points() -> Vec<(f64, f64)> {
    vec![(0.0, 0.0), (1.0, 0.0), (-1.0, 0.0)]
}

/// classify stability of a fixed point based on discrete-time Jacobian eigenvalues
fn classify_euler_map_stability(l1: f64, l2: f64) -> &'static str {
    let abs_l1 = l1.abs();
    let abs_l2 = l2.abs();

    if abs_l1 < 1.0 && abs_l2 < 1.0 {
        "Attractor"
    } else if abs_l1 > 1.0 && abs_l2 > 1.0 {
        "Repeller"
    } else {
        "Saddle"
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EulerMapTrajectoryRet {
    pub points: Vec<(f64, f64)>,
    pub stop_reason: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EulerMapManifoldResult {
    pub plus: EulerMapTrajectoryRet,
    pub minus: EulerMapTrajectoryRet,
    pub saddle_point: (f64, f64),
    pub eigenvalue: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EulerMapFixedPointResult {
    pub x: f64,
    pub y: f64,
    pub eigenvalues: (f64, f64),
    pub stability: String,
}

#[derive(Serialize, Deserialize)]
pub struct EulerMapComputeResult {
    pub manifolds: Vec<EulerMapManifoldResult>,
    pub fixed_points: Vec<EulerMapFixedPointResult>,
}

#[wasm_bindgen]
pub struct EulerMapSystemWasm {
    delta: f64,
    h: f64,
    epsilon: f64,
    max_period: usize,
}

#[wasm_bindgen]
impl EulerMapSystemWasm {
    #[wasm_bindgen(constructor)]
    pub fn new(
        delta: f64,
        h: f64,
        epsilon: f64,
        max_period: usize,
    ) -> Result<EulerMapSystemWasm, JsValue> {
        Ok(EulerMapSystemWasm {
            delta,
            h,
            epsilon,
            max_period,
        })
    }

    #[wasm_bindgen(js_name = getPeriodicOrbits)]
    pub fn get_periodic_orbits(&self) -> Result<JsValue, JsValue> {
        let orbits: Vec<()> = Vec::new();
        serde_wasm_bindgen::to_value(&orbits)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
    }

    #[wasm_bindgen(js_name = trackTrajectory)]
    pub fn track_trajectory(&mut self, _initial_x: f64, _initial_y: f64, _max_iterations: usize) {}

    #[wasm_bindgen(js_name = getCurrentPoint)]
    pub fn get_current_point(&self) -> Result<JsValue, JsValue> {
        Ok(JsValue::NULL)
    }

    #[wasm_bindgen(js_name = getTrajectory)]
    pub fn get_trajectory(&self, _start: usize, _end: usize) -> Result<JsValue, JsValue> {
        let points: Vec<()> = Vec::new();
        serde_wasm_bindgen::to_value(&points)
            .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
    }

    #[wasm_bindgen]
    pub fn step(&mut self) -> bool {
        false
    }

    #[wasm_bindgen]
    pub fn reset(&mut self) {}

    #[wasm_bindgen(js_name = getTotalIterations)]
    pub fn get_total_iterations(&self) -> usize {
        0
    }

    #[wasm_bindgen(js_name = getCurrentIteration)]
    pub fn get_current_iteration(&self) -> usize {
        0
    }

    #[wasm_bindgen(js_name = getOrbitCount)]
    pub fn get_orbit_count(&self) -> usize {
        0
    }
}

use crate::boundary_periodic::ExtendedPoint;
use core::f64;

struct BdeSimulator<S: OdeSystem> {
    ode: S,
    epsilon: f64,
    points: Vec<ExtendedPoint>,
}

impl<S: OdeSystem> BdeSimulator<S> {
    fn new(ode: S, epsilon: f64, cx: f64, cy: f64, r: f64, num_points: usize) -> Self {
        let mut points = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let theta = 2.0 * f64::consts::PI * (i as f64) / (num_points as f64);
            let nx = theta.cos();
            let ny = theta.sin();
            let x = cx + r * nx;
            let y = cy + r * ny;
            points.push(ExtendedPoint::new(x, y, nx, ny));
        }

        Self {
            ode,
            epsilon,
            points,
        }
    }

    fn step(&mut self, h: f64) {
        let mut next_points = Vec::with_capacity(self.points.len());
        for p in &self.points {
            let x_vec = Vector2::new(p.x, p.y);
            let n_vec = Vector2::new(p.nx, p.ny);
            match rk4_bde_step(&self.ode, self.epsilon, h, x_vec, n_vec) {
                Ok((xp, np)) => {
                    next_points.push(ExtendedPoint::new(xp.x, xp.y, np.x, np.y));
                }
                Err(_) => {
                    next_points.push(*p);
                }
            }
        }
        self.points = next_points;
    }

    fn get_points(&self) -> &Vec<ExtendedPoint> {
        &self.points
    }

    fn reparameterize(&mut self) {
        let n = self.points.len();
        if n < 3 {
            return;
        }

        // build cumulative arc-length array
        let mut cum = vec![0.0f64; n + 1];
        for i in 0..n {
            let j = (i + 1) % n;
            let dx = self.points[j].x - self.points[i].x;
            let dy = self.points[j].y - self.points[i].y;
            cum[i + 1] = cum[i] + (dx * dx + dy * dy).sqrt();
        }
        let total = cum[n];
        if total < 1e-14 {
            return;
        }

        let mut new_pts = Vec::with_capacity(n);
        for k in 0..n {
            let target = total * (k as f64) / (n as f64);
            // binary-search for the segment that contains target arc-length
            let seg = cum
                .partition_point(|&l| l <= target)
                .saturating_sub(1)
                .min(n - 1);
            let seg_len = cum[seg + 1] - cum[seg];
            let t = if seg_len > 1e-14 {
                (target - cum[seg]) / seg_len
            } else {
                0.0
            };
            let pa = self.points[seg];
            let pb = self.points[(seg + 1) % n];
            let x = pa.x + t * (pb.x - pa.x);
            let y = pa.y + t * (pb.y - pa.y);
            let nx = pa.nx + t * (pb.nx - pa.nx);
            let ny = pa.ny + t * (pb.ny - pa.ny);
            let nm = (nx * nx + ny * ny).sqrt().max(1e-14);
            new_pts.push(ExtendedPoint::new(x, y, nx / nm, ny / nm));
        }
        self.points = new_pts;
    }

    fn has_self_intersection(&self, gap: usize) -> u8 {
        let n = self.points.len();
        if n < 4 {
            return 0;
        }
        let threshold_sq = {
            // use twice the mean segment length as the detection distance
            let mut total = 0.0f64;
            for i in 0..n {
                let j = (i + 1) % n;
                let dx = self.points[j].x - self.points[i].x;
                let dy = self.points[j].y - self.points[i].y;
                total += (dx * dx + dy * dy).sqrt();
            }
            let mean = total / n as f64;
            (mean * 0.8).powi(2)
        };

        for i in 0..n {
            for j in (i + gap)..n {
                if j + gap > n && j < n {
                    break;
                }
                let dx = self.points[j].x - self.points[i].x;
                let dy = self.points[j].y - self.points[i].y;
                if dx * dx + dy * dy < threshold_sq {
                    return 1;
                }
            }
        }
        0
    }

    fn get_fold_indices(&self, speed_threshold: f64) -> Vec<usize> {
        let mut indices: Vec<usize> = Vec::new();
        for (i, p) in self.points.iter().enumerate() {
            let x = Vector2::new(p.x, p.y);
            let nhat = Vector2::new(p.nx, p.ny);
            if let Ok(f_val) = self.ode.vector_field(x) {
                let tangent = f_val + self.epsilon * nhat;
                if tangent.norm() < speed_threshold {
                    indices.push(i);
                }
            }
        }
        indices
    }
}

#[wasm_bindgen]
pub struct BdeSimulatorWasm {
    sim: BdeSimulator<DuffingODE>,
}

#[wasm_bindgen]
impl BdeSimulatorWasm {
    #[wasm_bindgen(constructor)]
    pub fn new(
        delta: f64,
        epsilon: f64,
        cx: f64,
        cy: f64,
        r: f64,
        num_points: usize,
    ) -> Result<BdeSimulatorWasm, JsValue> {
        let ode = DuffingODE::new(delta).map_err(|e| JsValue::from_str(&e))?;
        Ok(BdeSimulatorWasm {
            sim: BdeSimulator::new(ode, epsilon, cx, cy, r, num_points),
        })
    }

    pub fn step(&mut self, h: f64) -> JsValue {
        self.sim.step(h);
        serde_wasm_bindgen::to_value(self.sim.get_points()).unwrap()
    }

    pub fn get_points(&self) -> JsValue {
        serde_wasm_bindgen::to_value(self.sim.get_points()).unwrap()
    }

    /// arc-length reparameterize: redistribute points evenly along the curve.
    pub fn reparameterize(&mut self) {
        self.sim.reparameterize();
    }

    pub fn has_self_intersection(&self, gap: usize) -> u8 {
        self.sim.has_self_intersection(gap)
    }

    pub fn get_fold_indices(&self, speed_threshold: f64) -> JsValue {
        let indices = self.sim.get_fold_indices(speed_threshold);
        serde_wasm_bindgen::to_value(&indices).unwrap_or(JsValue::NULL)
    }
}

#[wasm_bindgen]
pub struct BdeSimulatorUserDefinedWasm {
    sim: BdeSimulator<UserDefinedOdeSystem>,
}

#[wasm_bindgen]
impl BdeSimulatorUserDefinedWasm {
    #[wasm_bindgen(constructor)]
    pub fn new(
        x_eq: &str,
        y_eq: &str,
        params: JsValue,
        epsilon: f64,
        cx: f64,
        cy: f64,
        r: f64,
        num_points: usize,
    ) -> Result<BdeSimulatorUserDefinedWasm, JsValue> {
        let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
        let ode =
            UserDefinedOdeSystem::new(x_eq, y_eq, param_set).map_err(|e| JsValue::from_str(&e))?;

        Ok(BdeSimulatorUserDefinedWasm {
            sim: BdeSimulator::new(ode, epsilon, cx, cy, r, num_points),
        })
    }

    pub fn step(&mut self, h: f64) -> JsValue {
        self.sim.step(h);
        serde_wasm_bindgen::to_value(self.sim.get_points()).unwrap()
    }

    pub fn get_points(&self) -> JsValue {
        serde_wasm_bindgen::to_value(self.sim.get_points()).unwrap()
    }

    pub fn reparameterize(&mut self) {
        self.sim.reparameterize();
    }

    pub fn has_self_intersection(&self, gap: usize) -> u8 {
        self.sim.has_self_intersection(gap)
    }

    pub fn get_fold_indices(&self, speed_threshold: f64) -> JsValue {
        let indices = self.sim.get_fold_indices(speed_threshold);
        serde_wasm_bindgen::to_value(&indices).unwrap_or(JsValue::NULL)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::ParameterEntry;
    use nalgebra::{Matrix2, Vector2};

    #[test]
    fn test_duffing_ode_eval() {
        let delta = 0.5;
        let ode = DuffingODE::new(delta).unwrap();

        let pos = Vector2::new(1.0, 2.0);
        let val = ode.vector_field(pos).unwrap();
        assert!((val.x - 2.0).abs() < 1e-10);
        assert!((val.y - -1.0).abs() < 1e-10);
    }

    #[test]
    fn test_user_defined_ode_vector_field() {
        let params = ParameterSet::new(vec![
            ParameterEntry {
                name: "a".to_string(),
                value: 2.0,
            },
            ParameterEntry {
                name: "b".to_string(),
                value: -1.5,
            },
        ])
        .unwrap();
        let ode = UserDefinedOdeSystem::new("a * x + y", "b * y", params).unwrap();

        let pos = Vector2::new(0.5, -2.0);
        let val = ode.vector_field(pos).unwrap();

        assert!((val.x - (2.0 * 0.5 - 2.0)).abs() < 1e-12);
        assert!((val.y - (-1.5 * -2.0)).abs() < 1e-12);
    }

    #[test]
    fn test_user_defined_ode_jacobian() {
        let params = ParameterSet::new(vec![
            ParameterEntry {
                name: "a".to_string(),
                value: 1.2,
            },
            ParameterEntry {
                name: "b".to_string(),
                value: -0.4,
            },
            ParameterEntry {
                name: "c".to_string(),
                value: 0.7,
            },
        ])
        .unwrap();
        let ode = UserDefinedOdeSystem::new("a * x + b * y", "c * x - y", params).unwrap();

        let pos = Vector2::new(1.0, -0.5);
        let jac = ode.jacobian(pos);

        assert!((jac.m11 - 1.2).abs() < 1e-6);
        assert!((jac.m12 - -0.4).abs() < 1e-6);
        assert!((jac.m21 - 0.7).abs() < 1e-6);
        assert!((jac.m22 - -1.0).abs() < 1e-6);
    }

    #[test]
    fn test_user_defined_ode_rk4_constant_field() {
        let params = ParameterSet::new(vec![
            ParameterEntry {
                name: "a".to_string(),
                value: 0.3,
            },
            ParameterEntry {
                name: "b".to_string(),
                value: -0.2,
            },
        ])
        .unwrap();
        let ode = UserDefinedOdeSystem::new("a", "b", params).unwrap();

        let pos = Vector2::new(-1.0, 2.0);
        let h = 0.5;
        let next = ode.rk4_step(pos, h).unwrap();

        assert!((next.x - (-1.0 + 0.3 * 0.5)).abs() < 1e-12);
        assert!((next.y - (2.0 - 0.2 * 0.5)).abs() < 1e-12);
    }

    #[test]
    fn test_user_defined_ode_many_parameters() {
        let params = ParameterSet::new(vec![
            ParameterEntry {
                name: "a".to_string(),
                value: 1.1,
            },
            ParameterEntry {
                name: "b".to_string(),
                value: -0.3,
            },
            ParameterEntry {
                name: "c".to_string(),
                value: 0.7,
            },
            ParameterEntry {
                name: "d".to_string(),
                value: 2.0,
            },
            ParameterEntry {
                name: "e".to_string(),
                value: -1.5,
            },
        ])
        .unwrap();

        let ode = UserDefinedOdeSystem::new("a * x + b * y + c", "d * x + e * y", params).unwrap();

        let pos = Vector2::new(2.0, -1.0);
        let val = ode.vector_field(pos).unwrap();

        assert!((val.x - (1.1 * 2.0 + -0.3 * -1.0 + 0.7)).abs() < 1e-12);
        assert!((val.y - (2.0 * 2.0 + -1.5 * -1.0)).abs() < 1e-12);
    }

    #[test]
    fn test_duffing_ode_is_odd() {
        let delta = 0.3;
        let ode = DuffingODE::new(delta).unwrap();

        let pos = Vector2::new(0.8, -1.2);
        let val = ode.vector_field(pos).unwrap();
        let neg_val = ode.vector_field(-pos).unwrap();

        assert!((val.x + neg_val.x).abs() < 1e-12);
        assert!((val.y + neg_val.y).abs() < 1e-12);
    }

    #[test]
    fn test_duffing_jacobian() {
        let delta = 0.3;
        let ode = DuffingODE::new(delta).unwrap();
        let pos = Vector2::new(2.0, -1.0);
        let jac = ode.jacobian(pos);

        assert!((jac.m11 - 0.0).abs() < 1e-12);
        assert!((jac.m12 - 1.0).abs() < 1e-12);
        assert!((jac.m21 - -11.0).abs() < 1e-12);
        assert!((jac.m22 + 0.3).abs() < 1e-12);
    }

    #[test]
    fn test_euler_step() {
        let delta = 0.5;
        let ode = DuffingODE::new(delta).unwrap();
        let h = 0.1;
        let eps = 0.05;
        let euler = EulerMap::new(ode, h, eps).unwrap();

        let pos = Vector2::new(1.0, 2.0);
        let next = euler.map(pos).unwrap();

        assert!((next.x - 1.2).abs() < 1e-10);
        assert!((next.y - 1.9).abs() < 1e-10);
    }

    #[test]
    fn test_euler_allows_zero_epsilon() {
        let delta = 0.1;
        let ode = DuffingODE::new(delta).unwrap();
        let h = 0.05;
        let euler = EulerMap::new(ode, h, 0.0).unwrap();
        assert!((euler.get_epsilon() - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_euler_rejects_negative_epsilon() {
        let delta = 0.1;
        let ode = DuffingODE::new(delta).unwrap();
        let h = 0.05;
        assert!(EulerMap::new(ode, h, -0.01).is_err());
    }

    #[test]
    fn test_euler_jacobian() {
        let delta = 0.2;
        let ode = DuffingODE::new(delta).unwrap();
        let h = 0.1;
        let eps = 0.05;
        let euler = EulerMap::new(ode, h, eps).unwrap();

        let pos = Vector2::new(2.0, 1.0);
        let jac = euler.jacobian(pos);

        assert!((jac.m11 - 1.0).abs() < 1e-10);
        assert!((jac.m12 - 0.1).abs() < 1e-10);
        assert!((jac.m21 - -1.1).abs() < 1e-10);
        assert!((jac.m22 - 0.98).abs() < 1e-10);
    }

    #[test]
    fn test_euler_get_epsilon() {
        let delta = 0.2;
        let ode = DuffingODE::new(delta).unwrap();
        let h = 0.3;
        let eps = 0.5;
        let euler = EulerMap::new(ode, h, eps).unwrap();

        assert!((euler.get_epsilon() - 0.15).abs() < 1e-10);
    }

    #[test]
    fn test_bde_rhs_zero_epsilon_matches_vector_field() {
        let delta = 0.2;
        let ode = DuffingODE::new(delta).unwrap();
        let x = Vector2::new(0.5, -0.25);
        let n = Vector2::new(0.6, 0.8);

        let (dx, _dn) = bde_rhs(&ode, 0.0, x, n).unwrap();
        let fx = ode.vector_field(x).unwrap();
        assert!((dx - fx).norm() < 1e-12);
    }

    #[test]
    fn test_bde_rhs_noise_adds_normal_component() {
        let delta = 0.2;
        let ode = DuffingODE::new(delta).unwrap();
        let x = Vector2::new(0.5, -0.25);
        let n = Vector2::new(0.6, 0.8);
        let eps = 0.2;

        let (dx0, _dn0) = bde_rhs(&ode, 0.0, x, n).unwrap();
        let (dx1, _dn1) = bde_rhs(&ode, eps, x, n).unwrap();
        let expected = dx0 + n * eps;

        assert!((dx1 - expected).norm() < 1e-12);
    }

    #[test]
    fn test_rk4_bde_step_matches_ode_when_epsilon_zero() {
        let delta = 0.15;
        let ode = DuffingODE::new(delta).unwrap();
        let x = Vector2::new(0.4, -0.7);
        let n = Vector2::new(0.3, 0.9539392014169457); // already unit-length
        let h = 0.05;

        let (xp, _np) = rk4_bde_step(&ode, 0.0, h, x, n).unwrap();
        let x_ode = ode.rk4_step(x, h).unwrap();

        assert!((xp - x_ode).norm() < 1e-12);
    }

    #[test]
    fn test_rk4_bde_step_user_defined_matches_ode_when_epsilon_zero() {
        let params = ParameterSet::new(vec![
            ParameterEntry {
                name: "a".to_string(),
                value: 1.0,
            },
            ParameterEntry {
                name: "b".to_string(),
                value: -0.5,
            },
        ])
        .unwrap();
        let ode = UserDefinedOdeSystem::new("a * x", "b * y", params).unwrap();
        let x = Vector2::new(0.4, -0.7);
        let n = Vector2::new(0.3, 0.9539392014169457);
        let h = 0.05;

        let (xp, _np) = rk4_bde_step(&ode, 0.0, h, x, n).unwrap();
        let x_ode = ode.rk4_step(x, h).unwrap();

        assert!((xp - x_ode).norm() < 1e-12);
    }

    #[test]
    fn test_rk4_bde_step_normal_is_unit() {
        let delta = 0.15;
        let ode = DuffingODE::new(delta).unwrap();
        let x = Vector2::new(-0.3, 0.45);
        let n = Vector2::new(0.8, 0.6);
        let h = 0.05;

        let (_xp, np) = rk4_bde_step(&ode, 0.1, h, x, n).unwrap();
        assert!((np.norm() - 1.0).abs() < 1e-10);
    }
}

#[wasm_bindgen]
pub fn boundary_map_duffing_ode(
    x: f64,
    y: f64,
    nx: f64,
    ny: f64,
    delta: f64,
    h: f64,
    epsilon: f64,
) -> JsValue {
    let ode = match DuffingODE::new(delta) {
        Ok(ode) => ode,
        Err(_) => return JsValue::NULL,
    };
    match rk4_bde_step(&ode, epsilon, h, Vector2::new(x, y), Vector2::new(nx, ny)) {
        Ok((next_x, next_n)) => {
            let p = ExtendedPoint::new(next_x.x, next_x.y, next_n.x, next_n.y);
            serde_wasm_bindgen::to_value(&p).unwrap_or(JsValue::NULL)
        }
        Err(_) => JsValue::NULL,
    }
}

#[derive(Serialize)]
struct PointResult {
    x: f64,
    y: f64,
}

#[wasm_bindgen]
pub fn evaluate_user_defined_ode(
    x: f64,
    y: f64,
    x_eq: &str,
    y_eq: &str,
    params: JsValue,
) -> Result<JsValue, JsValue> {
    let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
    let ode = UserDefinedOdeSystem::new(x_eq, y_eq, param_set)
        .map_err(|e| JsValue::from_str(&format!("Error parsing equations: {}", e)))?;

    let pos = Vector2::new(x, y);
    let vel = ode
        .vector_field(pos)
        .map_err(|e| JsValue::from_str(&format!("Error evaluating vector field: {}", e)))?;

    let result = PointResult { x: vel.x, y: vel.y };
    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    result.serialize(&serializer).map_err(|e| {
        console_error!("Serialization error: {:?}", e);
        JsValue::from_str("Failed to serialize result")
    })
}

#[wasm_bindgen]
pub fn boundary_map_user_defined_ode(
    x: f64,
    y: f64,
    nx: f64,
    ny: f64,
    x_eq: &str,
    y_eq: &str,
    params: JsValue,
    h: f64,
    epsilon: f64,
) -> Result<JsValue, JsValue> {
    let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
    let ode = UserDefinedOdeSystem::new(x_eq, y_eq, param_set)
        .map_err(|e| JsValue::from_str(&format!("Error parsing equations: {}", e)))?;

    let pos = Vector2::new(x, y);
    let normal = Vector2::new(nx, ny);
    let (next_x, next_n) = rk4_bde_step(&ode, epsilon, h, pos, normal)
        .map_err(|e| JsValue::from_str(&format!("Error evaluating BDE: {}", e)))?;

    let p = ExtendedPoint::new(next_x.x, next_x.y, next_n.x, next_n.y);
    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    p.serialize(&serializer).map_err(|e| {
        console_error!("Serialization error: {:?}", e);
        JsValue::from_str("Failed to serialize result")
    })
}
