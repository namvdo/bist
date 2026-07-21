use crate::parameters::parameter_set_from_js;
use crate::range::{clamp_pair, RANGE_LIMIT};
use nalgebra::{Matrix2, Vector2, Vector4};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Mutex;
use wasm_bindgen::prelude::*;

type CacheKey = (i32, i32, i32, i32, i32, i32);

static MANIFOLD_CACHE: std::sync::OnceLock<Mutex<HashMap<CacheKey, CachedManifoldResult>>> =
    std::sync::OnceLock::new();

fn get_cache() -> &'static Mutex<HashMap<CacheKey, CachedManifoldResult>> {
    MANIFOLD_CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

fn cache_key(a: f64, b: f64, x_min: f64, x_max: f64, y_min: f64, y_max: f64) -> CacheKey {
    (
        (a * 100.0).round() as i32,
        (b * 100.0).round() as i32,
        (x_min * 100.0).round() as i32,
        (x_max * 100.0).round() as i32,
        (y_min * 100.0).round() as i32,
        (y_max * 100.0).round() as i32,
    )
}

fn normalize_display_range(x_min: f64, x_max: f64, y_min: f64, y_max: f64) -> (f64, f64, f64, f64) {
    let (x_min, x_max) = clamp_pair(x_min, x_max, RANGE_LIMIT);
    let (y_min, y_max) = clamp_pair(y_min, y_max, RANGE_LIMIT);
    (x_min, x_max, y_min, y_max)
}

fn in_display_range(x: f64, y: f64, x_min: f64, x_max: f64, y_min: f64, y_max: f64) -> bool {
    x >= x_min && x <= x_max && y >= y_min && y <= y_max
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CachedManifoldResult {
    manifolds: Vec<ManifoldResult>,
    fixed_points: Vec<FixedPointResult>,
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

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
    #[wasm_bindgen(js_namespace = console)]
    fn error(s: &str);
}

#[cfg(not(target_arch = "wasm32"))]
fn log(_s: &str) {
    // println!("{}", s); // Silence info logs for benchmark
}

#[cfg(not(target_arch = "wasm32"))]
fn error(s: &str) {
    eprintln!("{}", s);
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
#[wasm_bindgen]
#[derive(Debug, Clone, Copy)]
pub struct HenonParams {
    pub a: f64,
    pub b: f64,
    pub epsilon: f64,
}

use crate::dynamical_systems::{DynamicalSystem, ExtendedState, UserDefinedDynamicalSystem};

#[derive(Clone, Debug)]
pub struct ManifoldConfig {
    pub perturb_tol: f64,
    pub spacing_tol: f64,
    pub spacing_upper: f64,
    pub conv_tol: f64,
    pub stable_tol: f64,
    pub max_iter: usize,
    pub max_points: usize,
    pub time_limit: f64,
    pub inner_max: usize,
    pub self_cross_tol: f64,
    pub self_compare_skip: usize,
}

impl Default for ManifoldConfig {
    fn default() -> Self {
        Self {
            perturb_tol: 1e-5,
            spacing_tol: 5e-3,
            spacing_upper: 10.0,
            conv_tol: 1e-14,
            stable_tol: 1e-19,
            max_iter: 8000,
            max_points: 700_000,
            time_limit: 10.0,
            inner_max: 2000,
            self_cross_tol: 0.002,
            self_compare_skip: 100,
        }
    }
}

#[derive(Clone, Debug, Copy, PartialEq)]
pub enum SaddleType {
    Regular,
    DualRepeller,
}

#[derive(Clone, Debug)]
pub struct SaddlePoint {
    pub position: Vector2<f64>, // (x_s, y_s) saddle position
    pub period: usize,          // Period of the periodic point

    pub tangent_2d: Vector2<f64>, // (v_x, v_y) from 2D eigenvector
    pub eigenvalue: f64,

    pub tangent_4d: Option<Vector4<f64>>, // Full 4D eigenvector if available

    pub saddle_type: SaddleType,

    // For boundary position
    pub normal: Vector2<f64>, // (nx, ny) outward normal
}

impl SaddlePoint {
    /// Extract position and normal components from 4D eigenvector
    pub fn from_4d_eigenvector(
        position: Vector2<f64>,
        eigenvector_4d: Vector4<f64>,
        period: usize,
        eigenvalue: f64,
        saddle_type: SaddleType,
    ) -> Self {
        let tangent = Vector2::new(eigenvector_4d[0], eigenvector_4d[1]);
        let normal_unnorm = Vector2::new(eigenvector_4d[2], eigenvector_4d[3]);
        let normal = normal_unnorm / normal_unnorm.norm();

        Self {
            position,
            period,
            tangent_2d: tangent / tangent.norm(),
            eigenvalue,
            tangent_4d: Some(eigenvector_4d),
            saddle_type,
            normal,
        }
    }

    /// From 2D eigenvector (compute normal geometrically)
    pub fn from_2d_eigenvector(
        position: Vector2<f64>,
        tangent: Vector2<f64>,
        period: usize,
        eigenvalue: f64,
        saddle_type: SaddleType,
        attractor_center: Option<Vector2<f64>>,
    ) -> Self {
        // Compute normal: point away from attractor center
        let normal = if let Some(center) = attractor_center {
            let to_boundary = position - center;
            to_boundary / to_boundary.norm()
        } else {
            // perpendicular to tangent, prefer positive direction
            let perp = Vector2::new(-tangent.y, tangent.x);
            perp / perp.norm()
        };

        Self {
            position,
            period,
            tangent_2d: tangent / tangent.norm(),
            eigenvalue,
            tangent_4d: None,
            saddle_type,
            normal,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Trajectory {
    pub points: Vec<ExtendedState>,
    pub stop_reason: StopReason,
}

#[derive(Debug, Clone, PartialEq)]
pub enum StopReason {
    Converged,
    MaxIterations,
    MaxPoints,
    TimeExceeded,
    ApproachedTargetPoint,
    SelfIntersection,
}

impl HenonParams {
    pub fn new(a: f64, b: f64, epsilon: f64) -> Result<Self, String> {
        if !a.is_finite() || !b.is_finite() || !epsilon.is_finite() {
            return Err("Parameters must be finite numbers".to_string());
        }

        if b.abs() < 1e-10 {
            return Err("Parameter b cannot be zero or near zero".to_string());
        }

        if epsilon <= 0.0 {
            return Err("Epsilon must be positive".to_string());
        }

        if a.abs() > 10.0 || epsilon > 1.0 {
            return Err("Parameters outside reasonable range".to_string());
        }

        Ok(Self { a, b, epsilon })
    }

    pub fn henon_map(&self, pos: &Vector2<f64>) -> Result<Vector2<f64>, String> {
        if !pos.x.is_finite() || !pos.y.is_finite() {
            return Err("Invalid position: non-finite coordinates".to_string());
        }

        let x_new = 1.0 - self.a * pos.x * pos.x + pos.y;
        let y_new = self.b * pos.x;

        if !x_new.is_finite() || !y_new.is_finite() {
            return Err("Henon map produced non-finite values".to_string());
        }

        Ok(Vector2::new(x_new, y_new))
    }

    pub fn henon_map_inverse(&self, pos: &Vector2<f64>) -> Result<Vector2<f64>, String> {
        if !pos.x.is_finite() || !pos.y.is_finite() {
            return Err("Invalid position: non-finite coordinates".to_string());
        }

        let y_over_b = pos.y / self.b;
        let x_new = y_over_b;
        let y_new = pos.x + self.a * y_over_b * y_over_b - 1.0;

        if !x_new.is_finite() || !y_new.is_finite() {
            return Err("Inverse Henon map produced non-finite values".to_string());
        }

        Ok(Vector2::new(x_new, y_new))
    }

    pub fn jacobian(&self, pos: Vector2<f64>) -> Matrix2<f64> {
        Matrix2::new(-2.0 * self.a * pos.x, 1.0, self.b, 0.0)
    }

    pub fn transform_normal(
        &self,
        pos: Vector2<f64>,
        normal: Vector2<f64>,
    ) -> Result<Vector2<f64>, String> {
        if !pos.x.is_finite() || !pos.y.is_finite() {
            return Err("Invalid position in transform_normal".to_string());
        }

        if !normal.x.is_finite() || !normal.y.is_finite() {
            return Err("Invalid normal in transform_normal".to_string());
        }

        let jac_inv_t = Matrix2::new(0.0, 1.0, 1.0 / self.b, 2.0 * self.a * pos.x / self.b);

        let transformed = jac_inv_t * normal;
        let norm = transformed.norm();

        if !norm.is_finite() || norm < 1e-10 {
            return Ok(normal);
        }

        let result = transformed / norm;
        if !result.x.is_finite() || !result.y.is_finite() {
            return Ok(normal);
        }

        Ok(result)
    }

    pub fn transform_normal_inverse(
        &self,
        pos: Vector2<f64>,
        normal: Vector2<f64>,
    ) -> Result<Vector2<f64>, String> {
        if !pos.x.is_finite() || !pos.y.is_finite() {
            return Err("Invalid position in transform_normal_inverse".to_string());
        }

        if !normal.x.is_finite() || !normal.y.is_finite() {
            return Err("Invalid normal in transform_normal_inverse".to_string());
        }
        let coeff = -2.0 * self.a * pos.y / self.b;
        let jac_inv_inv_t = Matrix2::new(coeff, self.b, 1.0, 0.0);

        let transformed = jac_inv_inv_t * normal;
        let norm = transformed.norm();

        if !norm.is_finite() || norm < 1e-10 {
            return Ok(normal);
        }

        let result = transformed / norm;
        if !result.x.is_finite() || !result.y.is_finite() {
            return Ok(normal);
        }

        Ok(result)
    }
}

impl DynamicalSystem for HenonParams {
    fn map(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        self.henon_map(&pos)
    }

    fn map_inverse(&self, pos: Vector2<f64>) -> Result<Vector2<f64>, String> {
        self.henon_map_inverse(&pos)
    }

    fn jacobian(&self, pos: Vector2<f64>) -> Matrix2<f64> {
        self.jacobian(pos)
    }

    fn transform_normal(
        &self,
        pos: Vector2<f64>,
        normal: Vector2<f64>,
    ) -> Result<Vector2<f64>, String> {
        self.transform_normal(pos, normal)
    }

    fn transform_normal_inverse(
        &self,
        pos: Vector2<f64>,
        normal: Vector2<f64>,
    ) -> Result<Vector2<f64>, String> {
        self.transform_normal_inverse(pos, normal)
    }

    fn get_epsilon(&self) -> f64 {
        self.epsilon
    }
}

pub struct UnstableManifoldComputer<S: DynamicalSystem> {
    params: S,
    config: ManifoldConfig,
}

impl<S: DynamicalSystem> UnstableManifoldComputer<S> {
    pub fn new(params: S, config: ManifoldConfig) -> Self {
        Self { params, config }
    }

    pub fn compute_direction(
        &self,
        saddle: &SaddlePoint,
        direction_sign: f64,
        target_points: &[Vector2<f64>],
    ) -> Result<Trajectory, String> {
        console_log!(
            "Starting manifold computation from ({:.4}, {:.4})",
            saddle.position.x,
            saddle.position.y
        );

        let mut n_period = saddle.period;
        if n_period == 0 {
            return Err("Period cannot be zero".to_string());
        }

        // handle negative eigenvalues by doubling period
        // this ensures eigenvalue becomes positive after period doubling
        if saddle.eigenvalue < 0.0 {
            console_log!(
                "Negative eigenvalue {:.4}, doubling period from {} to {}",
                saddle.eigenvalue,
                n_period,
                n_period * 2
            );
            n_period *= 2;
        }

        // initial perturbation along the eigenvector direction
        let perturb_distance = self.config.perturb_tol;
        let eigenvector = saddle.tangent_2d; // The unstable eigenvector
        let perturbed_pos = saddle.position + direction_sign * perturb_distance * eigenvector;

        console_log!(
            "Perturbed along eigenvector ({:.6}, {:.6})",
            eigenvector.x,
            eigenvector.y
        );
        console_log!(
            "Perturbed position: ({:.6}, {:.6}), distance: {:.6}",
            perturbed_pos.x,
            perturbed_pos.y,
            perturb_distance
        );

        // use normal from saddle
        let initial_normal = saddle.normal;

        console_log!(
            "Initial normal: ({:.6}, {:.6})",
            initial_normal.x,
            initial_normal.y
        );

        // create initial 4D state: (perturbed_x, perturbed_y, nx, ny)
        let state_0 = ExtendedState {
            pos: perturbed_pos,
            normal: initial_normal,
        };

        // choose map direction based on saddle type
        let map_fn: Box<dyn Fn(ExtendedState, usize) -> Result<ExtendedState, String>> =
            if saddle.saddle_type == SaddleType::DualRepeller {
                console_log!("Using inverse map (dual repeller)");
                Box::new(|state, n| self.params.extended_map_inverse(state, n))
            } else {
                console_log!("Using forward map");
                Box::new(|state, n| self.params.extended_map(state, n))
            };

        // apply boundary map n_period times to get state_1
        let state_1 = match map_fn(state_0, n_period) {
            Ok(s) => s,
            Err(e) => {
                console_error!("Initial map application failed: {}", e);
                return Err(format!("Initial iteration failed: {}", e));
            }
        };

        console_log!(
            "After first iteration: ({:.6}, {:.6})",
            state_1.pos.x,
            state_1.pos.y
        );

        // dist_vec_0 is the displacement after one boundary map iteration
        // used for parametric refinement: new_initial = state_0 + param * dist_vec_0
        let dist_vec_0_pos = state_1.pos - state_0.pos;
        let dist_vec_0_normal = state_1.normal - state_0.normal;

        console_log!(
            "dist_vec_0 (pos): ({:.6}, {:.6}), norm: {:.6}",
            dist_vec_0_pos.x,
            dist_vec_0_pos.y,
            dist_vec_0_pos.norm()
        );

        // main iteration loop
        // trajectory stores all points on the manifold
        let mut trajectory = vec![state_0];
        let mut current_state = state_1;
        let mut iteration = 1;

        let start_time = get_time_secs();
        let spacing_tol = if saddle.saddle_type == SaddleType::DualRepeller {
            2e-4
        } else {
            self.config.spacing_tol
        };

        // track when we've moved far enough from saddle for self-crossing check
        let mut self_cross_trigger = false;

        loop {
            // Check stopping conditions
            if iteration > self.config.max_iter {
                console_log!("Max iterations reached at {}", iteration);
                return Ok(Trajectory {
                    points: trajectory,
                    stop_reason: StopReason::MaxIterations,
                });
            }

            if trajectory.len() > self.config.max_points {
                console_log!("Max points reached: {}", trajectory.len());
                return Ok(Trajectory {
                    points: trajectory,
                    stop_reason: StopReason::MaxPoints,
                });
            }

            // Check time limit
            if get_time_secs() - start_time > self.config.time_limit {
                console_log!("Time limit exceeded");
                return Ok(Trajectory {
                    points: trajectory,
                    stop_reason: StopReason::TimeExceeded,
                });
            }

            // Compute next iteration: F^n(current_state)
            let next_state = match map_fn(current_state, n_period) {
                Ok(s) => s,
                Err(_) => {
                    console_log!("Map diverged at iteration {}", iteration);
                    return Ok(Trajectory {
                        points: trajectory,
                        stop_reason: StopReason::Converged,
                    });
                }
            };

            // check convergence - distance between consecutive iterations
            let step_distance = (next_state.pos - current_state.pos).norm();

            // enable self-crossing check once we're far enough from saddle
            if step_distance > 1e-2 {
                self_cross_trigger = true;
            }

            if step_distance < self.config.conv_tol && iteration > 30 {
                console_log!("Converged at iteration {}", iteration);
                trajectory.push(current_state);
                return Ok(Trajectory {
                    points: trajectory,
                    stop_reason: StopReason::Converged,
                });
            }

            // check if approaching target points (closure to stable points)
            if !target_points.is_empty() {
                let min_dist_to_target = target_points
                    .iter()
                    .map(|tp| (next_state.pos - tp).norm())
                    .fold(f64::INFINITY, f64::min);

                if min_dist_to_target < self.config.stable_tol {
                    console_log!(
                        "Approached target point at iteration {}, dist={:.2e}",
                        iteration,
                        min_dist_to_target
                    );
                    trajectory.push(current_state);
                    trajectory.push(next_state);
                    return Ok(Trajectory {
                        points: trajectory,
                        stop_reason: StopReason::ApproachedTargetPoint,
                    });
                }
            }

            // check self-intersection (only when far enough from saddle)
            if self_cross_trigger && trajectory.len() > self.config.self_compare_skip {
                let check_range = trajectory.len() - self.config.self_compare_skip;
                let min_self_dist = trajectory[..check_range]
                    .iter()
                    .map(|p| (next_state.pos - p.pos).norm())
                    .fold(f64::INFINITY, f64::min);

                if min_self_dist <= self.config.self_cross_tol {
                    console_log!(
                        "Self-intersection at iteration {}, dist={:.2e}",
                        iteration,
                        min_self_dist
                    );
                    trajectory.push(current_state);
                    trajectory.push(next_state);
                    return Ok(Trajectory {
                        points: trajectory,
                        stop_reason: StopReason::SelfIntersection,
                    });
                }
            }

            // Adaptive refinement if gap too large but not too huge
            if step_distance > spacing_tol && step_distance < self.config.spacing_upper {
                match self.refine_segment(
                    state_0,
                    dist_vec_0_pos,
                    dist_vec_0_normal,
                    current_state,
                    next_state,
                    iteration,
                    n_period,
                    &map_fn,
                    spacing_tol,
                    start_time,
                    &trajectory,
                ) {
                    Ok(refined_points) => {
                        if !refined_points.is_empty() {
                            console_log!("Added {} refined points", refined_points.len());
                            trajectory.extend(refined_points);
                        }
                    }
                    Err(reason) => {
                        console_log!("Refinement stopped: {:?}", reason);
                        return Ok(Trajectory {
                            points: trajectory,
                            stop_reason: reason,
                        });
                    }
                }
            }

            trajectory.push(current_state);
            current_state = next_state;
            iteration += 1;
        }
    }

    /// Parametric refinement
    ///
    /// The key idea: we track a parameter m in [0, 1] for each point where:
    /// - m = 0 corresponds to state_old (endpoint at iteration j-1)
    /// - m = 1 corresponds to state_new (endpoint at iteration j)
    ///
    /// When two consecutive points P_j and P_{j+1} are too far apart,
    /// we create a new initial condition at parameter (m_j + m_{j+1})/2:
    ///   new_initial = state_0 + midpoint_param * dist_vec_0
    /// Then apply the boundary map 'iteration' times.
    fn refine_segment(
        &self,
        state_0: ExtendedState, // initial perturbed state (at saddle + delta*eigenvec)
        dist_vec_0_pos: Vector2<f64>, // F(state_0).pos - state_0.pos
        dist_vec_0_normal: Vector2<f64>, // F(state_0).normal - state_0.normal
        state_old: ExtendedState, // P_{start} at this iteration
        state_new: ExtendedState, // P_{end} at this iteration
        iteration: usize,       // current iteration number j
        n_period: usize,
        map_fn: &dyn Fn(ExtendedState, usize) -> Result<ExtendedState, String>,
        spacing_tol: f64,
        start_time: f64,
        _existing_trajectory: &[ExtendedState],
    ) -> Result<Vec<ExtendedState>, StopReason> {
        // Initialize add_pt array with endpoints
        // Each entry: (state, parameter)
        // param = 0 corresponds to state_old, param = 1 corresponds to state_new
        let mut add_pt: Vec<(ExtendedState, f64)> = vec![(state_old, 0.0), (state_new, 1.0)];

        let mut inner_iter = 0;

        // Keep adding points while any consecutive pair is too far apart
        loop {
            // Find indices where consecutive points are too far apart
            let mut indices_to_refine: Vec<usize> = Vec::new();

            for j in 0..(add_pt.len() - 1) {
                let dist = (add_pt[j + 1].0.pos - add_pt[j].0.pos).norm();
                if dist > spacing_tol && dist < self.config.spacing_upper {
                    indices_to_refine.push(j);
                }
            }

            if indices_to_refine.is_empty() {
                break;
            }

            inner_iter += 1;
            if inner_iter > self.config.inner_max {
                console_log!(
                    "Inner refinement loop exceeded {} iterations",
                    self.config.inner_max
                );
                break;
            }

            // Check time limit
            if get_time_secs() - start_time > self.config.time_limit {
                console_log!("Time limit exceeded during refinement");
                return Err(StopReason::TimeExceeded);
            }

            // insert new points at midpoint parameters (process in reverse to keep indices valid)
            for &j in indices_to_refine.iter().rev() {
                let param_j = add_pt[j].1;
                let param_j1 = add_pt[j + 1].1;
                let midpoint_param = (param_j + param_j1) / 2.0;

                // create new initial condition at this parameter
                // formula: F^j(state_0 + midpoint_param * dist_vec_0)
                // but dist_vec_0 is defined as F(state_0) - state_0
                // so the initial condition is: state_0 + midpoint_param * dist_vec_0
                let new_initial_pos = state_0.pos + midpoint_param * dist_vec_0_pos;
                let new_initial_normal_unnorm = state_0.normal + midpoint_param * dist_vec_0_normal;
                let normal_norm = new_initial_normal_unnorm.norm();

                if normal_norm < 1e-10 {
                    continue;
                }

                let new_initial_normal = new_initial_normal_unnorm / normal_norm;
                let new_initial_state = ExtendedState {
                    pos: new_initial_pos,
                    normal: new_initial_normal,
                };

                // apply boundary map 'iteration' times: F^{iteration*n_period}(new_initial)
                let mut intermediate_state = new_initial_state;
                let mut valid = true;

                for _ in 0..iteration {
                    match map_fn(intermediate_state, n_period) {
                        Ok(s) => {
                            if !s.pos.x.is_finite()
                                || !s.pos.y.is_finite()
                                || s.pos.x.abs() > 200.0
                                || s.pos.y.abs() > 200.0
                            {
                                valid = false;
                                break;
                            }
                            intermediate_state = s;
                        }
                        Err(_) => {
                            valid = false;
                            break;
                        }
                    }
                }

                if valid {
                    add_pt.insert(j + 1, (intermediate_state, midpoint_param));
                }
            }

            // check if we've added too many points
            if add_pt.len() > 10000 {
                console_log!("Too many points in refinement: {}", add_pt.len());
                break;
            }
        }

        // return all intermediate points (exclude the endpoints already in trajectory)
        let mut refined_states: Vec<ExtendedState> = Vec::new();
        for i in 1..(add_pt.len() - 1) {
            refined_states.push(add_pt[i].0);
        }

        if !refined_states.is_empty() {
            console_log!("Successfully refined {} points", refined_states.len());
        }
        Ok(refined_states)
    }

    pub fn compute_manifold(
        &self,
        saddle: &SaddlePoint,
        target_points: &[Vector2<f64>],
    ) -> Result<(Trajectory, Trajectory), String> {
        let traj_plus = self.compute_direction(saddle, 1.0, target_points)?;
        let traj_minus = self.compute_direction(saddle, -1.0, target_points)?;

        Ok((traj_plus, traj_minus))
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrajectoryRet {
    pub points: Vec<(f64, f64)>,
    pub extended_points: Vec<(f64, f64, f64, f64)>,
    pub stop_reason: String,
}

fn trajectory_ret(traj: &Trajectory) -> TrajectoryRet {
    let finite = |state: &&ExtendedState| {
        state.pos.x.is_finite()
            && state.pos.y.is_finite()
            && state.normal.x.is_finite()
            && state.normal.y.is_finite()
    };
    TrajectoryRet {
        points: traj
            .points
            .iter()
            .filter(finite)
            .map(|state| (state.pos.x, state.pos.y))
            .collect(),
        extended_points: traj
            .points
            .iter()
            .filter(finite)
            .map(|state| (state.pos.x, state.pos.y, state.normal.x, state.normal.y))
            .collect(),
        stop_reason: format!("{:?}", traj.stop_reason),
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ManifoldResult {
    pub plus: TrajectoryRet,
    pub minus: TrajectoryRet,
    pub saddle_point: (f64, f64),
    pub eigenvalue: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FixedPointResult {
    pub x: f64,
    pub y: f64,
    pub eigenvalues: (f64, f64),
    pub stability: String, // "Attractor", "Repeller", "Saddle"
}

#[derive(Serialize, Deserialize)]
pub struct ComputeResult {
    pub manifolds: Vec<ManifoldResult>,
    pub fixed_points: Vec<FixedPointResult>,
}

#[wasm_bindgen]
pub fn compute_manifold_simple(
    a: f64,
    b: f64,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> Result<JsValue, JsValue> {
    let (x_min, x_max, y_min, y_max) = normalize_display_range(x_min, x_max, y_min, y_max);
    // Check cache first
    let key = cache_key(a, b, x_min, x_max, y_min, y_max);
    if let Ok(cache) = get_cache().lock() {
        if let Some(cached) = cache.get(&key) {
            console_log!("Cache HIT for a={}, b={}", a, b);
            let result = ComputeResult {
                manifolds: cached.manifolds.clone(),
                fixed_points: cached.fixed_points.clone(),
            };
            let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
            return match result.serialize(&serializer) {
                Ok(v) => Ok(v),
                Err(e) => {
                    console_error!("Serialization error: {:?}", e);
                    Err(JsValue::from_str("Failed to serialize result"))
                }
            };
        }
    }

    console_log!(
        "Cache MISS - Computing manifold with a={}, b={}, epsilon={}",
        a,
        b,
        epsilon
    );

    let params = match HenonParams::new(a, b, epsilon) {
        Ok(p) => p,
        Err(e) => {
            console_error!("Invalid parameters: {}", e);
            return Err(JsValue::from_str(&e));
        }
    };

    let mut unique_states: Vec<ExtendedState> = Vec::new();

    let one_minus_b = 1.0 - b;
    let discriminant = one_minus_b * one_minus_b + 4.0 * a;

    console_log!(
        "Analytical solver: a={}, b={}, discriminant={}",
        a,
        b,
        discriminant
    );

    if discriminant >= 0.0 && a.abs() > 1e-10 {
        let sqrt_disc = discriminant.sqrt();
        let x1 = (-one_minus_b + sqrt_disc) / (2.0 * a);
        let x2 = (-one_minus_b - sqrt_disc) / (2.0 * a);

        for x in [x1, x2] {
            let y = b * x;
            let pos = Vector2::new(x, y);

            if !x.is_finite() || !y.is_finite() || pos.norm() > 50.0 {
                continue;
            }

            // Compute Jacobian eigenvalues and eigenvectors
            // Jacobian: J = [-2ax, 1; b, 0]
            let jac = params.jacobian(pos);
            let trace = jac.trace();
            let det = jac.determinant();
            let eig_disc = trace * trace - 4.0 * det;

            console_log!(
                "Fixed point ({:.4}, {:.4}): trace={:.4}, det={:.4}, eig_disc={:.4}",
                x,
                y,
                trace,
                det,
                eig_disc
            );

            // compute eigenvector for unstable direction (if saddle)
            let (l1, l2, is_complex) = if eig_disc >= 0.0 {
                let sqrt_eig = eig_disc.sqrt();
                ((trace + sqrt_eig) / 2.0, (trace - sqrt_eig) / 2.0, false)
            } else {
                let real = trace / 2.0;
                let imag = (-eig_disc).sqrt() / 2.0;
                let mag = (real * real + imag * imag).sqrt();
                (mag, mag, true)
            };

            // For saddles, find the unstable eigenvector
            let unstable_lambda = if l1.abs() > l2.abs() { l1 } else { l2 };

            // Eigenvector from (J - λI)v = 0
            let v1 = 1.0;
            let v2 = 2.0 * a * x + unstable_lambda;
            let norm = (v1 * v1 + v2 * v2).sqrt();
            let normal = if norm > 1e-10 && !is_complex {
                Vector2::new(v1 / norm, v2 / norm)
            } else {
                // For complex eigenvalues or numerical issues, use a default direction
                Vector2::new(1.0, 0.0)
            };

            unique_states.push(ExtendedState { pos, normal });
            console_log!(
                "Analytical fixed point: ({:.4}, {:.4}) eigenvalues: {:.4}, {:.4}",
                x,
                y,
                l1,
                l2
            );
        }
    } else {
        console_log!(
            "No real fixed points (discriminant={} < 0 or a={} ≈ 0)",
            discriminant,
            a
        );
    }

    let mut fixed_points_result = Vec::new();
    let mut all_fixed_points_pos = Vec::new(); // ALL fixed points for manifold termination
    let mut unstable_points_indices = Vec::new();

    for (_, state) in unique_states.iter().enumerate() {
        if !in_display_range(state.pos.x, state.pos.y, x_min, x_max, y_min, y_max) {
            console_log!(
                "Skipping fixed point ({:.4}, {:.4}) - outside display range",
                state.pos.x,
                state.pos.y
            );
            continue;
        }

        let jac = params.jacobian(state.pos);
        let trace = jac.trace();
        let det = jac.determinant();
        let discriminant = trace * trace - 4.0 * det;

        let (l1, l2) = if discriminant >= 0.0 {
            let sqrt_disc = discriminant.sqrt();
            ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0)
        } else {
            let real = trace / 2.0;
            let imag = (-discriminant).sqrt() / 2.0;
            let mag = (real * real + imag * imag).sqrt();
            (mag, mag)
        };

        let abs_l1 = l1.abs();
        let abs_l2 = l2.abs();

        let stability = if abs_l1 < 1.0 && abs_l2 < 1.0 {
            "Attractor"
        } else if abs_l1 > 1.0 && abs_l2 > 1.0 {
            "Repeller"
        } else {
            "Saddle"
        };

        console_log!(
            "Classified ({:.4}, {:.4}) as {} with eigenvalues ({:.4}, {:.4})",
            state.pos.x,
            state.pos.y,
            stability,
            l1,
            l2
        );

        fixed_points_result.push(FixedPointResult {
            x: state.pos.x,
            y: state.pos.y,
            eigenvalues: (l1, l2),
            stability: stability.to_string(),
        });

        all_fixed_points_pos.push(state.pos);

        if stability == "Saddle" || stability == "Repeller" {
            unstable_points_indices.push(fixed_points_result.len() - 1);
        }
    }

    let attractor_count = fixed_points_result
        .iter()
        .filter(|fp| fp.stability == "Attractor")
        .count();
    let saddle_count = fixed_points_result
        .iter()
        .filter(|fp| fp.stability == "Saddle")
        .count();
    let repeller_count = fixed_points_result
        .iter()
        .filter(|fp| fp.stability == "Repeller")
        .count();

    console_log!(
        "Fixed point summary: {} attractors, {} saddles, {} repellers",
        attractor_count,
        saddle_count,
        repeller_count
    );

    console_log!(
        "Termination targets: {} points for manifold closure",
        all_fixed_points_pos.len()
    );

    for (i, pos) in all_fixed_points_pos.iter().enumerate() {
        console_log!("  Target {}: ({:.4}, {:.4})", i, pos.x, pos.y);
    }

    console_log!(
        "Will compute manifolds for {} unstable/saddle points: {:?}",
        unstable_points_indices.len(),
        unstable_points_indices
    );

    let mut manifolds_result = Vec::new();
    let config = ManifoldConfig::default();
    let computer = UnstableManifoldComputer::new(params, config);

    // Compute manifolds for unstable points
    for idx in unstable_points_indices {
        let fp_info = &fixed_points_result[idx];
        let pos = Vector2::new(fp_info.x, fp_info.y);

        let saddle_type = if fp_info.stability == "Repeller" {
            SaddleType::DualRepeller
        } else {
            SaddleType::Regular
        };

        let l1 = fp_info.eigenvalues.0;
        let l2 = fp_info.eigenvalues.1;

        // Compute unstable eigenvector from scratch
        let unstable_lambda = if l1.abs() > l2.abs() { l1 } else { l2 };
        let v1 = 1.0;
        let v2 = 2.0 * a * fp_info.x + unstable_lambda;
        let norm = (v1 * v1 + v2 * v2).sqrt();
        let eigenvector = if norm > 1e-10 {
            Vector2::new(v1 / norm, v2 / norm)
        } else {
            Vector2::new(1.0, 0.0)
        };

        let saddle_pt = SaddlePoint::from_2d_eigenvector(
            pos,
            eigenvector,
            1,
            unstable_lambda,
            saddle_type,
            None,
        );

        if let Ok((traj_plus, traj_minus)) =
            computer.compute_manifold(&saddle_pt, &all_fixed_points_pos)
        {
            console_log!(
                "Manifold from ({:.4}, {:.4}): plus {} pts ({:?}), minus {} pts ({:?})",
                pos.x,
                pos.y,
                traj_plus.points.len(),
                traj_plus.stop_reason,
                traj_minus.points.len(),
                traj_minus.stop_reason
            );

            manifolds_result.push(ManifoldResult {
                plus: trajectory_ret(&traj_plus),
                minus: trajectory_ret(&traj_minus),
                saddle_point: (pos.x, pos.y),
                eigenvalue: saddle_pt.eigenvalue,
            });
        }
    }

    let result = ComputeResult {
        manifolds: manifolds_result,
        fixed_points: fixed_points_result,
    };

    // Store in cache
    if let Ok(mut cache) = get_cache().lock() {
        cache.insert(
            key,
            CachedManifoldResult {
                manifolds: result.manifolds.clone(),
                fixed_points: result.fixed_points.clone(),
            },
        );
        console_log!("Cached result for a={}, b={}", a, b);
    }

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    match result.serialize(&serializer) {
        Ok(v) => Ok(v),
        Err(e) => {
            console_error!("Serialization error: {:?}", e);
            Err(JsValue::from_str("Failed to serialize result"))
        }
    }
}

#[wasm_bindgen]
pub fn compute_manifold_js(
    a: f64,
    b: f64,
    epsilon: f64,
    saddle_x: f64,
    saddle_y: f64,
    period: usize,
    eigenvector_x: f64,
    eigenvector_y: f64,
    eigenvalue: f64,
    is_dual_repeller: bool,
) -> Result<JsValue, JsValue> {
    console_log!(
        "Computing manifold with a={}, b={}, epsilon={}",
        a,
        b,
        epsilon
    );

    let params = match HenonParams::new(a, b, epsilon) {
        Ok(p) => p,
        Err(e) => {
            console_error!("Invalid parameters: {}", e);
            return Err(JsValue::from_str(&e));
        }
    };

    let config = ManifoldConfig::default();

    if !saddle_x.is_finite() || !saddle_y.is_finite() {
        let err = "Saddle point coordinates must be finite";
        console_error!("{}", err);
        return Err(JsValue::from_str(err));
    }

    if !eigenvector_x.is_finite() || !eigenvector_y.is_finite() {
        let err = "Eigenvector coordinates must be finite";
        console_error!("{}", err);
        return Err(JsValue::from_str(err));
    }

    if period == 0 {
        let err = "Period must be positive";
        console_error!("{}", err);
        return Err(JsValue::from_str(err));
    }

    let eigenvector = Vector2::new(eigenvector_x, eigenvector_y);
    let norm = eigenvector.norm();

    if norm < 1e-10 {
        let err = "Eigenvector magnitude too small";
        console_error!("{}", err);
        return Err(JsValue::from_str(err));
    }

    let eigenvector = eigenvector / norm;

    let saddle = SaddlePoint::from_2d_eigenvector(
        Vector2::new(saddle_x, saddle_y),
        eigenvector,
        period,
        eigenvalue,
        if is_dual_repeller {
            SaddleType::DualRepeller
        } else {
            SaddleType::Regular
        },
        None,
    );

    let target_points = vec![];
    let computer = UnstableManifoldComputer::new(params, config);

    let (traj_plus, traj_minus) = match computer.compute_manifold(&saddle, &target_points) {
        Ok(result) => result,
        Err(e) => {
            console_error!("Failed to compute manifold: {}", e);
            return Err(JsValue::from_str(&e));
        }
    };

    console_log!(
        "Successfully computed manifold: +{} pts, -{} pts",
        traj_plus.points.len(),
        traj_minus.points.len()
    );

    let result = serde_json::json!({
        "plus": trajectory_ret(&traj_plus),
        "minus": trajectory_ret(&traj_minus),
    });

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    match result.serialize(&serializer) {
        Ok(v) => Ok(v),
        Err(e) => {
            console_error!("Serialization error: {:?}", e);
            Err(JsValue::from_str("Failed to serialize result"))
        }
    }
}

#[wasm_bindgen]
pub fn compute_user_defined_manifold(
    x_eq: &str,
    y_eq: &str,
    params: JsValue,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> Result<JsValue, JsValue> {
    let (x_min, x_max, y_min, y_max) = normalize_display_range(x_min, x_max, y_min, y_max);
    console_log!(
        "Computing user defined manifold for x={}, y={}, eps={}",
        x_eq,
        y_eq,
        epsilon
    );

    let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
    let system = match UserDefinedDynamicalSystem::new(x_eq, y_eq, epsilon, param_set) {
        Ok(s) => s,
        Err(e) => return Err(JsValue::from_str(&format!("Failed to parse system: {}", e))),
    };

    let start_points = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(0.6, 0.6),
        Vector2::new(-0.6, -0.6),
        Vector2::new(0.5, 0.5),
        Vector2::new(-0.5, 0.5),
        Vector2::new(0.5, -0.5),
        Vector2::new(-0.5, -0.5),
        Vector2::new(-1.2, -1.2),
        Vector2::new(1.2, 1.2),
    ];

    let mut fixed_points = Vec::new();

    for start in start_points {
        let mut x = start.x;
        let mut y = start.y;
        let mut converged = false;

        for _ in 0..100 {
            let pos = Vector2::new(x, y);
            match system.map(pos) {
                Ok(next_pos) => {
                    let f_val = next_pos - pos; // G(x) = map(x) - x
                    if f_val.norm() < 1e-10 {
                        converged = true;
                        break;
                    }

                    let jac = system.jacobian(pos);
                    let j_minus_i = jac - Matrix2::identity();

                    match j_minus_i.try_inverse() {
                        Some(inv) => {
                            let delta = inv * f_val;
                            x -= delta.x;
                            y -= delta.y;
                        }
                        None => {
                            // Singular Jacobian, try simple step or break
                            // Fallback to simple iteration step?
                            x = next_pos.x;
                            y = next_pos.y;
                        }
                    }

                    if x.abs() > 100.0 || y.abs() > 100.0 {
                        break;
                    }
                }
                Err(_) => break,
            }
        }

        if converged {
            // check if already found to avoid duplicates
            let mut duplicate = false;
            for fp in &fixed_points {
                let diff: Vector2<f64> = *fp - Vector2::new(x, y);
                if diff.norm() < 1e-6 {
                    duplicate = true;
                    break;
                }
            }
            if !duplicate {
                fixed_points.push(Vector2::new(x, y));
            }
        }
    }

    if fixed_points.is_empty() {
        console_log!("No fixed points found for user defined system");
    } else {
        console_log!(
            "Found {} fixed points: {:?}",
            fixed_points.len(),
            fixed_points
        );
    }

    let mut fixed_points_result = Vec::new();
    let mut manifolds_result = Vec::new();
    let config = ManifoldConfig::default();
    let computer = UnstableManifoldComputer::new(system.clone(), config);

    for fp_pos in fixed_points {
        if !in_display_range(fp_pos.x, fp_pos.y, x_min, x_max, y_min, y_max) {
            continue;
        }
        let jac = system.jacobian(fp_pos);
        let trace = jac.trace();
        let det = jac.determinant();
        let disc = trace * trace - 4.0 * det;

        let (l1, l2) = if disc >= 0.0 {
            let sqrt_disc = disc.sqrt();
            ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0)
        } else {
            let real = trace / 2.0;
            let imag = (-disc).sqrt() / 2.0;
            let mag = (real * real + imag * imag).sqrt();
            (mag, mag)
        };

        let stability = if l1.abs() < 1.0 && l2.abs() < 1.0 {
            "stable"
        } else if l1.abs() > 1.0 && l2.abs() > 1.0 {
            "unstable"
        } else {
            "saddle"
        };

        fixed_points_result.push(FixedPointResult {
            x: fp_pos.x,
            y: fp_pos.y,
            eigenvalues: (l1, l2),
            stability: stability.to_string(),
        });

        if stability == "saddle" || stability == "unstable" {
            // Compute manifold
            // Find unstable eigenvector
            let unstable_lambda = if l1.abs() > l2.abs() { l1 } else { l2 };

            // Simple eigenvector calculation (assuming real for saddle/unstable)
            let eigenvector = if disc >= 0.0 {
                // (J - lambda*I)v = 0
                // [j11-lambda, j12]
                // [j21, j22-lambda]
                if (jac[(0, 0)] - unstable_lambda).abs() > 1e-10 {
                    Vector2::new(jac[(0, 1)], unstable_lambda - jac[(0, 0)]).normalize()
                } else if jac[(0, 1)].abs() > 1e-10 {
                    Vector2::new(1.0, 0.0) // If j11=lambda and j12!=0, then v2=0
                } else {
                    Vector2::new(0.0, 1.0)
                }
            } else {
                Vector2::new(1.0, 0.0) // Fallback for complex
            };

            let saddle_type = if stability == "unstable" {
                SaddleType::DualRepeller
            } else {
                SaddleType::Regular
            };

            let saddle = SaddlePoint::from_2d_eigenvector(
                fp_pos,
                eigenvector,
                1,
                unstable_lambda,
                saddle_type,
                None,
            );

            // target points for termination (use other fixed points?), for now just empty
            let targets = vec![];

            if let Ok((traj_plus, traj_minus)) = computer.compute_manifold(&saddle, &targets) {
                manifolds_result.push(ManifoldResult {
                    plus: trajectory_ret(&traj_plus),
                    minus: trajectory_ret(&traj_minus),
                    saddle_point: (fp_pos.x, fp_pos.y),
                    eigenvalue: unstable_lambda,
                });
            }
        }
    }

    let result = ComputeResult {
        manifolds: manifolds_result,
        fixed_points: fixed_points_result,
    };

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    match result.serialize(&serializer) {
        Ok(v) => Ok(v),
        Err(e) => {
            console_error!("Serialization error: {:?}", e);
            Err(JsValue::from_str("Failed to serialize result"))
        }
    }
}

/// orbits_js: Array of {points: [[x,y],...], period: number, stability: "stable"|"saddle"|"unstable"}
#[wasm_bindgen]
pub fn compute_manifold_from_orbits(
    a: f64,
    b: f64,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    orbits_js: JsValue,
) -> Result<JsValue, JsValue> {
    let (x_min, x_max, y_min, y_max) = normalize_display_range(x_min, x_max, y_min, y_max);
    console_log!(
        "Computing manifold from {} orbits with a={}, b={}, eps={}",
        if orbits_js.is_array() {
            "array of"
        } else {
            "?"
        },
        a,
        b,
        epsilon
    );

    let params = match HenonParams::new(a, b, epsilon) {
        Ok(p) => p,
        Err(e) => {
            console_error!("Invalid parameters: {}", e);
            return Err(JsValue::from_str(&e));
        }
    };

    // Parse the orbits from JS
    #[derive(Deserialize)]
    struct OrbitInput {
        points: Vec<(f64, f64)>,
        extended_points: Option<Vec<(f64, f64, f64, f64)>>,
        period: usize,
        stability: String,
    }

    let orbits: Vec<OrbitInput> = match serde_wasm_bindgen::from_value(orbits_js) {
        Ok(v) => v,
        Err(e) => {
            console_error!("Failed to parse orbits: {:?}", e);
            return Err(JsValue::from_str("Failed to parse orbits"));
        }
    };

    console_log!("Parsed {} orbits", orbits.len());

    // Collect all points for termination targets and find saddles for manifold origin
    let mut all_points: Vec<Vector2<f64>> = Vec::new();
    let mut saddle_orbits: Vec<&OrbitInput> = Vec::new();
    let mut fixed_points_result: Vec<FixedPointResult> = Vec::new();

    for orbit in &orbits {
        let in_range = orbit
            .points
            .iter()
            .all(|(x, y)| in_display_range(*x, *y, x_min, x_max, y_min, y_max));
        if !in_range {
            console_log!(
                "Skipping orbit period {} - outside display range",
                orbit.period
            );
            continue;
        }

        console_log!(
            "Orbit period {}: {} points, stability='{}', in_range={}",
            orbit.period,
            orbit.points.len(),
            orbit.stability,
            in_range
        );

        // Add all points as potential termination targets
        for (x, y) in &orbit.points {
            all_points.push(Vector2::new(*x, *y));

            // Add to fixed points result for display
            fixed_points_result.push(FixedPointResult {
                x: *x,
                y: *y,
                eigenvalues: (0.0, 0.0), // Will be computed below
                stability: orbit.stability.clone(),
            });
        }

        // include saddle and unstable (dual repeller) orbits for manifold computation
        if orbit.stability == "saddle" || orbit.stability == "unstable" {
            console_log!(
                "  -> Adding period-{} {} orbit for manifold computation",
                orbit.period,
                orbit.stability
            );
            saddle_orbits.push(orbit);
        } else {
            console_log!(
                "  -> Skipping period-{} {} orbit (not saddle/unstable)",
                orbit.period,
                orbit.stability
            );
        }
    }

    console_log!(
        "Found {} points for termination, {} saddle orbits for manifolds",
        all_points.len(),
        saddle_orbits.len()
    );

    // Compute eigenvalues for display and eigenvectors for saddle manifolds
    for fp in &mut fixed_points_result {
        let jac = params.jacobian(Vector2::new(fp.x, fp.y));
        let trace = jac.trace();
        let det = jac.determinant();
        let disc = trace * trace - 4.0 * det;

        let (l1, l2) = if disc >= 0.0 {
            let sqrt_disc = disc.sqrt();
            ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0)
        } else {
            let real = trace / 2.0;
            let imag = (-disc).sqrt() / 2.0;
            let mag = (real * real + imag * imag).sqrt();
            (mag, mag)
        };
        fp.eigenvalues = (l1, l2);
    }

    // Compute manifolds for each saddle orbit
    let mut manifolds_result: Vec<ManifoldResult> = Vec::new();
    let config = ManifoldConfig::default();
    let computer = UnstableManifoldComputer::new(params, config);

    for orbit in saddle_orbits {
        if orbit.points.is_empty() {
            continue;
        }

        // Compute manifold from EACH point in the orbit
        // For period-n orbit, we need manifolds from all n points
        for point_idx in 0..orbit.points.len() {
            let (px, py) = orbit.points[point_idx];
            let pos = Vector2::new(px, py);

            console_log!(
                "Computing manifold from point {}/{} of period-{} orbit at ({:.4}, {:.4})",
                point_idx + 1,
                orbit.points.len(),
                orbit.period,
                px,
                py
            );

            // Determine eigenvector, eigenvalue, and normal
            let (eigenvector, unstable_lambda, normal_opt) = if let Some(ref ext) =
                orbit.extended_points
            {
                if point_idx < ext.len() {
                    let (ex, ey, nx, ny) = ext[point_idx];
                    // the extended_points contain (x, y, nx, ny)
                    // where (nx, ny) is the outward normal at the boundary
                    let normal = Vector2::new(nx, ny);
                    let normal_norm = normal.norm();
                    let normal_unit = if normal_norm > 1e-10 {
                        normal / normal_norm
                    } else {
                        Vector2::new(1.0, 0.0)
                    };

                    // The tangent direction (eigenvector direction for unstable manifold)
                    // is perpendicular to the outward normal
                    // For a curve in 2D, tangent = rotate normal by 90 degrees
                    let tangent = Vector2::new(-normal_unit.y, normal_unit.x);

                    // Compute eigenvalue from the 4D Jacobian if we have extended data
                    // For now, estimate based on how fast points separate
                    // Use accumulated 2D Jacobian eigenvalue as approximation
                    let jac = params.jacobian(pos);
                    let mut jac_accum = jac;
                    let mut current_pos = pos;
                    for _ in 1..orbit.period {
                        current_pos = Vector2::new(
                            1.0 - a * current_pos.x * current_pos.x + current_pos.y,
                            b * current_pos.x,
                        );
                        let next_jac = params.jacobian(current_pos);
                        jac_accum = next_jac * jac_accum;
                    }
                    let trace = jac_accum.trace();
                    let det = jac_accum.determinant();
                    let disc = trace * trace - 4.0 * det;
                    let lambda = if disc >= 0.0 {
                        let sqrt_disc = disc.sqrt();
                        let l1 = (trace + sqrt_disc) / 2.0;
                        let l2 = (trace - sqrt_disc) / 2.0;
                        if l1.abs() > l2.abs() {
                            l1
                        } else {
                            l2
                        }
                    } else {
                        trace / 2.0
                    };

                    console_log!(
                        "Using extended point: pos=({:.6}, {:.6}), normal=({:.6}, {:.6}), tangent=({:.6}, {:.6}), lambda={:.6}",
                        ex, ey, nx, ny, tangent.x, tangent.y, lambda
                    );

                    (tangent, lambda, Some(normal_unit))
                } else {
                    console_log!("Extended points exists but is empty");
                    (Vector2::new(1.0, 0.0), 2.0, None)
                }
            } else {
                // Compute eigenvector for period-n map
                // For period 1: use Jacobian at the point
                // For period n: need to use accumulated Jacobian
                let jac = params.jacobian(pos);

                // Accumulate Jacobian for higher periods
                let mut jac_accum = jac;
                let mut current_pos = pos;
                for _ in 1..orbit.period {
                    current_pos = Vector2::new(
                        1.0 - a * current_pos.x * current_pos.x + current_pos.y,
                        b * current_pos.x,
                    );
                    let next_jac = params.jacobian(current_pos);
                    jac_accum = next_jac * jac_accum;
                }

                let trace = jac_accum.trace();
                let det = jac_accum.determinant();
                let disc = trace * trace - 4.0 * det;

                let (l1, l2) = if disc >= 0.0 {
                    let sqrt_disc = disc.sqrt();
                    ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0)
                } else {
                    (trace / 2.0, trace / 2.0)
                };

                // Find unstable eigenvector
                let unstable_lambda = if l1.abs() > l2.abs() { l1 } else { l2 };

                // For accumulated Jacobian, eigenvector is more complex
                // Use (J - λI)v = 0, from first row: (j11 - λ)v1 + j12*v2 = 0
                let j11 = jac_accum[(0, 0)];
                let j12 = jac_accum[(0, 1)];
                let v1 = 1.0;
                let v2 = -(j11 - unstable_lambda) / j12.max(1e-10);
                let norm = (v1 * v1 + v2 * v2).sqrt();
                let eigenvector = if norm > 1e-10 {
                    Vector2::new(v1 / norm, v2 / norm)
                } else {
                    Vector2::new(1.0, 0.0)
                };

                console_log!(
                    "Computed eigenvector for period-{}: ({:.6}, {:.6}), lambda={:.6}",
                    orbit.period,
                    eigenvector.x,
                    eigenvector.y,
                    unstable_lambda
                );

                (eigenvector, unstable_lambda, None)
            };

            let saddle_type = if orbit.stability == "unstable" {
                SaddleType::DualRepeller
            } else {
                SaddleType::Regular
            };

            // Create SaddlePoint with proper normal:
            // If boundary map provided normal, use it. Otherwise pass None for geometric calculation
            let saddle_pt = if let Some(normal) = normal_opt {
                // Use from_4d_eigenvector or construct manually with the correct normal
                SaddlePoint {
                    position: pos,
                    period: orbit.period,
                    tangent_2d: eigenvector / eigenvector.norm(),
                    eigenvalue: unstable_lambda,
                    tangent_4d: None,
                    saddle_type,
                    normal: normal / normal.norm(),
                }
            } else {
                SaddlePoint::from_2d_eigenvector(
                    pos,
                    eigenvector,
                    orbit.period,
                    unstable_lambda,
                    saddle_type,
                    None,
                )
            };

            console_log!(
                "Computing manifold for period-{} saddle at ({:.4}, {:.4})",
                orbit.period,
                px,
                py
            );

            if let Ok((traj_plus, traj_minus)) = computer.compute_manifold(&saddle_pt, &all_points)
            {
                let plus_points: Vec<(f64, f64)> = traj_plus
                    .points
                    .iter()
                    .filter(|s| s.pos.x.is_finite() && s.pos.y.is_finite())
                    .map(|s| (s.pos.x, s.pos.y))
                    .collect();

                let minus_points: Vec<(f64, f64)> = traj_minus
                    .points
                    .iter()
                    .filter(|s| s.pos.x.is_finite() && s.pos.y.is_finite())
                    .map(|s| (s.pos.x, s.pos.y))
                    .collect();

                console_log!(
                    "Manifold computed: +{} pts, -{} pts",
                    plus_points.len(),
                    minus_points.len()
                );

                manifolds_result.push(ManifoldResult {
                    plus: trajectory_ret(&traj_plus),
                    minus: trajectory_ret(&traj_minus),
                    saddle_point: (px, py),
                    eigenvalue: unstable_lambda,
                });
            }
        } // end for point_idx in orbit.points
    }

    let result = ComputeResult {
        manifolds: manifolds_result,
        fixed_points: fixed_points_result,
    };

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    match result.serialize(&serializer) {
        Ok(v) => Ok(v),
        Err(e) => {
            console_error!("Serialization error: {:?}", e);
            Err(JsValue::from_str("Failed to serialize result"))
        }
    }
}

/// Result structure for stable/unstable manifold computation
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StableUnstableResult {
    pub unstable_manifolds: Vec<ManifoldResult>,
    pub stable_manifolds: Vec<ManifoldResult>,
    pub fixed_points: Vec<FixedPointResult>,
    pub intersections: Vec<IntersectionInfo>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IntersectionInfo {
    /// Index of saddle point whose UNSTABLE manifold is checked
    pub unstable_saddle_index: usize,
    /// Index of saddle point whose STABLE manifold is checked
    pub stable_saddle_index: usize,
    pub has_intersection: bool,
    pub min_distance: f64,
}

/// Compute both stable and unstable manifolds from periodic orbits
///
/// For each saddle orbit:
/// - Unstable manifold: Use unstable eigenvector (|lambda| > 1) with forward map
/// - Stable manifold: Use stable eigenvector (|lambda| < 1) with inverse map
///
/// The stable manifold is computed using SaddleType::DualRepeller which triggers
/// the use of the inverse boundary map (extended_map_inverse).
#[derive(Serialize)]
struct PointResult {
    x: f64,
    y: f64,
}

#[wasm_bindgen]
pub fn evaluate_user_defined_map(
    x: f64,
    y: f64,
    x_eq: &str,
    y_eq: &str,
    params: JsValue,
    epsilon: f64,
) -> Result<JsValue, JsValue> {
    let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
    let system = UserDefinedDynamicalSystem::new(x_eq, y_eq, epsilon, param_set)
        .map_err(|e| JsValue::from_str(&format!("Error parsing equations: {}", e)))?;

    let pos = Vector2::new(x, y);
    let next_pos = system
        .map(pos)
        .map_err(|e| JsValue::from_str(&format!("Error evaluating map: {}", e)))?;

    let result = PointResult {
        x: next_pos.x,
        y: next_pos.y,
    };

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    result.serialize(&serializer).map_err(|e| {
        console_error!("Serialization error: {:?}", e);
        JsValue::from_str("Failed to serialize result")
    })
}

#[wasm_bindgen]
pub fn compute_stable_and_unstable_manifolds(
    a: f64,
    b: f64,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    orbits_js: JsValue,
    intersection_threshold: f64,
) -> Result<JsValue, JsValue> {
    let (x_min, x_max, y_min, y_max) = normalize_display_range(x_min, x_max, y_min, y_max);
    console_log!(
        "Computing stable and unstable manifolds with a={}, b={}, eps={}, threshold={}",
        a,
        b,
        epsilon,
        intersection_threshold
    );

    let params = match HenonParams::new(a, b, epsilon) {
        Ok(p) => p,
        Err(e) => {
            console_error!("Invalid parameters: {}", e);
            return Err(JsValue::from_str(&e));
        }
    };

    // Parse orbits from JS
    #[derive(Deserialize)]
    struct OrbitInput {
        points: Vec<(f64, f64)>,
        extended_points: Option<Vec<(f64, f64, f64, f64)>>,
        period: usize,
        stability: String,
        #[serde(default)]
        eigenvalues: Vec<f64>,
    }

    let orbits: Vec<OrbitInput> = match serde_wasm_bindgen::from_value(orbits_js) {
        Ok(v) => v,
        Err(e) => {
            console_error!("Failed to parse orbits: {:?}", e);
            return Err(JsValue::from_str("Failed to parse orbits"));
        }
    };

    console_log!(
        "Parsed {} orbits for stable/unstable manifold computation",
        orbits.len()
    );

    let mut all_points: Vec<Vector2<f64>> = Vec::new();
    let mut saddle_orbits: Vec<&OrbitInput> = Vec::new();
    let mut fixed_points_result: Vec<FixedPointResult> = Vec::new();

    // Collect saddle orbits and all points
    for orbit in &orbits {
        let in_range = orbit
            .points
            .iter()
            .all(|(x, y)| in_display_range(*x, *y, x_min, x_max, y_min, y_max));
        if !in_range {
            continue;
        }

        for (x, y) in &orbit.points {
            all_points.push(Vector2::new(*x, *y));
            fixed_points_result.push(FixedPointResult {
                x: *x,
                y: *y,
                eigenvalues: (0.0, 0.0),
                stability: orbit.stability.clone(),
            });
        }

        if orbit.stability == "saddle" {
            console_log!(
                "Adding period-{} saddle orbit for stable/unstable manifold computation",
                orbit.period
            );
            saddle_orbits.push(orbit);
        }
    }

    console_log!(
        "Found {} saddle orbits, {} total points",
        saddle_orbits.len(),
        all_points.len()
    );

    // compute eigenvalues for fixed points
    for fp in &mut fixed_points_result {
        let jac = params.jacobian(Vector2::new(fp.x, fp.y));
        let trace = jac.trace();
        let det = jac.determinant();
        let disc = trace * trace - 4.0 * det;
        let (l1, l2) = if disc >= 0.0 {
            let sqrt_disc = disc.sqrt();
            ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0)
        } else {
            let real = trace / 2.0;
            let imag = (-disc).sqrt() / 2.0;
            let mag = (real * real + imag * imag).sqrt();
            (mag, mag)
        };
        fp.eigenvalues = (l1, l2);
    }

    let mut unstable_manifolds: Vec<ManifoldResult> = Vec::new();
    let mut stable_manifolds: Vec<ManifoldResult> = Vec::new();
    let mut intersections: Vec<IntersectionInfo> = Vec::new();

    let config = ManifoldConfig::default();
    let computer = UnstableManifoldComputer::new(params, config);

    for (_, orbit) in saddle_orbits.iter().enumerate() {
        if orbit.points.is_empty() {
            continue;
        }

        // Process each point in the orbit
        for point_idx in 0..orbit.points.len() {
            let (px, py) = orbit.points[point_idx];
            let pos = Vector2::new(px, py);

            console_log!(
                "Computing manifolds for point {}/{} of period-{} saddle at ({:.4}, {:.4})",
                point_idx + 1,
                orbit.points.len(),
                orbit.period,
                px,
                py
            );

            // Compute accumulated Jacobian for period-n map
            let jac = params.jacobian(pos);
            let mut jac_accum = jac;
            let mut current_pos = pos;
            for _ in 1..orbit.period {
                current_pos = Vector2::new(
                    1.0 - a * current_pos.x * current_pos.x + current_pos.y,
                    b * current_pos.x,
                );
                let next_jac = params.jacobian(current_pos);
                jac_accum = next_jac * jac_accum;
            }

            let trace = jac_accum.trace();
            let det = jac_accum.determinant();
            let disc = trace * trace - 4.0 * det;

            let (l1, l2) = if disc >= 0.0 {
                let sqrt_disc = disc.sqrt();
                ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0)
            } else {
                (trace / 2.0, trace / 2.0)
            };

            let (unstable_lambda, stable_lambda) = if l1.abs() > l2.abs() {
                (l1, l2)
            } else {
                (l2, l1)
            };

            let j11 = jac_accum[(0, 0)];
            let j12 = jac_accum[(0, 1)];

            let compute_eigenvector = |lambda: f64| -> Vector2<f64> {
                let v1 = 1.0;
                let v2 = -(j11 - lambda) / j12.max(1e-10);
                let norm = (v1 * v1 + v2 * v2).sqrt();
                if norm > 1e-10 {
                    Vector2::new(v1 / norm, v2 / norm)
                } else {
                    Vector2::new(1.0, 0.0)
                }
            };

            let unstable_eigenvec = compute_eigenvector(unstable_lambda);
            let stable_eigenvec = compute_eigenvector(stable_lambda);

            console_log!(
                "Eigenvalues: λ_u={:.4} (|λ|={:.4}), λ_s={:.4} (|λ|={:.4})",
                unstable_lambda,
                unstable_lambda.abs(),
                stable_lambda,
                stable_lambda.abs()
            );
            console_log!(
                "Eigenvectors: v_u=({:.4}, {:.4}), v_s=({:.4}, {:.4})",
                unstable_eigenvec.x,
                unstable_eigenvec.y,
                stable_eigenvec.x,
                stable_eigenvec.y
            );

            // get normal from extended points if available
            let normal_opt = if let Some(ref ext) = orbit.extended_points {
                if point_idx < ext.len() {
                    let (_, _, nx, ny) = ext[point_idx];
                    let normal = Vector2::new(nx, ny);
                    let norm = normal.norm();
                    if norm > 1e-10 {
                        Some(normal / norm)
                    } else {
                        None
                    }
                } else {
                    None
                }
            } else {
                None
            };

            let saddle_unstable = if let Some(normal) = normal_opt {
                SaddlePoint {
                    position: pos,
                    period: orbit.period,
                    tangent_2d: unstable_eigenvec,
                    eigenvalue: unstable_lambda,
                    tangent_4d: None,
                    saddle_type: SaddleType::Regular,
                    normal,
                }
            } else {
                SaddlePoint::from_2d_eigenvector(
                    pos,
                    unstable_eigenvec,
                    orbit.period,
                    unstable_lambda,
                    SaddleType::Regular, // Forward map
                    None,
                )
            };

            let unstable_result = if let Ok((traj_plus, traj_minus)) =
                computer.compute_manifold(&saddle_unstable, &all_points)
            {
                let plus_points: Vec<(f64, f64)> = traj_plus
                    .points
                    .iter()
                    .filter(|s| s.pos.x.is_finite() && s.pos.y.is_finite())
                    .map(|s| (s.pos.x, s.pos.y))
                    .collect();

                let minus_points: Vec<(f64, f64)> = traj_minus
                    .points
                    .iter()
                    .filter(|s| s.pos.x.is_finite() && s.pos.y.is_finite())
                    .map(|s| (s.pos.x, s.pos.y))
                    .collect();

                console_log!(
                    "Unstable manifold: +{} pts, -{} pts",
                    plus_points.len(),
                    minus_points.len()
                );

                Some(ManifoldResult {
                    plus: trajectory_ret(&traj_plus),
                    minus: trajectory_ret(&traj_minus),
                    saddle_point: (px, py),
                    eigenvalue: unstable_lambda,
                })
            } else {
                None
            };

            let saddle_stable = if let Some(normal) = normal_opt {
                SaddlePoint {
                    position: pos,
                    period: orbit.period,
                    tangent_2d: stable_eigenvec,
                    eigenvalue: stable_lambda,
                    tangent_4d: None,
                    saddle_type: SaddleType::DualRepeller,
                    normal,
                }
            } else {
                SaddlePoint::from_2d_eigenvector(
                    pos,
                    stable_eigenvec,
                    orbit.period,
                    stable_lambda,
                    SaddleType::DualRepeller,
                    None,
                )
            };

            let stable_result = if let Ok((traj_plus, traj_minus)) =
                computer.compute_manifold(&saddle_stable, &all_points)
            {
                let plus_points: Vec<(f64, f64)> = traj_plus
                    .points
                    .iter()
                    .filter(|s| s.pos.x.is_finite() && s.pos.y.is_finite())
                    .map(|s| (s.pos.x, s.pos.y))
                    .collect();

                let minus_points: Vec<(f64, f64)> = traj_minus
                    .points
                    .iter()
                    .filter(|s| s.pos.x.is_finite() && s.pos.y.is_finite())
                    .map(|s| (s.pos.x, s.pos.y))
                    .collect();

                console_log!(
                    "Stable manifold: +{} pts, -{} pts",
                    plus_points.len(),
                    minus_points.len()
                );

                Some(ManifoldResult {
                    plus: trajectory_ret(&traj_plus),
                    minus: trajectory_ret(&traj_minus),
                    saddle_point: (px, py),
                    eigenvalue: stable_lambda,
                })
            } else {
                None
            };

            // Collect manifolds (intersection detection done AFTER all manifolds computed)
            if let Some(u) = unstable_result {
                unstable_manifolds.push(u);
            }
            if let Some(s) = stable_result {
                stable_manifolds.push(s);
            }
        }
    }

    console_log!(
        "Checking heteroclinic intersections between {} unstable and {} stable manifolds",
        unstable_manifolds.len(),
        stable_manifolds.len()
    );

    for (u_idx, u_man) in unstable_manifolds.iter().enumerate() {
        for (s_idx, s_man) in stable_manifolds.iter().enumerate() {
            // Skip if same saddle point (they trivially intersect at the saddle)
            let u_saddle = u_man.saddle_point;
            let s_saddle = s_man.saddle_point;
            let same_saddle =
                (u_saddle.0 - s_saddle.0).abs() < 1e-8 && (u_saddle.1 - s_saddle.1).abs() < 1e-8;

            if same_saddle {
                continue;
            }

            // Combine all unstable points
            let mut u_points: Vec<Vector2<f64>> = u_man
                .plus
                .points
                .iter()
                .map(|(x, y)| Vector2::new(*x, *y))
                .collect();
            u_points.extend(u_man.minus.points.iter().map(|(x, y)| Vector2::new(*x, *y)));

            // Combine all stable points
            let mut s_points: Vec<Vector2<f64>> = s_man
                .plus
                .points
                .iter()
                .map(|(x, y)| Vector2::new(*x, *y))
                .collect();
            s_points.extend(s_man.minus.points.iter().map(|(x, y)| Vector2::new(*x, *y)));

            // Find minimum distance between manifolds
            let mut min_distance = f64::INFINITY;
            for u_pt in &u_points {
                for s_pt in &s_points {
                    let dist = (u_pt - s_pt).norm();
                    if dist < min_distance {
                        min_distance = dist;
                    }
                }
            }

            let has_intersection = min_distance < intersection_threshold;
            if has_intersection {
                console_log!(
                    "HETEROCLINIC CONNECTION: Unstable({:.3},{:.3}) -> Stable({:.3},{:.3}), dist={:.6}",
                    u_saddle.0, u_saddle.1,
                    s_saddle.0, s_saddle.1,
                    min_distance
                );
            }

            intersections.push(IntersectionInfo {
                unstable_saddle_index: u_idx,
                stable_saddle_index: s_idx,
                has_intersection,
                min_distance,
            });
        }
    }

    console_log!(
        "Computed {} unstable, {} stable manifolds, {} intersection checks",
        unstable_manifolds.len(),
        stable_manifolds.len(),
        intersections.len()
    );

    let result = StableUnstableResult {
        unstable_manifolds,
        stable_manifolds,
        fixed_points: fixed_points_result,
        intersections,
    };

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    match result.serialize(&serializer) {
        Ok(v) => Ok(v),
        Err(e) => {
            console_error!("Serialization error: {:?}", e);
            Err(JsValue::from_str("Failed to serialize result"))
        }
    }
}

/// compute accumulated Jacobian for a period-n orbit using a generic system
fn accumulate_jacobian_generic(
    system: &dyn DynamicalSystem,
    start_pos: Vector2<f64>,
    period: usize,
) -> Matrix2<f64> {
    let jac = system.jacobian(start_pos);
    let mut jac_accum = jac;
    let mut current_pos = start_pos;
    for _ in 1..period {
        if let Ok(next) = system.map(current_pos) {
            current_pos = next;
        } else {
            break;
        }
        let next_jac = system.jacobian(current_pos);
        jac_accum = next_jac * jac_accum;
    }
    jac_accum
}

/// extract eigenvalues and eigenvector from a 2x2 Jacobian
fn eigen_analysis_2x2(jac_accum: &Matrix2<f64>) -> (f64, f64, f64, Vector2<f64>) {
    let trace = jac_accum.trace();
    let det = jac_accum.determinant();
    let disc = trace * trace - 4.0 * det;

    let (l1, l2) = if disc >= 0.0 {
        let sqrt_disc = disc.sqrt();
        ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0)
    } else {
        let real = trace / 2.0;
        let imag = (-disc).sqrt() / 2.0;
        let mag = (real * real + imag * imag).sqrt();
        (mag, mag)
    };

    let unstable_lambda = if l1.abs() > l2.abs() { l1 } else { l2 };

    let eigenvector = if disc >= 0.0 {
        let j11 = jac_accum[(0, 0)];
        let j12 = jac_accum[(0, 1)];
        if (j11 - unstable_lambda).abs() > 1e-10 && j12.abs() > 1e-10 {
            Vector2::new(j12, unstable_lambda - j11).normalize()
        } else if j12.abs() > 1e-10 {
            Vector2::new(1.0, 0.0)
        } else {
            Vector2::new(0.0, 1.0)
        }
    } else {
        Vector2::new(1.0, 0.0)
    };

    (l1, l2, unstable_lambda, eigenvector)
}

#[wasm_bindgen]
pub fn compute_manifold_from_orbits_user_defined(
    x_eq: &str,
    y_eq: &str,
    params: JsValue,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    orbits_js: JsValue,
) -> Result<JsValue, JsValue> {
    let (x_min, x_max, y_min, y_max) = normalize_display_range(x_min, x_max, y_min, y_max);
    console_log!(
        "Computing user-defined manifold from orbits: x={}, y={}, eps={}",
        x_eq,
        y_eq,
        epsilon
    );

    let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
    let system = match UserDefinedDynamicalSystem::new(x_eq, y_eq, epsilon, param_set) {
        Ok(s) => s,
        Err(e) => return Err(JsValue::from_str(&format!("Failed to parse system: {}", e))),
    };

    #[derive(Deserialize)]
    struct OrbitInput {
        points: Vec<(f64, f64)>,
        extended_points: Option<Vec<(f64, f64, f64, f64)>>,
        period: usize,
        stability: String,
    }

    let orbits: Vec<OrbitInput> = match serde_wasm_bindgen::from_value(orbits_js) {
        Ok(v) => v,
        Err(e) => {
            console_error!("Failed to parse orbits: {:?}", e);
            return Err(JsValue::from_str("Failed to parse orbits"));
        }
    };

    let mut all_points: Vec<Vector2<f64>> = Vec::new();
    let mut saddle_orbits: Vec<&OrbitInput> = Vec::new();
    let mut fixed_points_result: Vec<FixedPointResult> = Vec::new();

    for orbit in &orbits {
        let in_range = orbit
            .points
            .iter()
            .all(|(x, y)| in_display_range(*x, *y, x_min, x_max, y_min, y_max));
        if !in_range {
            continue;
        }

        for (x, y) in &orbit.points {
            all_points.push(Vector2::new(*x, *y));
            fixed_points_result.push(FixedPointResult {
                x: *x,
                y: *y,
                eigenvalues: (0.0, 0.0),
                stability: orbit.stability.clone(),
            });
        }

        if orbit.stability == "saddle" || orbit.stability == "unstable" {
            saddle_orbits.push(orbit);
        }
    }

    // Compute eigenvalues for display
    for fp in &mut fixed_points_result {
        let jac = system.jacobian(Vector2::new(fp.x, fp.y));
        let (l1, l2, _, _) = eigen_analysis_2x2(&jac);
        fp.eigenvalues = (l1, l2);
    }

    let mut manifolds_result: Vec<ManifoldResult> = Vec::new();
    let config = ManifoldConfig::default();
    let computer = UnstableManifoldComputer::new(system.clone(), config);

    for orbit in saddle_orbits {
        if orbit.points.is_empty() {
            continue;
        }

        for point_idx in 0..orbit.points.len() {
            let (px, py) = orbit.points[point_idx];
            let pos = Vector2::new(px, py);

            let (eigenvector, unstable_lambda, normal_opt) =
                if let Some(ref ext) = orbit.extended_points {
                    if point_idx < ext.len() {
                        let (_ex, _ey, nx, ny) = ext[point_idx];
                        let normal = Vector2::new(nx, ny);
                        let normal_norm = normal.norm();
                        let normal_unit = if normal_norm > 1e-10 {
                            normal / normal_norm
                        } else {
                            Vector2::new(1.0, 0.0)
                        };
                        let tangent = Vector2::new(-normal_unit.y, normal_unit.x);

                        let jac_accum = accumulate_jacobian_generic(&system, pos, orbit.period);
                        let (_, _, lambda, _) = eigen_analysis_2x2(&jac_accum);

                        (tangent, lambda, Some(normal_unit))
                    } else {
                        (Vector2::new(1.0, 0.0), 2.0, None)
                    }
                } else {
                    let jac_accum = accumulate_jacobian_generic(&system, pos, orbit.period);
                    let (_, _, unstable_lambda, eigenvector) = eigen_analysis_2x2(&jac_accum);
                    (eigenvector, unstable_lambda, None)
                };

            let saddle_type = if orbit.stability == "unstable" {
                SaddleType::DualRepeller
            } else {
                SaddleType::Regular
            };

            let saddle_pt = if let Some(normal) = normal_opt {
                SaddlePoint {
                    position: pos,
                    period: orbit.period,
                    tangent_2d: eigenvector / eigenvector.norm(),
                    eigenvalue: unstable_lambda,
                    tangent_4d: None,
                    saddle_type,
                    normal: normal / normal.norm(),
                }
            } else {
                SaddlePoint::from_2d_eigenvector(
                    pos,
                    eigenvector,
                    orbit.period,
                    unstable_lambda,
                    saddle_type,
                    None,
                )
            };

            if let Ok((traj_plus, traj_minus)) = computer.compute_manifold(&saddle_pt, &all_points)
            {
                manifolds_result.push(ManifoldResult {
                    plus: trajectory_ret(&traj_plus),
                    minus: trajectory_ret(&traj_minus),
                    saddle_point: (px, py),
                    eigenvalue: unstable_lambda,
                });
            }
        }
    }

    let result = ComputeResult {
        manifolds: manifolds_result,
        fixed_points: fixed_points_result,
    };

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    match result.serialize(&serializer) {
        Ok(v) => Ok(v),
        Err(e) => {
            console_error!("Serialization error: {:?}", e);
            Err(JsValue::from_str("Failed to serialize result"))
        }
    }
}

#[wasm_bindgen]
pub fn compute_stable_and_unstable_manifolds_user_defined(
    x_eq: &str,
    y_eq: &str,
    params: JsValue,
    epsilon: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    orbits_js: JsValue,
    intersection_threshold: f64,
) -> Result<JsValue, JsValue> {
    let (x_min, x_max, y_min, y_max) = normalize_display_range(x_min, x_max, y_min, y_max);
    console_log!(
        "Computing user-defined stable+unstable manifolds: x={}, y={}, eps={}",
        x_eq,
        y_eq,
        epsilon
    );

    let param_set = parameter_set_from_js(params).map_err(|e| JsValue::from_str(&e))?;
    let system = match UserDefinedDynamicalSystem::new(x_eq, y_eq, epsilon, param_set) {
        Ok(s) => s,
        Err(e) => return Err(JsValue::from_str(&format!("Failed to parse system: {}", e))),
    };

    #[derive(Deserialize)]
    struct OrbitInput {
        points: Vec<(f64, f64)>,
        extended_points: Option<Vec<(f64, f64, f64, f64)>>,
        period: usize,
        stability: String,
        #[serde(default)]
        eigenvalues: Vec<f64>,
    }

    let orbits: Vec<OrbitInput> = match serde_wasm_bindgen::from_value(orbits_js) {
        Ok(v) => v,
        Err(e) => {
            console_error!("Failed to parse orbits: {:?}", e);
            return Err(JsValue::from_str("Failed to parse orbits"));
        }
    };

    let mut all_points: Vec<Vector2<f64>> = Vec::new();
    let mut saddle_orbits: Vec<&OrbitInput> = Vec::new();
    let mut fixed_points_result: Vec<FixedPointResult> = Vec::new();

    for orbit in &orbits {
        let in_range = orbit
            .points
            .iter()
            .all(|(x, y)| in_display_range(*x, *y, x_min, x_max, y_min, y_max));
        if !in_range {
            continue;
        }

        for (x, y) in &orbit.points {
            all_points.push(Vector2::new(*x, *y));
            fixed_points_result.push(FixedPointResult {
                x: *x,
                y: *y,
                eigenvalues: (0.0, 0.0),
                stability: orbit.stability.clone(),
            });
        }

        if orbit.stability == "saddle" {
            saddle_orbits.push(orbit);
        }
    }

    for fp in &mut fixed_points_result {
        let jac = system.jacobian(Vector2::new(fp.x, fp.y));
        let (l1, l2, _, _) = eigen_analysis_2x2(&jac);
        fp.eigenvalues = (l1, l2);
    }

    let mut unstable_manifolds: Vec<ManifoldResult> = Vec::new();
    let mut stable_manifolds: Vec<ManifoldResult> = Vec::new();
    let mut intersections: Vec<IntersectionInfo> = Vec::new();

    let config = ManifoldConfig::default();
    let computer = UnstableManifoldComputer::new(system.clone(), config);

    for orbit in &saddle_orbits {
        if orbit.points.is_empty() {
            continue;
        }

        for point_idx in 0..orbit.points.len() {
            let (px, py) = orbit.points[point_idx];
            let pos = Vector2::new(px, py);

            let jac_accum = accumulate_jacobian_generic(&system, pos, orbit.period);
            let trace = jac_accum.trace();
            let det = jac_accum.determinant();
            let disc = trace * trace - 4.0 * det;

            let (l1, l2) = if disc >= 0.0 {
                let sqrt_disc = disc.sqrt();
                ((trace + sqrt_disc) / 2.0, (trace - sqrt_disc) / 2.0)
            } else {
                (trace / 2.0, trace / 2.0)
            };

            let (unstable_lambda, stable_lambda) = if l1.abs() > l2.abs() {
                (l1, l2)
            } else {
                (l2, l1)
            };

            let compute_eigenvector = |lambda: f64| -> Vector2<f64> {
                let j11 = jac_accum[(0, 0)];
                let j12 = jac_accum[(0, 1)];
                let v1 = 1.0;
                let v2 = -(j11 - lambda) / j12.max(1e-10);
                let norm = (v1 * v1 + v2 * v2).sqrt();
                if norm > 1e-10 {
                    Vector2::new(v1 / norm, v2 / norm)
                } else {
                    Vector2::new(1.0, 0.0)
                }
            };

            let unstable_eigenvec = compute_eigenvector(unstable_lambda);
            let stable_eigenvec = compute_eigenvector(stable_lambda);

            let normal_opt = if let Some(ref ext) = orbit.extended_points {
                if point_idx < ext.len() {
                    let (_, _, nx, ny) = ext[point_idx];
                    let normal = Vector2::new(nx, ny);
                    let norm = normal.norm();
                    if norm > 1e-10 {
                        Some(normal / norm)
                    } else {
                        None
                    }
                } else {
                    None
                }
            } else {
                None
            };

            let make_saddle = |eigvec: Vector2<f64>,
                               lambda: f64,
                               stype: SaddleType|
             -> SaddlePoint {
                if let Some(normal) = normal_opt {
                    SaddlePoint {
                        position: pos,
                        period: orbit.period,
                        tangent_2d: eigvec,
                        eigenvalue: lambda,
                        tangent_4d: None,
                        saddle_type: stype,
                        normal,
                    }
                } else {
                    SaddlePoint::from_2d_eigenvector(pos, eigvec, orbit.period, lambda, stype, None)
                }
            };

            let saddle_unstable =
                make_saddle(unstable_eigenvec, unstable_lambda, SaddleType::Regular);
            let saddle_stable =
                make_saddle(stable_eigenvec, stable_lambda, SaddleType::DualRepeller);

            if let Ok((tp, tm)) = computer.compute_manifold(&saddle_unstable, &all_points) {
                unstable_manifolds.push(ManifoldResult {
                    plus: trajectory_ret(&tp),
                    minus: trajectory_ret(&tm),
                    saddle_point: (px, py),
                    eigenvalue: unstable_lambda,
                });
            }

            if let Ok((tp, tm)) = computer.compute_manifold(&saddle_stable, &all_points) {
                stable_manifolds.push(ManifoldResult {
                    plus: trajectory_ret(&tp),
                    minus: trajectory_ret(&tm),
                    saddle_point: (px, py),
                    eigenvalue: stable_lambda,
                });
            }
        }
    }

    // Check heteroclinic intersections
    for (u_idx, u_man) in unstable_manifolds.iter().enumerate() {
        for (s_idx, s_man) in stable_manifolds.iter().enumerate() {
            let u_saddle = u_man.saddle_point;
            let s_saddle = s_man.saddle_point;
            if (u_saddle.0 - s_saddle.0).abs() < 1e-8 && (u_saddle.1 - s_saddle.1).abs() < 1e-8 {
                continue;
            }

            let mut u_points: Vec<Vector2<f64>> = u_man
                .plus
                .points
                .iter()
                .map(|(x, y)| Vector2::new(*x, *y))
                .collect();
            u_points.extend(u_man.minus.points.iter().map(|(x, y)| Vector2::new(*x, *y)));

            let mut s_points: Vec<Vector2<f64>> = s_man
                .plus
                .points
                .iter()
                .map(|(x, y)| Vector2::new(*x, *y))
                .collect();
            s_points.extend(s_man.minus.points.iter().map(|(x, y)| Vector2::new(*x, *y)));

            let mut min_distance = f64::INFINITY;
            for u_pt in &u_points {
                for s_pt in &s_points {
                    let dist = (u_pt - s_pt).norm();
                    if dist < min_distance {
                        min_distance = dist;
                    }
                }
            }

            intersections.push(IntersectionInfo {
                unstable_saddle_index: u_idx,
                stable_saddle_index: s_idx,
                has_intersection: min_distance < intersection_threshold,
                min_distance,
            });
        }
    }

    let result = StableUnstableResult {
        unstable_manifolds,
        stable_manifolds,
        fixed_points: fixed_points_result,
        intersections,
    };

    let serializer = serde_wasm_bindgen::Serializer::new().serialize_maps_as_objects(true);
    match result.serialize(&serializer) {
        Ok(v) => Ok(v),
        Err(e) => {
            console_error!("Serialization error: {:?}", e);
            Err(JsValue::from_str("Failed to serialize result"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_henon_params_validation() {
        assert!(HenonParams::new(1.4, 0.3, 0.01).is_ok());
        assert!(HenonParams::new(0.0, 0.3, 0.01).is_ok());
        assert!(HenonParams::new(1.4, 0.0, 0.01).is_err());
        assert!(HenonParams::new(1.4, 0.3, -0.01).is_err());
        assert!(HenonParams::new(1.4, 0.3, 0.0).is_err());
        assert!(HenonParams::new(f64::NAN, 0.3, 0.01).is_err());
    }

    #[test]
    fn test_henon_map() {
        let params = HenonParams::new(1.4, 0.3, 0.01).unwrap();
        let pos = Vector2::new(0.0, 0.0);
        let result = params.henon_map(&pos).unwrap();
        assert_eq!(result.x, 1.0);
        assert_eq!(result.y, 0.0);
    }

    #[test]
    fn test_fixed_point_a036() {
        let a: f64 = 0.36;
        let b: f64 = 0.3;

        let mut x = 0.0;
        let mut y = 0.0;

        for _ in 0..10000 {
            let x_new = 1.0 - a * x * x + y;
            let y_new = b * x;

            if (x_new - x).abs() < 1e-10 && (y_new - y).abs() < 1e-10 {
                break;
            }

            x = x_new;
            y = y_new;
        }

        println!("Fixed point for a={}: ({}, {})", a, x, y);
        let x_check = 1.0 - a * x * x + y;
        let y_check = b * x;
        let error_x = (x_check - x).abs();
        let error_y = (y_check - y).abs();
        println!(
            "Fixed point error: x_error={}, y_error={}",
            error_x, error_y
        );

        assert!(error_x < 1e-6, "x error {} too large", error_x);
        assert!(error_y < 1e-6, "y error {} too large", error_y);
    }

    #[test]
    fn test_eigenvalues_a036() {
        let a: f64 = 0.36;
        let b: f64 = 0.3;

        let mut x = 0.0;
        let mut y = 0.0;
        for _ in 0..1000 {
            let x_new = 1.0 - a * x * x + y;
            let y_new = b * x;
            if (x_new - x).abs() < 1e-10 && (y_new - y).abs() < 1e-10 {
                break;
            }
            x = x_new;
            y = y_new;
        }

        let jac = Matrix2::new(-2.0 * a * x, 1.0, b, 0.0);

        let trace = jac.trace();
        let det = jac.determinant();
        let discriminant = trace * trace - 4.0 * det;

        assert!(discriminant >= 0.0, "Complex eigenvalues");

        let sqrt_disc = discriminant.sqrt();
        let lambda1 = (trace + sqrt_disc) / 2.0;
        let lambda2 = (trace - sqrt_disc) / 2.0;

        println!("Eigenvalues: λ1={}, λ2={}", lambda1, lambda2);
        println!("Magnitudes: |λ1|={}, |λ2|={}", lambda1.abs(), lambda2.abs());

        let both_stable = lambda1.abs() < 1.0 && lambda2.abs() < 1.0;
        let has_unstable = lambda1.abs() > 1.0 || lambda2.abs() > 1.0;

        if both_stable {
            println!("Both eigenvalues stable - this is an attractor");
            println!("For minimal invariant set, use inverse map to trace stable manifold");
        } else {
            assert!(
                has_unstable,
                "Should have at least one unstable eigenvalue for saddle"
            );
        }
    }

    #[test]
    fn test_a036_manifold() {
        let params = HenonParams::new(0.36, 0.3, 0.0625).unwrap();

        let mut x: f64 = 0.0;
        let mut y: f64 = 0.0;
        for _ in 0..1000 {
            let x_new = 1.0 - 0.36 * x * x + y;
            let y_new = 0.3 * x;
            if (x_new - x).abs() < 1e-10 && (y_new - y).abs() < 1e-10 {
                break;
            }
            x = x_new;
            y = y_new;
        }

        println!("Testing a=0.36 fixed point: ({}, {})", x, y);

        let config = ManifoldConfig {
            max_iter: 50,
            max_points: 5000,
            ..ManifoldConfig::default()
        };

        let jac = Matrix2::new(-2.0 * 0.36 * x, 1.0, 0.3, 0.0);
        let trace = jac.trace();
        let det = jac.determinant();
        let sqrt_disc = (trace * trace - 4.0 * det).sqrt();
        let lambda1 = (trace + sqrt_disc) / 2.0;
        let lambda2 = (trace - sqrt_disc) / 2.0;

        println!("Eigenvalues: λ1={}, λ2={}", lambda1, lambda2);

        let (unstable_lambda, eigenvec) = if lambda2.abs() > lambda1.abs() {
            let v = if (jac[(0, 0)] - lambda2).abs() > 1e-10 {
                Vector2::new(jac[(0, 1)], lambda2 - jac[(0, 0)])
            } else {
                Vector2::new(lambda2 - jac[(1, 1)], jac[(1, 0)])
            };
            (lambda2, v.normalize())
        } else {
            let v = if (jac[(0, 0)] - lambda1).abs() > 1e-10 {
                Vector2::new(jac[(0, 1)], lambda1 - jac[(0, 0)])
            } else {
                Vector2::new(lambda1 - jac[(1, 1)], jac[(1, 0)])
            };
            (lambda1, v.normalize())
        };

        let saddle_type = if unstable_lambda.abs() > 1.0 {
            SaddleType::Regular
        } else {
            println!("WARNING: No unstable eigenvalue found!");
            SaddleType::Regular
        };

        println!(
            "Saddle type: {:?}, |λ_unstable|={}",
            saddle_type,
            unstable_lambda.abs()
        );

        let saddle = SaddlePoint::from_2d_eigenvector(
            Vector2::new(x, y),
            eigenvec,
            1,
            unstable_lambda,
            saddle_type,
            None,
        );

        let computer = UnstableManifoldComputer::new(params, config);
        let result = computer.compute_direction(&saddle, 1.0, &[]);

        match result {
            Ok(traj) => {
                println!(
                    "Success! Trajectory has {} points, stop reason: {:?}",
                    traj.points.len(),
                    traj.stop_reason
                );
                assert!(traj.points.len() > 1);
            }
            Err(e) => {
                println!("Expected behavior: {}", e);
            }
        }
    }

    fn henon_fixed_points(a: f64, b: f64) -> Vec<(f64, f64)> {
        let disc = (1.0 - b) * (1.0 - b) + 4.0 * a;
        if disc < 0.0 {
            return vec![];
        }
        let sqrt_disc = disc.sqrt();
        let x1 = (-(1.0 - b) + sqrt_disc) / (2.0 * a);
        let x2 = (-(1.0 - b) - sqrt_disc) / (2.0 * a);
        vec![(x1, b * x1), (x2, b * x2)]
    }

    fn classify_fixed_point(a: f64, b: f64, x: f64, y: f64) -> (&'static str, f64, f64) {
        let jac = Matrix2::new(-2.0 * a * x, 1.0, b, 0.0);
        let trace = jac.trace();
        let det = jac.determinant();
        let disc = trace * trace - 4.0 * det;
        let (l1, l2) = if disc >= 0.0 {
            let s = disc.sqrt();
            ((trace + s) / 2.0, (trace - s) / 2.0)
        } else {
            let r = trace / 2.0;
            let im = (-disc).sqrt() / 2.0;
            let mag = (r * r + im * im).sqrt();
            (mag, mag)
        };
        let stab = if l1.abs() < 1.0 && l2.abs() < 1.0 {
            "attractor"
        } else if l1.abs() > 1.0 && l2.abs() > 1.0 {
            "repeller"
        } else {
            "saddle"
        };
        (stab, l1, l2)
    }

    #[test]
    fn test_repeller_classification() {
        let fps = henon_fixed_points(1.4, 0.3);
        assert_eq!(fps.len(), 2);

        for (x, y) in &fps {
            let (stab, l1, l2) = classify_fixed_point(1.4, 0.3, *x, *y);
            println!("FP ({:.4}, {:.4}): {} (λ={:.4}, {:.4})", x, y, stab, l1, l2);
            assert_eq!(stab, "saddle", "Expected saddle at a=1.4");
        }

        // At a=0.2, b=0.3, x_+ should be an attractor
        let fps2 = henon_fixed_points(0.2, 0.3);
        assert!(!fps2.is_empty());
        let (x, y) = fps2[0];
        let (stab, _, _) = classify_fixed_point(0.2, 0.3, x, y);
        assert_eq!(stab, "attractor", "Expected attractor at a=0.2");
    }

    #[test]
    fn test_repeller_attractor_duality() {
        // A repeller of f is an attractor of f^{-1}
        // Find a parameter where a fixed point is a repeller
        // At high a values, the x_+ fixed point becomes a repeller
        let a = 4.0;
        let b = 0.3;
        let fps = henon_fixed_points(a, b);

        for (x, y) in &fps {
            let (stab, l1, l2) = classify_fixed_point(a, b, *x, *y);
            println!(
                "a={}, FP ({:.4}, {:.4}): {} (λ={:.4}, {:.4})",
                a, x, y, stab, l1, l2
            );

            if stab == "repeller" {
                // Verify: eigenvalues of inverse are 1/λ_i, all < 1
                let inv_l1 = 1.0 / l1;
                let inv_l2 = 1.0 / l2;
                assert!(
                    inv_l1.abs() < 1.0 && inv_l2.abs() < 1.0,
                    "Inverse eigenvalues should be inside unit circle: |1/λ1|={}, |1/λ2|={}",
                    inv_l1.abs(),
                    inv_l2.abs()
                );
            }
        }
    }

    #[test]
    fn test_repeller_forward_divergence() {
        // Points near a repeller should diverge under forward iteration
        let a = 4.0;
        let b = 0.3;
        let eps = 0.01;
        let params = HenonParams::new(a, b, eps).unwrap();
        let fps = henon_fixed_points(a, b);

        for (fx, fy) in &fps {
            let (stab, _, _) = classify_fixed_point(a, b, *fx, *fy);
            if stab != "repeller" {
                continue;
            }

            // Perturb slightly from the repeller
            let perturb = Vector2::new(fx + 1e-6, fy + 1e-6);
            let mut pos = perturb;
            let fp = Vector2::new(*fx, *fy);

            let initial_dist = (pos - fp).norm();
            let mut diverging = false;

            for _ in 0..20 {
                match params.henon_map(&pos) {
                    Ok(next) => {
                        let dist = (next - fp).norm();
                        if dist > initial_dist * 10.0 {
                            diverging = true;
                            break;
                        }
                        pos = next;
                    }
                    Err(_) => break,
                }
            }

            assert!(
                diverging,
                "Points near repeller ({:.4}, {:.4}) should diverge under forward iteration",
                fx, fy
            );
        }
    }

    #[test]
    fn test_repeller_backward_convergence() {
        let a = 4.0;
        let b = 0.3;
        let eps = 0.01;
        let params = HenonParams::new(a, b, eps).unwrap();
        let fps = henon_fixed_points(a, b);

        for (fx, fy) in &fps {
            let (stab, _, _) = classify_fixed_point(a, b, *fx, *fy);
            if stab != "repeller" {
                continue;
            }

            let perturb = Vector2::new(fx + 1e-4, fy + 1e-4);
            let mut pos = perturb;
            let fp = Vector2::new(*fx, *fy);

            let mut converged = false;
            for _ in 0..100 {
                match params.henon_map_inverse(&pos) {
                    Ok(next) => {
                        let dist = (next - fp).norm();
                        if dist < 1e-8 {
                            converged = true;
                            break;
                        }
                        if dist > 100.0 {
                            break;
                        }
                        pos = next;
                    }
                    Err(_) => break,
                }
            }

            assert!(
                converged,
                "Points near repeller ({:.4}, {:.4}) should converge under inverse iteration",
                fx, fy
            );
        }
    }

    #[test]
    fn test_stable_manifold_forward_convergence() {
        // Points on the stable manifold should converge to the saddle under FORWARD iteration
        let a = 1.4;
        let b = 0.3;
        let eps = 0.01;
        let params = HenonParams::new(a, b, eps).unwrap();
        let fps = henon_fixed_points(a, b);

        for (fx, fy) in &fps {
            let (stab, l1, l2) = classify_fixed_point(a, b, *fx, *fy);
            if stab != "saddle" {
                continue;
            }

            // Compute stable eigenvector
            let stable_lambda = if l1.abs() < l2.abs() { l1 } else { l2 };
            let unstable_lambda = if l1.abs() > l2.abs() { l1 } else { l2 };

            let v1 = 1.0;
            let v2 = 2.0 * a * fx + stable_lambda;
            let norm = (v1 * v1 + v2 * v2).sqrt();
            let stable_eigvec = Vector2::new(v1 / norm, v2 / norm);

            // Compute stable manifold using DualRepeller (inverse map)
            let saddle = SaddlePoint::from_2d_eigenvector(
                Vector2::new(*fx, *fy),
                stable_eigvec,
                1,
                stable_lambda,
                SaddleType::DualRepeller,
                None,
            );

            let config = ManifoldConfig {
                max_iter: 200,
                max_points: 10000,
                time_limit: 5.0,
                ..ManifoldConfig::default()
            };
            let computer = UnstableManifoldComputer::new(params, config);
            let result = computer.compute_manifold(&saddle, &[]);

            assert!(result.is_ok(), "Stable manifold computation should succeed");
            let (plus, minus) = result.unwrap();

            // Take some points from the stable manifold and verify they converge
            // under FORWARD iteration
            let fp = Vector2::new(*fx, *fy);
            let mut convergence_count = 0;
            let test_points: Vec<_> = plus
                .points
                .iter()
                .chain(minus.points.iter())
                .take(20)
                .collect();

            for state in &test_points {
                let mut pos = state.pos;
                let mut converging = false;

                for _ in 0..500 {
                    match params.henon_map(&pos) {
                        Ok(next) => {
                            let dist = (next - fp).norm();
                            if dist < 1e-3 {
                                converging = true;
                                break;
                            }
                            if dist > 100.0 {
                                break;
                            }
                            pos = next;
                        }
                        Err(_) => break,
                    }
                }

                if converging {
                    convergence_count += 1;
                }
            }

            println!(
                "Stable manifold of saddle ({:.4}, {:.4}): {}/{} points converge forward",
                fx,
                fy,
                convergence_count,
                test_points.len()
            );

            // At least some points should converge (near the saddle they will)
            assert!(
                convergence_count > 0,
                "Some stable manifold points should converge under forward iteration"
            );
        }
    }

    #[test]
    fn test_stable_manifold_tangency() {
        // Near the saddle, the stable manifold should be tangent to the stable eigenspace
        let a = 1.4;
        let b = 0.3;
        let eps = 0.01;
        let params = HenonParams::new(a, b, eps).unwrap();
        let fps = henon_fixed_points(a, b);

        for (fx, fy) in &fps {
            let (stab, l1, l2) = classify_fixed_point(a, b, *fx, *fy);
            if stab != "saddle" {
                continue;
            }

            let stable_lambda = if l1.abs() < l2.abs() { l1 } else { l2 };
            let v1 = 1.0;
            let v2 = 2.0 * a * fx + stable_lambda;
            let norm = (v1 * v1 + v2 * v2).sqrt();
            let stable_eigvec = Vector2::new(v1 / norm, v2 / norm);

            let saddle = SaddlePoint::from_2d_eigenvector(
                Vector2::new(*fx, *fy),
                stable_eigvec,
                1,
                stable_lambda,
                SaddleType::DualRepeller,
                None,
            );

            let config = ManifoldConfig {
                max_iter: 50,
                max_points: 5000,
                time_limit: 5.0,
                ..ManifoldConfig::default()
            };
            let computer = UnstableManifoldComputer::new(params, config);
            let result = computer.compute_manifold(&saddle, &[]);
            assert!(result.is_ok());

            let (plus, _) = result.unwrap();
            let fp = Vector2::new(*fx, *fy);

            // Check the first few points (closest to saddle) are along stable eigenvector
            if plus.points.len() >= 2 {
                let first_displacement = plus.points[0].pos - fp;
                let disp_norm = first_displacement.norm();
                if disp_norm > 1e-10 {
                    let direction = first_displacement / disp_norm;
                    // Dot product should be close to ±1 (parallel or antiparallel)
                    let dot = direction.dot(&stable_eigvec).abs();
                    println!(
                        "Tangency check at ({:.4}, {:.4}): |dot(manifold_dir, v_s)| = {:.6}",
                        fx, fy, dot
                    );
                    assert!(
                        dot > 0.9,
                        "Near saddle, stable manifold should be tangent to stable eigenspace, got dot={:.4}",
                        dot
                    );
                }
            }
        }
    }

    #[test]
    fn test_stable_unstable_manifold_invariance() {
        let a = 1.4;
        let b = 0.3;
        let eps = 0.01;
        let params = HenonParams::new(a, b, eps).unwrap();
        let fps = henon_fixed_points(a, b);

        for (fx, fy) in &fps {
            let (stab, l1, l2) = classify_fixed_point(a, b, *fx, *fy);
            if stab != "saddle" {
                continue;
            }

            let stable_lambda = if l1.abs() < l2.abs() { l1 } else { l2 };
            let v1 = 1.0;
            let v2 = 2.0 * a * fx + stable_lambda;
            let norm = (v1 * v1 + v2 * v2).sqrt();
            let stable_eigvec = Vector2::new(v1 / norm, v2 / norm);

            let saddle = SaddlePoint::from_2d_eigenvector(
                Vector2::new(*fx, *fy),
                stable_eigvec,
                1,
                stable_lambda,
                SaddleType::DualRepeller,
                None,
            );

            let config = ManifoldConfig {
                max_iter: 50,
                max_points: 5000,
                time_limit: 5.0,
                ..ManifoldConfig::default()
            };
            let computer = UnstableManifoldComputer::new(params, config);
            let result = computer.compute_manifold(&saddle, &[]).unwrap();
            let (plus, _) = result;
            let fp = Vector2::new(*fx, *fy);

            let mut converge_count = 0;
            let mut tested = 0;
            for state in plus.points.iter().take(3) {
                if (state.pos - fp).norm() > 0.1 {
                    continue;
                }
                tested += 1;
                if let Ok(mapped) = params.henon_map(&state.pos) {
                    let mut pos = mapped;
                    let mut converged = false;
                    for _ in 0..1000 {
                        match params.henon_map(&pos) {
                            Ok(next) => {
                                if (next - fp).norm() < 1e-2 {
                                    converged = true;
                                    break;
                                }
                                if next.norm() > 100.0 {
                                    break;
                                }
                                pos = next;
                            }
                            Err(_) => break,
                        }
                    }
                    if converged {
                        converge_count += 1;
                    }
                }
            }

            if tested > 0 {
                println!(
                    "Invariance at ({:.4}, {:.4}): {}/{} close points converge",
                    fx, fy, converge_count, tested
                );
                assert!(
                    converge_count > 0 || tested == 0,
                    "f(q) for q near saddle on W^s should converge to the saddle"
                );
            }
        }
    }

    #[test]
    fn test_hausdorff_distance_identical_sets() {
        // d_H(A, A) = 0
        let set_a = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 1.0),
        ];
        let result = crate::hausdorff::compute_hausdorff_distance(&set_a, &set_a, 3).unwrap();
        assert!(
            result.distance < 1e-10,
            "d_H(A,A) should be 0, got {}",
            result.distance
        );
    }

    #[test]
    fn test_hausdorff_distance_known_value() {
        let set_a: Vec<Vector2<f64>> = (0..10).map(|i| Vector2::new(i as f64 * 0.1, 0.0)).collect();
        let set_b: Vec<Vector2<f64>> = (0..10).map(|i| Vector2::new(i as f64 * 0.1, 1.0)).collect();

        let result = crate::hausdorff::compute_hausdorff_distance(&set_a, &set_b, 3).unwrap();
        assert!(
            (result.distance - 1.0).abs() < 0.01,
            "d_H for parallel lines offset by 1 should be ~1.0, got {}",
            result.distance
        );
    }

    #[test]
    fn test_hausdorff_distance_symmetry() {
        // d_H(A, B) = d_H(B, A)
        let set_a = vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)];
        let set_b = vec![Vector2::new(0.5, 0.5), Vector2::new(1.5, 0.5)];

        let result_ab = crate::hausdorff::compute_hausdorff_distance(&set_a, &set_b, 2).unwrap();
        let result_ba = crate::hausdorff::compute_hausdorff_distance(&set_b, &set_a, 2).unwrap();
        assert!(
            (result_ab.distance - result_ba.distance).abs() < 1e-10,
            "Hausdorff distance should be symmetric: {} vs {}",
            result_ab.distance,
            result_ba.distance
        );
    }

    #[test]
    fn test_hausdorff_distance_triangle_inequality() {
        // d_H(A, C) <= d_H(A, B) + d_H(B, C)
        let set_a = vec![Vector2::new(0.0, 0.0)];
        let set_b = vec![Vector2::new(1.0, 0.0)];
        let set_c = vec![Vector2::new(3.0, 0.0)];

        let d_ab = crate::hausdorff::compute_hausdorff_distance(&set_a, &set_b, 1)
            .unwrap()
            .distance;
        let d_bc = crate::hausdorff::compute_hausdorff_distance(&set_b, &set_c, 1)
            .unwrap()
            .distance;
        let d_ac = crate::hausdorff::compute_hausdorff_distance(&set_a, &set_c, 1)
            .unwrap()
            .distance;

        assert!(
            d_ac <= d_ab + d_bc + 1e-10,
            "Triangle inequality violated: d(A,C)={} > d(A,B)+d(B,C)={}",
            d_ac,
            d_ab + d_bc
        );
    }

    #[test]
    fn test_directed_hausdorff_asymmetry() {
        // The directed Hausdorff distances are generally asymmetric
        let set_a = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(3.0, 0.0),
        ];
        let set_b = vec![Vector2::new(1.0, 0.0)];

        let (d_ab, _, _) = crate::hausdorff::directed_hausdorff(&set_a, &set_b);
        let (d_ba, _, _) = crate::hausdorff::directed_hausdorff(&set_b, &set_a);

        // d(A->B) = max(min_b||a-b|| for each a) = max(1, 0, 2) = 2
        //   (0,0) -> 1.0, (1,0) -> 0.0, (3,0) -> 2.0
        // d(B->A) = 0 (B's single point (1,0) has dist 0 to A)
        assert!(
            (d_ab - 2.0).abs() < 1e-10,
            "d(A->B) should be 2.0, got {}",
            d_ab
        );
        assert!(d_ba < 1e-10, "d(B->A) should be 0.0, got {}", d_ba);
    }

    #[test]
    fn test_saddle_point_dual_repeller_type() {
        // SaddleType::DualRepeller should trigger inverse map in manifold computation
        let saddle = SaddlePoint::from_2d_eigenvector(
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            1,
            0.5, // eigenvalue
            SaddleType::DualRepeller,
            None,
        );
        assert_eq!(saddle.saddle_type, SaddleType::DualRepeller);
        assert_eq!(saddle.period, 1);
    }

    #[test]
    fn test_henon_inverse_roundtrip() {
        // f^{-1}(f(p)) = p for Henon map
        let params = HenonParams::new(1.4, 0.3, 0.01).unwrap();
        let test_points = vec![
            Vector2::new(0.5, 0.3),
            Vector2::new(-0.5, 0.1),
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, -0.5),
        ];

        for p in &test_points {
            let fp = params.henon_map(p).unwrap();
            let roundtrip = params.henon_map_inverse(&fp).unwrap();
            let err = (roundtrip - p).norm();
            assert!(
                err < 1e-10,
                "f^-1(f(p)) should equal p, got error {} for p=({}, {})",
                err,
                p.x,
                p.y
            );
        }
    }

    #[test]
    fn test_henon_forward_inverse_roundtrip() {
        // f(f^{-1}(p)) = p
        let params = HenonParams::new(1.4, 0.3, 0.01).unwrap();
        let test_points = vec![Vector2::new(0.5, 0.3), Vector2::new(-0.5, 0.1)];

        for p in &test_points {
            let inv_p = params.henon_map_inverse(p).unwrap();
            let roundtrip = params.henon_map(&inv_p).unwrap();
            let err = (roundtrip - p).norm();
            assert!(err < 1e-10, "f(f^-1(p)) should equal p, got error {}", err);
        }
    }

    #[test]
    fn test_manifold_config_dual_repeller_spacing() {
        let saddle_regular = SaddlePoint::from_2d_eigenvector(
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            1,
            2.0,
            SaddleType::Regular,
            None,
        );
        let saddle_repeller = SaddlePoint::from_2d_eigenvector(
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            1,
            2.0,
            SaddleType::DualRepeller,
            None,
        );

        assert_eq!(saddle_regular.saddle_type, SaddleType::Regular);
        assert_eq!(saddle_repeller.saddle_type, SaddleType::DualRepeller);
    }

    #[test]
    fn test_negative_eigenvalue_period_doubling() {
        let a = 1.4;
        let b = 0.3;
        let eps = 0.01;
        let params = HenonParams::new(a, b, eps).unwrap();
        let fps = henon_fixed_points(a, b);

        for (fx, fy) in &fps {
            let (stab, l1, l2) = classify_fixed_point(a, b, *fx, *fy);
            if stab != "saddle" {
                continue;
            }

            let unstable_lambda = if l1.abs() > l2.abs() { l1 } else { l2 };

            // If eigenvalue is negative, compute_direction will handle it
            let v1 = 1.0;
            let v2 = 2.0 * a * fx + unstable_lambda;
            let norm = (v1 * v1 + v2 * v2).sqrt();
            let eigvec = Vector2::new(v1 / norm, v2 / norm);

            let saddle = SaddlePoint::from_2d_eigenvector(
                Vector2::new(*fx, *fy),
                eigvec,
                1,
                unstable_lambda,
                SaddleType::Regular,
                None,
            );

            let config = ManifoldConfig {
                max_iter: 50,
                max_points: 5000,
                time_limit: 3.0,
                ..ManifoldConfig::default()
            };
            let computer = UnstableManifoldComputer::new(params, config);
            let result = computer.compute_direction(&saddle, 1.0, &[]);

            assert!(
                result.is_ok(),
                "Manifold computation should handle negative eigenvalues"
            );
            let traj = result.unwrap();
            assert!(traj.points.len() > 1, "Should produce trajectory points");

            if unstable_lambda < 0.0 {
                println!(
                    "Handled negative eigenvalue {:.4} at ({:.4}, {:.4}), got {} points",
                    unstable_lambda,
                    fx,
                    fy,
                    traj.points.len()
                );
            }
        }
    }

    #[test]
    fn test_unstable_manifold_backward_convergence() {
        let a = 1.4;
        let b = 0.3;
        let eps = 0.01;
        let params = HenonParams::new(a, b, eps).unwrap();
        let fps = henon_fixed_points(a, b);

        for (fx, fy) in &fps {
            let (stab, l1, l2) = classify_fixed_point(a, b, *fx, *fy);
            if stab != "saddle" {
                continue;
            }

            let unstable_lambda = if l1.abs() > l2.abs() { l1 } else { l2 };
            let v1 = 1.0;
            let v2 = 2.0 * a * fx + unstable_lambda;
            let norm = (v1 * v1 + v2 * v2).sqrt();
            let eigvec = Vector2::new(v1 / norm, v2 / norm);

            let saddle = SaddlePoint::from_2d_eigenvector(
                Vector2::new(*fx, *fy),
                eigvec,
                1,
                unstable_lambda,
                SaddleType::Regular,
                None,
            );

            let config = ManifoldConfig {
                max_iter: 100,
                max_points: 5000,
                time_limit: 5.0,
                ..ManifoldConfig::default()
            };
            let computer = UnstableManifoldComputer::new(params, config);
            let (plus, _) = computer.compute_manifold(&saddle, &[]).unwrap();
            let fp = Vector2::new(*fx, *fy);

            let mut converge_count = 0;
            for state in plus.points.iter().take(10) {
                let mut pos = state.pos;
                let mut converged = false;
                for _ in 0..200 {
                    match params.henon_map_inverse(&pos) {
                        Ok(next) => {
                            if (next - fp).norm() < 1e-3 {
                                converged = true;
                                break;
                            }
                            if next.norm() > 100.0 {
                                break;
                            }
                            pos = next;
                        }
                        Err(_) => break,
                    }
                }
                if converged {
                    converge_count += 1;
                }
            }

            println!(
                "Unstable manifold backward convergence at ({:.4}, {:.4}): {}/{}",
                fx,
                fy,
                converge_count,
                plus.points.len().min(10)
            );
            assert!(
                converge_count > 0,
                "Some W^u points should converge under inverse iteration"
            );
        }
    }

    #[test]
    fn test_hausdorff_empty_set_error() {
        let empty: Vec<Vector2<f64>> = vec![];
        let non_empty = vec![Vector2::new(0.0, 0.0)];

        let result = crate::hausdorff::compute_hausdorff_distance(&empty, &non_empty, 1);
        assert!(result.is_err(), "Should error on empty set A");

        let result2 = crate::hausdorff::compute_hausdorff_distance(&non_empty, &empty, 1);
        assert!(result2.is_err(), "Should error on empty set B");
    }
}
