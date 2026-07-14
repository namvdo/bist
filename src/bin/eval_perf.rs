use henon_periodic_orbits::{
    BoundaryHenonSystemAnalysis, DuffingODE, HenonParams, ManifoldConfig, SaddlePoint, SaddleType,
    UlamComputer, UnstableManifoldComputer,
};
use nalgebra::{Matrix2, Vector2};
use serde::Serialize;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const X_MIN: f64 = -2.0;
const X_MAX: f64 = 2.0;
const Y_MIN: f64 = -1.5;
const Y_MAX: f64 = 1.5;

#[derive(Serialize)]
struct PerfRecord {
    case_id: String,
    category: String,
    scenario: String,
    duration_ms: f64,
    params: Value,
    outputs: Value,
}

fn parse_out_arg() -> Option<PathBuf> {
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--out" {
            return args.next().map(PathBuf::from);
        }
    }
    None
}

fn now_unix_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn unstable_eigenpair(jac: Matrix2<f64>) -> (f64, Vector2<f64>) {
    let trace = jac[(0, 0)] + jac[(1, 1)];
    let det = jac[(0, 0)] * jac[(1, 1)] - jac[(0, 1)] * jac[(1, 0)];
    let disc = (trace * trace - 4.0 * det).max(0.0);
    let sqrt_disc = disc.sqrt();

    let lambda1 = (trace + sqrt_disc) / 2.0;
    let lambda2 = (trace - sqrt_disc) / 2.0;
    let unstable = if lambda1.abs() >= lambda2.abs() {
        lambda1
    } else {
        lambda2
    };

    let mut eigvec = if jac[(0, 1)].abs() > 1e-12 {
        Vector2::new(1.0, (unstable - jac[(0, 0)]) / jac[(0, 1)])
    } else if jac[(1, 0)].abs() > 1e-12 {
        Vector2::new((unstable - jac[(1, 1)]) / jac[(1, 0)], 1.0)
    } else {
        Vector2::new(1.0, 0.0)
    };

    if eigvec.norm() > 1e-12 {
        eigvec /= eigvec.norm();
    } else {
        eigvec = Vector2::new(1.0, 0.0);
    }

    (unstable, eigvec)
}

fn principal_henon_fixed_point(a: f64, b: f64) -> Vector2<f64> {
    let disc = (1.0 - b) * (1.0 - b) + 4.0 * a;
    let x = (-(1.0 - b) + disc.sqrt()) / (2.0 * a);
    Vector2::new(x, b * x)
}

fn benchmark_periodic_case(
    case_id: &str,
    a: f64,
    b: f64,
    epsilon: f64,
    max_period: usize,
    grid_size: usize,
    theta_grid_size: usize,
    residual_threshold: f64,
) -> PerfRecord {
    let start = Instant::now();
    let analysis = BoundaryHenonSystemAnalysis::new_with_search_settings(
        a,
        b,
        epsilon,
        max_period,
        X_MIN,
        X_MAX,
        Y_MIN,
        Y_MAX,
        grid_size,
        theta_grid_size,
        residual_threshold,
    );
    let duration_ms = start.elapsed().as_secs_f64() * 1000.0;

    let mut period_histogram: BTreeMap<String, usize> = BTreeMap::new();
    let mut stability_histogram: BTreeMap<String, usize> = BTreeMap::new();

    for orbit in &analysis.orbit_database.orbits {
        *period_histogram
            .entry(format!("p{}", orbit.period))
            .or_insert(0) += 1;
        let stability_label = format!("{:?}", orbit.stability).to_lowercase();
        *stability_histogram.entry(stability_label).or_insert(0) += 1;
    }

    let has_period_4 = period_histogram.get("p4").copied().unwrap_or(0) > 0;
    let has_period_7 = period_histogram.get("p7").copied().unwrap_or(0) > 0;

    PerfRecord {
        case_id: case_id.to_string(),
        category: "backend_periodic_orbits".to_string(),
        scenario: if case_id.contains("stress") {
            "stress".to_string()
        } else if case_id.contains("bifurcation") {
            "interesting".to_string()
        } else {
            "typical".to_string()
        },
        duration_ms,
        params: json!({
            "a": a,
            "b": b,
            "epsilon": epsilon,
            "max_period": max_period,
            "grid_size": grid_size,
            "theta_grid_size": theta_grid_size,
            "residual_threshold": residual_threshold,
            "range": {"x_min": X_MIN, "x_max": X_MAX, "y_min": Y_MIN, "y_max": Y_MAX}
        }),
        outputs: json!({
            "total_orbits": analysis.orbit_database.total_count(),
            "period_histogram": period_histogram,
            "stability_histogram": stability_histogram,
            "has_period_4": has_period_4,
            "has_period_7": has_period_7
        }),
    }
}

fn benchmark_manifold_case(
    case_id: &str,
    a: f64,
    b: f64,
    epsilon: f64,
    max_iter: usize,
    time_limit: f64,
    max_points: usize,
) -> PerfRecord {
    let (duration_ms, output) = match HenonParams::new(a, b, epsilon) {
        Ok(params) => {
            let fp = principal_henon_fixed_point(a, b);
            let jac = params.jacobian(fp);
            let (unstable_lambda, unstable_vec) = unstable_eigenpair(jac);

            let saddle = SaddlePoint::from_2d_eigenvector(
                fp,
                unstable_vec,
                1,
                unstable_lambda,
                SaddleType::Regular,
                None,
            );

            let config = ManifoldConfig {
                max_iter,
                time_limit,
                max_points,
                ..ManifoldConfig::default()
            };

            let computer = UnstableManifoldComputer::new(params, config);
            let start = Instant::now();
            match computer.compute_manifold(&saddle, &[]) {
                Ok((plus, minus)) => {
                    let duration_ms = start.elapsed().as_secs_f64() * 1000.0;
                    let output = json!({
                        "plus_points": plus.points.len(),
                        "minus_points": minus.points.len(),
                        "total_points": plus.points.len() + minus.points.len(),
                        "plus_stop_reason": format!("{:?}", plus.stop_reason),
                        "minus_stop_reason": format!("{:?}", minus.stop_reason),
                        "unstable_eigenvalue": unstable_lambda
                    });
                    (duration_ms, output)
                }
                Err(err) => {
                    let duration_ms = start.elapsed().as_secs_f64() * 1000.0;
                    (duration_ms, json!({ "error": err }))
                }
            }
        }
        Err(err) => (0.0, json!({ "error": err })),
    };

    PerfRecord {
        case_id: case_id.to_string(),
        category: "backend_unstable_manifold".to_string(),
        scenario: if case_id.contains("stress") {
            "stress".to_string()
        } else if case_id.contains("bifurcation") {
            "interesting".to_string()
        } else {
            "typical".to_string()
        },
        duration_ms,
        params: json!({
            "a": a,
            "b": b,
            "epsilon": epsilon,
            "max_iter": max_iter,
            "time_limit_secs": time_limit,
            "max_points": max_points
        }),
        outputs: output,
    }
}

fn benchmark_ulam_case(
    case_id: &str,
    a: f64,
    b: f64,
    epsilon: f64,
    subdivisions: usize,
    points_per_box: usize,
) -> PerfRecord {
    let start = Instant::now();
    let result = UlamComputer::new(
        a,
        b,
        subdivisions,
        points_per_box,
        epsilon,
        X_MIN,
        X_MAX,
        Y_MIN,
        Y_MAX,
    );
    let duration_ms = start.elapsed().as_secs_f64() * 1000.0;

    let samples_per_dim = (points_per_box as f64).sqrt().ceil() as usize;
    let total_boxes = subdivisions * subdivisions;
    let total_samples = total_boxes * samples_per_dim * samples_per_dim;

    let outputs = match result {
        Ok(_) => json!({
            "status": "ok",
            "total_boxes": total_boxes,
            "samples_per_box": samples_per_dim * samples_per_dim,
            "total_samples": total_samples
        }),
        Err(err) => json!({
            "status": "error",
            "error": err,
            "total_boxes": total_boxes,
            "samples_per_box": samples_per_dim * samples_per_dim,
            "total_samples": total_samples
        }),
    };

    PerfRecord {
        case_id: case_id.to_string(),
        category: "backend_ulam".to_string(),
        scenario: if case_id.contains("stress") {
            "stress".to_string()
        } else if case_id.contains("interesting") {
            "interesting".to_string()
        } else {
            "typical".to_string()
        },
        duration_ms,
        params: json!({
            "a": a,
            "b": b,
            "epsilon": epsilon,
            "subdivisions": subdivisions,
            "points_per_box": points_per_box,
            "range": {"x_min": X_MIN, "x_max": X_MAX, "y_min": Y_MIN, "y_max": Y_MAX}
        }),
        outputs,
    }
}

fn benchmark_duffing_rk4_case(
    case_id: &str,
    delta: f64,
    h: f64,
    steps: usize,
    x0: f64,
    y0: f64,
) -> PerfRecord {
    let (duration_ms, output) = match DuffingODE::new(delta) {
        Ok(ode) => {
            let mut state = Vector2::new(x0, y0);
            let start = Instant::now();
            let mut error: Option<String> = None;

            for _ in 0..steps {
                match ode.rk4_step(state, h) {
                    Ok(next) => state = next,
                    Err(err) => {
                        error = Some(err);
                        break;
                    }
                }
            }

            let duration_ms = start.elapsed().as_secs_f64() * 1000.0;
            let output = if let Some(err) = error {
                json!({ "status": "error", "error": err, "steps_completed": 0 })
            } else {
                json!({
                    "status": "ok",
                    "final_x": state.x,
                    "final_y": state.y,
                    "steps_completed": steps
                })
            };
            (duration_ms, output)
        }
        Err(err) => (
            0.0,
            json!({ "status": "error", "error": err, "steps_completed": 0 }),
        ),
    };

    PerfRecord {
        case_id: case_id.to_string(),
        category: "backend_continuous_discretization".to_string(),
        scenario: if case_id.contains("stress") {
            "stress".to_string()
        } else {
            "typical".to_string()
        },
        duration_ms,
        params: json!({
            "delta": delta,
            "h": h,
            "steps": steps,
            "x0": x0,
            "y0": y0
        }),
        outputs: output,
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let out_path = parse_out_arg();
    let mut records = Vec::new();

    records.push(benchmark_periodic_case(
        "periodic_typical_henon",
        1.4,
        0.3,
        0.0625,
        7,
        15,
        12,
        1e-10,
    ));
    records.push(benchmark_periodic_case(
        "periodic_interesting_bifurcation_pre",
        0.55,
        0.3,
        0.0625,
        7,
        15,
        12,
        1e-10,
    ));
    records.push(benchmark_periodic_case(
        "periodic_interesting_bifurcation_post",
        0.60,
        0.3,
        0.0625,
        7,
        15,
        12,
        1e-10,
    ));
    records.push(benchmark_periodic_case(
        "periodic_stress_dense_search",
        1.4,
        0.3,
        0.0625,
        8,
        20,
        16,
        1e-10,
    ));

    records.push(benchmark_manifold_case(
        "manifold_typical_henon",
        1.4,
        0.3,
        0.0625,
        4000,
        8.0,
        300_000,
    ));
    records.push(benchmark_manifold_case(
        "manifold_interesting_near_bifurcation",
        0.55,
        0.3,
        0.0625,
        4000,
        8.0,
        300_000,
    ));
    records.push(benchmark_manifold_case(
        "manifold_stress_large_budget",
        1.4,
        0.3,
        0.0625,
        8000,
        12.0,
        700_000,
    ));

    records.push(benchmark_ulam_case(
        "ulam_typical_grid48",
        1.4,
        0.3,
        0.0625,
        48,
        64,
    ));
    records.push(benchmark_ulam_case(
        "ulam_interesting_grid64",
        1.4,
        0.3,
        0.0625,
        64,
        100,
    ));
    records.push(benchmark_ulam_case(
        "ulam_stress_grid80",
        1.4,
        0.3,
        0.0625,
        80,
        144,
    ));

    records.push(benchmark_duffing_rk4_case(
        "continuous_rk4_typical",
        0.15,
        0.05,
        50_000,
        0.1,
        0.1,
    ));
    records.push(benchmark_duffing_rk4_case(
        "continuous_rk4_stress",
        0.15,
        0.05,
        200_000,
        0.1,
        0.1,
    ));

    let payload = json!({
        "generated_at_unix": now_unix_secs(),
        "records": records
    });

    if let Some(path) = out_path {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::write(&path, serde_json::to_vec_pretty(&payload)?)?;
        println!("Wrote backend performance results to {}", path.display());
    } else {
        println!("{}", serde_json::to_string_pretty(&payload)?);
    }

    Ok(())
}
