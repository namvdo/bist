//! Geometric signed-distance offset contours around a closed MIS seed.

use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use wasm_bindgen::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Bounds2D {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

impl Bounds2D {
    pub fn width(&self) -> f64 {
        self.x_max - self.x_min
    }
    pub fn height(&self) -> f64 {
        self.y_max - self.y_min
    }
    pub fn is_valid(&self) -> bool {
        [self.x_min, self.x_max, self.y_min, self.y_max]
            .iter()
            .all(|v| v.is_finite())
            && self.x_min < self.x_max
            && self.y_min < self.y_max
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Orientation {
    CounterClockwise,
    Clockwise,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ExtendedBoundaryPoint {
    pub x: f64,
    pub y: f64,
    pub nx: f64,
    pub ny: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundaryComponent {
    pub id: usize,
    pub points: Vec<ExtendedBoundaryPoint>,
    pub orientation: Orientation,
    pub is_hole: bool,
    pub signed_area: f64,
    pub perimeter: f64,
    pub is_simple: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GeometricOffsetStopReason {
    RequestedLevelsCompleted,
    EscapedDomain,
    ContourExtractionFailed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeometricOffsetLevel {
    pub level: usize,
    pub target_distance: f64,
    pub boundary_components: Vec<BoundaryComponent>,
    pub area: f64,
    pub component_count: usize,
    pub hole_count: usize,
    pub offset_residual: f64,
    pub gap_residual: f64,
    pub uncertainty: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeometricOffsetResult {
    pub levels: Vec<GeometricOffsetLevel>,
    pub completed_levels: usize,
    pub epsilon: f64,
    pub seed_area: f64,
    pub seed_uncertainty: f64,
    pub stop_reason: GeometricOffsetStopReason,
    pub bounds: Bounds2D,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Point2 {
    x: f64,
    y: f64,
}

impl Point2 {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    fn distance(self, other: Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}

#[derive(Debug, Clone)]
struct SignedDistanceGrid {
    bounds: Bounds2D,
    width: usize,
    height: usize,
    dx: f64,
    dy: f64,
    values: Vec<f64>,
}

impl SignedDistanceGrid {
    fn new(bounds: Bounds2D, width: usize, height: usize) -> Result<Self, String> {
        if !bounds.is_valid() {
            return Err("Invalid geometric offset bounds".to_string());
        }
        if width < 8 || height < 8 {
            return Err("Grid dimensions must be at least 8".to_string());
        }
        let count = width.checked_mul(height).ok_or("Grid size overflow")?;
        Ok(Self {
            bounds,
            width,
            height,
            dx: bounds.width() / (width - 1) as f64,
            dy: bounds.height() / (height - 1) as f64,
            values: vec![0.0; count],
        })
    }
    fn index(&self, row: usize, col: usize) -> usize {
        row * self.width + col
    }
    fn point(&self, row: usize, col: usize) -> Point2 {
        Point2::new(
            self.bounds.x_min + col as f64 * self.dx,
            self.bounds.y_min + row as f64 * self.dy,
        )
    }
    fn value(&self, row: usize, col: usize) -> f64 {
        self.values[self.index(row, col)]
    }
}

fn signed_area(points: &[Point2]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        sum += a.x * b.y - b.x * a.y;
    }
    sum * 0.5
}

fn perimeter(points: &[Point2]) -> f64 {
    (0..points.len())
        .map(|i| points[i].distance(points[(i + 1) % points.len()]))
        .sum()
}

fn point_segment_distance(p: Point2, a: Point2, b: Point2) -> f64 {
    let vx = b.x - a.x;
    let vy = b.y - a.y;
    let len2 = vx * vx + vy * vy;
    if len2 <= 1e-24 {
        return p.distance(a);
    }
    let t = (((p.x - a.x) * vx + (p.y - a.y) * vy) / len2).clamp(0.0, 1.0);
    p.distance(Point2::new(a.x + t * vx, a.y + t * vy))
}

fn point_in_loop(p: Point2, loop_points: &[Point2]) -> bool {
    let mut inside = false;
    let mut j = loop_points.len() - 1;
    for i in 0..loop_points.len() {
        let pi = loop_points[i];
        let pj = loop_points[j];
        if ((pi.y > p.y) != (pj.y > p.y))
            && p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn signed_distance_to_set(p: Point2, loops: &[Vec<Point2>]) -> f64 {
    let mut distance = f64::INFINITY;
    let mut inside = false;
    for lp in loops {
        if lp.len() < 3 {
            continue;
        }
        let mut inside_loop = false;
        let mut previous = lp.len() - 1;
        for i in 0..lp.len() {
            distance = distance.min(point_segment_distance(p, lp[i], lp[(i + 1) % lp.len()]));
            let current_point = lp[i];
            let previous_point = lp[previous];
            if ((current_point.y > p.y) != (previous_point.y > p.y))
                && p.x
                    < (previous_point.x - current_point.x) * (p.y - current_point.y)
                        / (previous_point.y - current_point.y)
                        + current_point.x
            {
                inside_loop = !inside_loop;
            }
            previous = i;
        }
        if inside_loop {
            inside = !inside;
        }
    }
    if inside {
        -distance
    } else {
        distance
    }
}

fn build_distance_grid(
    bounds: Bounds2D,
    width: usize,
    height: usize,
    loops: &[Vec<Point2>],
) -> Result<SignedDistanceGrid, String> {
    let mut grid = SignedDistanceGrid::new(bounds, width, height)?;
    for row in 0..height {
        for col in 0..width {
            let idx = grid.index(row, col);
            grid.values[idx] = signed_distance_to_set(grid.point(row, col), loops);
        }
    }
    Ok(grid)
}

fn build_tube_grid(
    bounds: Bounds2D,
    width: usize,
    height: usize,
    loops: &[Vec<Point2>],
    radius: f64,
) -> Result<SignedDistanceGrid, String> {
    let mut grid = SignedDistanceGrid::new(bounds, width, height)?;
    for row in 0..height {
        for col in 0..width {
            let point = grid.point(row, col);
            let mut distance = f64::INFINITY;
            for lp in loops {
                for i in 0..lp.len() {
                    distance =
                        distance.min(point_segment_distance(point, lp[i], lp[(i + 1) % lp.len()]));
                }
            }
            let idx = grid.index(row, col);
            grid.values[idx] = distance - radius;
        }
    }
    Ok(grid)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum EdgeKey {
    Horizontal(usize, usize),
    Vertical(usize, usize),
}

fn crossing_point(grid: &SignedDistanceGrid, edge: EdgeKey, epsilon: f64) -> Point2 {
    let (p0, p1, v0, v1) = match edge {
        EdgeKey::Horizontal(row, col) => (
            grid.point(row, col),
            grid.point(row, col + 1),
            grid.value(row, col) + epsilon,
            grid.value(row, col + 1) + epsilon,
        ),
        EdgeKey::Vertical(row, col) => (
            grid.point(row, col),
            grid.point(row + 1, col),
            grid.value(row, col) + epsilon,
            grid.value(row + 1, col) + epsilon,
        ),
    };
    let denom = v0 - v1;
    let t = if denom.abs() < 1e-14 {
        0.5
    } else {
        (v0 / denom).clamp(0.0, 1.0)
    };
    Point2::new(p0.x + t * (p1.x - p0.x), p0.y + t * (p1.y - p0.y))
}

fn marching_squares(grid: &SignedDistanceGrid, epsilon: f64) -> Vec<Vec<Point2>> {
    let mut segments: Vec<(EdgeKey, EdgeKey)> = Vec::new();
    for row in 0..grid.height - 1 {
        for col in 0..grid.width - 1 {
            let bl = grid.value(row, col) + epsilon;
            let br = grid.value(row, col + 1) + epsilon;
            let tr = grid.value(row + 1, col + 1) + epsilon;
            let tl = grid.value(row + 1, col) + epsilon;
            let case_id = (bl <= 0.0) as u8
                | (((br <= 0.0) as u8) << 1)
                | (((tr <= 0.0) as u8) << 2)
                | (((tl <= 0.0) as u8) << 3);
            let bottom = EdgeKey::Horizontal(row, col);
            let right = EdgeKey::Vertical(row, col + 1);
            let top = EdgeKey::Horizontal(row + 1, col);
            let left = EdgeKey::Vertical(row, col);
            let center_inside = (bl + br + tr + tl) * 0.25 <= 0.0;
            let mut add = |a, b| segments.push((a, b));
            match case_id {
                1 => add(left, bottom),
                2 => add(bottom, right),
                3 => add(left, right),
                4 => add(right, top),
                5 if center_inside => {
                    add(bottom, right);
                    add(top, left);
                }
                5 => {
                    add(left, bottom);
                    add(right, top);
                }
                6 => add(bottom, top),
                7 => add(left, top),
                8 => add(top, left),
                9 => add(top, bottom),
                10 if center_inside => {
                    add(left, bottom);
                    add(right, top);
                }
                10 => {
                    add(bottom, right);
                    add(top, left);
                }
                11 => add(top, right),
                12 => add(right, left),
                13 => add(bottom, right),
                14 => add(left, bottom),
                _ => {}
            }
        }
    }

    let mut adjacency: HashMap<EdgeKey, Vec<EdgeKey>> = HashMap::new();
    for &(a, b) in &segments {
        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);
    }
    let canonical = |a: EdgeKey, b: EdgeKey| if a <= b { (a, b) } else { (b, a) };
    let mut visited: HashSet<(EdgeKey, EdgeKey)> = HashSet::new();
    let mut loops = Vec::new();
    for &(start, first) in &segments {
        if visited.contains(&canonical(start, first)) {
            continue;
        }
        let mut keys = vec![start];
        let mut previous = start;
        let mut current = first;
        visited.insert(canonical(start, first));
        let mut closed = false;
        for _ in 0..segments.len() + 2 {
            if current == start {
                closed = true;
                break;
            }
            keys.push(current);
            let Some(neighbors) = adjacency.get(&current) else {
                break;
            };
            let next = neighbors.iter().copied().find(|candidate| {
                *candidate != previous && !visited.contains(&canonical(current, *candidate))
            });
            let Some(next) = next else {
                break;
            };
            visited.insert(canonical(current, next));
            previous = current;
            current = next;
        }
        if closed && keys.len() >= 3 {
            loops.push(
                keys.into_iter()
                    .map(|key| crossing_point(grid, key, epsilon))
                    .collect(),
            );
        }
    }
    loops
}

fn orient_and_classify(mut loops: Vec<Vec<Point2>>) -> Vec<(Vec<Point2>, bool)> {
    loops.retain(|lp| lp.len() >= 3 && signed_area(lp).abs() > 1e-10);
    let snapshot = loops.clone();
    loops
        .into_iter()
        .enumerate()
        .map(|(index, mut lp)| {
            let probe = lp[0];
            let depth = snapshot
                .iter()
                .enumerate()
                .filter(|(other, candidate)| *other != index && point_in_loop(probe, candidate))
                .count();
            let is_hole = depth % 2 == 1;
            let area = signed_area(&lp);
            if (!is_hole && area < 0.0) || (is_hole && area > 0.0) {
                lp.reverse();
            }
            (lp, is_hole)
        })
        .collect()
}

fn largest_simple_exterior(loops: Vec<Vec<Point2>>) -> Option<Vec<Point2>> {
    orient_and_classify(loops)
        .into_iter()
        .filter(|(lp, hole)| !*hole && !has_self_intersection(lp))
        .max_by(|(left, _), (right, _)| {
            signed_area(left).abs().total_cmp(&signed_area(right).abs())
        })
        .map(|(lp, _)| lp)
}

fn orientation(a: f64) -> Orientation {
    if a >= 0.0 {
        Orientation::CounterClockwise
    } else {
        Orientation::Clockwise
    }
}

fn component_from_loop(id: usize, points: &[Point2], is_hole: bool) -> BoundaryComponent {
    let area = signed_area(points);
    let mut extended = Vec::with_capacity(points.len());
    for i in 0..points.len() {
        let previous = points[(i + points.len() - 1) % points.len()];
        let next = points[(i + 1) % points.len()];
        let tx = next.x - previous.x;
        let ty = next.y - previous.y;
        let length = tx.hypot(ty).max(1e-14);
        let (nx, ny) = if area >= 0.0 {
            (ty / length, -tx / length)
        } else {
            (-ty / length, tx / length)
        };
        extended.push(ExtendedBoundaryPoint {
            x: points[i].x,
            y: points[i].y,
            nx,
            ny,
        });
    }
    BoundaryComponent {
        id,
        points: extended,
        orientation: orientation(area),
        is_hole,
        signed_area: area,
        perimeter: perimeter(points),
        is_simple: !has_self_intersection(points),
    }
}

fn cross(a: Point2, b: Point2, c: Point2) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}
fn segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    let (ab_c, ab_d, cd_a, cd_b) = (
        cross(a, b, c),
        cross(a, b, d),
        cross(c, d, a),
        cross(c, d, b),
    );
    ab_c * ab_d < -1e-12 && cd_a * cd_b < -1e-12
}
fn has_self_intersection(points: &[Point2]) -> bool {
    if points.len() > 6000 {
        return true;
    }
    for i in 0..points.len() {
        for j in i + 1..points.len() {
            if j == i || j == (i + 1) % points.len() || i == (j + 1) % points.len() {
                continue;
            }
            if segments_intersect(
                points[i],
                points[(i + 1) % points.len()],
                points[j],
                points[(j + 1) % points.len()],
            ) {
                return true;
            }
        }
    }
    false
}

fn set_area(classified: &[(Vec<Point2>, bool)]) -> f64 {
    classified
        .iter()
        .map(|(lp, hole)| {
            if *hole {
                -signed_area(lp).abs()
            } else {
                signed_area(lp).abs()
            }
        })
        .sum::<f64>()
        .max(0.0)
}

struct PreparedSeed {
    boundary: Vec<Point2>,
    uncertainty: f64,
}

fn prepare_seed_boundary(
    seed: &[(f64, f64)],
    bounds: Bounds2D,
    grid_width: usize,
    grid_height: usize,
) -> Result<PreparedSeed, String> {
    if seed.len() < 3 {
        return Err("MIS boundary seed needs at least three points".to_string());
    }
    if !bounds.is_valid() {
        return Err("Invalid geometric offset bounds".to_string());
    }
    let mut initial = seed
        .iter()
        .map(|&(x, y)| Point2::new(x, y))
        .collect::<Vec<_>>();
    if initial.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
        return Err("MIS seed contains non-finite points".to_string());
    }
    if initial
        .first()
        .zip(initial.last())
        .is_some_and(|(a, b)| a.distance(*b) < 1e-10)
    {
        initial.pop();
    }
    if signed_area(&initial).abs() < 1e-10 {
        return Err("MIS boundary seed has degenerate area".to_string());
    }

    let mut uncertainty = 0.0;
    if has_self_intersection(&initial) {
        let grid = build_distance_grid(bounds, grid_width, grid_height, &[initial.clone()])?;
        let grid_scale = grid.dx.hypot(grid.dy);
        if let Some(largest) = largest_simple_exterior(marching_squares(&grid, 0.0)) {
            initial = largest;
            uncertainty = 0.5 * grid_scale;
        } else {
            let tube_radius = 0.75 * grid_scale;
            let tube_grid = build_tube_grid(
                bounds,
                grid_width,
                grid_height,
                &[initial.clone()],
                tube_radius,
            )?;
            let Some(largest) = largest_simple_exterior(marching_squares(&tube_grid, 0.0)) else {
                return Err(
                    "MIS boundary seed self-intersects and could not be normalized in the current view"
                        .to_string(),
                );
            };
            initial = largest;
            uncertainty = tube_radius + 0.5 * grid_scale;
        }
    }
    if signed_area(&initial) < 0.0 {
        initial.reverse();
    }

    Ok(PreparedSeed {
        boundary: initial,
        uncertainty,
    })
}

pub fn compute_geometric_offset_contours(
    seed: &[(f64, f64)],
    epsilon: f64,
    num_levels: usize,
    resolution: usize,
    bounds: Bounds2D,
) -> Result<GeometricOffsetResult, String> {
    if !epsilon.is_finite() || epsilon <= 0.0 {
        return Err("Geometric offset distance must be positive and finite".to_string());
    }
    if num_levels == 0 {
        return Err("At least one geometric offset level is required".to_string());
    }
    let resolution = resolution.clamp(32, 512);
    let prepared = prepare_seed_boundary(seed, bounds, resolution, resolution)?;
    let initial = prepared.boundary;
    let seed_area = signed_area(&initial).abs();
    let initial_loops = vec![initial.clone()];
    let grid = build_distance_grid(bounds, resolution, resolution, &initial_loops)?;
    let grid_uncertainty = 0.5 * grid.dx.hypot(grid.dy);
    let x_min = initial.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let x_max = initial
        .iter()
        .map(|p| p.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let y_min = initial.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let y_max = initial
        .iter()
        .map(|p| p.y)
        .fold(f64::NEG_INFINITY, f64::max);

    let mut levels = Vec::with_capacity(num_levels);
    let mut stop_reason = GeometricOffsetStopReason::RequestedLevelsCompleted;
    let mut previous_loops = initial_loops.clone();
    for level in 1..=num_levels {
        let target_distance = level as f64 * epsilon;
        if x_min - target_distance <= bounds.x_min
            || x_max + target_distance >= bounds.x_max
            || y_min - target_distance <= bounds.y_min
            || y_max + target_distance >= bounds.y_max
        {
            stop_reason = GeometricOffsetStopReason::EscapedDomain;
            break;
        }

        // marching_squares extracts phi + offset = 0.  Passing a negative
        // target therefore extracts the exterior signed-distance contour
        // phi = target_distance.
        let classified = orient_and_classify(marching_squares(&grid, -target_distance));
        if classified.is_empty() {
            stop_reason = GeometricOffsetStopReason::ContourExtractionFailed;
            break;
        }
        let loops = classified
            .iter()
            .map(|(lp, _)| lp.clone())
            .collect::<Vec<_>>();
        let offset_residual = loops
            .iter()
            .flatten()
            .map(|p| (signed_distance_to_set(*p, &initial_loops) - target_distance).abs())
            .fold(0.0, f64::max);
        let gap_residual = loops
            .iter()
            .flatten()
            .map(|p| (signed_distance_to_set(*p, &previous_loops) - epsilon).abs())
            .fold(0.0, f64::max);
        let boundary_components = classified
            .iter()
            .enumerate()
            .map(|(id, (lp, is_hole))| component_from_loop(id, lp, *is_hole))
            .collect::<Vec<_>>();
        levels.push(GeometricOffsetLevel {
            level,
            target_distance,
            area: set_area(&classified),
            component_count: boundary_components.iter().filter(|c| !c.is_hole).count(),
            hole_count: boundary_components.iter().filter(|c| c.is_hole).count(),
            boundary_components,
            offset_residual,
            gap_residual,
            uncertainty: prepared.uncertainty + grid_uncertainty,
        });
        previous_loops = loops;
    }

    Ok(GeometricOffsetResult {
        completed_levels: levels.len(),
        levels,
        epsilon,
        seed_area,
        seed_uncertainty: prepared.uncertainty,
        stop_reason,
        bounds,
    })
}

#[wasm_bindgen(js_name = "computeGeometricOffsetContours")]
pub fn compute_geometric_offset_contours_js(
    boundary: JsValue,
    epsilon: f64,
    num_levels: usize,
    resolution: usize,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> Result<JsValue, JsValue> {
    let seed: Vec<(f64, f64)> = serde_wasm_bindgen::from_value(boundary)
        .map_err(|e| JsValue::from_str(&format!("Invalid MIS boundary: {e}")))?;
    let result = compute_geometric_offset_contours(
        &seed,
        epsilon,
        num_levels.clamp(1, 12),
        resolution,
        Bounds2D {
            x_min,
            x_max,
            y_min,
            y_max,
        },
    )
    .map_err(|e| JsValue::from_str(&e))?;
    serde_wasm_bindgen::to_value(&result)
        .map_err(|e| JsValue::from_str(&format!("Failed to serialize offset contours: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn circle(radius: f64, count: usize) -> Vec<(f64, f64)> {
        (0..count)
            .map(|i| {
                let t = std::f64::consts::TAU * i as f64 / count as f64;
                (radius * t.cos(), radius * t.sin())
            })
            .collect()
    }

    #[test]
    fn signed_distance_has_correct_sign() {
        let lp = circle(1.0, 128)
            .into_iter()
            .map(|(x, y)| Point2::new(x, y))
            .collect::<Vec<_>>();
        assert!(signed_distance_to_set(Point2::new(0.0, 0.0), &[lp.clone()]) < -0.99);
        assert!(signed_distance_to_set(Point2::new(2.0, 0.0), &[lp]) > 0.99);
    }

    #[test]
    fn invalid_offset_inputs_fail_fast() {
        let bounds = Bounds2D {
            x_min: -2.0,
            x_max: 2.0,
            y_min: -2.0,
            y_max: 2.0,
        };
        assert!(
            compute_geometric_offset_contours(&[(0.0, 0.0), (1.0, 0.0)], 0.1, 2, 128, bounds)
                .is_err()
        );
        assert!(compute_geometric_offset_contours(&circle(1.0, 64), 0.0, 2, 128, bounds).is_err());
        assert!(compute_geometric_offset_contours(&circle(1.0, 64), 0.1, 0, 128, bounds).is_err());
    }

    #[test]
    fn geometric_offsets_normalize_a_self_intersecting_seed() {
        let bounds = Bounds2D {
            x_min: -2.0,
            x_max: 2.0,
            y_min: -2.0,
            y_max: 2.0,
        };
        let bow_tie = [
            (0.0, 1.0),
            (0.588, -0.809),
            (-0.951, 0.309),
            (0.951, 0.309),
            (-0.588, -0.809),
        ];
        let result = compute_geometric_offset_contours(&bow_tie, 0.1, 1, 128, bounds).unwrap();
        assert_eq!(result.completed_levels, 1);
        assert!(result.seed_uncertainty > 0.0);
        assert!(result.levels[0]
            .boundary_components
            .iter()
            .all(|component| component.is_simple));
    }

    #[test]
    fn reconstructed_boundary_normals_are_unit_and_outward() {
        let points = circle(1.0, 96)
            .into_iter()
            .map(|(x, y)| Point2::new(x, y))
            .collect::<Vec<_>>();
        let component = component_from_loop(0, &points, false);
        for point in component.points {
            assert!((point.nx.hypot(point.ny) - 1.0).abs() < 1e-12);
            assert!(point.x * point.nx + point.y * point.ny > 0.99);
        }
    }

    #[test]
    fn geometric_offsets_follow_requested_signed_distances() {
        let bounds = Bounds2D {
            x_min: -2.0,
            x_max: 2.0,
            y_min: -2.0,
            y_max: 2.0,
        };
        let result =
            compute_geometric_offset_contours(&circle(1.0, 256), 0.1, 3, 256, bounds).unwrap();
        assert_eq!(result.completed_levels, 3);
        assert_eq!(
            result.stop_reason,
            GeometricOffsetStopReason::RequestedLevelsCompleted
        );
        for (index, level) in result.levels.iter().enumerate() {
            let radius = 1.0 + (index + 1) as f64 * 0.1;
            assert!((level.target_distance - (index + 1) as f64 * 0.1).abs() < 1e-12);
            assert!((level.area - std::f64::consts::PI * radius * radius).abs() < 0.03);
            assert!(level.offset_residual <= level.uncertainty);
            assert_eq!(level.component_count, 1);
            assert_eq!(level.hole_count, 0);
        }
    }

    #[test]
    fn consecutive_geometric_boundaries_are_one_epsilon_apart() {
        let bounds = Bounds2D {
            x_min: -2.0,
            x_max: 2.0,
            y_min: -2.0,
            y_max: 2.0,
        };
        let epsilon = 0.1;
        let result =
            compute_geometric_offset_contours(&circle(0.6, 192), epsilon, 3, 256, bounds).unwrap();
        let mut previous = vec![circle(0.6, 192)
            .into_iter()
            .map(|(x, y)| Point2::new(x, y))
            .collect::<Vec<_>>()];
        for level in &result.levels {
            let current = level
                .boundary_components
                .iter()
                .map(|component| {
                    component
                        .points
                        .iter()
                        .map(|p| Point2::new(p.x, p.y))
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>();
            let max_gap_error = current
                .iter()
                .flatten()
                .map(|p| (signed_distance_to_set(*p, &previous) - epsilon).abs())
                .fold(0.0, f64::max);
            assert!(max_gap_error < 0.02, "gap error was {max_gap_error}");
            assert!((level.gap_residual - max_gap_error).abs() < 1e-12);
            assert!(level.gap_residual <= level.uncertainty);
            previous = current;
        }
    }

    #[test]
    fn geometric_offsets_stop_before_leaving_the_view() {
        let bounds = Bounds2D {
            x_min: -1.0,
            x_max: 1.0,
            y_min: -1.0,
            y_max: 1.0,
        };
        let result =
            compute_geometric_offset_contours(&circle(0.8, 128), 0.15, 3, 128, bounds).unwrap();
        assert_eq!(result.completed_levels, 1);
        assert_eq!(result.stop_reason, GeometricOffsetStopReason::EscapedDomain);
    }
}
