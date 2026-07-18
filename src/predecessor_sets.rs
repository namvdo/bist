use serde::{Deserialize, Serialize};


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
        self.x_min.is_finite() &&
        self.x_max.is_finite() &&
        self.y_min.is_finite() && 
        self.y_max.is_finite() && 
        self.x_min < self.x_max && 
        self.y_min < self.y_max

        
    }


    pub fn contains(&self, x: f64, y: f64) -> bool {
        x >= self.x_min && x <= self.x_max 
        && y >= self.y_min && y <= self.y_max
    }

}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredecessorConfig {
    pub num_levels: usize,
    pub grid_width: usize,
    pub grid_height: usize,
    pub max_segment_length: f64,
    pub curve_tolerance: f64,
    pub nesting_tolerance: f64,
    pub closure_tolerance: f64,
    // dist(f(x), complement(M_k) = epsilon)
    pub predecessor_tolerance: f64, 
    pub max_subdivision_depth: usize,
    pub max_points_per_component: usize,
    pub require_nestedness: bool, 
    // whether to stop when the number of components or holes changed
    pub stop_on_topology_change: bool 
}


impl Default for PredecessorConfig {
    fn default() -> Self {
        Self {
            num_levels: 5,
            grid_width: 512,
            grid_height: 512,
            max_segment_length: 0.01,
            curve_tolerance: 1e-4,
            closure_tolerance: 1e-6,
            nesting_tolerance: 1e-6,
            predecessor_tolerance: 1e-3,
            max_subdivision_depth: 16,
            max_points_per_component: 200_000,
            require_nestedness: true,
            stop_on_topology_change: false
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Orientation {
    CounterClockwise,
    Clockwise
}

impl Orientation {
    pub fn from_signed_area(signed_area: f64) -> Self {
        if signed_area >= 0.0 {
            Self::CounterClockwise
        } else {
            Self::Clockwise
        }
    }
}

#[derive(Clone, Debug, Copy, PartialEq, Serialize, Deserialize)]
pub struct ExtendedBoundaryPoint {
    pub x: f64,
    pub y: f64,
    pub nx: f64,
    pub ny: f64
}

impl ExtendedBoundaryPoint {
    pub fn new(x: f64, y: f64, nx: f64, ny: f64) -> Result<Self, String> {
        let values = [x, y, nx, ny];
        if values.iter().any(|value| !value.is_finite()) {
            return Err("Boundary point and normal must be finite".to_string());
        }
        let normal_norm = nx.hypot(ny);
        if normal_norm < 1e-12 {
            return Err("Boundary normal cannot be zero".to_string());
        }

        Ok(Self {
            x, 
            y,
            nx / normal_norm,
            ny / normal_norm
        })
    }

    pub fn position(&self) -> (f64, f64) {
        (self.x, self.y)
    }

    pub fn normal(&self) -> (f64, f64) {
        (self.nx, self.ny)
    }

    pub fn normal_norm(&self) -> f64 {
        self.nx.hypot(self.ny)  
    }
}


pub struct BoundarySeedComponent {
    pub points: Vec<ExtendedBoundaryPoint>,
    pub is_hole: bool,
}

pub struct PredecessorInput {
    pub boundary_components: Vec<BoundarySeedComponent>,
    pub bounds: Bounds2D
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundaryComponent {
    pub id: usize,
    pub points: Vec<ExtendedBoundaryPoint>,
    pub orientation: Orientation,
    pub is_hole: bool,
    pub signed_area: f64,
    pub perimeter: f64,
    pub closure_error: f64,
    pub is_simple: bool
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationIssue {
    pub severity: ValidationSeverity,
    pub code: ValidationCode,
    pub message: String,

    pub component_id: Option<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ValidationSeverity {
    Warning,
    Error,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ValidationCode {
    InvalidBounds,
    EmptyBoundary,
    TooFewPoints,
    NonFinitePoint,
    DegenerateNormal,
    ClosureFailure,
    SelfIntersection,
    DegenerateArea,
    RefinementLimit,
    PointLimit,
    TopologyChanged,
    NestingFailure,
    PredecessorResidualTooLarge,
    StateOutsideBounds
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LevelDiagnostics {
    pub nesting_residual: f64,
    pub predecessor_residual: f64,
    pub uncertainty: f64,
    pub max_closure_error: f64,
    pub max_segment_length: f64,
    pub nested: bool,
    pub boundar_valid: bool, 
    pub issues: Vec<ValidationIssue>,
}

impl Default for LevelDiagnostics {
    fn default() -> Self {
        Self {
            nesting_residual: 0.0,
            predecessor_residual: 0.0,
            uncertainty: 0.0,
            max_closure_error: 0.0,
            max_segment_length: 0.0,
            nested: true,
            boundar_valid: true,
            issues: Vec::new()
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredecessorLevel {
    pub level: usize,
    pub boundary_components: Vec<BoundaryComponent>,
    pub erosion_components: Vec<BoundaryComponent>,
    pub area: f64,
    pub eroded_area: Option<f64>,
    pub component_count: usize,
    pub hole_count: usize,
    pub diagnostics: LevelDiagnostics,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PredecessorStopReason {
    RequestedLevelsCompleted,
    FixedPointReached,
    EmptyErosion,
    TopologyChanged,
    ValidationFailed,
    RefinementLimit,
    PointLimit,
    EscapedDomain,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredecessorResult {
    pub levels: Vec<PredecessorLevel>,
    pub completed_levels: usize,
    pub stop_reason: PredecessorStopReason,
    pub config: PredecessorConfig,
    pub bounds: Bounds2D
}