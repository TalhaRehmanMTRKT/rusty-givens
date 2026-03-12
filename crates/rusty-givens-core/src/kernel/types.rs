use crate::model::{AcModel, MeasurementSet, PowerSystem};

/// Choice of linear algebra backend for the gain matrix factorization
/// (applicable to Normal Equations and Equality-Constrained formulations).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Factorization {
    /// Dense Cholesky via `faer` — robust but O(n³) per iteration.
    DenseCholesky,
    /// Sparse Cholesky (LLT) via `faer::sparse` — fast for large networks.
    SparseCholesky,
    /// Sparse LU via `faer::sparse` — fallback when gain is not SPD.
    SparseLU,
}

/// WLS solver formulation, corresponding to the alternative methods described
/// in Chapters 2 and 3 of the textbook.
#[derive(Debug, Clone, PartialEq)]
pub enum SolverFormulation {
    /// Normal Equations: G Δx = Hᵀ W Δz, G = Hᵀ W H  (Chapter 2.6).
    /// The gain matrix is factorized by the chosen `Factorization` backend.
    NormalEquations { factorization: Factorization },

    /// Orthogonal (QR) factorization via Givens rotations (Chapter 3.2 / B.8).
    /// Works on H̃ = W^{1/2} H directly — never forms G.
    /// Numerically robust; handles very large weights without ill-conditioning.
    OrthogonalQR,

    /// Peters and Wilkinson method (Chapter 3.4).
    /// Performs LU decomposition of H̃, then solves (LᵀL) Δy = Lᵀ Δz̃.
    /// LᵀL is better conditioned than H̃ᵀH̃.
    PetersWilkinson,

    /// Equality-Constrained WLS via Lagrangian (Chapter 3.5).
    /// Virtual measurements (zero injections) become explicit constraints c(x) = 0.
    /// Optional scaling factor α improves conditioning.
    EqualityConstrained {
        factorization: Factorization,
        alpha: Option<f64>,
    },

    /// Fast Decoupled WLS (Chapter 2.7).
    /// Separates into P-θ and Q-V sub-problems with constant gain sub-matrices.
    /// Does not support branch current magnitude measurements.
    FastDecoupled,

    /// DC State Estimation — linear model (Chapter 2.8).
    /// Single linear solve: z_A = H_AA θ + e_A. Active power only.
    DcEstimation,
}

/// Configuration for the WLS solver.
#[derive(Debug, Clone)]
pub struct EstimationConfig {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub formulation: SolverFormulation,
}

impl Default for EstimationConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance: 1e-4,
            formulation: SolverFormulation::NormalEquations {
                factorization: Factorization::SparseCholesky,
            },
        }
    }
}

impl EstimationConfig {
    /// Convenience: extract the NE factorization backend (panics for non-NE formulations).
    pub(crate) fn ne_factorization(&self) -> Factorization {
        match &self.formulation {
            SolverFormulation::NormalEquations { factorization } => *factorization,
            SolverFormulation::EqualityConstrained { factorization, .. } => *factorization,
            _ => Factorization::SparseCholesky,
        }
    }
}

/// Diagnostic data collected for each Gauss-Newton iteration.
#[derive(Debug, Clone)]
pub struct IterationDiagnostic {
    pub iteration: usize,
    pub max_delta_x: f64,
    pub jacobian_time_s: f64,
    pub gain_time_s: f64,
    pub solve_time_s: f64,
    pub total_time_s: f64,
}

/// Which measurement an equation row corresponds to.
#[derive(Debug, Clone)]
pub enum MeasurementRef {
    Voltmeter { label: String },
    Ammeter { label: String },
    Wattmeter { label: String },
    Varmeter { label: String },
    PmuMagnitude { label: String },
    PmuAngle { label: String },
    CurrentAngle { label: String },
}

/// Artifacts from the converged WLS solution needed by post-estimation
/// analysis (bad data detection, observability, etc.).
#[derive(Debug, Clone)]
pub struct SolverArtifacts {
    /// Jacobian H at the converged state (k × s CSC).
    pub jacobian: sprs::CsMat<f64>,
    /// Measurement residuals r = z − h(x̂), length k.
    pub residuals: Vec<f64>,
    /// Measurement values z, length k.
    pub measurement_z: Vec<f64>,
    /// Computed h(x̂), length k.
    pub measurement_h: Vec<f64>,
    /// Diagonal precision entries W_ii = 1/σ²_i, length k.
    pub precision_diag: Vec<f64>,
    /// Off-diagonal precision entries (row1, row2, value) for correlated PMUs.
    pub precision_off: Vec<(usize, usize, f64)>,
    /// Gain matrix G = Jᵀ W J at the converged state (s × s CSC).
    pub gain_matrix: sprs::CsMat<f64>,
    /// Number of state variables s = 2n.
    pub n_states: usize,
    /// Slack bus index.
    pub slack_index: usize,
    /// Map from equation row index → measurement identity.
    pub measurement_map: Vec<MeasurementRef>,
}

/// Result of the AC state estimation.
#[derive(Debug, Clone)]
pub struct EstimationResult {
    pub voltage_magnitude: Vec<f64>,
    pub voltage_angle: Vec<f64>,
    pub converged: bool,
    pub iterations: usize,
    pub final_increment: f64,
    pub diagnostics: Vec<IterationDiagnostic>,
    /// Solver artifacts for post-estimation analysis.
    pub artifacts: Option<SolverArtifacts>,
}

/// Errors that can occur during state estimation.
#[derive(Debug, thiserror::Error)]
pub enum SolverError {
    #[error("factorization failed at iteration {iteration}: {detail}")]
    FactorizationFailed { iteration: usize, detail: String },

    #[error("numerical instability at iteration {iteration}: {detail}")]
    NumericalInstability { iteration: usize, detail: String },

    #[error("invalid input: {0}")]
    InvalidInput(String),
}

/// Trait that any state estimation solver backend must implement.
///
/// This is the core abstraction that allows plugging different SE backends
/// (Rust WLS, pandapower, etc.) behind a common interface.
pub trait SeSolver {
    fn estimate(
        &self,
        system: &PowerSystem,
        model: &AcModel,
        measurements: &MeasurementSet,
        config: &EstimationConfig,
    ) -> Result<EstimationResult, SolverError>;
}
