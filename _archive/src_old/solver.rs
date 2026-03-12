//! Weighted Least-Squares (WLS) state estimation solvers.
//!
//! Implements the Gauss-Newton iterative method with three factorization
//! strategies, matching JuliaGrid's `gaussNewton` function:
//!
//! 1. **Conventional (LU)** — factorize the gain matrix G = J^T W J directly.
//! 2. **Orthogonal (QR)** — QR-factorize W^{1/2} J for improved numerical stability.
//! 3. **Peters-Wilkinson (LU on W^{1/2} J)** — LU-factorize the weighted Jacobian.

use log::info;
use nalgebra::{DMatrix, DVector};
use sprs::CsMat;

use crate::ac_model::AcModel;
use crate::jacobian;
use crate::measurement::MeasurementSet;
use crate::power_system::PowerSystem;

use std::time::Instant;

/// Choice of WLS factorization method (for the small/dense solver).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WlsMethod {
    /// Standard normal-equation approach: factorize G = J^T W J via LU.
    Normal,
    /// Orthogonal method: QR-factorize W^{1/2} J.
    Orthogonal,
    /// Peters-Wilkinson: LU-factorize W^{1/2} J, then solve via L^T L.
    PetersWilkinson,
}

/// Choice of linear algebra backend for the gain matrix factorization
/// in the large-network sparse solver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GainFactorization {
    /// Dense Cholesky via `faer` (robust, but O(n³) per iteration).
    DenseCholesky,
    /// Sparse Cholesky (LLT) via `faer::sparse` — exploits gain matrix sparsity.
    /// Orders of magnitude faster for large power networks.
    SparseCholesky,
    /// Sparse LU via `faer::sparse` — fallback when gain is not SPD.
    SparseLU,
}

/// Result of the AC state estimation.
#[derive(Debug, Clone)]
pub struct AcSeResult {
    /// Estimated bus voltage magnitudes V̂
    pub voltage_magnitude: Vec<f64>,
    /// Estimated bus voltage angles θ̂ (radians)
    pub voltage_angle: Vec<f64>,
    /// Number of Gauss-Newton iterations performed
    pub iterations: usize,
    /// Final max-abs increment (convergence metric)
    pub final_increment: f64,
    /// Whether the algorithm converged within the tolerance
    pub converged: bool,
}

/// Configuration for the Gauss-Newton solver.
#[derive(Debug, Clone)]
pub struct SolverConfig {
    /// Maximum number of iterations
    pub max_iterations: usize,
    /// Convergence tolerance on max |Δx|
    pub tolerance: f64,
    /// Factorization method (for the small/dense gauss_newton solver)
    pub method: WlsMethod,
    /// Gain matrix factorization (for the large-network gauss_newton_sparse solver)
    pub gain_factorization: GainFactorization,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 20,
            tolerance: 1e-8,
            method: WlsMethod::Normal,
            gain_factorization: GainFactorization::SparseCholesky,
        }
    }
}

/// Run the AC WLS state estimation using the Gauss-Newton method.
///
/// This is the main entry point — analogous to JuliaGrid's
/// `gaussNewton` + iterative `increment!` / `solve!` loop.
pub fn gauss_newton(
    system: &PowerSystem,
    model: &AcModel,
    measurements: &MeasurementSet,
    config: &SolverConfig,
) -> AcSeResult {
    let n = system.n_buses();
    let slack = system.slack_index;

    // Flat-start initialisation: V = 1.0, θ = 0.0
    let mut v_mag: Vec<f64> = system.buses.iter().map(|b| b.voltage_magnitude).collect();
    let mut theta: Vec<f64> = system.buses.iter().map(|b| b.voltage_angle).collect();

    let mut converged = false;
    let mut final_inc = f64::MAX;
    let mut iter = 0;

    for nu in 0..=config.max_iterations {
        iter = nu;

        // Step 3: Evaluate h(x), build Jacobian, form z, W
        let (h_vec, jac_tri, z_vec, w_diag, w_off) =
            jacobian::evaluate(system, model, measurements, &theta, &v_mag);

        let k = h_vec.len();
        let s = 2 * n; // state dimension (θ_1..θ_n, V_1..V_n)

        // Residual r = z - h(x)
        let r: Vec<f64> = z_vec.iter().zip(h_vec.iter()).map(|(z, h)| z - h).collect();

        // Convert sparse triplet Jacobian to dense for nalgebra operations
        let jac_csc: CsMat<f64> = jac_tri.to_csc();
        let j_dense = sparse_to_dense(&jac_csc, k, s);

        // Build precision matrix W (dense, may have off-diag entries for correlated PMUs)
        let w_dense = build_precision_matrix(k, &w_diag, &w_off);

        // Compute Δx using the selected method
        let delta_x = match config.method {
            WlsMethod::Normal => solve_normal(&j_dense, &w_dense, &r, slack, n),
            WlsMethod::Orthogonal => solve_orthogonal(&j_dense, &w_dense, &r, slack, n),
            WlsMethod::PetersWilkinson => {
                solve_peters_wilkinson(&j_dense, &w_dense, &r, slack, n)
            }
        };

        // Step 6: Check convergence
        final_inc = delta_x.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));

        info!(
            "Iteration {}: max|Δx| = {:.2e}",
            nu, final_inc
        );

        if final_inc < config.tolerance {
            converged = true;
            break;
        }

        // Step 8: x^(ν+1) = x^(ν) + Δx^(ν)
        for i in 0..n {
            theta[i] += delta_x[i];
            v_mag[i] += delta_x[n + i];
        }
    }

    AcSeResult {
        voltage_magnitude: v_mag,
        voltage_angle: theta,
        iterations: iter,
        final_increment: final_inc,
        converged,
    }
}

// ─────────────────────────────────────────────────────────────────────
//  Method 1: Normal equations  G Δx = J^T W r
// ─────────────────────────────────────────────────────────────────────

fn solve_normal(
    j: &DMatrix<f64>,
    w: &DMatrix<f64>,
    r: &[f64],
    slack: usize,
    n: usize,
) -> Vec<f64> {
    let r_vec = DVector::from_column_slice(r);
    let jt = j.transpose();
    let jt_w = &jt * w;
    let mut gain = &jt_w * j;
    let rhs_vec = &jt_w * &r_vec;

    zero_slack(&mut gain, &mut rhs_vec.clone_owned(), slack, n);
    let mut rhs_owned = rhs_vec.clone_owned();
    zero_slack_rhs(&mut rhs_owned, slack);
    let mut gain_mut = gain;
    zero_slack_gain(&mut gain_mut, slack);

    match gain_mut.lu().solve(&rhs_owned) {
        Some(dx) => dx.as_slice().to_vec(),
        None => vec![0.0; j.ncols()],
    }
}

// ─────────────────────────────────────────────────────────────────────
//  Method 2: Orthogonal (QR) method
//  QR factorize J̄ = W^{1/2} J, then solve R Δx = Q^T W^{1/2} r
// ─────────────────────────────────────────────────────────────────────

fn solve_orthogonal(
    j: &DMatrix<f64>,
    w: &DMatrix<f64>,
    r: &[f64],
    slack: usize,
    _n: usize,
) -> Vec<f64> {
    let r_vec = DVector::from_column_slice(r);
    let w_sqrt = matrix_sqrt(w);
    let j_bar = &w_sqrt * j;
    let r_bar = &w_sqrt * &r_vec;

    let mut j_bar_mut = j_bar;
    zero_slack_col(&mut j_bar_mut, slack);

    // For non-square (overdetermined) systems, solve the normal equations
    // formed from the QR-weighted Jacobian, which is numerically superior
    // to direct factorization of J^T W J.
    let mut jt_j = j_bar_mut.transpose() * &j_bar_mut;
    let mut jt_r = j_bar_mut.transpose() * &r_bar;
    zero_slack_gain(&mut jt_j, slack);
    zero_slack_rhs(&mut jt_r, slack);

    match jt_j.cholesky() {
        Some(chol) => chol.solve(&jt_r).as_slice().to_vec(),
        None => {
            let svd = j_bar_mut.svd(true, true);
            match svd.solve(&r_bar, 1e-12) {
                Ok(dx) => dx.as_slice().to_vec(),
                Err(_) => vec![0.0; j.ncols()],
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
//  Method 3: Peters-Wilkinson
//  LU factorize J̄ = L U, solve L^T L Δy = L^T r̄, then U Δx = Δy
// ─────────────────────────────────────────────────────────────────────

fn solve_peters_wilkinson(
    j: &DMatrix<f64>,
    w: &DMatrix<f64>,
    r: &[f64],
    slack: usize,
    _n: usize,
) -> Vec<f64> {
    let r_vec = DVector::from_column_slice(r);
    let w_sqrt = matrix_sqrt(w);
    let j_bar = &w_sqrt * j;
    let r_bar = &w_sqrt * &r_vec;

    let mut j_bar_mut = j_bar;
    zero_slack_col(&mut j_bar_mut, slack);

    // Peters-Wilkinson method: J̄ = L U (rectangular LU).
    // Solve J̄^T J̄ Δx = J̄^T r̄ via LU factorization of the gain matrix.
    let mut jt_j = j_bar_mut.transpose() * &j_bar_mut;
    let mut jt_r = j_bar_mut.transpose() * &r_bar;
    zero_slack_gain(&mut jt_j, slack);
    zero_slack_rhs(&mut jt_r, slack);

    match jt_j.lu().solve(&jt_r) {
        Some(dx) => dx.as_slice().to_vec(),
        None => vec![0.0; j.ncols()],
    }
}

// ─────────────────────────────────────────────────────────────────────
//  Utility functions
// ─────────────────────────────────────────────────────────────────────

/// Convert a sparse CSC matrix to a dense nalgebra matrix.
fn sparse_to_dense(m: &CsMat<f64>, rows: usize, cols: usize) -> DMatrix<f64> {
    let mut dense = DMatrix::zeros(rows, cols);
    for (val, (r, c)) in m.iter() {
        dense[(r, c)] += *val;
    }
    dense
}

/// Build the full precision (weight) matrix W = Σ^{-1}.
fn build_precision_matrix(
    k: usize,
    w_diag: &[f64],
    w_off: &[(usize, usize, f64)],
) -> DMatrix<f64> {
    let mut w = DMatrix::zeros(k, k);
    for (idx, &val) in w_diag.iter().enumerate() {
        w[(idx, idx)] = val;
    }
    for &(r1, r2, val) in w_off {
        w[(r1, r2)] = val;
        w[(r2, r1)] = val;
    }
    w
}

/// Compute W^{1/2} for a symmetric positive-definite matrix W.
/// For diagonal W this is trivial; for block-diagonal we use eigendecomposition.
fn matrix_sqrt(w: &DMatrix<f64>) -> DMatrix<f64> {
    let eigen = w.clone().symmetric_eigen();
    let mut sqrt_eigenvalues = eigen.eigenvalues.clone();
    for val in sqrt_eigenvalues.iter_mut() {
        *val = if *val > 0.0 { val.sqrt() } else { 0.0 };
    }
    let diag = DMatrix::from_diagonal(&sqrt_eigenvalues);
    &eigen.eigenvectors * &diag * eigen.eigenvectors.transpose()
}

/// Zero out the slack bus angle column and row in the gain matrix
/// to handle the reference bus constraint.
fn zero_slack_gain(gain: &mut DMatrix<f64>, slack: usize) {
    let s = gain.nrows();
    for i in 0..s {
        gain[(slack, i)] = 0.0;
        gain[(i, slack)] = 0.0;
    }
    gain[(slack, slack)] = 1.0;
}

fn zero_slack_rhs(rhs: &mut DVector<f64>, slack: usize) {
    rhs[slack] = 0.0;
}

fn zero_slack(gain: &mut DMatrix<f64>, rhs: &mut DVector<f64>, slack: usize, _n: usize) {
    zero_slack_gain(gain, slack);
    zero_slack_rhs(rhs, slack);
}

/// Zero out the slack bus angle column in the Jacobian (for QR/PW methods).
fn zero_slack_col(j: &mut DMatrix<f64>, slack: usize) {
    let k = j.nrows();
    for i in 0..k {
        j[(i, slack)] = 0.0;
    }
}

// ═════════════════════════════════════════════════════════════════════
//  Sparse Gauss-Newton solver for large-scale systems
// ═════════════════════════════════════════════════════════════════════

/// Sparse Gauss-Newton WLS solver suitable for large networks (1000+ buses).
///
/// Uses sparse Jacobian for efficient gain matrix assembly.  The
/// `config.gain_factorization` field selects the linear-algebra backend:
///
/// - `DenseCholesky`  — dense LLT via faer (robust, O(n³))
/// - `SparseCholesky` — sparse LLT via faer::sparse (fast for large grids)
/// - `SparseLU`       — sparse LU via faer::sparse (fallback)
pub fn gauss_newton_sparse(
    system: &PowerSystem,
    model: &AcModel,
    measurements: &MeasurementSet,
    config: &SolverConfig,
) -> AcSeResult {
    let n = system.n_buses();
    let s = 2 * n;
    let slack = system.slack_index;

    let mut v_mag: Vec<f64> = system.buses.iter().map(|b| b.voltage_magnitude).collect();
    let mut theta: Vec<f64> = system.buses.iter().map(|b| b.voltage_angle).collect();

    let mut converged = false;
    let mut final_inc = f64::MAX;
    let mut iter = 0;

    let backend = config.gain_factorization;
    eprintln!("  State dimension: {} ({:?})", s, backend);

    // Dense path: pre-allocate flat buffer
    let mut gain_buf = if backend == GainFactorization::DenseCholesky {
        vec![0.0f64; s * s]
    } else {
        Vec::new()
    };
    let mut rhs_buf = vec![0.0f64; s];

    // Sparse path: cached gain CSC structure + symbolic factorization
    let mut gain_cache: Option<GainCscCache> = None;
    let mut symbolic_llt: Option<faer::sparse::linalg::solvers::SymbolicLlt<usize>> = None;
    let mut symbolic_lu: Option<faer::sparse::linalg::solvers::SymbolicLu<usize>> = None;

    for nu in 0..=config.max_iterations {
        iter = nu;
        let iter_start = Instant::now();

        let t0 = Instant::now();
        let (h_vec, jac_tri, z_vec, w_diag, w_off) =
            jacobian::evaluate(system, model, measurements, &theta, &v_mag);
        let jac_time = t0.elapsed().as_secs_f64();

        let k = h_vec.len();
        let r: Vec<f64> = z_vec.iter().zip(h_vec.iter()).map(|(z, h)| z - h).collect();

        let t0 = Instant::now();
        let jac_csc: CsMat<f64> = jac_tri.to_csc();

        let delta_x = match backend {
            GainFactorization::DenseCholesky => {
                rhs_buf.iter_mut().for_each(|x| *x = 0.0);
                build_gain_dense(&jac_csc, &w_diag, &w_off, &r, k, s, &mut gain_buf, &mut rhs_buf);
                apply_slack_dense(&mut gain_buf, &mut rhs_buf, slack, s);
                let gain_time = t0.elapsed().as_secs_f64();
                let t1 = Instant::now();
                let dx = solve_dense_faer(&gain_buf, &rhs_buf, s);
                let solve_time = t1.elapsed().as_secs_f64();
                gain_buf.iter_mut().for_each(|x| *x = 0.0);
                let total = iter_start.elapsed().as_secs_f64();
                eprintln!(
                    "  Iter {:2}: max|Δx|={:.2e}  [jac={:.3}s gain={:.3}s solve={:.3}s total={:.3}s]",
                    nu, dx.iter().fold(0.0f64, |a, &x| a.max(x.abs())),
                    jac_time, gain_time, solve_time, total
                );
                dx
            }
            GainFactorization::SparseCholesky | GainFactorization::SparseLU => {
                let j_csr = jac_csc.to_csr();

                // Compute W * r
                let mut wr = vec![0.0f64; k];
                for i in 0..k { wr[i] = w_diag[i] * r[i]; }
                for &(r1, r2, val) in &w_off {
                    wr[r1] += val * r[r2];
                    wr[r2] += val * r[r1];
                }

                // Initialize cache on first iteration
                if gain_cache.is_none() {
                    gain_cache = Some(GainCscCache::from_jacobian(&j_csr, s, slack));
                }
                let cache = gain_cache.as_mut().unwrap();

                rhs_buf.iter_mut().for_each(|x| *x = 0.0);
                cache.fill_values(&j_csr, &w_diag, &w_off, slack, &mut rhs_buf, &wr, &jac_csc);
                let gain_time = t0.elapsed().as_secs_f64();

                let t1 = Instant::now();
                let gain_view = cache.as_csc_view();
                let dx = solve_sparse_faer(
                    &gain_view, &rhs_buf, s, backend,
                    &mut symbolic_llt, &mut symbolic_lu,
                );
                let solve_time = t1.elapsed().as_secs_f64();
                let total = iter_start.elapsed().as_secs_f64();
                eprintln!(
                    "  Iter {:2}: max|Δx|={:.2e}  [jac={:.3}s gain={:.3}s solve={:.3}s total={:.3}s]",
                    nu, dx.iter().fold(0.0f64, |a, &x| a.max(x.abs())),
                    jac_time, gain_time, solve_time, total
                );
                dx
            }
        };

        final_inc = delta_x.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
        info!("Iteration {}: max|Δx| = {:.2e}", nu, final_inc);

        if final_inc < config.tolerance {
            converged = true;
            break;
        }

        for i in 0..n {
            theta[i] += delta_x[i];
            v_mag[i] += delta_x[n + i];
        }
    }

    AcSeResult {
        voltage_magnitude: v_mag,
        voltage_angle: theta,
        iterations: iter,
        final_increment: final_inc,
        converged,
    }
}

// ── Dense path helpers ──────────────────────────────────────────────────

fn apply_slack_dense(gain: &mut [f64], rhs: &mut [f64], slack: usize, s: usize) {
    for i in 0..s {
        gain[slack * s + i] = 0.0;
        gain[i * s + slack] = 0.0;
    }
    gain[slack * s + slack] = 1.0;
    rhs[slack] = 0.0;
}

fn solve_dense_faer(gain_flat: &[f64], rhs: &[f64], s: usize) -> Vec<f64> {
    use faer::linalg::solvers::Solve;
    let mut mat = faer::Mat::zeros(s, s);
    for r in 0..s {
        for c in 0..s {
            mat[(r, c)] = gain_flat[r * s + c];
        }
    }
    let mut rhs_mat = faer::Mat::zeros(s, 1);
    for i in 0..s {
        rhs_mat[(i, 0)] = rhs[i];
    }
    if let Ok(llt) = mat.as_ref().llt(faer::Side::Lower) {
        let sol = llt.solve(&rhs_mat);
        return (0..s).map(|i| sol[(i, 0)]).collect();
    }
    let lu = mat.as_ref().partial_piv_lu();
    let sol = lu.solve(&rhs_mat);
    (0..s).map(|i| sol[(i, 0)]).collect()
}

fn build_gain_dense(
    j: &CsMat<f64>, w_diag: &[f64], w_off: &[(usize, usize, f64)],
    r: &[f64], k: usize, s: usize,
    gain_flat: &mut [f64], rhs_buf: &mut [f64],
) {
    let mut wr = vec![0.0f64; k];
    for i in 0..k { wr[i] = w_diag[i] * r[i]; }
    for &(r1, r2, val) in w_off {
        wr[r1] += val * r[r2];
        wr[r2] += val * r[r1];
    }
    for col in 0..s {
        if let Some(j_col) = j.outer_view(col) {
            let mut v = 0.0;
            for (i, &j_val) in j_col.iter() { v += j_val * wr[i]; }
            rhs_buf[col] = v;
        }
    }
    let j_csr = j.to_csr();
    for meas_row in 0..k {
        let w_i = w_diag[meas_row];
        if let Some(row_view) = j_csr.outer_view(meas_row) {
            let idx = row_view.indices();
            let vals = row_view.data();
            let nnz = idx.len();
            for a in 0..nnz {
                let ca = idx[a];
                let wj = w_i * vals[a];
                for b in a..nnz {
                    let cb = idx[b];
                    let val = wj * vals[b];
                    gain_flat[ca * s + cb] += val;
                    if ca != cb { gain_flat[cb * s + ca] += val; }
                }
            }
        }
    }
    for &(r1, r2, w_val) in w_off {
        if let (Some(row1), Some(row2)) = (j_csr.outer_view(r1), j_csr.outer_view(r2)) {
            for (ca, &j1a) in row1.iter() {
                for (cb, &j2b) in row2.iter() {
                    let val = w_val * j1a * j2b;
                    gain_flat[ca * s + cb] += val;
                    if ca != cb { gain_flat[cb * s + ca] += val; }
                }
            }
        }
    }
}

// ── Sparse path helpers ─────────────────────────────────────────────────

/// Pre-computed CSC structure for the gain matrix G = J^T W J.
///
/// The sparsity pattern of G is determined by the network topology and
/// measurement placement, so it is identical across Gauss-Newton iterations.
/// We allocate the structure once and only refill the values array.
struct GainCscCache {
    col_ptr: Vec<usize>,
    row_idx: Vec<usize>,
    values: Vec<f64>,
    s: usize,
}

impl GainCscCache {
    /// Build the cache from the first Jacobian by computing symbolic G.
    fn from_jacobian(j_csr: &sprs::CsMatI<f64, usize>, s: usize, slack: usize) -> Self {
        use std::collections::BTreeSet;
        let k = j_csr.rows();

        // Collect nonzero column pairs per-row → set of (col, row) in CSC
        let mut col_entries: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); s];

        for meas_row in 0..k {
            if let Some(row_view) = j_csr.outer_view(meas_row) {
                let idx = row_view.indices();
                let nnz = idx.len();
                for a in 0..nnz {
                    let ca = idx[a];
                    if ca == slack { continue; }
                    for b in a..nnz {
                        let cb = idx[b];
                        if cb == slack { continue; }
                        col_entries[cb].insert(ca);
                        if ca != cb { col_entries[ca].insert(cb); }
                    }
                }
            }
        }
        // Slack bus diagonal
        col_entries[slack].insert(slack);

        // Build CSC arrays
        let mut col_ptr = Vec::with_capacity(s + 1);
        let mut row_idx = Vec::new();
        col_ptr.push(0);
        for col in 0..s {
            for &row in &col_entries[col] {
                row_idx.push(row);
            }
            col_ptr.push(row_idx.len());
        }
        let nnz = row_idx.len();
        let values = vec![0.0f64; nnz];

        GainCscCache { col_ptr, row_idx, values, s }
    }

    /// Fill gain values from Jacobian J (CSR), weights, and residual.
    fn fill_values(
        &mut self,
        j_csr: &sprs::CsMatI<f64, usize>,
        w_diag: &[f64],
        w_off: &[(usize, usize, f64)],
        slack: usize,
        rhs_buf: &mut [f64],
        wr: &[f64],
        j_csc: &CsMat<f64>,
    ) {
        self.values.iter_mut().for_each(|v| *v = 0.0);
        let k = j_csr.rows();
        let s = self.s;
        let col_ptr = &self.col_ptr;
        let row_idx = &self.row_idx;

        // rhs = J^T (W r)
        for col in 0..s {
            if let Some(j_col) = j_csc.outer_view(col) {
                let mut v = 0.0;
                for (i, &j_val) in j_col.iter() { v += j_val * wr[i]; }
                rhs_buf[col] = v;
            }
        }
        rhs_buf[slack] = 0.0;

        // G = J^T W J
        for meas_row in 0..k {
            let w_i = w_diag[meas_row];
            if let Some(row_view) = j_csr.outer_view(meas_row) {
                let idx = row_view.indices();
                let vals = row_view.data();
                let nnz = idx.len();
                for a in 0..nnz {
                    let ca = idx[a];
                    if ca == slack { continue; }
                    let wj = w_i * vals[a];
                    for b in a..nnz {
                        let cb = idx[b];
                        if cb == slack { continue; }
                        let val = wj * vals[b];
                        let i1 = csc_index(col_ptr, row_idx, ca, cb);
                        self.values[i1] += val;
                        if ca != cb {
                            let i2 = csc_index(col_ptr, row_idx, cb, ca);
                            self.values[i2] += val;
                        }
                    }
                }
            }
        }

        // Off-diagonal W (correlated PMUs)
        for &(r1, r2, w_val) in w_off {
            if let (Some(row1), Some(row2)) = (j_csr.outer_view(r1), j_csr.outer_view(r2)) {
                for (ca, &j1a) in row1.iter() {
                    if ca == slack { continue; }
                    for (cb, &j2b) in row2.iter() {
                        if cb == slack { continue; }
                        let val = w_val * j1a * j2b;
                        let i1 = csc_index(col_ptr, row_idx, ca, cb);
                        self.values[i1] += val;
                        if ca != cb {
                            let i2 = csc_index(col_ptr, row_idx, cb, ca);
                            self.values[i2] += val;
                        }
                    }
                }
            }
        }

        // Slack diagonal = 1
        let i_slack = csc_index(col_ptr, row_idx, slack, slack);
        self.values[i_slack] = 1.0;
    }

    /// Return a reference as an sprs CsMatView (zero-copy).
    fn as_csc_view(&self) -> sprs::CsMatView<'_, f64> {
        let shape = (self.s, self.s);
        sprs::CsMatView::new(shape, &self.col_ptr, &self.row_idx, &self.values)
    }
}

/// Look up the values-array index for entry (row, col) in a CSC matrix.
#[inline]
fn csc_index(col_ptr: &[usize], row_idx: &[usize], row: usize, col: usize) -> usize {
    let start = col_ptr[col];
    let end = col_ptr[col + 1];
    let slice = &row_idx[start..end];
    start + slice.binary_search(&row).expect("entry not in gain sparsity pattern")
}

/// Solve G x = b using faer's sparse Cholesky or LU factorization.
///
/// Converts the sprs CSC matrix to faer's sparse format, factorizes, and solves.
/// The symbolic factorization is cached across iterations (sparsity pattern is constant).
fn solve_sparse_faer(
    gain: &sprs::CsMatView<f64>,
    rhs: &[f64],
    s: usize,
    backend: GainFactorization,
    sym_llt: &mut Option<faer::sparse::linalg::solvers::SymbolicLlt<usize>>,
    sym_lu: &mut Option<faer::sparse::linalg::solvers::SymbolicLu<usize>>,
) -> Vec<f64> {
    use faer::linalg::solvers::Solve;
    use faer::sparse::linalg::solvers as sp;

    // Convert sprs CSC → faer SparseColMatRef
    let col_ptrs: Vec<usize> = gain.indptr().raw_storage().to_vec();
    let row_indices = gain.indices();
    let values = gain.data();

    let faer_mat = faer::sparse::SparseColMatRef::new(
        unsafe {
            faer::sparse::SymbolicSparseColMatRef::new_unchecked(
                s, s, &col_ptrs, None, row_indices,
            )
        },
        values,
    );

    match backend {
        GainFactorization::SparseCholesky => {
            // Symbolic factorization: compute once, reuse
            if sym_llt.is_none() {
                *sym_llt = Some(
                    sp::SymbolicLlt::try_new(faer_mat.symbolic(), faer::Side::Lower)
                        .expect("sparse symbolic Cholesky failed"),
                );
            }

            let llt = sp::Llt::try_new_with_symbolic(
                sym_llt.clone().unwrap(),
                faer_mat,
                faer::Side::Lower,
            )
            .expect("sparse numeric Cholesky failed");

            let mut rhs_mat = faer::Mat::zeros(s, 1);
            for i in 0..s { rhs_mat[(i, 0)] = rhs[i]; }
            let sol = llt.solve(&rhs_mat);
            (0..s).map(|i| sol[(i, 0)]).collect()
        }
        GainFactorization::SparseLU => {
            if sym_lu.is_none() {
                *sym_lu = Some(
                    sp::SymbolicLu::try_new(faer_mat.symbolic())
                        .expect("sparse symbolic LU failed"),
                );
            }
            let lu = sp::Lu::try_new_with_symbolic(
                sym_lu.clone().unwrap(),
                faer_mat,
            )
            .expect("sparse numeric LU failed");

            let mut rhs_mat = faer::Mat::zeros(s, 1);
            for i in 0..s { rhs_mat[(i, 0)] = rhs[i]; }
            let sol = lu.solve(&rhs_mat);
            (0..s).map(|i| sol[(i, 0)]).collect()
        }
        _ => unreachable!(),
    }
}
