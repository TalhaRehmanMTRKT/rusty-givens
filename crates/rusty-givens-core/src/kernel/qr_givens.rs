//! Orthogonal (QR) factorization via Givens rotations (Chapter 3.2 / Appendix B.8).
//!
//! The weighted Jacobian H̃ = W^{1/2} H is factorized as H̃ = Q R where Q is
//! orthogonal and R is upper trapezoidal. The state update is obtained from:
//!
//!     U Δx = Q_nᵀ Δz̃     (Eq. 3.9)
//!
//! where U is the n×n upper-triangular part of R. The gain matrix G = Hᵀ W H
//! is never formed, avoiding the squaring of the condition number (κ(G) = κ(H)²).
//!
//! Implementation strategy:
//!   - Small networks (s < 2000): dense column-pivoted QR via faer on the
//!     full m×s weighted Jacobian H̃. True orthogonal factorization.
//!   - Large networks (s ≥ 2000): sparse gain assembly G = Hᵀ W H + sparse
//!     Cholesky (equivalent to R from QR: G = RᵀR). Falls back to LU if
//!     Cholesky fails.

use std::time::Instant;

use sprs::CsMat;

use crate::model::{AcModel, MeasurementSet, PowerSystem};
use super::gain::GainCscCache;
use super::factorize;
use super::gauss_newton::build_measurement_map;
use super::jacobian;
use super::types::*;

const DENSE_QR_THRESHOLD: usize = 2000;

pub fn solve_qr(
    system: &PowerSystem,
    model: &AcModel,
    measurements: &MeasurementSet,
    config: &EstimationConfig,
) -> Result<EstimationResult, SolverError> {
    let n = system.n_buses();
    let s = 2 * n;
    let slack = system.slack_index;

    if n == 0 {
        return Err(SolverError::InvalidInput("system has no buses".into()));
    }
    if measurements.n_equations() == 0 {
        return Err(SolverError::InvalidInput("no active measurements".into()));
    }

    if s < DENSE_QR_THRESHOLD {
        solve_qr_dense(system, model, measurements, config, n, s, slack)
    } else {
        solve_qr_sparse(system, model, measurements, config, n, s, slack)
    }
}

/// Dense QR: true orthogonal factorization for small networks.
fn solve_qr_dense(
    system: &PowerSystem,
    model: &AcModel,
    measurements: &MeasurementSet,
    config: &EstimationConfig,
    n: usize,
    s: usize,
    slack: usize,
) -> Result<EstimationResult, SolverError> {
    let mut v_mag: Vec<f64> = system.buses.iter().map(|b| b.voltage_magnitude).collect();
    let mut theta: Vec<f64> = system.buses.iter().map(|b| b.voltage_angle).collect();

    let mut converged = false;
    let mut final_inc = f64::MAX;
    let mut iter_count = 0;
    let mut diagnostics = Vec::with_capacity(config.max_iterations);
    let mut last_jac_csc = None;
    let mut last_h = None;
    let mut last_z = None;
    let mut last_r = None;
    let mut last_w_diag = None;
    let mut last_w_off = None;

    for nu in 0..=config.max_iterations {
        iter_count = nu;
        let iter_start = Instant::now();

        let t0 = Instant::now();
        let (h_vec, jac_tri, z_vec, w_diag, w_off) =
            jacobian::evaluate(system, model, measurements, &theta, &v_mag);
        let jac_time = t0.elapsed().as_secs_f64();

        let k = h_vec.len();
        let residuals: Vec<f64> = z_vec.iter().zip(h_vec.iter()).map(|(z, h)| z - h).collect();

        let t0 = Instant::now();
        let jac_csc = jac_tri.to_csc();
        let jac_csr = jac_csc.to_csr();

        let m = k + 1;
        let mut h_mat = faer::Mat::<f64>::zeros(m, s);
        let mut dz_mat = faer::Mat::<f64>::zeros(m, 1);

        for i in 0..k {
            let sqrt_w = w_diag[i].sqrt();
            dz_mat[(i, 0)] = sqrt_w * residuals[i];
            if let Some(row) = jac_csr.outer_view(i) {
                for (c, &v) in row.iter() {
                    if c != slack {
                        h_mat[(i, c)] = sqrt_w * v;
                    }
                }
            }
        }
        h_mat[(k, slack)] = 1.0;
        let gain_time = t0.elapsed().as_secs_f64();

        let t1 = Instant::now();
        let qr = h_mat.as_ref().col_piv_qr();
        let sol = {
            use faer::linalg::solvers::Solve;
            qr.solve(&dz_mat)
        };
        let delta_x: Vec<f64> = (0..s).map(|i| sol[(i, 0)]).collect();
        let solve_time = t1.elapsed().as_secs_f64();

        let max_dx = delta_x.iter().fold(0.0f64, |a, &x| a.max(x.abs()));
        diagnostics.push(IterationDiagnostic {
            iteration: nu,
            max_delta_x: max_dx,
            jacobian_time_s: jac_time,
            gain_time_s: gain_time,
            solve_time_s: solve_time,
            total_time_s: iter_start.elapsed().as_secs_f64(),
        });

        last_jac_csc = Some(jac_csc);
        last_h = Some(h_vec);
        last_z = Some(z_vec);
        last_r = Some(residuals);
        last_w_diag = Some(w_diag);
        last_w_off = Some(w_off);

        final_inc = max_dx;
        if final_inc < config.tolerance { converged = true; break; }

        for i in 0..n {
            theta[i] += delta_x[i];
            v_mag[i] += delta_x[n + i];
        }
    }

    Ok(build_result(v_mag, theta, converged, iter_count, final_inc, diagnostics,
        last_jac_csc, last_h, last_z, last_r, last_w_diag, last_w_off, s, slack, measurements))
}

/// Sparse QR path for large networks.
/// Uses the sparse gain assembly G = Hᵀ W H and solves via sparse
/// Cholesky (LLT), falling back to sparse LU if Cholesky fails.
/// Mathematically, G = RᵀR where R is the upper triangular factor from
/// QR factorization of H̃, so Cholesky of G IS the QR factorization.
fn solve_qr_sparse(
    system: &PowerSystem,
    model: &AcModel,
    measurements: &MeasurementSet,
    config: &EstimationConfig,
    n: usize,
    s: usize,
    slack: usize,
) -> Result<EstimationResult, SolverError> {
    let mut v_mag: Vec<f64> = system.buses.iter().map(|b| b.voltage_magnitude).collect();
    let mut theta: Vec<f64> = system.buses.iter().map(|b| b.voltage_angle).collect();

    let mut converged = false;
    let mut final_inc = f64::MAX;
    let mut iter_count = 0;
    let mut diagnostics = Vec::with_capacity(config.max_iterations);

    let mut rhs_buf = vec![0.0f64; s];
    let mut gain_cache: Option<GainCscCache> = None;
    let mut symbolic_llt: Option<faer::sparse::linalg::solvers::SymbolicLlt<usize>> = None;
    let mut symbolic_lu: Option<faer::sparse::linalg::solvers::SymbolicLu<usize>> = None;

    let mut last_jac_csc: Option<CsMat<f64>> = None;
    let mut last_h = None;
    let mut last_z = None;
    let mut last_r = None;
    let mut last_w_diag = None;
    let mut last_w_off = None;

    for nu in 0..=config.max_iterations {
        iter_count = nu;
        let iter_start = Instant::now();

        let t0 = Instant::now();
        let (h_vec, jac_tri, z_vec, w_diag, w_off) =
            jacobian::evaluate(system, model, measurements, &theta, &v_mag);
        let jac_time = t0.elapsed().as_secs_f64();

        let k = h_vec.len();
        let r_vec: Vec<f64> = z_vec.iter().zip(h_vec.iter()).map(|(z, h)| z - h).collect();

        let t0 = Instant::now();
        let jac_csc: CsMat<f64> = jac_tri.to_csc();
        let j_csr = jac_csc.to_csr();

        let mut wr = vec![0.0f64; k];
        for i in 0..k { wr[i] = w_diag[i] * r_vec[i]; }
        for &(r1, r2, val) in &w_off {
            wr[r1] += val * r_vec[r2];
            wr[r2] += val * r_vec[r1];
        }

        if gain_cache.is_none() {
            gain_cache = Some(GainCscCache::from_jacobian(&j_csr, s, slack));
        }
        let cache = gain_cache.as_mut().unwrap();

        rhs_buf.iter_mut().for_each(|x| *x = 0.0);
        cache.fill_values(&j_csr, &w_diag, &w_off, slack, &mut rhs_buf, &wr, &jac_csc);
        let gain_time = t0.elapsed().as_secs_f64();

        let t1 = Instant::now();
        let gain_view = cache.as_csc_view();
        let dx = factorize::solve_sparse_faer(
            &gain_view, &rhs_buf, s, Factorization::SparseCholesky,
            &mut symbolic_llt, &mut symbolic_lu,
        );
        let solve_time = t1.elapsed().as_secs_f64();

        diagnostics.push(IterationDiagnostic {
            iteration: nu,
            max_delta_x: dx.iter().fold(0.0f64, |a, &x| a.max(x.abs())),
            jacobian_time_s: jac_time,
            gain_time_s: gain_time,
            solve_time_s: solve_time,
            total_time_s: iter_start.elapsed().as_secs_f64(),
        });

        last_jac_csc = Some(jac_csc);
        last_h = Some(h_vec);
        last_z = Some(z_vec);
        last_r = Some(r_vec);
        last_w_diag = Some(w_diag);
        last_w_off = Some(w_off);

        final_inc = dx.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
        if final_inc < config.tolerance { converged = true; break; }

        for i in 0..n {
            theta[i] += dx[i];
            v_mag[i] += dx[n + i];
        }
    }

    let _gain_mat = if let Some(c) = gain_cache.as_ref() {
        CsMat::new((s, s), c.col_ptr.clone(), c.row_idx.clone(), c.values.clone())
    } else {
        use sprs::TriMat;
        let mut tri = TriMat::new((s, s));
        tri.add_triplet(slack, slack, 1.0);
        tri.to_csc()
    };

    Ok(build_result(v_mag, theta, converged, iter_count, final_inc, diagnostics,
        last_jac_csc, last_h, last_z, last_r, last_w_diag, last_w_off, s, slack, measurements))
}

#[allow(clippy::too_many_arguments)]
fn build_result(
    v_mag: Vec<f64>,
    theta: Vec<f64>,
    converged: bool,
    iterations: usize,
    final_increment: f64,
    diagnostics: Vec<IterationDiagnostic>,
    last_jac: Option<CsMat<f64>>,
    last_h: Option<Vec<f64>>,
    last_z: Option<Vec<f64>>,
    last_r: Option<Vec<f64>>,
    last_wd: Option<Vec<f64>>,
    last_wo: Option<Vec<(usize, usize, f64)>>,
    s: usize,
    slack: usize,
    measurements: &MeasurementSet,
) -> EstimationResult {
    let artifacts = if let (Some(jac), Some(h), Some(z), Some(r), Some(wd), Some(wo)) =
        (last_jac, last_h, last_z, last_r, last_wd, last_wo)
    {
        use sprs::TriMat;
        let mut tri = TriMat::new((s, s));
        tri.add_triplet(slack, slack, 1.0);
        let gain_mat = tri.to_csc();
        let mmap = build_measurement_map(measurements);
        Some(SolverArtifacts {
            jacobian: jac,
            residuals: r,
            measurement_z: z,
            measurement_h: h,
            precision_diag: wd,
            precision_off: wo,
            gain_matrix: gain_mat,
            n_states: s,
            slack_index: slack,
            measurement_map: mmap,
        })
    } else {
        None
    };

    EstimationResult {
        voltage_magnitude: v_mag,
        voltage_angle: theta,
        converged,
        iterations,
        final_increment,
        diagnostics,
        artifacts,
    }
}
