//! Peters and Wilkinson method (Chapter 3.4).
//!
//! The key insight of P&W is that solving the Normal Equations via LU
//! decomposition (with partial pivoting) is more robust than Cholesky
//! for ill-conditioned gain matrices. When the gain matrix G = Hᵀ W H
//! has near-zero pivots, Cholesky fails but LU with pivoting can still
//! produce a valid factorization.
//!
//! This implementation uses the sparse gain assembly from the existing
//! NE path (GainCscCache) and solves with faer's sparse LU, providing
//! the robustness of P&W while maintaining sparsity for large networks.

use std::time::Instant;

use sprs::CsMat;

use crate::model::{AcModel, MeasurementSet, PowerSystem};
use super::gain::GainCscCache;
use super::factorize;
use super::gauss_newton::build_measurement_map;
use super::jacobian;
use super::types::*;

pub fn solve_pw(
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

    let mut v_mag: Vec<f64> = system.buses.iter().map(|b| b.voltage_magnitude).collect();
    let mut theta: Vec<f64> = system.buses.iter().map(|b| b.voltage_angle).collect();

    let mut converged = false;
    let mut final_inc = f64::MAX;
    let mut iter_count = 0;
    let mut diagnostics = Vec::with_capacity(config.max_iterations);

    let mut rhs_buf = vec![0.0f64; s];
    let mut gain_cache: Option<GainCscCache> = None;
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
        let r: Vec<f64> = z_vec.iter().zip(h_vec.iter()).map(|(z, h)| z - h).collect();

        let t0 = Instant::now();
        let jac_csc: CsMat<f64> = jac_tri.to_csc();
        let j_csr = jac_csc.to_csr();

        let mut wr = vec![0.0f64; k];
        for i in 0..k { wr[i] = w_diag[i] * r[i]; }
        for &(r1, r2, val) in &w_off {
            wr[r1] += val * r[r2];
            wr[r2] += val * r[r1];
        }

        if gain_cache.is_none() {
            gain_cache = Some(GainCscCache::from_jacobian(&j_csr, s, slack));
        }
        let cache = gain_cache.as_mut().unwrap();

        rhs_buf.iter_mut().for_each(|x| *x = 0.0);
        cache.fill_values(&j_csr, &w_diag, &w_off, slack, &mut rhs_buf, &wr, &jac_csc);
        let gain_time = t0.elapsed().as_secs_f64();

        // Solve G Δx = rhs using sparse LU (not Cholesky).
        // LU with partial pivoting is more robust than Cholesky for
        // ill-conditioned or near-singular gain matrices.
        let t1 = Instant::now();
        let gain_view = cache.as_csc_view();
        let mut dummy_llt = None;
        let dx = factorize::solve_sparse_faer(
            &gain_view,
            &rhs_buf,
            s,
            Factorization::SparseLU,
            &mut dummy_llt,
            &mut symbolic_lu,
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
        last_r = Some(r);
        last_w_diag = Some(w_diag);
        last_w_off = Some(w_off);

        final_inc = dx.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
        if final_inc < config.tolerance {
            converged = true;
            break;
        }

        for i in 0..n {
            theta[i] += dx[i];
            v_mag[i] += dx[n + i];
        }
    }

    let artifacts = if let (Some(jac), Some(h), Some(z), Some(r), Some(wd), Some(wo)) =
        (last_jac_csc, last_h, last_z, last_r, last_w_diag, last_w_off)
    {
        let gain_mat = {
            let c = gain_cache.as_ref().unwrap();
            CsMat::new(
                (s, s),
                c.col_ptr.clone(),
                c.row_idx.clone(),
                c.values.clone(),
            )
        };
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

    Ok(EstimationResult {
        voltage_magnitude: v_mag,
        voltage_angle: theta,
        converged,
        iterations: iter_count,
        final_increment: final_inc,
        diagnostics,
        artifacts,
        zi_buses: Vec::new(),
        zi_virtual_pairs_injected: 0,
    })
}
