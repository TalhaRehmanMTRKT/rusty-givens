//! Fast Decoupled WLS State Estimation (Chapter 2.7).
//!
//! Partitions measurements into:
//!   A (active): real power injections & flows  → P-θ sub-problem
//!   R (reactive): reactive power injections, flows & voltage magnitudes → Q-V sub-problem
//!
//! Assumptions (flat start):
//!   1. All bus voltages ≈ 1.0 p.u. and in phase
//!   2. Off-diagonal blocks H_AR, H_RA in the Jacobian are negligible
//!   3. Gain sub-matrices G_AA, G_RR are constant (computed once)
//!
//! The algorithm alternates between θ and V updates until convergence.
//! Branch current magnitude measurements are not supported (Section 2.7).

use std::time::Instant;

use crate::model::{AcModel, MeasurementSet, PowerSystem};
use super::gauss_newton::build_measurement_map;
use super::jacobian;
use super::types::*;

pub fn solve_fd(
    system: &PowerSystem,
    model: &AcModel,
    measurements: &MeasurementSet,
    config: &EstimationConfig,
) -> Result<EstimationResult, SolverError> {
    let n = system.n_buses();
    let slack = system.slack_index;

    if n == 0 {
        return Err(SolverError::InvalidInput("system has no buses".into()));
    }

    // FD ignores branch current magnitude measurements (Section 2.7).
    // They are simply excluded from the P-θ / Q-V partitioning.

    let mut v_mag: Vec<f64> = system.buses.iter().map(|b| b.voltage_magnitude).collect();
    let mut theta: Vec<f64> = system.buses.iter().map(|b| b.voltage_angle).collect();

    let (active_idx, reactive_idx) = classify_measurements(measurements, n);
    let n_a = active_idx.len();
    let n_r = reactive_idx.len();

    if n_a == 0 && n_r == 0 {
        return Err(SolverError::InvalidInput(
            "fast decoupled: no active or reactive measurements".into(),
        ));
    }

    // Constant gain sub-matrices from flat-start Jacobian
    let flat_v = vec![1.0f64; n];
    let flat_theta = vec![0.0f64; n];

    let (_h_flat, jac_tri_flat, _z_flat, w_diag_flat, _w_off_flat) =
        jacobian::evaluate(system, model, measurements, &flat_theta, &flat_v);
    let jac_flat_csc = jac_tri_flat.to_csc();
    let jac_flat_csr = jac_flat_csc.to_csr();

    // Build sparse sub-gains G_AA (n×n) and G_RR (n×n)
    let g_aa_csc = build_sparse_sub_gain(&jac_flat_csr, &w_diag_flat, &active_idx, n, 0, slack);
    let g_rr_csc = build_sparse_sub_gain(&jac_flat_csr, &w_diag_flat, &reactive_idx, n, n, slack);

    // Pre-compute symbolic factorizations (reused every iteration)
    let mut sym_llt_a: Option<faer::sparse::linalg::solvers::SymbolicLlt<usize>> = None;
    let mut sym_lu_a: Option<faer::sparse::linalg::solvers::SymbolicLu<usize>> = None;
    let mut sym_llt_r: Option<faer::sparse::linalg::solvers::SymbolicLlt<usize>> = None;
    let mut sym_lu_r: Option<faer::sparse::linalg::solvers::SymbolicLu<usize>> = None;

    let mut converged = false;
    let mut final_inc = f64::MAX;
    let mut iter_count = 0;
    let mut diagnostics = Vec::with_capacity(config.max_iterations * 2);

    let mut last_jac = None;
    let mut last_h = None;
    let mut last_z = None;
    let mut last_r = None;
    let mut last_w_diag = None;

    for nu in 0..config.max_iterations {
        iter_count = nu + 1;
        let iter_start = Instant::now();

        // ---- P-θ half iteration ----
        let t0 = Instant::now();
        let (h_vec, jac_tri, z_vec, w_diag, _w_off) =
            jacobian::evaluate(system, model, measurements, &theta, &v_mag);
        let jac_time = t0.elapsed().as_secs_f64();

        let r_vec: Vec<f64> = z_vec.iter().zip(h_vec.iter()).map(|(z, h)| z - h).collect();

        let t0 = Instant::now();
        let jac_csr: sprs::CsMat<f64> = jac_tri.to_csr();
        let mut rhs_a = vec![0.0f64; n];
        for &meas_row in &active_idx {
            let wi = w_diag[meas_row];
            let dz = r_vec[meas_row];
            if let Some(row_view) = jac_csr.outer_view(meas_row) {
                for (col, &jval) in row_view.iter() {
                    if col < n {
                        rhs_a[col] += wi * jval * dz;
                    }
                }
            }
        }
        rhs_a[slack] = 0.0;
        let gain_time_a = t0.elapsed().as_secs_f64();

        let t1 = Instant::now();
        let d_theta = solve_sparse_sub(
            &g_aa_csc, &rhs_a, n,
            &mut sym_llt_a, &mut sym_lu_a,
        );
        let solve_time_a = t1.elapsed().as_secs_f64();

        let max_dt = d_theta.iter().fold(0.0f64, |a, &x| a.max(x.abs()));
        diagnostics.push(IterationDiagnostic {
            iteration: nu * 2,
            max_delta_x: max_dt,
            jacobian_time_s: jac_time,
            gain_time_s: gain_time_a,
            solve_time_s: solve_time_a,
            total_time_s: iter_start.elapsed().as_secs_f64(),
        });

        for i in 0..n { theta[i] += d_theta[i]; }

        // ---- Q-V half iteration ----
        let (h_vec2, jac_tri2, z_vec2, w_diag2, _w_off2) =
            jacobian::evaluate(system, model, measurements, &theta, &v_mag);
        let r_vec2: Vec<f64> = z_vec2.iter().zip(h_vec2.iter()).map(|(z, h)| z - h).collect();

        let t0 = Instant::now();
        let jac_csr2: sprs::CsMat<f64> = jac_tri2.to_csr();
        let mut rhs_r = vec![0.0f64; n];
        for &meas_row in &reactive_idx {
            let wi = w_diag2[meas_row];
            let dz = r_vec2[meas_row];
            if let Some(row_view) = jac_csr2.outer_view(meas_row) {
                for (col, &jval) in row_view.iter() {
                    if col >= n && col < 2 * n {
                        rhs_r[col - n] += wi * jval * dz;
                    }
                }
            }
        }
        rhs_r[slack] = 0.0;
        let gain_time_r = t0.elapsed().as_secs_f64();

        let t1 = Instant::now();
        let d_v = solve_sparse_sub(
            &g_rr_csc, &rhs_r, n,
            &mut sym_llt_r, &mut sym_lu_r,
        );
        let solve_time_r = t1.elapsed().as_secs_f64();

        let max_dv = d_v.iter().fold(0.0f64, |a, &x| a.max(x.abs()));
        diagnostics.push(IterationDiagnostic {
            iteration: nu * 2 + 1,
            max_delta_x: max_dv,
            jacobian_time_s: 0.0,
            gain_time_s: gain_time_r,
            solve_time_s: solve_time_r,
            total_time_s: iter_start.elapsed().as_secs_f64(),
        });

        for i in 0..n { v_mag[i] += d_v[i]; }

        final_inc = max_dt.max(max_dv);
        last_jac = Some(jac_csr2.to_csc());
        last_h = Some(h_vec2);
        last_z = Some(z_vec2);
        last_r = Some(r_vec2);
        last_w_diag = Some(w_diag2);

        if max_dt < config.tolerance && max_dv < config.tolerance {
            converged = true;
            break;
        }
    }

    let s = 2 * n;
    let artifacts = if let (Some(jac), Some(h), Some(z), Some(r), Some(wd)) =
        (last_jac, last_h, last_z, last_r, last_w_diag)
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
            precision_off: Vec::new(),
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
    })
}

fn classify_measurements(meas: &MeasurementSet, _n: usize) -> (Vec<usize>, Vec<usize>) {
    let mut active = Vec::new();
    let mut reactive = Vec::new();
    let mut row = 0usize;

    for vm in meas.voltmeters.iter().filter(|m| m.status) {
        let _ = vm;
        reactive.push(row);
        row += 1;
    }
    for am in meas.ammeters.iter().filter(|m| m.status) {
        let _ = am;
        row += 1;
    }
    for _wm in meas.wattmeters.iter().filter(|m| m.status) {
        active.push(row);
        row += 1;
    }
    for _vm in meas.varmeters.iter().filter(|m| m.status) {
        reactive.push(row);
        row += 1;
    }
    for _pmu in meas.pmus.iter().filter(|m| m.status) {
        row += 2;
    }
    for _cam in meas.current_angle_meters.iter().filter(|m| m.status) {
        row += 1;
    }
    let _ = row;

    (active, reactive)
}

/// Build a sparse n×n sub-gain matrix from the Jacobian.
/// `col_offset` selects which block of columns (0 for θ, n for V).
fn build_sparse_sub_gain(
    j_csr: &sprs::CsMatI<f64, usize>,
    w_diag: &[f64],
    meas_rows: &[usize],
    n: usize,
    col_offset: usize,
    slack: usize,
) -> sprs::CsMat<f64> {
    let mut tri = sprs::TriMat::new((n, n));

    for &meas_row in meas_rows {
        let wi = w_diag[meas_row];
        if let Some(row_view) = j_csr.outer_view(meas_row) {
            let idx = row_view.indices();
            let vals = row_view.data();
            for a in 0..idx.len() {
                let ca = idx[a];
                if ca < col_offset || ca >= col_offset + n { continue; }
                let la = ca - col_offset;
                if la == slack { continue; }
                let wj = wi * vals[a];
                for b in 0..idx.len() {
                    let cb = idx[b];
                    if cb < col_offset || cb >= col_offset + n { continue; }
                    let lb = cb - col_offset;
                    if lb == slack { continue; }
                    tri.add_triplet(la, lb, wj * vals[b]);
                }
            }
        }
    }
    tri.add_triplet(slack, slack, 1.0);
    tri.to_csc()
}

/// Solve a sparse n×n sub-gain system via sparse Cholesky.
fn solve_sparse_sub(
    g_csc: &sprs::CsMat<f64>,
    rhs: &[f64],
    n: usize,
    sym_llt: &mut Option<faer::sparse::linalg::solvers::SymbolicLlt<usize>>,
    sym_lu: &mut Option<faer::sparse::linalg::solvers::SymbolicLu<usize>>,
) -> Vec<f64> {
    super::factorize::solve_sparse_faer(
        &g_csc.view(),
        rhs,
        n,
        Factorization::SparseCholesky,
        sym_llt,
        sym_lu,
    )
}
