//! Equality-Constrained WLS State Estimation (Chapter 3.5).
//!
//! Virtual measurements (zero injections at transit buses) are modeled as
//! explicit equality constraints c(x) = 0 via the Lagrangian method, rather
//! than using very large weights that cause ill-conditioning of G.
//!
//! The augmented KKT system solved at each iteration is (Eq. 3.18):
//!
//!     [ αH^T W H   C^T ] [ Δx ] = [ αH^T W Δz^k ]
//!     [    C        0   ] [ -λ ]   [  -c(x^k)     ]
//!
//! where C = ∂c/∂x is the constraint Jacobian, α is an optional scaling
//! factor that improves the condition number (Eq. 3.19–3.20).
//!
//! The matrix is indefinite and requires LU with pivoting.

use std::time::Instant;

use crate::model::measurement::*;
use crate::model::network::BusType;
use crate::model::{AcModel, MeasurementSet, PowerSystem};
use super::gauss_newton::build_measurement_map;
use super::jacobian;
use super::types::*;

pub fn solve_ec(
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

    let (alpha_cfg, _factorization) = match &config.formulation {
        SolverFormulation::EqualityConstrained { alpha, factorization } => {
            (*alpha, *factorization)
        }
        _ => (None, Factorization::SparseCholesky),
    };

    let zero_inj_buses = find_zero_injection_buses(system, measurements);
    let n_c = zero_inj_buses.len() * 2;

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

        let alpha = alpha_cfg.unwrap_or_else(|| {
            if w_diag.is_empty() { return 1.0; }
            let sum_w: f64 = w_diag.iter().sum();
            if sum_w < 1e-30 { 1.0 } else { k as f64 / sum_w }
        });

        let dim = s + n_c;
        let mut kkt = vec![0.0f64; dim * dim];
        let mut rhs = vec![0.0f64; dim];

        fill_htwh(&jac_csr, &w_diag, &w_off, alpha, k, s, &mut kkt, dim);
        fill_htw_dz(&jac_csc, &w_diag, &w_off, &residuals, alpha, s, &mut rhs);

        for i in 0..s {
            kkt[slack * dim + i] = 0.0;
            kkt[i * dim + slack] = 0.0;
        }
        kkt[slack * dim + slack] = 1.0;
        rhs[slack] = 0.0;

        let c_vals = eval_constraints(system, model, &zero_inj_buses, &theta, &v_mag);
        let c_jac = eval_constraint_jacobian(system, model, &zero_inj_buses, &theta, &v_mag, n);

        for (ci, cj_row) in c_jac.iter().enumerate() {
            for &(col, val) in cj_row {
                kkt[(s + ci) * dim + col] = val;
                kkt[col * dim + (s + ci)] = val;
            }
            rhs[s + ci] = -c_vals[ci];
        }

        let gain_time = t0.elapsed().as_secs_f64();

        let t1 = Instant::now();
        let sol = solve_indefinite(&kkt, &rhs, dim);
        let solve_time = t1.elapsed().as_secs_f64();

        let delta_x: Vec<f64> = sol[..s].to_vec();
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
        if final_inc < config.tolerance {
            converged = true;
            break;
        }

        for i in 0..n {
            theta[i] += delta_x[i];
            v_mag[i] += delta_x[n + i];
        }
    }

    let artifacts = if let (Some(jac), Some(h), Some(z), Some(r), Some(wd), Some(wo)) =
        (last_jac_csc, last_h, last_z, last_r, last_w_diag, last_w_off)
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

/// Identify buses with zero injection (no load, no generation, no measurement
/// attached as injection). These become c(x) = 0 constraints.
fn find_zero_injection_buses(system: &PowerSystem, measurements: &MeasurementSet) -> Vec<usize> {
    let mut injected = vec![false; system.n_buses()];

    for bus in &system.buses {
        let idx = system.bus_idx(bus.label);
        if bus.bus_type == BusType::Slack || bus.bus_type == BusType::PV {
            injected[idx] = true;
        }
        if bus.active_demand.abs() > 1e-12 || bus.reactive_demand.abs() > 1e-12 {
            injected[idx] = true;
        }
    }

    for wm in measurements.wattmeters.iter().filter(|m| m.status) {
        if let WattmeterLocation::Bus(bus) = &wm.location {
            if wm.active.abs() > 1e-12 {
                injected[system.bus_idx(*bus)] = true;
            }
        }
    }

    (0..system.n_buses())
        .filter(|&i| !injected[i])
        .collect()
}

/// Evaluate the constraint functions c(x) = [P_i(x), Q_i(x)] for each
/// zero-injection bus.
fn eval_constraints(
    _system: &PowerSystem,
    model: &AcModel,
    zero_buses: &[usize],
    theta: &[f64],
    v_mag: &[f64],
) -> Vec<f64> {
    let mut vals = Vec::with_capacity(zero_buses.len() * 2);
    for &i in zero_buses {
        let vi = v_mag[i];
        let mut p_sum = 0.0;
        let mut q_sum = 0.0;
        if let (Some(g_r), Some(b_r)) = (model.g_bus.outer_view(i), model.b_bus.outer_view(i)) {
            for (j, &gij) in g_r.iter() {
                let bij = b_r.get(j).copied().unwrap_or(0.0);
                let tij = theta[i] - theta[j];
                let vj = v_mag[j];
                p_sum += (gij * tij.cos() + bij * tij.sin()) * vj;
                q_sum += (gij * tij.sin() - bij * tij.cos()) * vj;
            }
        }
        vals.push(vi * p_sum);
        vals.push(vi * q_sum);
    }
    vals
}

/// Evaluate the constraint Jacobian C = ∂c/∂x. Returns a Vec of sparse rows.
fn eval_constraint_jacobian(
    _system: &PowerSystem,
    model: &AcModel,
    zero_buses: &[usize],
    theta: &[f64],
    v_mag: &[f64],
    n: usize,
) -> Vec<Vec<(usize, f64)>> {
    let mut rows = Vec::with_capacity(zero_buses.len() * 2);
    for &i in zero_buses {
        let vi = v_mag[i];
        let gii = model.g_bus.outer_view(i).and_then(|r| r.get(i).copied()).unwrap_or(0.0);
        let bii = model.b_bus.outer_view(i).and_then(|r| r.get(i).copied()).unwrap_or(0.0);

        let mut p_row = Vec::new();
        let mut q_row = Vec::new();
        let mut p_sum = 0.0;
        let mut dp_dti = 0.0;
        let mut q_sum = 0.0;
        let mut dq_dti = 0.0;

        if let (Some(g_r), Some(b_r)) = (model.g_bus.outer_view(i), model.b_bus.outer_view(i)) {
            for (j, &gij) in g_r.iter() {
                let bij = b_r.get(j).copied().unwrap_or(0.0);
                let tij = theta[i] - theta[j];
                let ct = tij.cos();
                let st = tij.sin();
                let vj = v_mag[j];

                p_sum += (gij * ct + bij * st) * vj;
                dp_dti += (-gij * st + bij * ct) * vj;
                q_sum += (gij * st - bij * ct) * vj;
                dq_dti += (gij * ct + bij * st) * vj;

                if j != i {
                    p_row.push((j, (gij * st - bij * ct) * vi * vj));
                    p_row.push((n + j, (gij * ct + bij * st) * vi));
                    q_row.push((j, -(gij * ct + bij * st) * vi * vj));
                    q_row.push((n + j, (gij * st - bij * ct) * vi));
                }
            }
        }

        p_row.push((i, vi * dp_dti - bii * vi * vi));
        p_row.push((n + i, p_sum + gii * vi));
        q_row.push((i, vi * dq_dti - gii * vi * vi));
        q_row.push((n + i, q_sum - bii * vi));

        rows.push(p_row);
        rows.push(q_row);
    }
    rows
}

fn fill_htwh(
    j_csr: &sprs::CsMatI<f64, usize>,
    w_diag: &[f64],
    w_off: &[(usize, usize, f64)],
    alpha: f64,
    k: usize,
    _s: usize,
    kkt: &mut [f64],
    dim: usize,
) {
    for meas_row in 0..k {
        let wi = alpha * w_diag[meas_row];
        if let Some(row_view) = j_csr.outer_view(meas_row) {
            let idx = row_view.indices();
            let vals = row_view.data();
            for a in 0..idx.len() {
                let ca = idx[a];
                let wj = wi * vals[a];
                for b in 0..idx.len() {
                    let cb = idx[b];
                    kkt[ca * dim + cb] += wj * vals[b];
                }
            }
        }
    }
    for &(r1, r2, w_val) in w_off {
        let aw = alpha * w_val;
        if let (Some(row1), Some(row2)) = (j_csr.outer_view(r1), j_csr.outer_view(r2)) {
            for (ca, &j1a) in row1.iter() {
                for (cb, &j2b) in row2.iter() {
                    let val = aw * j1a * j2b;
                    kkt[ca * dim + cb] += val;
                    kkt[cb * dim + ca] += val;
                }
            }
        }
    }
}

fn fill_htw_dz(
    j_csc: &sprs::CsMat<f64>,
    w_diag: &[f64],
    w_off: &[(usize, usize, f64)],
    residuals: &[f64],
    alpha: f64,
    s: usize,
    rhs: &mut [f64],
) {
    let k = residuals.len();
    let mut wr = vec![0.0f64; k];
    for i in 0..k { wr[i] = alpha * w_diag[i] * residuals[i]; }
    for &(r1, r2, val) in w_off {
        wr[r1] += alpha * val * residuals[r2];
        wr[r2] += alpha * val * residuals[r1];
    }
    for col in 0..s {
        if let Some(j_col) = j_csc.outer_view(col) {
            let mut v = 0.0;
            for (i, &j_val) in j_col.iter() { v += j_val * wr[i]; }
            rhs[col] = v;
        }
    }
}

/// Solve the indefinite KKT system using dense LU with partial pivoting (faer).
fn solve_indefinite(kkt: &[f64], rhs: &[f64], dim: usize) -> Vec<f64> {
    use faer::linalg::solvers::Solve;
    let mut mat = faer::Mat::zeros(dim, dim);
    for r in 0..dim {
        for c in 0..dim {
            mat[(r, c)] = kkt[r * dim + c];
        }
    }
    let mut rhs_mat = faer::Mat::zeros(dim, 1);
    for i in 0..dim { rhs_mat[(i, 0)] = rhs[i]; }
    let lu = mat.as_ref().partial_piv_lu();
    let sol = lu.solve(&rhs_mat);
    (0..dim).map(|i| sol[(i, 0)]).collect()
}
