//! DC State Estimation — linear model (Chapter 2.8).
//!
//! The DC approximation assumes flat voltage magnitudes (1.0 p.u.) and
//! neglects shunt elements and branch resistances. The active power flow
//! from bus k to m is approximated as:
//!
//!     P_km = (θ_k − θ_m) / x_km
//!
//! The resulting linear measurement model is:
//!
//!     z_A = H_AA · θ + e_A
//!
//! where z_A includes active power flow and injection measurements only,
//! and H_AA depends solely on branch reactances. The solution is a single
//! weighted least-squares solve (no iteration).

use std::time::Instant;

use crate::model::measurement::*;
use crate::model::{AcModel, MeasurementSet, PowerSystem};
use super::gauss_newton::build_measurement_map;
use super::types::*;

/// Solve the DC state estimation problem.
pub fn solve_dc(
    system: &PowerSystem,
    _model: &AcModel,
    measurements: &MeasurementSet,
    _config: &EstimationConfig,
) -> Result<EstimationResult, SolverError> {
    let n = system.n_buses();
    let slack = system.slack_index;

    if n == 0 {
        return Err(SolverError::InvalidInput("system has no buses".into()));
    }

    let t_start = Instant::now();

    let n_theta = n;
    let mut h_rows: Vec<Vec<(usize, f64)>> = Vec::new();
    let mut z_vec: Vec<f64> = Vec::new();
    let mut w_vec: Vec<f64> = Vec::new();

    for wm in measurements.wattmeters.iter().filter(|m| m.status) {
        match &wm.location {
            WattmeterLocation::Branch { branch, end } => {
                let br = &system.branches[branch - 1];
                let from = system.bus_idx(br.from);
                let to = system.bus_idx(br.to);
                let x = br.reactance;
                if x.abs() < 1e-15 { continue; }
                let b_inv = 1.0 / x;
                let mut row = Vec::new();
                match end {
                    BranchEnd::From => {
                        row.push((from, b_inv));
                        row.push((to, -b_inv));
                    }
                    BranchEnd::To => {
                        row.push((from, -b_inv));
                        row.push((to, b_inv));
                    }
                }
                h_rows.push(row);
                z_vec.push(wm.active);
                w_vec.push(1.0 / wm.variance);
            }
            WattmeterLocation::Bus(bus) => {
                let i = system.bus_idx(*bus);
                let mut row = Vec::new();
                for br in &system.branches {
                    if !br.status { continue; }
                    let from = system.bus_idx(br.from);
                    let to = system.bus_idx(br.to);
                    let x = br.reactance;
                    if x.abs() < 1e-15 { continue; }
                    let b_inv = 1.0 / x;
                    if from == i {
                        add_or_update(&mut row, from, b_inv);
                        add_or_update(&mut row, to, -b_inv);
                    } else if to == i {
                        add_or_update(&mut row, to, b_inv);
                        add_or_update(&mut row, from, -b_inv);
                    }
                }
                h_rows.push(row);
                z_vec.push(wm.active);
                w_vec.push(1.0 / wm.variance);
            }
        }
    }

    let m = h_rows.len();
    if m == 0 {
        return Err(SolverError::InvalidInput(
            "DC estimation requires active power measurements".into(),
        ));
    }

    let t_jac = t_start.elapsed().as_secs_f64();

    let t0 = Instant::now();
    let mut g_flat = vec![0.0f64; n_theta * n_theta];
    let mut rhs = vec![0.0f64; n_theta];

    for (row_idx, row) in h_rows.iter().enumerate() {
        let wi = w_vec[row_idx];
        let zi = z_vec[row_idx];
        for &(ca, ha) in row {
            rhs[ca] += wi * ha * zi;
            for &(cb, hb) in row {
                g_flat[ca * n_theta + cb] += wi * ha * hb;
            }
        }
    }

    for i in 0..n_theta {
        g_flat[slack * n_theta + i] = 0.0;
        g_flat[i * n_theta + slack] = 0.0;
    }
    g_flat[slack * n_theta + slack] = 1.0;
    rhs[slack] = 0.0;

    let gain_time = t0.elapsed().as_secs_f64();

    let t1 = Instant::now();
    let theta = super::factorize::solve_dense_faer(&g_flat, &rhs, n_theta);
    let solve_time = t1.elapsed().as_secs_f64();

    let v_mag: Vec<f64> = vec![1.0; n];
    let total_time = t_start.elapsed().as_secs_f64();

    let residuals: Vec<f64> = h_rows
        .iter()
        .zip(z_vec.iter())
        .map(|(row, &zi)| {
            let h_val: f64 = row.iter().map(|&(c, hc)| hc * theta[c]).sum();
            zi - h_val
        })
        .collect();

    let gain_csc = {
        use sprs::TriMat;
        let mut tri = TriMat::new((n_theta, n_theta));
        for r in 0..n_theta {
            for c in 0..n_theta {
                let v = g_flat[r * n_theta + c];
                if v.abs() > 1e-20 { tri.add_triplet(r, c, v); }
            }
        }
        tri.to_csc()
    };

    let jac_csc = {
        use sprs::TriMat;
        let mut tri = TriMat::new((m, n_theta));
        for (r, row) in h_rows.iter().enumerate() {
            for &(c, v) in row { tri.add_triplet(r, c, v); }
        }
        tri.to_csc()
    };

    let mmap = build_measurement_map(measurements);

    let diagnostics = vec![IterationDiagnostic {
        iteration: 0,
        max_delta_x: theta.iter().fold(0.0f64, |a, &x| a.max(x.abs())),
        jacobian_time_s: t_jac,
        gain_time_s: gain_time,
        solve_time_s: solve_time,
        total_time_s: total_time,
    }];

    let artifacts = Some(SolverArtifacts {
        jacobian: jac_csc,
        residuals,
        measurement_z: z_vec.clone(),
        measurement_h: h_rows
            .iter()
            .map(|row| row.iter().map(|&(c, hc)| hc * theta[c]).sum())
            .collect(),
        precision_diag: w_vec,
        precision_off: Vec::new(),
        gain_matrix: gain_csc,
        n_states: n_theta,
        slack_index: slack,
        measurement_map: mmap,
    });

    Ok(EstimationResult {
        voltage_magnitude: v_mag,
        voltage_angle: theta,
        converged: true,
        iterations: 1,
        final_increment: 0.0,
        diagnostics,
        artifacts,
    })
}

fn add_or_update(row: &mut Vec<(usize, f64)>, col: usize, val: f64) {
    if let Some(entry) = row.iter_mut().find(|(c, _)| *c == col) {
        entry.1 += val;
    } else {
        row.push((col, val));
    }
}
