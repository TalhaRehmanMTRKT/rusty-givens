//! Measurement function evaluation h(x) and Jacobian matrix J(x) construction.
//!
//! Implements every measurement type from JuliaGrid's AC State Estimation
//! tutorial, including voltmeters, ammeters, wattmeters, varmeters, and PMUs
//! in both polar and rectangular coordinate systems.

use sprs::TriMat;

use crate::ac_model::{AcModel, BranchParams};
use crate::measurement::*;
use crate::power_system::PowerSystem;

/// State vector layout: x = [θ_1, ..., θ_n, V_1, ..., V_n].
/// Column indices in the Jacobian are [0..n) for angles and [n..2n) for magnitudes.
/// The slack bus angle is held fixed via the gain-matrix zeroing technique.
#[inline]
fn theta_col(bus_idx: usize) -> usize {
    bus_idx
}
#[inline]
fn v_col(bus_idx: usize, n: usize) -> usize {
    n + bus_idx
}

// ───────────────────────────────────────────────────────────────────────
//  h(x) evaluation — returns the measurement function vector and fills
//  the Jacobian triplet list simultaneously.
// ───────────────────────────────────────────────────────────────────────

/// Evaluate h(x) and build the Jacobian for all active measurements.
///
/// Returns `(h_vec, jacobian, z_vec, w_diag, w_off)` where:
///   - `h_vec`  : measurement function values h(x)  (length k)
///   - `jacobian`: sparse Jacobian J(x) of size k × 2n
///   - `z_vec`  : measurement values z              (length k)
///   - `w_diag` : diagonal precision entries 1/v     (length k)
///   - `w_off`  : off-diagonal precision pairs (row1, row2, value) for correlated PMUs
pub fn evaluate(
    system: &PowerSystem,
    model: &AcModel,
    measurements: &MeasurementSet,
    theta: &[f64],
    v_mag: &[f64],
) -> (Vec<f64>, TriMat<f64>, Vec<f64>, Vec<f64>, Vec<(usize, usize, f64)>) {
    let n = system.n_buses();
    let k = measurements.n_equations();

    let mut h_vec = Vec::with_capacity(k);
    let mut z_vec = Vec::with_capacity(k);
    let mut w_diag = Vec::with_capacity(k);
    let mut w_off: Vec<(usize, usize, f64)> = Vec::new();
    let mut jac = TriMat::new((k, 2 * n));
    let mut row = 0usize;

    // ── 1. Voltmeters ──
    for vm in measurements.voltmeters.iter().filter(|m| m.status) {
        let i = system.bus_idx(vm.bus);
        let hi = v_mag[i];
        h_vec.push(hi);
        z_vec.push(vm.magnitude);
        w_diag.push(1.0 / vm.variance);
        // ∂h/∂V_i = 1
        jac.add_triplet(row, v_col(i, n), 1.0);
        row += 1;
    }

    // ── 2. Ammeters ──
    for am in measurements.ammeters.iter().filter(|m| m.status) {
        let bp = &model.branch_params[am.branch - 1];
        add_current_magnitude(
            &mut h_vec,
            &mut z_vec,
            &mut w_diag,
            &mut jac,
            &mut row,
            bp,
            am.end,
            am.magnitude,
            am.variance,
            am.square,
            theta,
            v_mag,
            n,
        );
    }

    // ── 3. Wattmeters ──
    for wm in measurements.wattmeters.iter().filter(|m| m.status) {
        match &wm.location {
            WattmeterLocation::Bus(bus) => {
                let i = system.bus_idx(*bus);
                add_active_injection(
                    &mut h_vec, &mut z_vec, &mut w_diag, &mut jac, &mut row, model, i, wm.active,
                    wm.variance, theta, v_mag, n,
                );
            }
            WattmeterLocation::Branch { branch, end } => {
                let bp = &model.branch_params[branch - 1];
                add_active_flow(
                    &mut h_vec, &mut z_vec, &mut w_diag, &mut jac, &mut row, bp, *end, wm.active,
                    wm.variance, theta, v_mag, n,
                );
            }
        }
    }

    // ── 4. Varmeters ──
    for vm in measurements.varmeters.iter().filter(|m| m.status) {
        match &vm.location {
            VarmeterLocation::Bus(bus) => {
                let i = system.bus_idx(*bus);
                add_reactive_injection(
                    &mut h_vec, &mut z_vec, &mut w_diag, &mut jac, &mut row, model, i,
                    vm.reactive, vm.variance, theta, v_mag, n,
                );
            }
            VarmeterLocation::Branch { branch, end } => {
                let bp = &model.branch_params[branch - 1];
                add_reactive_flow(
                    &mut h_vec, &mut z_vec, &mut w_diag, &mut jac, &mut row, bp, *end,
                    vm.reactive, vm.variance, theta, v_mag, n,
                );
            }
        }
    }

    // ── 5. PMUs ──
    for pmu in measurements.pmus.iter().filter(|m| m.status) {
        match &pmu.location {
            PmuLocation::Bus(bus) => {
                let i = system.bus_idx(*bus);
                add_bus_pmu(
                    &mut h_vec, &mut z_vec, &mut w_diag, &mut w_off, &mut jac, &mut row, pmu, i,
                    theta, v_mag, n,
                );
            }
            PmuLocation::Branch { branch, end } => {
                let bp = &model.branch_params[branch - 1];
                add_branch_pmu(
                    &mut h_vec, &mut z_vec, &mut w_diag, &mut w_off, &mut jac, &mut row, pmu, bp,
                    *end, theta, v_mag, n,
                );
            }
        }
    }

    (h_vec, jac, z_vec, w_diag, w_off)
}

// ─────────────────────────────────────────────────────────────────────
//  Measurement function + Jacobian helpers (private)
// ─────────────────────────────────────────────────────────────────────

/// Current magnitude measurement at from-bus or to-bus end.
fn add_current_magnitude(
    h: &mut Vec<f64>,
    z: &mut Vec<f64>,
    w: &mut Vec<f64>,
    jac: &mut TriMat<f64>,
    row: &mut usize,
    bp: &BranchParams,
    end: BranchEnd,
    meas_mag: f64,
    variance: f64,
    square: bool,
    theta: &[f64],
    v_mag: &[f64],
    n: usize,
) {
    let i = bp.from_idx;
    let j = bp.to_idx;
    let vi = v_mag[i];
    let vj = v_mag[j];
    let theta_ij = theta[i] - theta[j];
    let phi = bp.phi;

    let (a, b_coeff, c, d, _cos_sign_d) = match end {
        BranchEnd::From => (
            bp.a_from(),
            bp.b_from(),
            bp.c_from(),
            bp.d_from(),
            -1.0f64, // C cos - D sin for from
        ),
        BranchEnd::To => (
            bp.a_to(),
            bp.b_to(),
            bp.c_to(),
            bp.d_to(),
            1.0f64, // C cos + D sin for to
        ),
    };

    let cos_td = (theta_ij - phi).cos();
    let sin_td = (theta_ij - phi).sin();

    // Inner expression (under the sqrt)
    let _expr_cos = c * cos_td + _cos_sign_d * d * sin_td;
    // Re-derive properly for from and to:
    let (inner, dh_dtheta_i_num, dh_dtheta_j_num, dh_dvi_num, dh_dvj_num) = match end {
        BranchEnd::From => {
            // h² = A Vi² + B Vj² - 2[C cos(θ-φ) - D sin(θ-φ)] Vi Vj
            let bracket = c * cos_td - d * sin_td;
            let sq = a * vi * vi + b_coeff * vj * vj - 2.0 * bracket * vi * vj;
            let dth = (c * sin_td + d * cos_td) * vi * vj;
            let dvi = a * vi - bracket * vj;
            let dvj = b_coeff * vj - bracket * vi;
            (sq, dth, -dth, dvi, dvj)
        }
        BranchEnd::To => {
            // h² = A Vi² + B Vj² - 2[C cos(θ-φ) + D sin(θ-φ)] Vi Vj
            let bracket = c * cos_td + d * sin_td;
            let sq = a * vi * vi + b_coeff * vj * vj - 2.0 * bracket * vi * vj;
            let dth = (c * sin_td - d * cos_td) * vi * vj;
            let dvi = a * vi - bracket * vj;
            let dvj = b_coeff * vj - bracket * vi;
            (sq, dth, -dth, dvi, dvj)
        }
    };

    let inner_safe = inner.max(1e-20);

    if square {
        // Squared form: h = inner, z = meas², v = 2*variance
        h.push(inner_safe);
        z.push(meas_mag * meas_mag);
        w.push(1.0 / (2.0 * variance));
        // Jacobians are doubled (derivative of h² w.r.t. x is 2h * dh/dx, but since h=inner here...)
        jac.add_triplet(*row, theta_col(i), 2.0 * dh_dtheta_i_num);
        jac.add_triplet(*row, theta_col(j), 2.0 * dh_dtheta_j_num);
        jac.add_triplet(*row, v_col(i, n), 2.0 * dh_dvi_num);
        jac.add_triplet(*row, v_col(j, n), 2.0 * dh_dvj_num);
    } else {
        let h_val = inner_safe.sqrt();
        h.push(h_val);
        z.push(meas_mag);
        w.push(1.0 / variance);
        jac.add_triplet(*row, theta_col(i), dh_dtheta_i_num / h_val);
        jac.add_triplet(*row, theta_col(j), dh_dtheta_j_num / h_val);
        jac.add_triplet(*row, v_col(i, n), dh_dvi_num / h_val);
        jac.add_triplet(*row, v_col(j, n), dh_dvj_num / h_val);
    }
    *row += 1;
}

/// Active power injection h_{P_i}(x) = V_i Σ_j (G_ij cos θ_ij + B_ij sin θ_ij) V_j.
fn add_active_injection(
    h: &mut Vec<f64>,
    z: &mut Vec<f64>,
    w: &mut Vec<f64>,
    jac: &mut TriMat<f64>,
    row: &mut usize,
    model: &AcModel,
    i: usize,
    meas: f64,
    variance: f64,
    theta: &[f64],
    v_mag: &[f64],
    n: usize,
) {
    let vi = v_mag[i];

    let gii = model.g_bus.outer_view(i).and_then(|r| r.get(i).copied()).unwrap_or(0.0);
    let bii = model.b_bus.outer_view(i).and_then(|r| r.get(i).copied()).unwrap_or(0.0);

    // Compute the inner sum: Σ_j (G_ij cos θ_ij + B_ij sin θ_ij) V_j
    // and all Jacobian entries in a single pass.
    let mut inner_sum = 0.0;
    let mut dh_dthetai = 0.0;

    if let (Some(g_r), Some(b_r)) = (model.g_bus.outer_view(i), model.b_bus.outer_view(i)) {
        for (j, &gij) in g_r.iter() {
            let bij = b_r.get(j).copied().unwrap_or(0.0);
            let theta_ij = theta[i] - theta[j];
            let cos_t = theta_ij.cos();
            let sin_t = theta_ij.sin();
            let vj = v_mag[j];

            inner_sum += (gij * cos_t + bij * sin_t) * vj;
            dh_dthetai += (-gij * sin_t + bij * cos_t) * vj;

            if j != i {
                // dh/dθ_j = (G_ij sin θ_ij - B_ij cos θ_ij) Vi Vj
                jac.add_triplet(*row, theta_col(j), (gij * sin_t - bij * cos_t) * vi * vj);
                // dh/dV_j = (G_ij cos θ_ij + B_ij sin θ_ij) Vi
                jac.add_triplet(*row, v_col(j, n), (gij * cos_t + bij * sin_t) * vi);
            }
        }
    }

    let hi = vi * inner_sum;
    // dh/dθ_i = Vi Σ_j (-G sin + B cos) Vj - B_ii Vi²
    dh_dthetai = vi * dh_dthetai - bii * vi * vi;
    // dh/dV_i = Σ_j (G cos + B sin) Vj + G_ii Vi
    let dh_dvi = inner_sum + gii * vi;

    h.push(hi);
    z.push(meas);
    w.push(1.0 / variance);
    jac.add_triplet(*row, theta_col(i), dh_dthetai);
    jac.add_triplet(*row, v_col(i, n), dh_dvi);
    *row += 1;
}

/// Active power flow from-bus: h_{P_ij}(x).
fn add_active_flow(
    h: &mut Vec<f64>,
    z: &mut Vec<f64>,
    w: &mut Vec<f64>,
    jac: &mut TriMat<f64>,
    row: &mut usize,
    bp: &BranchParams,
    end: BranchEnd,
    meas: f64,
    variance: f64,
    theta: &[f64],
    v_mag: &[f64],
    n: usize,
) {
    let i = bp.from_idx;
    let j = bp.to_idx;
    let vi = v_mag[i];
    let vj = v_mag[j];
    let theta_ij = theta[i] - theta[j];
    let phi = bp.phi;
    let cos_tp = (theta_ij - phi).cos();
    let sin_tp = (theta_ij - phi).sin();
    let tau = bp.tau;
    let g = bp.g;
    let b = bp.b;
    let g_s = bp.g_s;

    let (hi, dh_dti, dh_dtj, dh_dvi_val, dh_dvj_val) = match end {
        BranchEnd::From => {
            // h = (g+g_s)/τ² Vi² - (1/τ)[g cos(θ-φ) + b sin(θ-φ)] Vi Vj
            let h_val = (g + g_s) / (tau * tau) * vi * vi
                - (1.0 / tau) * (g * cos_tp + b * sin_tp) * vi * vj;
            let dt = (1.0 / tau) * (g * sin_tp - b * cos_tp) * vi * vj;
            let dvi = 2.0 * (g + g_s) / (tau * tau) * vi
                - (1.0 / tau) * (g * cos_tp + b * sin_tp) * vj;
            let dvj = -(1.0 / tau) * (g * cos_tp + b * sin_tp) * vi;
            (h_val, dt, -dt, dvi, dvj)
        }
        BranchEnd::To => {
            // h = (g+g_s) Vj² - (1/τ)[g cos(θ-φ) - b sin(θ-φ)] Vi Vj
            let h_val =
                (g + g_s) * vj * vj - (1.0 / tau) * (g * cos_tp - b * sin_tp) * vi * vj;
            let dt = (1.0 / tau) * (g * sin_tp + b * cos_tp) * vi * vj;
            let dvi = -(1.0 / tau) * (g * cos_tp - b * sin_tp) * vj;
            let dvj =
                2.0 * (g + g_s) * vj - (1.0 / tau) * (g * cos_tp - b * sin_tp) * vi;
            (h_val, dt, -dt, dvi, dvj)
        }
    };

    h.push(hi);
    z.push(meas);
    w.push(1.0 / variance);
    jac.add_triplet(*row, theta_col(i), dh_dti);
    jac.add_triplet(*row, theta_col(j), dh_dtj);
    jac.add_triplet(*row, v_col(i, n), dh_dvi_val);
    jac.add_triplet(*row, v_col(j, n), dh_dvj_val);
    *row += 1;
}

/// Reactive power injection h_{Q_i}(x).
fn add_reactive_injection(
    h: &mut Vec<f64>,
    z: &mut Vec<f64>,
    w: &mut Vec<f64>,
    jac: &mut TriMat<f64>,
    row: &mut usize,
    model: &AcModel,
    i: usize,
    meas: f64,
    variance: f64,
    theta: &[f64],
    v_mag: &[f64],
    n: usize,
) {
    let vi = v_mag[i];
    let mut sum = 0.0;
    let mut dh_dthetai = 0.0;

    let gii = model
        .g_bus
        .outer_view(i)
        .and_then(|r| r.get(i).copied())
        .unwrap_or(0.0);
    let bii = model
        .b_bus
        .outer_view(i)
        .and_then(|r| r.get(i).copied())
        .unwrap_or(0.0);

    if let (Some(g_r), Some(b_r)) = (model.g_bus.outer_view(i), model.b_bus.outer_view(i)) {
        for (j_idx, &gij) in g_r.iter() {
            let bij = b_r.get(j_idx).copied().unwrap_or(0.0);
            let theta_ij = theta[i] - theta[j_idx];
            let cos_t = theta_ij.cos();
            let sin_t = theta_ij.sin();
            let vj = v_mag[j_idx];

            sum += (gij * sin_t - bij * cos_t) * vj;

            if j_idx != i {
                // dh/dθ_j = -(G cos + B sin) Vi Vj
                let dh_dthetaj = -(gij * cos_t + bij * sin_t) * vi * vj;
                jac.add_triplet(*row, theta_col(j_idx), dh_dthetaj);
                // dh/dV_j = (G sin - B cos) Vi
                let dh_dvj = (gij * sin_t - bij * cos_t) * vi;
                jac.add_triplet(*row, v_col(j_idx, n), dh_dvj);
            }

            dh_dthetai += (gij * cos_t + bij * sin_t) * vj;
        }
    }

    let hi = vi * sum;
    dh_dthetai = vi * dh_dthetai - gii * vi * vi;
    let dh_dvi = sum - bii * vi;

    h.push(hi);
    z.push(meas);
    w.push(1.0 / variance);
    jac.add_triplet(*row, theta_col(i), dh_dthetai);
    jac.add_triplet(*row, v_col(i, n), dh_dvi);
    *row += 1;
}

/// Reactive power flow from/to bus.
fn add_reactive_flow(
    h: &mut Vec<f64>,
    z: &mut Vec<f64>,
    w: &mut Vec<f64>,
    jac: &mut TriMat<f64>,
    row: &mut usize,
    bp: &BranchParams,
    end: BranchEnd,
    meas: f64,
    variance: f64,
    theta: &[f64],
    v_mag: &[f64],
    n: usize,
) {
    let i = bp.from_idx;
    let j = bp.to_idx;
    let vi = v_mag[i];
    let vj = v_mag[j];
    let theta_ij = theta[i] - theta[j];
    let phi = bp.phi;
    let cos_tp = (theta_ij - phi).cos();
    let sin_tp = (theta_ij - phi).sin();
    let tau = bp.tau;
    let g = bp.g;
    let b = bp.b;
    let (hi, dh_dti, dh_dtj, dh_dvi_val, dh_dvj_val) = match end {
        BranchEnd::From => {
            let b_s = bp.b_s;
            // h = -(b+b_s)/τ² Vi² - (1/τ)[g sin(θ-φ) - b cos(θ-φ)] Vi Vj
            let h_val = -(b + b_s) / (tau * tau) * vi * vi
                - (1.0 / tau) * (g * sin_tp - b * cos_tp) * vi * vj;
            let dt = -(1.0 / tau) * (g * cos_tp + b * sin_tp) * vi * vj;
            let dvi = -2.0 * (b + b_s) / (tau * tau) * vi
                - (1.0 / tau) * (g * sin_tp - b * cos_tp) * vj;
            let dvj = -(1.0 / tau) * (g * sin_tp - b * cos_tp) * vi;
            (h_val, dt, -dt, dvi, dvj)
        }
        BranchEnd::To => {
            let b_s = bp.b_s;
            // h = -(b+b_s) Vj² + (1/τ)[g sin(θ-φ) + b cos(θ-φ)] Vi Vj
            let h_val =
                -(b + b_s) * vj * vj + (1.0 / tau) * (g * sin_tp + b * cos_tp) * vi * vj;
            let dt = (1.0 / tau) * (g * cos_tp - b * sin_tp) * vi * vj;
            let dvi = (1.0 / tau) * (g * sin_tp + b * cos_tp) * vj;
            let dvj =
                -2.0 * (b + b_s) * vj + (1.0 / tau) * (g * sin_tp + b * cos_tp) * vi;
            (h_val, dt, -dt, dvi, dvj)
        }
    };

    h.push(hi);
    z.push(meas);
    w.push(1.0 / variance);
    jac.add_triplet(*row, theta_col(i), dh_dti);
    jac.add_triplet(*row, theta_col(j), dh_dtj);
    jac.add_triplet(*row, v_col(i, n), dh_dvi_val);
    jac.add_triplet(*row, v_col(j, n), dh_dvj_val);
    *row += 1;
}

/// Bus PMU measurement (voltage phasor).
fn add_bus_pmu(
    h: &mut Vec<f64>,
    z: &mut Vec<f64>,
    w: &mut Vec<f64>,
    w_off: &mut Vec<(usize, usize, f64)>,
    jac: &mut TriMat<f64>,
    row: &mut usize,
    pmu: &Pmu,
    i: usize,
    theta: &[f64],
    v_mag: &[f64],
    n: usize,
) {
    let vi = v_mag[i];
    let ti = theta[i];
    let v_m = pmu.variance_magnitude;
    let v_a = pmu.variance_angle;

    match pmu.coordinate {
        PmuCoordinate::Polar => {
            // h_V = V_i, h_θ = θ_i
            h.push(vi);
            z.push(pmu.magnitude);
            w.push(1.0 / v_m);
            jac.add_triplet(*row, v_col(i, n), 1.0);
            *row += 1;

            h.push(ti);
            z.push(pmu.angle);
            w.push(1.0 / v_a);
            jac.add_triplet(*row, theta_col(i), 1.0);
            *row += 1;
        }
        PmuCoordinate::Rectangular => {
            let z_v = pmu.magnitude;
            let z_t = pmu.angle;
            let cos_z = z_t.cos();
            let sin_z = z_t.sin();

            // Measurement values in rectangular
            let z_re = z_v * cos_z;
            let z_im = z_v * sin_z;

            // Variances via uncertainty propagation
            let v_re = v_m * cos_z * cos_z + v_a * (z_v * sin_z).powi(2);
            let v_im = v_m * sin_z * sin_z + v_a * (z_v * cos_z).powi(2);

            // h_Re = V_i cos θ_i
            h.push(vi * ti.cos());
            z.push(z_re);
            w.push(1.0 / v_re);
            jac.add_triplet(*row, theta_col(i), -vi * ti.sin());
            jac.add_triplet(*row, v_col(i, n), ti.cos());
            let row_re = *row;
            *row += 1;

            // h_Im = V_i sin θ_i
            h.push(vi * ti.sin());
            z.push(z_im);
            w.push(1.0 / v_im);
            jac.add_triplet(*row, theta_col(i), vi * ti.cos());
            jac.add_triplet(*row, v_col(i, n), ti.sin());
            let row_im = *row;
            *row += 1;

            if pmu.correlated {
                let cov = cos_z * sin_z * (v_m - v_a * z_v * z_v);
                // Invert the 2×2 block [[v_re, cov], [cov, v_im]]
                let det = v_re * v_im - cov * cov;
                if det.abs() > 1e-30 {
                    let inv_re = v_im / det;
                    let inv_im = v_re / det;
                    let inv_off = -cov / det;
                    w[row_re] = inv_re;
                    w[row_im] = inv_im;
                    w_off.push((row_re, row_im, inv_off));
                }
            }
        }
    }
}

/// Branch PMU measurement (current phasor).
fn add_branch_pmu(
    h: &mut Vec<f64>,
    z: &mut Vec<f64>,
    w: &mut Vec<f64>,
    w_off: &mut Vec<(usize, usize, f64)>,
    jac: &mut TriMat<f64>,
    row: &mut usize,
    pmu: &Pmu,
    bp: &BranchParams,
    end: BranchEnd,
    theta: &[f64],
    v_mag: &[f64],
    n: usize,
) {
    let from = bp.from_idx;
    let to = bp.to_idx;
    let vi = v_mag[from];
    let vj = v_mag[to];
    let ti = theta[from];
    let tj = theta[to];
    let phi = bp.phi;
    let v_m = pmu.variance_magnitude;
    let v_a = pmu.variance_angle;

    match pmu.coordinate {
        PmuCoordinate::Polar => {
            // Magnitude (same as ammeter)
            add_current_magnitude(
                h,
                z,
                w,
                jac,
                row,
                bp,
                end,
                pmu.magnitude,
                v_m,
                pmu.square,
                theta,
                v_mag,
                n,
            );

            // Angle measurement
            let (h_psi, dp_dti, dp_dtj, dp_dvi, dp_dvj) = match end {
                BranchEnd::From => {
                    let a = bp.a_psi_from();
                    let b = bp.b_psi_from();
                    let c = bp.c_psi_from();
                    let d = bp.d_psi_from();

                    let re = (a * ti.cos() - b * ti.sin()) * vi
                        - (c * (tj + phi).cos() - d * (tj + phi).sin()) * vj;
                    let im = (a * ti.sin() + b * ti.cos()) * vi
                        - (c * (tj + phi).sin() + d * (tj + phi).cos()) * vj;
                    let psi = im.atan2(re);

                    let i_sq = re * re + im * im;
                    let i_sq_safe = if i_sq < 1e-20 { 1e-20 } else { i_sq };

                    let a_i = bp.a_from();
                    let b_i = bp.b_from();
                    let c_i = bp.c_from();
                    let d_i = bp.d_from();
                    let theta_ij = ti - tj;
                    let cos_td = (theta_ij - phi).cos();
                    let sin_td = (theta_ij - phi).sin();

                    let dp_ti = (a_i * vi * vi
                        - (c_i * cos_td - d_i * sin_td) * vi * vj)
                        / i_sq_safe;
                    let dp_tj = (b_i * vj * vj
                        - (c_i * cos_td - d_i * sin_td) * vi * vj)
                        / i_sq_safe;
                    let dp_vi = -(c_i * sin_td + d_i * cos_td) * vj / i_sq_safe;
                    let dp_vj = (c_i * sin_td + d_i * cos_td) * vi / i_sq_safe;

                    (psi, dp_ti, dp_tj, dp_vi, dp_vj)
                }
                BranchEnd::To => {
                    let a = bp.a_psi_to();
                    let b = bp.b_psi_to();
                    let c = bp.c_psi_to();
                    let d = bp.d_psi_to();

                    let re = (a * tj.cos() - b * tj.sin()) * vj
                        - (c * (ti - phi).cos() - d * (ti - phi).sin()) * vi;
                    let im = (a * tj.sin() + b * tj.cos()) * vj
                        - (c * (ti - phi).sin() + d * (ti - phi).cos()) * vi;
                    let psi = im.atan2(re);

                    let i_sq = re * re + im * im;
                    let i_sq_safe = if i_sq < 1e-20 { 1e-20 } else { i_sq };

                    let a_i = bp.a_to();
                    let b_i = bp.b_to();
                    let c_i = bp.c_to();
                    let d_i = bp.d_to();
                    let theta_ij = ti - tj;
                    let cos_td = (theta_ij - phi).cos();
                    let sin_td = (theta_ij - phi).sin();

                    let dp_ti = (a_i * vi * vi
                        - (c_i * cos_td + d_i * sin_td) * vi * vj)
                        / i_sq_safe;
                    let dp_tj = (b_i * vj * vj
                        - (c_i * cos_td + d_i * sin_td) * vi * vj)
                        / i_sq_safe;
                    let dp_vi = -(c_i * sin_td - d_i * cos_td) * vj / i_sq_safe;
                    let dp_vj = (c_i * sin_td - d_i * cos_td) * vi / i_sq_safe;

                    (psi, dp_ti, dp_tj, dp_vi, dp_vj)
                }
            };

            h.push(h_psi);
            z.push(pmu.angle);
            w.push(1.0 / v_a);
            jac.add_triplet(*row, theta_col(from), dp_dti);
            jac.add_triplet(*row, theta_col(to), dp_dtj);
            jac.add_triplet(*row, v_col(from, n), dp_dvi);
            jac.add_triplet(*row, v_col(to, n), dp_dvj);
            *row += 1;
        }
        PmuCoordinate::Rectangular => {
            let z_i = pmu.magnitude;
            let z_psi = pmu.angle;
            let cos_z = z_psi.cos();
            let sin_z = z_psi.sin();

            let z_re = z_i * cos_z;
            let z_im = z_i * sin_z;

            let v_re = v_m * cos_z * cos_z + v_a * (z_i * sin_z).powi(2);
            let v_im = v_m * sin_z * sin_z + v_a * (z_i * cos_z).powi(2);

            match end {
                BranchEnd::From => {
                    let a = bp.a_psi_from();
                    let b_coeff = bp.b_psi_from();
                    let c = bp.c_psi_from();
                    let d = bp.d_psi_from();

                    // h_Re = (A cos θ_i - B sin θ_i)Vi - [C cos(θ_j+φ) - D sin(θ_j+φ)]Vj
                    let h_re = (a * ti.cos() - b_coeff * ti.sin()) * vi
                        - (c * (tj + phi).cos() - d * (tj + phi).sin()) * vj;
                    h.push(h_re);
                    z.push(z_re);
                    w.push(1.0 / v_re);

                    jac.add_triplet(*row, theta_col(from), -(a * ti.sin() + b_coeff * ti.cos()) * vi);
                    jac.add_triplet(
                        *row,
                        theta_col(to),
                        (c * (tj + phi).sin() + d * (tj + phi).cos()) * vj,
                    );
                    jac.add_triplet(*row, v_col(from, n), a * ti.cos() - b_coeff * ti.sin());
                    jac.add_triplet(
                        *row,
                        v_col(to, n),
                        -c * (tj + phi).cos() + d * (tj + phi).sin(),
                    );
                    let row_re = *row;
                    *row += 1;

                    // h_Im = (A sin θ_i + B cos θ_i)Vi - [C sin(θ_j+φ) + D cos(θ_j+φ)]Vj
                    let h_im = (a * ti.sin() + b_coeff * ti.cos()) * vi
                        - (c * (tj + phi).sin() + d * (tj + phi).cos()) * vj;
                    h.push(h_im);
                    z.push(z_im);
                    w.push(1.0 / v_im);

                    jac.add_triplet(*row, theta_col(from), (a * ti.cos() - b_coeff * ti.sin()) * vi);
                    jac.add_triplet(
                        *row,
                        theta_col(to),
                        (-c * (tj + phi).cos() + d * (tj + phi).sin()) * vj,
                    );
                    jac.add_triplet(*row, v_col(from, n), a * ti.sin() + b_coeff * ti.cos());
                    jac.add_triplet(
                        *row,
                        v_col(to, n),
                        -c * (tj + phi).sin() - d * (tj + phi).cos(),
                    );
                    let row_im = *row;
                    *row += 1;

                    if pmu.correlated {
                        let cov = sin_z * cos_z * (v_m - v_a * z_i * z_i);
                        let det = v_re * v_im - cov * cov;
                        if det.abs() > 1e-30 {
                            w[row_re] = v_im / det;
                            w[row_im] = v_re / det;
                            w_off.push((row_re, row_im, -cov / det));
                        }
                    }
                }
                BranchEnd::To => {
                    let a = bp.a_psi_to();
                    let b_coeff = bp.b_psi_to();
                    let c = bp.c_psi_to();
                    let d = bp.d_psi_to();

                    // h_Re = (A cos θ_j - B sin θ_j)Vj - [C cos(θ_i-φ) - D sin(θ_i-φ)]Vi
                    let h_re = (a * tj.cos() - b_coeff * tj.sin()) * vj
                        - (c * (ti - phi).cos() - d * (ti - phi).sin()) * vi;
                    h.push(h_re);
                    z.push(z_re);
                    w.push(1.0 / v_re);

                    jac.add_triplet(
                        *row,
                        theta_col(from),
                        (c * (ti - phi).sin() + d * (ti - phi).cos()) * vi,
                    );
                    jac.add_triplet(
                        *row,
                        theta_col(to),
                        -(a * tj.sin() + b_coeff * tj.cos()) * vj,
                    );
                    jac.add_triplet(
                        *row,
                        v_col(from, n),
                        -c * (ti - phi).cos() + d * (ti - phi).sin(),
                    );
                    jac.add_triplet(*row, v_col(to, n), a * tj.cos() - b_coeff * tj.sin());
                    let row_re = *row;
                    *row += 1;

                    // h_Im = (A sin θ_j + B cos θ_j)Vj - [C sin(θ_i-φ) + D cos(θ_i-φ)]Vi
                    let h_im = (a * tj.sin() + b_coeff * tj.cos()) * vj
                        - (c * (ti - phi).sin() + d * (ti - phi).cos()) * vi;
                    h.push(h_im);
                    z.push(z_im);
                    w.push(1.0 / v_im);

                    jac.add_triplet(
                        *row,
                        theta_col(from),
                        (-c * (ti - phi).cos() + d * (ti - phi).sin()) * vi,
                    );
                    jac.add_triplet(
                        *row,
                        theta_col(to),
                        (a * tj.cos() - b_coeff * tj.sin()) * vj,
                    );
                    jac.add_triplet(
                        *row,
                        v_col(from, n),
                        -c * (ti - phi).sin() - d * (ti - phi).cos(),
                    );
                    jac.add_triplet(*row, v_col(to, n), a * tj.sin() + b_coeff * tj.cos());
                    let row_im = *row;
                    *row += 1;

                    if pmu.correlated {
                        let cov = sin_z * cos_z * (v_m - v_a * z_i * z_i);
                        let det = v_re * v_im - cov * cov;
                        if det.abs() > 1e-30 {
                            w[row_re] = v_im / det;
                            w[row_im] = v_re / det;
                            w_off.push((row_re, row_im, -cov / det));
                        }
                    }
                }
            }
        }
    }
}
