//! Post-estimation evaluation: computes dependent results from the estimated
//! state variables (V, θ) and the network admittance model.
//!
//! These are deterministic evaluations — no iteration required.
//!
//! Computed quantities:
//!   - Branch flows: P, Q, I at both from/to terminals
//!   - Branch losses: ΔP = P_from + P_to, ΔQ = Q_from + Q_to
//!   - Bus injections: P_i, Q_i from the Y_bus model
//!   - Bus current injection magnitudes

use crate::model::ac_model::{AcModel, BranchParams};
use crate::model::network::PowerSystem;

/// Power flow result at one terminal of a branch.
#[derive(Debug, Clone)]
pub struct TerminalFlow {
    pub p: f64,
    pub q: f64,
    pub i_mag: f64,
}

/// Complete power flow result for a single branch.
#[derive(Debug, Clone)]
pub struct BranchFlowResult {
    pub branch_index: usize,
    pub from_bus: usize,
    pub to_bus: usize,
    pub from: TerminalFlow,
    pub to: TerminalFlow,
    pub p_loss: f64,
    pub q_loss: f64,
}

/// Net injection at a bus computed from the Y_bus model.
#[derive(Debug, Clone)]
pub struct BusInjectionResult {
    pub bus_index: usize,
    pub p_inj: f64,
    pub q_inj: f64,
}

/// All dependent results computed from the estimated state.
#[derive(Debug, Clone)]
pub struct PostEstimationResult {
    pub branches: Vec<BranchFlowResult>,
    pub buses: Vec<BusInjectionResult>,
    pub total_p_loss: f64,
    pub total_q_loss: f64,
    pub total_p_generation: f64,
    pub total_q_generation: f64,
    pub total_p_load: f64,
    pub total_q_load: f64,
}

/// Evaluate all dependent results from the estimated state variables.
pub fn evaluate_post_estimation(
    system: &PowerSystem,
    model: &AcModel,
    v_mag: &[f64],
    theta: &[f64],
) -> PostEstimationResult {
    let branches = compute_branch_flows(system, model, v_mag, theta);
    let buses = compute_bus_injections(system, model, v_mag, theta);

    let total_p_loss: f64 = branches.iter().map(|b| b.p_loss).sum();
    let total_q_loss: f64 = branches.iter().map(|b| b.q_loss).sum();

    let total_p_load: f64 = system.buses.iter().map(|b| b.active_demand).sum();
    let total_q_load: f64 = system.buses.iter().map(|b| b.reactive_demand).sum();

    let total_p_generation: f64 = buses.iter().map(|b| b.p_inj).sum::<f64>() + total_p_load;
    let total_q_generation: f64 = buses.iter().map(|b| b.q_inj).sum::<f64>() + total_q_load;

    PostEstimationResult {
        branches,
        buses,
        total_p_loss,
        total_q_loss,
        total_p_generation,
        total_q_generation,
        total_p_load,
        total_q_load,
    }
}

/// Compute P, Q, I at both terminals and losses for every active branch.
fn compute_branch_flows(
    system: &PowerSystem,
    model: &AcModel,
    v_mag: &[f64],
    theta: &[f64],
) -> Vec<BranchFlowResult> {
    let mut results = Vec::with_capacity(system.branches.len());

    for (br_idx, (br, bp)) in system
        .branches
        .iter()
        .zip(model.branch_params.iter())
        .enumerate()
    {
        if !br.status {
            results.push(BranchFlowResult {
                branch_index: br_idx,
                from_bus: bp.from_idx,
                to_bus: bp.to_idx,
                from: TerminalFlow { p: 0.0, q: 0.0, i_mag: 0.0 },
                to: TerminalFlow { p: 0.0, q: 0.0, i_mag: 0.0 },
                p_loss: 0.0,
                q_loss: 0.0,
            });
            continue;
        }

        let from = compute_terminal_flow(bp, v_mag, theta, true);
        let to = compute_terminal_flow(bp, v_mag, theta, false);

        results.push(BranchFlowResult {
            branch_index: br_idx,
            from_bus: bp.from_idx,
            to_bus: bp.to_idx,
            p_loss: from.p + to.p,
            q_loss: from.q + to.q,
            from,
            to,
        });
    }

    results
}

/// Compute P, Q, I at one terminal of a branch.
fn compute_terminal_flow(
    bp: &BranchParams,
    v_mag: &[f64],
    theta: &[f64],
    is_from: bool,
) -> TerminalFlow {
    let i = bp.from_idx;
    let j = bp.to_idx;
    let vi = v_mag[i];
    let vj = v_mag[j];
    let theta_ij = theta[i] - theta[j];
    let phi = bp.phi;
    let tau = bp.tau;
    let g = bp.g;
    let b = bp.b;
    let g_s = bp.g_s;
    let b_s = bp.b_s;
    let cos_tp = (theta_ij - phi).cos();
    let sin_tp = (theta_ij - phi).sin();

    let (p, q) = if is_from {
        // P_from = (g+g_s)/τ² Vi² - (1/τ)[g cos(θ-φ) + b sin(θ-φ)] Vi Vj
        let p_val = (g + g_s) / (tau * tau) * vi * vi
            - (1.0 / tau) * (g * cos_tp + b * sin_tp) * vi * vj;
        // Q_from = -(b+b_s)/τ² Vi² - (1/τ)[g sin(θ-φ) - b cos(θ-φ)] Vi Vj
        let q_val = -(b + b_s) / (tau * tau) * vi * vi
            - (1.0 / tau) * (g * sin_tp - b * cos_tp) * vi * vj;
        (p_val, q_val)
    } else {
        // P_to = (g+g_s) Vj² - (1/τ)[g cos(θ-φ) - b sin(θ-φ)] Vi Vj
        let p_val = (g + g_s) * vj * vj
            - (1.0 / tau) * (g * cos_tp - b * sin_tp) * vi * vj;
        // Q_to = -(b+b_s) Vj² + (1/τ)[g sin(θ-φ) + b cos(θ-φ)] Vi Vj
        let q_val = -(b + b_s) * vj * vj
            + (1.0 / tau) * (g * sin_tp + b * cos_tp) * vi * vj;
        (p_val, q_val)
    };

    let s_mag = (p * p + q * q).sqrt();
    let v_terminal = if is_from { vi } else { vj };
    let i_mag = if v_terminal > 1e-10 { s_mag / v_terminal } else { 0.0 };

    TerminalFlow { p, q, i_mag }
}

/// Compute net complex power injection at every bus: S_i = V_i Σ_j Y*_ij V*_j.
fn compute_bus_injections(
    _system: &PowerSystem,
    model: &AcModel,
    v_mag: &[f64],
    theta: &[f64],
) -> Vec<BusInjectionResult> {
    let n = v_mag.len();
    let mut results = Vec::with_capacity(n);

    for i in 0..n {
        let vi = v_mag[i];
        let mut p_sum = 0.0;
        let mut q_sum = 0.0;

        if let (Some(g_row), Some(b_row)) = (
            model.g_bus.outer_view(i),
            model.b_bus.outer_view(i),
        ) {
            for (j, &gij) in g_row.iter() {
                let bij = b_row.get(j).copied().unwrap_or(0.0);
                let vj = v_mag[j];
                let theta_ij = theta[i] - theta[j];
                let cos_t = theta_ij.cos();
                let sin_t = theta_ij.sin();

                p_sum += (gij * cos_t + bij * sin_t) * vj;
                q_sum += (gij * sin_t - bij * cos_t) * vj;
            }
        }

        results.push(BusInjectionResult {
            bus_index: i,
            p_inj: vi * p_sum,
            q_inj: vi * q_sum,
        });
    }

    results
}
