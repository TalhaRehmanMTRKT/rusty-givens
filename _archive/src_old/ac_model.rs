use num_complex::Complex64;
use sprs::{CsMat, TriMat};

use crate::power_system::PowerSystem;

/// AC model data computed from the power system topology.
///
/// Contains the nodal admittance matrix (Y-bus) and per-branch
/// parameters needed for measurement functions and Jacobians.
#[derive(Debug, Clone)]
pub struct AcModel {
    /// Nodal admittance matrix Y = G + jB  (n × n, sparse)
    pub y_bus: CsMat<Complex64>,
    /// Real part of Y-bus (conductance matrix)
    pub g_bus: CsMat<f64>,
    /// Imaginary part of Y-bus (susceptance matrix)
    pub b_bus: CsMat<f64>,
    /// Per-branch precomputed parameters for measurement functions
    pub branch_params: Vec<BranchParams>,
}

/// Precomputed branch parameters derived from the unified π-model.
///
/// For a branch (i,j): series admittance y = g + jb, half-shunt y_s = g_s + jb_s,
/// tap ratio τ, shift angle φ.
#[derive(Debug, Clone)]
pub struct BranchParams {
    pub from_idx: usize,
    pub to_idx: usize,
    /// Series conductance g_ij
    pub g: f64,
    /// Series susceptance b_ij
    pub b: f64,
    /// Half-shunt conductance g_s
    pub g_s: f64,
    /// Half-shunt susceptance b_s
    pub b_s: f64,
    /// Tap ratio τ (≠ 0)
    pub tau: f64,
    /// Phase shift angle φ (rad)
    pub phi: f64,
}

impl BranchParams {
    // ── From-bus current magnitude squared coefficients ──
    pub fn a_from(&self) -> f64 {
        let t2 = self.tau * self.tau;
        ((self.g + self.g_s).powi(2) + (self.b + self.b_s).powi(2)) / (t2 * t2)
    }
    pub fn b_from(&self) -> f64 {
        (self.g.powi(2) + self.b.powi(2)) / (self.tau * self.tau)
    }
    pub fn c_from(&self) -> f64 {
        let t3 = self.tau.powi(3);
        (self.g * (self.g + self.g_s) + self.b * (self.b + self.b_s)) / t3
    }
    pub fn d_from(&self) -> f64 {
        (self.g * self.b_s - self.b * self.g_s) / self.tau.powi(3)
    }

    // ── To-bus current magnitude squared coefficients ──
    pub fn a_to(&self) -> f64 {
        (self.g.powi(2) + self.b.powi(2)) / (self.tau * self.tau)
    }
    pub fn b_to(&self) -> f64 {
        (self.g + self.g_s).powi(2) + (self.b + self.b_s).powi(2)
    }
    pub fn c_to(&self) -> f64 {
        (self.g * (self.g + self.g_s) + self.b * (self.b + self.b_s)) / self.tau
    }
    pub fn d_to(&self) -> f64 {
        (self.g * self.b_s - self.b * self.g_s) / self.tau
    }

    // ── Phasor angle coefficients (from-bus) ──
    pub fn a_psi_from(&self) -> f64 {
        (self.g + self.g_s) / (self.tau * self.tau)
    }
    pub fn b_psi_from(&self) -> f64 {
        (self.b + self.b_s) / (self.tau * self.tau)
    }
    pub fn c_psi_from(&self) -> f64 {
        self.g / self.tau
    }
    pub fn d_psi_from(&self) -> f64 {
        self.b / self.tau
    }

    // ── Phasor angle coefficients (to-bus) ──
    pub fn a_psi_to(&self) -> f64 {
        self.g + self.g_s
    }
    pub fn b_psi_to(&self) -> f64 {
        self.b + self.b_s
    }
    pub fn c_psi_to(&self) -> f64 {
        self.g / self.tau
    }
    pub fn d_psi_to(&self) -> f64 {
        self.b / self.tau
    }
}

/// Build the AC model (Y-bus and branch parameters) from the power system.
///
/// Follows JuliaGrid's `acModel!` logic based on the unified branch model
/// from equation (1) of the paper.
pub fn build_ac_model(system: &PowerSystem) -> AcModel {
    let n = system.n_buses();

    let mut y_trip = TriMat::new((n, n));

    let mut branch_params = Vec::with_capacity(system.n_branches());

    for br in &system.branches {
        if !br.status {
            continue;
        }
        let i = system.bus_idx(br.from);
        let j = system.bus_idx(br.to);

        let y_series = br.series_admittance();
        let y_shunt = br.shunt_admittance_half();
        let tau = br.tap_ratio;
        let phi = br.shift_angle;
        let alpha = br.transformer_ratio(); // (1/τ) e^{-jφ}
        let alpha_conj = alpha.conj();

        // From equation (1) in the paper — the 2×2 primitive admittance matrix:
        //   Y_ii += (y + y_s) / τ²
        //   Y_jj += (y + y_s)
        //   Y_ij += -α* y
        //   Y_ji += -α  y
        let y_ii = (y_series + y_shunt) / (tau * tau);
        let y_jj = y_series + y_shunt;
        let y_ij = -alpha_conj * y_series;
        let y_ji = -alpha * y_series;

        y_trip.add_triplet(i, i, y_ii);
        y_trip.add_triplet(j, j, y_jj);
        y_trip.add_triplet(i, j, y_ij);
        y_trip.add_triplet(j, i, y_ji);

        branch_params.push(BranchParams {
            from_idx: i,
            to_idx: j,
            g: y_series.re,
            b: y_series.im,
            g_s: y_shunt.re,
            b_s: y_shunt.im,
            tau,
            phi,
        });
    }

    // Add bus shunt elements to Y-bus diagonal
    for (idx, bus) in system.buses.iter().enumerate() {
        if bus.shunt_conductance != 0.0 || bus.shunt_susceptance != 0.0 {
            let y_sh = Complex64::new(bus.shunt_conductance, bus.shunt_susceptance);
            y_trip.add_triplet(idx, idx, y_sh);
        }
    }

    let y_bus: CsMat<Complex64> = y_trip.to_csc();

    let mut g_trip = TriMat::new((n, n));
    let mut b_trip = TriMat::new((n, n));
    for (val, (row, col)) in y_bus.iter() {
        g_trip.add_triplet(row, col, val.re);
        b_trip.add_triplet(row, col, val.im);
    }

    AcModel {
        y_bus,
        g_bus: g_trip.to_csc(),
        b_bus: b_trip.to_csc(),
        branch_params,
    }
}
