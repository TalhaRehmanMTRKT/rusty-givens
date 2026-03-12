use num_complex::Complex64;
use sprs::{CsMat, TriMat};

use super::network::PowerSystem;

#[derive(Debug, Clone)]
pub struct AcModel {
    pub y_bus: CsMat<Complex64>,
    pub g_bus: CsMat<f64>,
    pub b_bus: CsMat<f64>,
    pub branch_params: Vec<BranchParams>,
}

#[derive(Debug, Clone)]
pub struct BranchParams {
    pub from_idx: usize,
    pub to_idx: usize,
    pub g: f64,
    pub b: f64,
    pub g_s: f64,
    pub b_s: f64,
    pub tau: f64,
    pub phi: f64,
}

impl BranchParams {
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
        let alpha = br.transformer_ratio();
        let alpha_conj = alpha.conj();

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
