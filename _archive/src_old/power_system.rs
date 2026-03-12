use num_complex::Complex64;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Bus types matching JuliaGrid conventions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BusType {
    /// PQ bus (load bus) – default
    PQ = 1,
    /// PV bus (generator bus)
    PV = 2,
    /// Slack / reference bus
    Slack = 3,
}

/// A single bus in the power system network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bus {
    pub label: usize,
    pub bus_type: BusType,
    /// Active power demand (p.u.)
    pub active_demand: f64,
    /// Reactive power demand (p.u.)
    pub reactive_demand: f64,
    /// Shunt conductance at this bus (p.u.)
    pub shunt_conductance: f64,
    /// Shunt susceptance at this bus (p.u.)
    pub shunt_susceptance: f64,
    /// Initial voltage magnitude (p.u.) for flat-start
    pub voltage_magnitude: f64,
    /// Initial voltage angle (rad) for flat-start
    pub voltage_angle: f64,
}

impl Bus {
    pub fn new(label: usize) -> Self {
        Self {
            label,
            bus_type: BusType::PQ,
            active_demand: 0.0,
            reactive_demand: 0.0,
            shunt_conductance: 0.0,
            shunt_susceptance: 0.0,
            voltage_magnitude: 1.0,
            voltage_angle: 0.0,
        }
    }
}

/// A single branch described by the unified π-model.
///
/// The primary (from) side carries the transformer tap.
/// Series admittance y_ij = 1 / (r_ij + j x_ij).
/// Transformer complex ratio α_ij = (1/τ_ij) e^{-j φ_ij}.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Branch {
    pub label: usize,
    pub from: usize,
    pub to: usize,
    /// Series resistance (p.u.)
    pub resistance: f64,
    /// Series reactance (p.u.)
    pub reactance: f64,
    /// Total line-charging susceptance (p.u.) — split equally to each side
    pub susceptance: f64,
    /// Total line-charging conductance (p.u.) — split equally to each side
    pub conductance: f64,
    /// Off-nominal transformer turns ratio (τ), default = 1.0
    pub tap_ratio: f64,
    /// Phase shift angle (φ) in radians, default = 0.0
    pub shift_angle: f64,
    /// Whether the branch is in service
    pub status: bool,
}

impl Branch {
    pub fn new(label: usize, from: usize, to: usize) -> Self {
        Self {
            label,
            from,
            to,
            resistance: 0.0,
            reactance: 0.0,
            susceptance: 0.0,
            conductance: 0.0,
            tap_ratio: 1.0,
            shift_angle: 0.0,
            status: true,
        }
    }

    /// Series admittance y_ij = 1 / (r + jx).
    pub fn series_admittance(&self) -> Complex64 {
        let z = Complex64::new(self.resistance, self.reactance);
        if z.norm() < 1e-15 {
            Complex64::new(0.0, 0.0)
        } else {
            1.0 / z
        }
    }

    /// Half-line shunt admittance y_s = (g_s + j b_s) / 2.
    /// JuliaGrid puts half on each side of the π-model.
    pub fn shunt_admittance_half(&self) -> Complex64 {
        Complex64::new(self.conductance / 2.0, self.susceptance / 2.0)
    }

    /// Transformer complex ratio α = (1/τ) e^{-jφ}.
    pub fn transformer_ratio(&self) -> Complex64 {
        let inv_tau = 1.0 / self.tap_ratio;
        Complex64::new(
            inv_tau * self.shift_angle.cos(),
            -inv_tau * self.shift_angle.sin(),
        )
    }
}

/// Top-level power system container analogous to JuliaGrid's `PowerSystem` type.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerSystem {
    pub buses: Vec<Bus>,
    pub branches: Vec<Branch>,
    /// Map from bus label to its index in `buses`
    #[serde(skip)]
    pub bus_index: HashMap<usize, usize>,
    /// Index of the slack bus in `buses`
    pub slack_index: usize,
}

impl PowerSystem {
    pub fn new() -> Self {
        Self {
            buses: Vec::new(),
            branches: Vec::new(),
            bus_index: HashMap::new(),
            slack_index: 0,
        }
    }

    pub fn add_bus(&mut self, mut bus: Bus) -> &mut Self {
        let idx = self.buses.len();
        if bus.bus_type == BusType::Slack {
            self.slack_index = idx;
        }
        self.bus_index.insert(bus.label, idx);
        // Ensure bus label is set
        if bus.label == 0 {
            bus.label = idx + 1;
        }
        self.buses.push(bus);
        self
    }

    pub fn add_branch(&mut self, branch: Branch) -> &mut Self {
        self.branches.push(branch);
        self
    }

    pub fn n_buses(&self) -> usize {
        self.buses.len()
    }

    pub fn n_branches(&self) -> usize {
        self.branches.len()
    }

    /// Get the internal index of a bus by its label.
    pub fn bus_idx(&self, label: usize) -> usize {
        self.bus_index[&label]
    }
}
