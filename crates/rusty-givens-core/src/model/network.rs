use num_complex::Complex64;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusType {
    PQ = 1,
    PV = 2,
    Slack = 3,
}

#[derive(Debug, Clone)]
pub struct Bus {
    pub label: usize,
    pub bus_type: BusType,
    pub active_demand: f64,
    pub reactive_demand: f64,
    pub shunt_conductance: f64,
    pub shunt_susceptance: f64,
    pub voltage_magnitude: f64,
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

#[derive(Debug, Clone)]
pub struct Branch {
    pub label: usize,
    pub from: usize,
    pub to: usize,
    pub resistance: f64,
    pub reactance: f64,
    pub susceptance: f64,
    pub conductance: f64,
    pub tap_ratio: f64,
    pub shift_angle: f64,
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

    pub fn series_admittance(&self) -> Complex64 {
        let z = Complex64::new(self.resistance, self.reactance);
        if z.norm() < 1e-15 {
            Complex64::new(0.0, 0.0)
        } else {
            1.0 / z
        }
    }

    pub fn shunt_admittance_half(&self) -> Complex64 {
        Complex64::new(self.conductance / 2.0, self.susceptance / 2.0)
    }

    pub fn transformer_ratio(&self) -> Complex64 {
        let inv_tau = 1.0 / self.tap_ratio;
        Complex64::new(
            inv_tau * self.shift_angle.cos(),
            -inv_tau * self.shift_angle.sin(),
        )
    }
}

#[derive(Debug, Clone)]
pub struct PowerSystem {
    pub buses: Vec<Bus>,
    pub branches: Vec<Branch>,
    pub bus_index: HashMap<usize, usize>,
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

    pub fn bus_idx(&self, label: usize) -> usize {
        self.bus_index[&label]
    }
}

impl Default for PowerSystem {
    fn default() -> Self {
        Self::new()
    }
}
