//! JSON I/O for loading power system case study data exported by the Python
//! extraction scripts (e.g., `extract_gb_network.py`).

use serde::Deserialize;
use std::collections::HashMap;
use std::fs;
use std::path::Path;

use crate::measurement::*;
use crate::power_system::*;

#[derive(Deserialize)]
pub struct CaseData {
    pub name: String,
    pub description: String,
    pub n_buses: usize,
    pub n_branches: usize,
    pub slack_bus_index: usize,
    pub base_mva: f64,
    pub buses: Vec<BusJson>,
    pub branches: Vec<BranchJson>,
    pub measurements: MeasurementsJson,
    pub true_state: TrueStateJson,
}

#[derive(Deserialize)]
pub struct BusJson {
    pub label: usize,
    pub bus_type: u8,
    pub p_demand_pu: f64,
    pub q_demand_pu: f64,
    pub g_shunt_pu: f64,
    pub b_shunt_pu: f64,
    pub vm_init: f64,
    pub va_init: f64,
    pub vn_kv: f64,
    #[serde(default)]
    pub geo_x: Option<f64>,
    #[serde(default)]
    pub geo_y: Option<f64>,
}

#[derive(Deserialize)]
pub struct BranchJson {
    pub label: usize,
    pub from_bus: usize,
    pub to_bus: usize,
    pub resistance: f64,
    pub reactance: f64,
    pub susceptance: f64,
    pub conductance: f64,
    pub tap_ratio: f64,
    pub shift_angle: f64,
    pub status: bool,
}

#[derive(Deserialize)]
pub struct MeasurementsJson {
    pub voltmeters: Vec<VoltmeterJson>,
    pub ammeters: Vec<AmmeterJson>,
    pub wattmeters: Vec<WattmeterJson>,
    pub varmeters: Vec<VarmeterJson>,
    pub pmus: Vec<PmuJson>,
}

#[derive(Deserialize)]
pub struct VoltmeterJson {
    pub label: String,
    pub bus: usize,
    pub magnitude: f64,
    pub variance: f64,
}

#[derive(Deserialize)]
pub struct AmmeterJson {
    pub label: String,
    pub branch: usize,
    pub end: String,
    pub magnitude: f64,
    pub variance: f64,
    pub square: bool,
}

#[derive(Deserialize)]
pub struct WattmeterJson {
    pub label: String,
    pub location: LocationJson,
    pub active: f64,
    pub variance: f64,
}

#[derive(Deserialize)]
pub struct VarmeterJson {
    pub label: String,
    pub location: LocationJson,
    pub reactive: f64,
    pub variance: f64,
}

#[derive(Deserialize)]
#[serde(untagged)]
pub enum LocationJson {
    Bus(BusLocJson),
    Branch(BranchLocJson),
}

#[derive(Deserialize)]
pub struct BusLocJson {
    #[serde(rename = "Bus")]
    pub bus: usize,
}

#[derive(Deserialize)]
pub struct BranchLocJson {
    #[serde(rename = "Branch")]
    pub branch: BranchEndJson,
}

#[derive(Deserialize)]
pub struct BranchEndJson {
    pub branch: usize,
    pub end: String,
}

#[derive(Deserialize)]
pub struct PmuJson {
    pub label: String,
    pub location: LocationJson,
    pub magnitude: f64,
    pub angle: f64,
    pub variance_magnitude: f64,
    pub variance_angle: f64,
    pub coordinate: String,
    pub correlated: bool,
    pub square: bool,
}

#[derive(Deserialize)]
pub struct TrueStateJson {
    pub voltage_magnitude: Vec<f64>,
    pub voltage_angle: Vec<f64>,
}

/// Load a case JSON file and build `PowerSystem` + `MeasurementSet`.
pub fn load_case(path: &Path) -> Result<(CaseData, PowerSystem, MeasurementSet), String> {
    let json_str = fs::read_to_string(path).map_err(|e| format!("Failed to read {}: {}", path.display(), e))?;
    let case: CaseData = serde_json::from_str(&json_str).map_err(|e| format!("JSON parse error: {}", e))?;

    let mut system = PowerSystem::new();
    let mut bus_label_remap: HashMap<usize, usize> = HashMap::new();

    for (seq, bus) in case.buses.iter().enumerate() {
        let bus_type = match bus.bus_type {
            3 => BusType::Slack,
            2 => BusType::PV,
            _ => BusType::PQ,
        };
        let mut b = Bus::new(bus.label);
        b.bus_type = bus_type;
        b.active_demand = bus.p_demand_pu;
        b.reactive_demand = bus.q_demand_pu;
        b.shunt_conductance = bus.g_shunt_pu;
        b.shunt_susceptance = bus.b_shunt_pu;
        b.voltage_magnitude = bus.vm_init;
        b.voltage_angle = bus.va_init;
        system.add_bus(b);
        bus_label_remap.insert(bus.label, seq);
    }

    for br in &case.branches {
        if !br.status {
            continue;
        }
        let mut branch = Branch::new(br.label, br.from_bus, br.to_bus);
        branch.resistance = br.resistance;
        branch.reactance = br.reactance;
        branch.susceptance = br.susceptance;
        branch.conductance = br.conductance;
        branch.tap_ratio = br.tap_ratio;
        branch.shift_angle = br.shift_angle;
        system.add_branch(branch);
    }

    let mut meas = MeasurementSet::new();

    for vm in &case.measurements.voltmeters {
        meas.add_bus_voltmeter(&vm.label, vm.bus, vm.magnitude, vm.variance);
    }

    for am in &case.measurements.ammeters {
        let end = if am.end == "From" { BranchEnd::From } else { BranchEnd::To };
        meas.add_branch_ammeter(&am.label, am.branch, end, am.magnitude, am.variance, am.square);
    }

    for wm in &case.measurements.wattmeters {
        match &wm.location {
            LocationJson::Bus(b) => {
                meas.add_bus_wattmeter(&wm.label, b.bus, wm.active, wm.variance);
            }
            LocationJson::Branch(b) => {
                let end = if b.branch.end == "From" { BranchEnd::From } else { BranchEnd::To };
                meas.add_branch_wattmeter(&wm.label, b.branch.branch, end, wm.active, wm.variance);
            }
        }
    }

    for vm in &case.measurements.varmeters {
        match &vm.location {
            LocationJson::Bus(b) => {
                meas.add_bus_varmeter(&vm.label, b.bus, vm.reactive, vm.variance);
            }
            LocationJson::Branch(b) => {
                let end = if b.branch.end == "From" { BranchEnd::From } else { BranchEnd::To };
                meas.add_branch_varmeter(&vm.label, b.branch.branch, end, vm.reactive, vm.variance);
            }
        }
    }

    for pmu in &case.measurements.pmus {
        let polar = pmu.coordinate == "Polar";
        match &pmu.location {
            LocationJson::Bus(b) => {
                meas.add_bus_pmu(
                    &pmu.label, b.bus, pmu.magnitude, pmu.angle,
                    pmu.variance_magnitude, pmu.variance_angle, polar, pmu.correlated,
                );
            }
            LocationJson::Branch(b) => {
                let end = if b.branch.end == "From" { BranchEnd::From } else { BranchEnd::To };
                meas.add_branch_pmu(
                    &pmu.label, b.branch.branch, end, pmu.magnitude, pmu.angle,
                    pmu.variance_magnitude, pmu.variance_angle, polar, pmu.correlated,
                );
            }
        }
    }

    Ok((case, system, meas))
}
