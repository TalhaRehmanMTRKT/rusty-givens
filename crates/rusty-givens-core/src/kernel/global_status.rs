//! Global SE status: comprehensive result summary for the complete network
//! and per voltage level, as required by operational SE standards.

use std::collections::BTreeMap;
use std::time::SystemTime;

use crate::model::{MeasurementSet, PowerSystem};
use super::post_estimation::PostEstimationResult;
use super::types::{EstimationResult, SolverArtifacts, MeasurementRef};

/// Measurement counts broken down by type.
#[derive(Debug, Clone, Default)]
pub struct MeasurementCounts {
    pub voltmeters: usize,
    pub ammeters: usize,
    pub wattmeters: usize,
    pub varmeters: usize,
    pub pmu_pairs: usize,
    pub current_angle_meters: usize,
    pub total: usize,
}

impl MeasurementCounts {
    pub fn from_measurement_set(meas: &MeasurementSet) -> Self {
        let voltmeters = meas.voltmeters.iter().filter(|m| m.status).count();
        let ammeters = meas.ammeters.iter().filter(|m| m.status).count();
        let wattmeters = meas.wattmeters.iter().filter(|m| m.status).count();
        let varmeters = meas.varmeters.iter().filter(|m| m.status).count();
        let pmu_pairs = meas.pmus.iter().filter(|m| m.status).count();
        let current_angle_meters = meas.current_angle_meters.iter().filter(|m| m.status).count();
        let total = voltmeters + ammeters + wattmeters + varmeters + 2 * pmu_pairs + current_angle_meters;
        Self { voltmeters, ammeters, wattmeters, varmeters, pmu_pairs, current_angle_meters, total }
    }

    pub fn from_artifact_map(map: &[MeasurementRef]) -> Self {
        let mut c = Self::default();
        for m in map {
            match m {
                MeasurementRef::Voltmeter { .. } => c.voltmeters += 1,
                MeasurementRef::Ammeter { .. } => c.ammeters += 1,
                MeasurementRef::Wattmeter { .. } => c.wattmeters += 1,
                MeasurementRef::Varmeter { .. } => c.varmeters += 1,
                MeasurementRef::PmuMagnitude { .. } => c.pmu_pairs += 1,
                MeasurementRef::PmuAngle { .. } => {}
                MeasurementRef::CurrentAngle { .. } => c.current_angle_meters += 1,
            }
        }
        c.total = c.voltmeters + c.ammeters + c.wattmeters + c.varmeters
            + 2 * c.pmu_pairs + c.current_angle_meters;
        c
    }
}

/// Chi-squared (objective function) summary.
#[derive(Debug, Clone)]
pub struct ObjectiveSummary {
    /// J(x̂) = Σ w_i r_i²  (weighted sum of squared residuals)
    pub objective_value: f64,
    /// E[J] = m - n  (degrees of freedom)
    pub expected_value: f64,
    /// Degrees of freedom: m - (2n - 1)
    pub degrees_of_freedom: i64,
}

/// Per-voltage-level breakdown of network and measurement statistics.
#[derive(Debug, Clone)]
pub struct VoltageLevelStats {
    pub voltage_kv: f64,
    pub n_buses: usize,
    pub n_branches: usize,
    pub measurement_counts: MeasurementCounts,
}

/// Complete global status for the SE result.
#[derive(Debug, Clone)]
pub struct SeGlobalStatus {
    pub timestamp: SystemTime,
    pub n_buses: usize,
    pub n_branches: usize,
    pub n_state_variables: usize,

    pub converged: bool,
    pub iterations: usize,
    pub max_iterations_config: usize,
    pub tolerance_config: f64,
    pub final_increment: f64,
    pub se_time_seconds: f64,
    pub formulation_name: String,

    pub objective: Option<ObjectiveSummary>,
    pub measurement_counts: MeasurementCounts,

    pub per_voltage_level: Vec<VoltageLevelStats>,

    pub post_estimation: Option<PostEstimationResult>,
}

/// Compute the WLS objective J(x̂) = rᵀ W r from solver artifacts.
pub fn compute_objective(artifacts: &SolverArtifacts) -> ObjectiveSummary {
    let m = artifacts.residuals.len();
    let s = artifacts.n_states;
    let n_free = if s > 0 { s - 1 } else { 0 };

    let mut j_val = 0.0;
    for i in 0..m {
        let r = artifacts.residuals[i];
        let w = artifacts.precision_diag[i];
        j_val += w * r * r;
    }
    for &(r1, r2, w_off) in &artifacts.precision_off {
        if r1 < m && r2 < m {
            j_val += 2.0 * w_off * artifacts.residuals[r1] * artifacts.residuals[r2];
        }
    }

    ObjectiveSummary {
        objective_value: j_val,
        expected_value: (m as i64 - n_free as i64) as f64,
        degrees_of_freedom: m as i64 - n_free as i64,
    }
}

/// Build the global status from the SE result and related data.
pub fn build_global_status(
    system: &PowerSystem,
    measurements: &MeasurementSet,
    se_result: &EstimationResult,
    se_time_seconds: f64,
    formulation_name: &str,
    max_iterations_config: usize,
    tolerance_config: f64,
    bus_vn_kv: &[f64],
    post_estimation: Option<PostEstimationResult>,
) -> SeGlobalStatus {
    let n = system.n_buses();
    let s = 2 * n;

    let objective = se_result
        .artifacts
        .as_ref()
        .map(compute_objective);

    let measurement_counts = MeasurementCounts::from_measurement_set(measurements);

    let per_voltage_level = build_voltage_level_stats(system, measurements, bus_vn_kv);

    SeGlobalStatus {
        timestamp: SystemTime::now(),
        n_buses: n,
        n_branches: system.n_branches(),
        n_state_variables: s,
        converged: se_result.converged,
        iterations: se_result.iterations,
        max_iterations_config,
        tolerance_config,
        final_increment: se_result.final_increment,
        se_time_seconds,
        formulation_name: formulation_name.to_string(),
        objective,
        measurement_counts,
        per_voltage_level,
        post_estimation,
    }
}

/// Group buses/branches/measurements by voltage level.
fn build_voltage_level_stats(
    system: &PowerSystem,
    measurements: &MeasurementSet,
    bus_vn_kv: &[f64],
) -> Vec<VoltageLevelStats> {
    if bus_vn_kv.is_empty() {
        return Vec::new();
    }

    let mut vl_buses: BTreeMap<u64, Vec<usize>> = BTreeMap::new();
    for (i, &kv) in bus_vn_kv.iter().enumerate() {
        let key = (kv * 100.0).round() as u64;
        vl_buses.entry(key).or_default().push(i);
    }

    let mut results = Vec::new();
    for (&key, bus_indices) in &vl_buses {
        let voltage_kv = key as f64 / 100.0;
        let n_buses = bus_indices.len();

        let mut bus_set = vec![false; system.n_buses()];
        for &bi in bus_indices {
            bus_set[bi] = true;
        }

        let n_branches = system
            .branches
            .iter()
            .filter(|b| b.status && (bus_set[system.bus_idx(b.from)] || bus_set[system.bus_idx(b.to)]))
            .count();

        let mut counts = MeasurementCounts::default();
        for vm in measurements.voltmeters.iter().filter(|m| m.status) {
            if bus_set[system.bus_idx(vm.bus)] { counts.voltmeters += 1; }
        }
        for wm in measurements.wattmeters.iter().filter(|m| m.status) {
            match &wm.location {
                crate::model::measurement::WattmeterLocation::Bus(bus) => {
                    if bus_set[system.bus_idx(*bus)] { counts.wattmeters += 1; }
                }
                crate::model::measurement::WattmeterLocation::Branch { branch, .. } => {
                    let bp = &system.branches[*branch - 1];
                    if bus_set[system.bus_idx(bp.from)] || bus_set[system.bus_idx(bp.to)] {
                        counts.wattmeters += 1;
                    }
                }
            }
        }
        for vm in measurements.varmeters.iter().filter(|m| m.status) {
            match &vm.location {
                crate::model::measurement::VarmeterLocation::Bus(bus) => {
                    if bus_set[system.bus_idx(*bus)] { counts.varmeters += 1; }
                }
                crate::model::measurement::VarmeterLocation::Branch { branch, .. } => {
                    let bp = &system.branches[*branch - 1];
                    if bus_set[system.bus_idx(bp.from)] || bus_set[system.bus_idx(bp.to)] {
                        counts.varmeters += 1;
                    }
                }
            }
        }
        for am in measurements.ammeters.iter().filter(|m| m.status) {
            let bp = &system.branches[am.branch - 1];
            if bus_set[system.bus_idx(bp.from)] || bus_set[system.bus_idx(bp.to)] {
                counts.ammeters += 1;
            }
        }
        for pmu in measurements.pmus.iter().filter(|m| m.status) {
            let in_vl = match &pmu.location {
                crate::model::measurement::PmuLocation::Bus(bus) => bus_set[system.bus_idx(*bus)],
                crate::model::measurement::PmuLocation::Branch { branch, .. } => {
                    let bp = &system.branches[*branch - 1];
                    bus_set[system.bus_idx(bp.from)] || bus_set[system.bus_idx(bp.to)]
                }
            };
            if in_vl { counts.pmu_pairs += 1; }
        }
        counts.total = counts.voltmeters + counts.ammeters + counts.wattmeters
            + counts.varmeters + 2 * counts.pmu_pairs + counts.current_angle_meters;

        results.push(VoltageLevelStats {
            voltage_kv,
            n_buses,
            n_branches,
            measurement_counts: counts,
        });
    }

    results
}
