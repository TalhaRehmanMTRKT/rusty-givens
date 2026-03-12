//! HTTP API server for the State Estimation frontend.
//!
//! Provides two API transports:
//!   - REST (JSON over HTTP/1.1) on port 3001
//!   - gRPC (protobuf over HTTP/2) on port 50051
//!
//! Free edition endpoints:
//!   GET  /api/network      — topology (buses + branches) for vis-network
//!   GET  /api/true-state   — true power-flow state (for error comparison)
//!   POST /api/estimate     — run WLS SE with a given configuration
//!   GET  /api/last-result  — most recent estimation result
//!
//! Pro edition endpoints (compiled with `--features pro`):
//!   POST /api/observability  — run observability analysis
//!   POST /api/redundancy     — run redundancy analysis
//!   POST /api/bdd/run        — iterative BDD analysis
//!   GET  /api/bdd/bad-data   — active bad data set
//!   GET  /api/bdd/historical — historical bad data set
//!   GET  /api/bdd/alerts     — current BDD alerts
//!   POST /api/bdd/reset      — clear BDD state
//!   gRPC: BddService, ObsService, RedService

mod grpc;
#[cfg(feature = "pro")]
mod pro;

pub(crate) mod pb {
    pub mod rusty_givens {
        pub mod v1 {
            tonic::include_proto!("rusty_givens.v1");
        }
    }
}

use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use axum::extract::State;
use axum::http::StatusCode;
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;
use tower_http::cors::CorsLayer;

use rusty_givens_core::kernel::{
    EstimationConfig, EstimationResult, Factorization, MeasurementRef,
    SolverFormulation,
    evaluate_post_estimation, build_global_status,
    SeGlobalStatus, MeasurementCounts,
};
use rusty_givens_core::model::{build_ac_model, AcModel, MeasurementSet, PowerSystem};
use rusty_givens_io::{load_case, BusMetadata, CaseInfo, TrueState};

#[cfg(feature = "pro")]
use rusty_givens_bdd::BddState;

// ── Shared application state ────────────────────────────────────────

pub(crate) struct AppState {
    pub(crate) info: CaseInfo,
    pub(crate) system: PowerSystem,
    pub(crate) model: AcModel,
    pub(crate) measurements: MeasurementSet,
    pub(crate) true_state: TrueState,
    pub(crate) bus_metadata: Vec<BusMetadata>,
    pub(crate) last_result: Mutex<Option<SeResultPayload>>,
    pub(crate) last_se_result: Mutex<Option<EstimationResult>>,
    #[cfg(feature = "pro")]
    pub(crate) bdd_state: Mutex<BddState>,
}

// ── JSON payloads (REST) ────────────────────────────────────────────

#[derive(Serialize)]
struct BusNode {
    index: usize,
    label: usize,
    vn_kv: f64,
    bus_type: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    geo_x: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    geo_y: Option<f64>,
}

#[derive(Serialize)]
struct BranchEdge {
    index: usize,
    from_bus: usize,
    to_bus: usize,
}

#[derive(Serialize)]
struct NetworkPayload {
    n_buses: usize,
    n_branches: usize,
    slack_bus_index: usize,
    base_mva: f64,
    buses: Vec<BusNode>,
    branches: Vec<BranchEdge>,
}

#[derive(Serialize)]
struct TrueStatePayload {
    voltage_magnitude: Vec<f64>,
    voltage_angle_deg: Vec<f64>,
}

#[derive(Deserialize)]
#[allow(dead_code)]
struct EstimateRequest {
    factorization: String,
    #[serde(default)]
    formulation: Option<String>,
    max_iterations: Option<usize>,
    tolerance: Option<f64>,
    skip_obs_check: Option<bool>,
    obs_method: Option<String>,
}

#[derive(Serialize, Clone)]
pub(crate) struct BusResult {
    pub index: usize,
    pub label: usize,
    pub est_vm: f64,
    pub est_va_deg: f64,
    pub true_vm: f64,
    pub true_va_deg: f64,
    pub vm_error: f64,
    pub va_error_deg: f64,
}

#[derive(Serialize, Clone)]
pub(crate) struct ChiSquaredPayload {
    pub bad_data_suspected: bool,
    pub objective: f64,
    pub threshold: f64,
    pub degrees_of_freedom: usize,
}

#[derive(Serialize, Clone)]
pub(crate) struct FlaggedMeasurementPayload {
    pub index: usize,
    pub residual_value: f64,
    pub raw_residual: f64,
    pub omega_ii: f64,
    pub measurement_type: String,
    pub measurement_label: String,
    pub category: String,
    pub residual_type: String,
}

#[derive(Serialize, Clone)]
pub(crate) struct ResidualTestPayload {
    pub bad_data_detected: bool,
    pub residual_type: String,
    pub compute_time_s: f64,
    pub top_residuals: Vec<FlaggedMeasurementPayload>,
}

#[derive(Serialize, Clone)]
pub(crate) struct BddPayload {
    pub chi_squared: Option<ChiSquaredPayload>,
    pub residual_test: Option<ResidualTestPayload>,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct BadMarkPayload {
    pub element_type: String,
    pub element_index: usize,
    pub category: String,
    pub count: u32,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct BddAlertPayload {
    pub meas_index: usize,
    pub label: String,
    pub category: String,
    pub consecutive_count: u32,
    pub message: String,
    pub run_number: u64,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct BadDataEntryPayload {
    pub meas_index: usize,
    pub label: String,
    pub category: String,
    pub identification_count: u32,
    pub deactivated: bool,
    pub last_residual: f64,
    pub residual_type: String,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct HistoricalBadDataPayload {
    pub meas_index: usize,
    pub label: String,
    pub category: String,
    pub total_identifications: u32,
    pub release_count: u32,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct IterativeBddPayload {
    pub chi_squared: Option<ChiSquaredPayload>,
    pub residual_test: Option<ResidualTestPayload>,
    pub eliminated: Vec<FlaggedMeasurementPayload>,
    pub bad_marks: Vec<BadMarkPayload>,
    pub pseudo_measurements_injected: usize,
    pub alerts: Vec<BddAlertPayload>,
    pub reestimation_count: usize,
    pub used_compensation: bool,
    pub total_time_s: f64,
}

#[cfg(feature = "pro")]
#[derive(Deserialize)]
pub(crate) struct IterativeBddRequest {
    pub residual_type: Option<String>,
    pub residual_threshold: Option<f64>,
    pub chi2_enabled: Option<bool>,
    pub chi2_confidence: Option<f64>,
    pub max_bad_data: Option<usize>,
    pub elimination_strategy: Option<String>,
    pub inject_pseudo: Option<bool>,
    pub alert_threshold_n: Option<u32>,
    pub deactivation_m1: Option<u32>,
    pub release_cycle_m2: Option<u32>,
    pub bad_data_set_n1: Option<u32>,
    pub historical_n2: Option<u32>,
}

#[derive(Serialize, Clone)]
pub(crate) struct SeResultPayload {
    pub converged: bool,
    pub iterations: usize,
    pub se_time_seconds: f64,
    pub final_increment: f64,
    pub factorization: String,
    pub tolerance: f64,
    pub max_iterations: usize,
    pub vm_mae: f64,
    pub vm_max_error: f64,
    pub va_mae_deg: f64,
    pub va_max_error_deg: f64,
    pub buses: Vec<BusResult>,
    pub bdd: Option<BddPayload>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub obs_check: Option<ObsResultPayload>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub global_status: Option<GlobalStatusPayload>,
}

// ── Global status + dependent results payloads ──────────────────────

#[derive(Serialize, Clone)]
pub(crate) struct TerminalFlowPayload {
    pub p: f64,
    pub q: f64,
    pub i_mag: f64,
}

#[derive(Serialize, Clone)]
pub(crate) struct BranchFlowPayload {
    pub branch_index: usize,
    pub from_bus: usize,
    pub to_bus: usize,
    pub from: TerminalFlowPayload,
    pub to: TerminalFlowPayload,
    pub p_loss: f64,
    pub q_loss: f64,
}

#[derive(Serialize, Clone)]
pub(crate) struct BusInjectionPayload {
    pub bus_index: usize,
    pub p_inj: f64,
    pub q_inj: f64,
}

#[derive(Serialize, Clone)]
pub(crate) struct MeasCountsPayload {
    pub voltmeters: usize,
    pub ammeters: usize,
    pub wattmeters: usize,
    pub varmeters: usize,
    pub pmu_pairs: usize,
    pub current_angle_meters: usize,
    pub total: usize,
}

#[derive(Serialize, Clone)]
pub(crate) struct ObjectivePayload {
    pub objective_value: f64,
    pub expected_value: f64,
    pub degrees_of_freedom: i64,
}

#[derive(Serialize, Clone)]
pub(crate) struct VoltageLevelPayload {
    pub voltage_kv: f64,
    pub n_buses: usize,
    pub n_branches: usize,
    pub measurement_counts: MeasCountsPayload,
}

#[derive(Serialize, Clone)]
pub(crate) struct PowerBalancePayload {
    pub total_p_loss: f64,
    pub total_q_loss: f64,
    pub total_p_generation: f64,
    pub total_q_generation: f64,
    pub total_p_load: f64,
    pub total_q_load: f64,
}

#[derive(Serialize, Clone)]
pub(crate) struct GlobalStatusPayload {
    pub timestamp: String,
    pub n_buses: usize,
    pub n_branches: usize,
    pub n_state_variables: usize,
    pub objective: Option<ObjectivePayload>,
    pub measurement_counts: MeasCountsPayload,
    pub per_voltage_level: Vec<VoltageLevelPayload>,
    pub branch_flows: Vec<BranchFlowPayload>,
    pub bus_injections: Vec<BusInjectionPayload>,
    pub power_balance: PowerBalancePayload,
}

#[derive(Serialize, Clone)]
pub(crate) struct MeasurementExportEntry {
    pub index: usize,
    pub measurement_type: String,
    pub label: String,
    pub measured_value: f64,
    pub standard_deviation: f64,
    pub status: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estimated_value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub residual: Option<f64>,
}

// ── Observability Analysis payloads ─────────────────────────────────

#[cfg(feature = "pro")]
#[derive(Deserialize)]
pub(crate) struct ObsRequest {
    pub method: Option<String>,
    pub zero_pivot_threshold: Option<f64>,
}

#[derive(Serialize, Clone)]
pub(crate) struct ObsIslandPayload {
    pub island_id: usize,
    pub bus_indices: Vec<usize>,
    pub branch_indices: Vec<usize>,
}

#[derive(Serialize, Clone)]
pub(crate) struct SubModelPayload {
    pub kind: String,
    pub observable: bool,
    pub islands: Vec<ObsIslandPayload>,
    pub unobservable_buses: Vec<usize>,
    pub unobservable_branches: Vec<usize>,
    pub zero_pivot_states: Vec<usize>,
    pub n_measurements_used: usize,
}

#[derive(Serialize, Clone)]
pub(crate) struct PqPairPayload {
    pub all_paired: bool,
    pub n_paired: usize,
    pub unpaired_wattmeters: Vec<String>,
    pub unpaired_varmeters: Vec<String>,
    pub paired_bus_indices: Vec<usize>,
}

#[derive(Serialize, Clone)]
pub(crate) struct BusObsPayload {
    pub bus_index: usize,
    pub p_theta_observable: bool,
    pub q_v_observable: bool,
    pub p_theta_island_id: Option<usize>,
    pub q_v_island_id: Option<usize>,
    pub needs_pseudo: bool,
    pub pseudo_models: Vec<String>,
    pub detected_in: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pseudo_basis: Option<String>,
    pub paired_observable: bool,
}

#[derive(Serialize, Clone)]
pub(crate) struct BranchObsPayload {
    pub branch_index: usize,
    pub p_theta_observable: bool,
    pub q_v_observable: bool,
    pub detected_in: String,
}

#[derive(Serialize, Clone)]
pub(crate) struct ObsResultPayload {
    pub observable: bool,
    pub p_theta: SubModelPayload,
    pub q_v: SubModelPayload,
    pub pq_pairs: PqPairPayload,
    pub bus_observability: Vec<BusObsPayload>,
    pub branch_observability: Vec<BranchObsPayload>,
    pub method: String,
    pub analysis_time_s: f64,
}

// ── Redundancy Analysis payloads ─────────────────────────────────────

#[cfg(feature = "pro")]
#[derive(Deserialize)]
pub(crate) struct RedundancyRequest {
    pub w_ii_critical_threshold: Option<f64>,
    pub detection_sensitivity_min: Option<f64>,
    pub lambda_k: Option<f64>,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct MeasRedundancyPayload {
    pub index: usize,
    pub measurement_type: String,
    pub measurement_label: String,
    pub sub_system: String,
    pub w_ii: f64,
    pub max_abs_k_ik: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_k_ik_partner: Option<usize>,
    pub redundancy_class: String,
    pub detection_reliable: bool,
    pub coupling_indicator: f64,
    pub associated_buses: Vec<usize>,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct GlobalRedundancyPayload {
    pub n_buses: usize,
    pub n_state_variables: usize,
    pub n_measurements: usize,
    pub degrees_of_freedom: i64,
    pub redundancy_ratio: f64,
    pub n_voltmeters: usize,
    pub n_ammeters: usize,
    pub n_wattmeters: usize,
    pub n_varmeters: usize,
    pub n_pmu_pairs: usize,
    pub sufficient: bool,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct LocalRedundancyPayload {
    pub bus_index: usize,
    pub n_connected_measurements: usize,
    pub min_w_ii: f64,
    pub n_critical: usize,
    pub n_simply_redundant: usize,
    pub n_multiply_redundant: usize,
}

#[cfg(feature = "pro")]
#[derive(Serialize, Clone)]
pub(crate) struct RedundancyResultPayload {
    pub global: GlobalRedundancyPayload,
    pub measurements: Vec<MeasRedundancyPayload>,
    pub local: Vec<LocalRedundancyPayload>,
    pub n_critical: usize,
    pub n_simply_redundant: usize,
    pub n_multiply_redundant: usize,
    pub analysis_time_s: f64,
}

// ── Shared estimation logic ─────────────────────────────────────────

pub(crate) struct SeExecution {
    pub payload: SeResultPayload,
    pub se_result: EstimationResult,
}

pub(crate) fn execute_estimation(
    state: &AppState,
    formulation: SolverFormulation,
    formulation_name: &str,
    tolerance: f64,
    max_iterations: usize,
) -> Result<SeExecution, String> {
    use rusty_givens_core::kernel::WlsSolver;
    use rusty_givens_core::kernel::SeSolver;

    let config = EstimationConfig {
        max_iterations,
        tolerance,
        formulation,
    };

    let solver = WlsSolver;
    let t0 = Instant::now();
    let result = solver
        .estimate(&state.system, &state.model, &state.measurements, &config)
        .map_err(|e| e.to_string())?;
    let se_time = t0.elapsed().as_secs_f64();

    #[cfg(feature = "pro")]
    let bdd = if result.converged {
        pro::run_bdd_analysis(&result)
    } else {
        None
    };
    #[cfg(not(feature = "pro"))]
    let bdd: Option<BddPayload> = None;

    let true_vm = &state.true_state.voltage_magnitude;
    let true_va = &state.true_state.voltage_angle;
    let n = state.system.n_buses();

    let mut vm_err_sum = 0.0;
    let mut va_err_sum = 0.0;
    let mut vm_max = 0.0f64;
    let mut va_max = 0.0f64;

    let buses: Vec<BusResult> = (0..n)
        .map(|i| {
            let vm_e = (result.voltage_magnitude[i] - true_vm[i]).abs();
            let va_e_deg = (result.voltage_angle[i] - true_va[i]).abs().to_degrees();
            vm_err_sum += vm_e;
            va_err_sum += va_e_deg;
            vm_max = vm_max.max(vm_e);
            va_max = va_max.max(va_e_deg);
            BusResult {
                index: i,
                label: state.bus_metadata[i].label,
                est_vm: result.voltage_magnitude[i],
                est_va_deg: result.voltage_angle[i].to_degrees(),
                true_vm: true_vm[i],
                true_va_deg: true_va[i].to_degrees(),
                vm_error: vm_e,
                va_error_deg: va_e_deg,
            }
        })
        .collect();

    // ── Dependent results (post-estimation evaluation) ──────────────
    let global_status = if result.converged {
        let post_est = evaluate_post_estimation(
            &state.system, &state.model,
            &result.voltage_magnitude, &result.voltage_angle,
        );

        let bus_vn_kv: Vec<f64> = state.bus_metadata.iter().map(|b| b.vn_kv).collect();
        let gs = build_global_status(
            &state.system, &state.measurements, &result,
            se_time, formulation_name, max_iterations, tolerance,
            &bus_vn_kv, Some(post_est),
        );
        Some(global_status_to_payload(&gs))
    } else {
        None
    };

    let payload = SeResultPayload {
        converged: result.converged,
        iterations: result.iterations,
        se_time_seconds: se_time,
        final_increment: result.final_increment,
        factorization: formulation_name.to_string(),
        tolerance,
        max_iterations,
        vm_mae: vm_err_sum / n as f64,
        vm_max_error: vm_max,
        va_mae_deg: va_err_sum / n as f64,
        va_max_error_deg: va_max,
        buses,
        bdd,
        obs_check: None,
        global_status,
    };

    Ok(SeExecution {
        payload,
        se_result: result,
    })
}

// ── Payload conversion helpers ──────────────────────────────────────

fn global_status_to_payload(gs: &SeGlobalStatus) -> GlobalStatusPayload {
    let objective = gs.objective.as_ref().map(|o| ObjectivePayload {
        objective_value: o.objective_value,
        expected_value: o.expected_value,
        degrees_of_freedom: o.degrees_of_freedom,
    });

    let measurement_counts = meas_counts_to_payload(&gs.measurement_counts);

    let per_voltage_level = gs.per_voltage_level.iter().map(|vl| VoltageLevelPayload {
        voltage_kv: vl.voltage_kv,
        n_buses: vl.n_buses,
        n_branches: vl.n_branches,
        measurement_counts: meas_counts_to_payload(&vl.measurement_counts),
    }).collect();

    let (branch_flows, bus_injections, power_balance) =
        if let Some(pe) = &gs.post_estimation {
            let bf: Vec<BranchFlowPayload> = pe.branches.iter().map(|b| BranchFlowPayload {
                branch_index: b.branch_index,
                from_bus: b.from_bus,
                to_bus: b.to_bus,
                from: TerminalFlowPayload { p: b.from.p, q: b.from.q, i_mag: b.from.i_mag },
                to: TerminalFlowPayload { p: b.to.p, q: b.to.q, i_mag: b.to.i_mag },
                p_loss: b.p_loss,
                q_loss: b.q_loss,
            }).collect();
            let bi: Vec<BusInjectionPayload> = pe.buses.iter().map(|b| BusInjectionPayload {
                bus_index: b.bus_index,
                p_inj: b.p_inj,
                q_inj: b.q_inj,
            }).collect();
            let pb = PowerBalancePayload {
                total_p_loss: pe.total_p_loss,
                total_q_loss: pe.total_q_loss,
                total_p_generation: pe.total_p_generation,
                total_q_generation: pe.total_q_generation,
                total_p_load: pe.total_p_load,
                total_q_load: pe.total_q_load,
            };
            (bf, bi, pb)
        } else {
            (vec![], vec![], PowerBalancePayload {
                total_p_loss: 0.0,
                total_q_loss: 0.0,
                total_p_generation: 0.0,
                total_q_generation: 0.0,
                total_p_load: 0.0,
                total_q_load: 0.0,
            })
        };

    GlobalStatusPayload {
        timestamp: humantime::format_rfc3339(gs.timestamp).to_string(),
        n_buses: gs.n_buses,
        n_branches: gs.n_branches,
        n_state_variables: gs.n_state_variables,
        objective,
        measurement_counts,
        per_voltage_level,
        branch_flows,
        bus_injections,
        power_balance,
    }
}

fn meas_counts_to_payload(mc: &MeasurementCounts) -> MeasCountsPayload {
    MeasCountsPayload {
        voltmeters: mc.voltmeters,
        ammeters: mc.ammeters,
        wattmeters: mc.wattmeters,
        varmeters: mc.varmeters,
        pmu_pairs: mc.pmu_pairs,
        current_angle_meters: mc.current_angle_meters,
        total: mc.total,
    }
}

fn build_measurement_export(
    _system: &PowerSystem,
    measurements: &MeasurementSet,
    artifacts: Option<&rusty_givens_core::kernel::SolverArtifacts>,
) -> Vec<MeasurementExportEntry> {
    let mut entries = Vec::new();
    let mut idx = 0usize;

    for vm in &measurements.voltmeters {
        let (est, resid) = get_artifact_entry(artifacts, idx, vm.status);
        entries.push(MeasurementExportEntry {
            index: idx,
            measurement_type: "voltmeter".to_string(),
            label: format!("V_bus{}", vm.bus),
            measured_value: vm.magnitude,
            standard_deviation: vm.variance.sqrt(),
            status: vm.status,
            estimated_value: est,
            residual: resid,
        });
        if vm.status { idx += 1; }
    }

    for am in &measurements.ammeters {
        let end_label = match am.end { rusty_givens_core::model::measurement::BranchEnd::From => "from", _ => "to" };
        let (est, resid) = get_artifact_entry(artifacts, idx, am.status);
        entries.push(MeasurementExportEntry {
            index: idx,
            measurement_type: "ammeter".to_string(),
            label: format!("I_br{}_{}", am.branch, end_label),
            measured_value: am.magnitude,
            standard_deviation: am.variance.sqrt(),
            status: am.status,
            estimated_value: est,
            residual: resid,
        });
        if am.status { idx += 1; }
    }

    for wm in &measurements.wattmeters {
        let label = match &wm.location {
            rusty_givens_core::model::measurement::WattmeterLocation::Bus(bus) =>
                format!("P_bus{}", bus),
            rusty_givens_core::model::measurement::WattmeterLocation::Branch { branch, end } => {
                let e = match end { rusty_givens_core::model::measurement::BranchEnd::From => "from", _ => "to" };
                format!("P_br{}_{}", branch, e)
            }
        };
        let (est, resid) = get_artifact_entry(artifacts, idx, wm.status);
        entries.push(MeasurementExportEntry {
            index: idx,
            measurement_type: "wattmeter".to_string(),
            label,
            measured_value: wm.active,
            standard_deviation: wm.variance.sqrt(),
            status: wm.status,
            estimated_value: est,
            residual: resid,
        });
        if wm.status { idx += 1; }
    }

    for vm in &measurements.varmeters {
        let label = match &vm.location {
            rusty_givens_core::model::measurement::VarmeterLocation::Bus(bus) =>
                format!("Q_bus{}", bus),
            rusty_givens_core::model::measurement::VarmeterLocation::Branch { branch, end } => {
                let e = match end { rusty_givens_core::model::measurement::BranchEnd::From => "from", _ => "to" };
                format!("Q_br{}_{}", branch, e)
            }
        };
        let (est, resid) = get_artifact_entry(artifacts, idx, vm.status);
        entries.push(MeasurementExportEntry {
            index: idx,
            measurement_type: "varmeter".to_string(),
            label,
            measured_value: vm.reactive,
            standard_deviation: vm.variance.sqrt(),
            status: vm.status,
            estimated_value: est,
            residual: resid,
        });
        if vm.status { idx += 1; }
    }

    for pmu in &measurements.pmus {
        let loc_label = match &pmu.location {
            rusty_givens_core::model::measurement::PmuLocation::Bus(bus) =>
                format!("bus{}", bus),
            rusty_givens_core::model::measurement::PmuLocation::Branch { branch, end } => {
                let e = match end { rusty_givens_core::model::measurement::BranchEnd::From => "from", _ => "to" };
                format!("br{}_{}", branch, e)
            }
        };
        let (est_mag, resid_mag) = get_artifact_entry(artifacts, idx, pmu.status);
        entries.push(MeasurementExportEntry {
            index: idx,
            measurement_type: "pmu_magnitude".to_string(),
            label: format!("PMU_mag_{}", loc_label),
            measured_value: pmu.magnitude,
            standard_deviation: pmu.variance_magnitude.sqrt(),
            status: pmu.status,
            estimated_value: est_mag,
            residual: resid_mag,
        });
        if pmu.status { idx += 1; }

        let (est_ang, resid_ang) = get_artifact_entry(artifacts, idx, pmu.status);
        entries.push(MeasurementExportEntry {
            index: idx,
            measurement_type: "pmu_angle".to_string(),
            label: format!("PMU_ang_{}", loc_label),
            measured_value: pmu.angle,
            standard_deviation: pmu.variance_angle.sqrt(),
            status: pmu.status,
            estimated_value: est_ang,
            residual: resid_ang,
        });
        if pmu.status { idx += 1; }
    }

    entries
}

fn get_artifact_entry(
    artifacts: Option<&rusty_givens_core::kernel::SolverArtifacts>,
    row: usize,
    status: bool,
) -> (Option<f64>, Option<f64>) {
    if !status { return (None, None); }
    match artifacts {
        Some(a) if row < a.measurement_h.len() => {
            let est = a.measurement_h[row];
            let resid = if row < a.residuals.len() { a.residuals[row] } else { 0.0 };
            (Some(est), Some(resid))
        }
        _ => (None, None),
    }
}

// ── REST handlers ───────────────────────────────────────────────────

async fn get_network(State(state): State<Arc<AppState>>) -> Json<NetworkPayload> {
    let buses: Vec<BusNode> = state
        .bus_metadata
        .iter()
        .enumerate()
        .map(|(i, b)| BusNode {
            index: i,
            label: b.label,
            vn_kv: b.vn_kv,
            bus_type: b.bus_type,
            geo_x: b.geo_x,
            geo_y: b.geo_y,
        })
        .collect();

    let branches: Vec<BranchEdge> = state
        .system
        .branches
        .iter()
        .enumerate()
        .filter(|(_, b)| b.status)
        .map(|(i, b)| BranchEdge {
            index: i,
            from_bus: b.from,
            to_bus: b.to,
        })
        .collect();

    Json(NetworkPayload {
        n_buses: state.info.n_buses,
        n_branches: state.info.n_branches,
        slack_bus_index: state.info.slack_bus_index,
        base_mva: state.info.base_mva,
        buses,
        branches,
    })
}

async fn get_true_state(State(state): State<Arc<AppState>>) -> Json<TrueStatePayload> {
    let ts = &state.true_state;
    Json(TrueStatePayload {
        voltage_magnitude: ts.voltage_magnitude.clone(),
        voltage_angle_deg: ts.voltage_angle.iter().map(|a| a.to_degrees()).collect(),
    })
}

async fn run_estimate(
    State(state): State<Arc<AppState>>,
    Json(req): Json<EstimateRequest>,
) -> Result<Json<SeResultPayload>, (StatusCode, String)> {
    let (formulation, formulation_name) = parse_formulation_str(
        req.formulation.as_deref(),
        &req.factorization,
    )?;
    let tolerance = req.tolerance.unwrap_or(1e-4);
    let max_iterations = req.max_iterations.unwrap_or(50);

    // ── OA gate (Pro only): run observability check before SE ───────
    #[cfg(feature = "pro")]
    let obs_payload: Option<ObsResultPayload> = {
        let skip_obs = req.skip_obs_check.unwrap_or(false);
        if !skip_obs {
            pro::obs_gate(&state, req.obs_method.as_deref())?
        } else {
            None
        }
    };
    #[cfg(not(feature = "pro"))]
    let obs_payload: Option<ObsResultPayload> = None;

    // ── Run SE ──────────────────────────────────────────────────────
    let mut exec = execute_estimation(&state, formulation, &formulation_name, tolerance, max_iterations)
        .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e))?;
    exec.payload.obs_check = obs_payload;

    *state.last_result.lock().await = Some(exec.payload.clone());
    *state.last_se_result.lock().await = Some(exec.se_result);
    Ok(Json(exec.payload))
}

async fn get_last_result(
    State(state): State<Arc<AppState>>,
) -> Result<Json<SeResultPayload>, StatusCode> {
    let guard = state.last_result.lock().await;
    match guard.as_ref() {
        Some(r) => Ok(Json(r.clone())),
        None => Err(StatusCode::NOT_FOUND),
    }
}

async fn get_measurement_export(
    State(state): State<Arc<AppState>>,
) -> Result<Json<Vec<MeasurementExportEntry>>, StatusCode> {
    let guard = state.last_se_result.lock().await;
    let artifacts = guard.as_ref().and_then(|r| r.artifacts.as_ref());
    let entries = build_measurement_export(&state.system, &state.measurements, artifacts);
    Ok(Json(entries))
}

// ── Helper functions ────────────────────────────────────────────────

/// Parse the solver formulation from the request.  Accepts either the new
/// `formulation` field or falls back to the legacy `factorization` field.
fn parse_formulation_str(
    formulation: Option<&str>,
    factorization_legacy: &str,
) -> Result<(SolverFormulation, String), (StatusCode, String)> {
    if let Some(f) = formulation {
        match f {
            "NormalEquations" | "NE" => {
                let fac = parse_factorization_inner(factorization_legacy)?;
                Ok((
                    SolverFormulation::NormalEquations { factorization: fac },
                    format!("NormalEquations/{factorization_legacy}"),
                ))
            }
            "OrthogonalQR" | "QR" | "Givens" => {
                Ok((SolverFormulation::OrthogonalQR, "OrthogonalQR".into()))
            }
            "PetersWilkinson" | "PW" => {
                Ok((SolverFormulation::PetersWilkinson, "PetersWilkinson".into()))
            }
            "EqualityConstrained" | "EC" => {
                let fac = parse_factorization_inner(factorization_legacy)?;
                Ok((
                    SolverFormulation::EqualityConstrained { factorization: fac, alpha: None },
                    format!("EqualityConstrained/{factorization_legacy}"),
                ))
            }
            "FastDecoupled" | "FD" => {
                Ok((SolverFormulation::FastDecoupled, "FastDecoupled".into()))
            }
            "DcEstimation" | "DC" => {
                Ok((SolverFormulation::DcEstimation, "DcEstimation".into()))
            }
            other => Err((
                StatusCode::BAD_REQUEST,
                format!(
                    "Unknown formulation: {other}. Use NormalEquations, OrthogonalQR, \
                     PetersWilkinson, EqualityConstrained, FastDecoupled, or DcEstimation"
                ),
            )),
        }
    } else {
        let fac = parse_factorization_inner(factorization_legacy)?;
        Ok((
            SolverFormulation::NormalEquations { factorization: fac },
            format!("NormalEquations/{factorization_legacy}"),
        ))
    }
}

fn parse_factorization_inner(s: &str) -> Result<Factorization, (StatusCode, String)> {
    match s {
        "DenseCholesky" => Ok(Factorization::DenseCholesky),
        "SparseCholesky" => Ok(Factorization::SparseCholesky),
        "SparseLU" => Ok(Factorization::SparseLU),
        other => Err((
            StatusCode::BAD_REQUEST,
            format!("Unknown factorization: {other}. Use DenseCholesky, SparseCholesky, or SparseLU"),
        )),
    }
}

#[allow(dead_code)]
pub(crate) fn meas_ref_type(m: &MeasurementRef) -> &str {
    match m {
        MeasurementRef::Voltmeter { .. } => "Voltmeter",
        MeasurementRef::Ammeter { .. } => "Ammeter",
        MeasurementRef::Wattmeter { .. } => "Wattmeter",
        MeasurementRef::Varmeter { .. } => "Varmeter",
        MeasurementRef::PmuMagnitude { .. } => "PMU_Magnitude",
        MeasurementRef::PmuAngle { .. } => "PMU_Angle",
        MeasurementRef::CurrentAngle { .. } => "CurrentAngle",
    }
}

#[allow(dead_code)]
pub(crate) fn meas_ref_label(m: &MeasurementRef) -> &str {
    match m {
        MeasurementRef::Voltmeter { label } => label,
        MeasurementRef::Ammeter { label } => label,
        MeasurementRef::Wattmeter { label } => label,
        MeasurementRef::Varmeter { label } => label,
        MeasurementRef::PmuMagnitude { label } => label,
        MeasurementRef::PmuAngle { label } => label,
        MeasurementRef::CurrentAngle { label } => label,
    }
}


// ── Main ────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() {
    env_logger::init();

    let case_path = Path::new("case_study/gb_network.json");
    println!("Loading GB network from {}...", case_path.display());
    let loaded = load_case(case_path).expect("Failed to load case data");
    println!(
        "Loaded: {} buses, {} branches, {} measurements",
        loaded.system.n_buses(),
        loaded.system.n_branches(),
        loaded.measurements.n_equations()
    );

    println!("Building AC model...");
    let model = build_ac_model(&loaded.system);

    let state = Arc::new(AppState {
        info: loaded.info,
        system: loaded.system,
        model,
        measurements: loaded.measurements,
        true_state: loaded.true_state,
        bus_metadata: loaded.bus_metadata,
        last_result: Mutex::new(None),
        last_se_result: Mutex::new(None),
        #[cfg(feature = "pro")]
        bdd_state: Mutex::new(BddState::new()),
    });

    // REST server (JSON over HTTP/1.1)
    #[allow(unused_mut)]
    let mut rest_app = Router::new()
        .route("/api/network", get(get_network))
        .route("/api/true-state", get(get_true_state))
        .route("/api/estimate", post(run_estimate))
        .route("/api/last-result", get(get_last_result))
        .route("/api/measurements", get(get_measurement_export));

    #[cfg(feature = "pro")]
    {
        rest_app = rest_app
            .route("/api/observability", post(pro::run_observability))
            .route("/api/redundancy", post(pro::run_redundancy))
            .route("/api/bdd/run", post(pro::run_iterative_bdd_handler))
            .route("/api/bdd/bad-data", get(pro::get_bad_data_set))
            .route("/api/bdd/historical", get(pro::get_historical_bad_data))
            .route("/api/bdd/alerts", get(pro::get_bdd_alerts))
            .route("/api/bdd/reset", post(pro::reset_bdd_state));
    }

    let rest_app = rest_app
        .layer(CorsLayer::permissive())
        .with_state(state.clone());

    let rest_addr = "0.0.0.0:3001";
    #[cfg(feature = "pro")]
    println!("REST  API listening on http://{rest_addr} [Pro edition]");
    #[cfg(not(feature = "pro"))]
    println!("REST  API listening on http://{rest_addr} [Free edition]");
    let rest_listener = tokio::net::TcpListener::bind(rest_addr)
        .await
        .expect("Failed to bind REST port 3001");

    // gRPC server (protobuf over HTTP/2)
    let grpc_addr: std::net::SocketAddr = "0.0.0.0:50051"
        .parse()
        .expect("invalid gRPC address");
    println!("gRPC  API listening on http://{grpc_addr}");

    let estimate_svc =
        pb::rusty_givens::v1::estimate_service_server::EstimateServiceServer::new(
            grpc::EstimateServiceImpl {
                state: state.clone(),
            },
        );

    #[allow(unused_mut)]
    let mut grpc_builder = tonic::transport::Server::builder()
        .add_service(estimate_svc);

    #[cfg(feature = "pro")]
    {
        let bdd_svc = pb::rusty_givens::v1::bdd_service_server::BddServiceServer::new(
            grpc::BddServiceImpl {
                state: state.clone(),
            },
        );
        let obs_svc = pb::rusty_givens::v1::obs_service_server::ObsServiceServer::new(
            grpc::ObsServiceImpl {
                state: state.clone(),
            },
        );
        let red_svc = pb::rusty_givens::v1::red_service_server::RedServiceServer::new(
            grpc::RedServiceImpl {
                state: state.clone(),
            },
        );
        grpc_builder = grpc_builder
            .add_service(bdd_svc)
            .add_service(obs_svc)
            .add_service(red_svc);
    }

    let grpc_server = grpc_builder.serve(grpc_addr);

    // Run both servers concurrently; shut down if either exits.
    tokio::select! {
        r = axum::serve(rest_listener, rest_app) => {
            eprintln!("REST server exited: {r:?}");
        }
        r = grpc_server => {
            eprintln!("gRPC server exited: {r:?}");
        }
    }
}
