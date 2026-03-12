//! HTTP API server for the State Estimation frontend.
//!
//! Endpoints:
//!   GET  /api/network    — topology (buses + branches) for vis-network
//!   GET  /api/true-state — true power-flow state (for error comparison)
//!   POST /api/estimate   — run WLS SE with a given configuration

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

use state_rustimation::ac_model::{build_ac_model, AcModel};
use state_rustimation::io::{load_case, CaseData};
use state_rustimation::measurement::MeasurementSet;
use state_rustimation::power_system::PowerSystem;
use state_rustimation::solver::{
    gauss_newton_sparse, AcSeResult, GainFactorization, SolverConfig, WlsMethod,
};

// ── Shared application state ────────────────────────────────────────────

struct AppState {
    case_data: CaseData,
    system: PowerSystem,
    model: AcModel,
    measurements: MeasurementSet,
    last_result: Mutex<Option<SeResultPayload>>,
}

// ── JSON payloads ───────────────────────────────────────────────────────

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
struct EstimateRequest {
    factorization: String,
    max_iterations: Option<usize>,
    tolerance: Option<f64>,
}

#[derive(Serialize, Clone)]
struct BusResult {
    index: usize,
    label: usize,
    est_vm: f64,
    est_va_deg: f64,
    true_vm: f64,
    true_va_deg: f64,
    vm_error: f64,
    va_error_deg: f64,
}

#[derive(Serialize, Clone)]
struct SeResultPayload {
    converged: bool,
    iterations: usize,
    se_time_seconds: f64,
    final_increment: f64,
    factorization: String,
    tolerance: f64,
    max_iterations: usize,
    vm_mae: f64,
    vm_max_error: f64,
    va_mae_deg: f64,
    va_max_error_deg: f64,
    buses: Vec<BusResult>,
}

// ── Handlers ────────────────────────────────────────────────────────────

async fn get_network(State(state): State<Arc<AppState>>) -> Json<NetworkPayload> {
    let buses: Vec<BusNode> = state
        .case_data
        .buses
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
        .case_data
        .branches
        .iter()
        .enumerate()
        .filter(|(_, b)| b.status)
        .map(|(i, b)| BranchEdge {
            index: i,
            from_bus: b.from_bus,
            to_bus: b.to_bus,
        })
        .collect();

    Json(NetworkPayload {
        n_buses: state.case_data.n_buses,
        n_branches: state.case_data.n_branches,
        slack_bus_index: state.case_data.slack_bus_index,
        base_mva: state.case_data.base_mva,
        buses,
        branches,
    })
}

async fn get_true_state(State(state): State<Arc<AppState>>) -> Json<TrueStatePayload> {
    let ts = &state.case_data.true_state;
    Json(TrueStatePayload {
        voltage_magnitude: ts.voltage_magnitude.clone(),
        voltage_angle_deg: ts.voltage_angle.iter().map(|a| a.to_degrees()).collect(),
    })
}

async fn run_estimate(
    State(state): State<Arc<AppState>>,
    Json(req): Json<EstimateRequest>,
) -> Result<Json<SeResultPayload>, (StatusCode, String)> {
    let factorization = match req.factorization.as_str() {
        "DenseCholesky" => GainFactorization::DenseCholesky,
        "SparseCholesky" => GainFactorization::SparseCholesky,
        "SparseLU" => GainFactorization::SparseLU,
        other => {
            return Err((
                StatusCode::BAD_REQUEST,
                format!("Unknown factorization: {other}. Use DenseCholesky, SparseCholesky, or SparseLU"),
            ))
        }
    };

    let tolerance = req.tolerance.unwrap_or(1e-4);
    let max_iterations = req.max_iterations.unwrap_or(50);

    let config = SolverConfig {
        max_iterations,
        tolerance,
        method: WlsMethod::Normal,
        gain_factorization: factorization,
    };

    let t0 = Instant::now();
    let result: AcSeResult =
        gauss_newton_sparse(&state.system, &state.model, &state.measurements, &config);
    let se_time = t0.elapsed().as_secs_f64();

    let true_vm = &state.case_data.true_state.voltage_magnitude;
    let true_va = &state.case_data.true_state.voltage_angle;
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
                label: state.case_data.buses[i].label,
                est_vm: result.voltage_magnitude[i],
                est_va_deg: result.voltage_angle[i].to_degrees(),
                true_vm: true_vm[i],
                true_va_deg: true_va[i].to_degrees(),
                vm_error: vm_e,
                va_error_deg: va_e_deg,
            }
        })
        .collect();

    let payload = SeResultPayload {
        converged: result.converged,
        iterations: result.iterations,
        se_time_seconds: se_time,
        final_increment: result.final_increment,
        factorization: req.factorization,
        tolerance,
        max_iterations,
        vm_mae: vm_err_sum / n as f64,
        vm_max_error: vm_max,
        va_mae_deg: va_err_sum / n as f64,
        va_max_error_deg: va_max,
        buses,
    };

    *state.last_result.lock().await = Some(payload.clone());
    Ok(Json(payload))
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

// ── Main ────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() {
    env_logger::init();

    let case_path = Path::new("case_study/gb_network.json");
    println!("Loading GB network from {}...", case_path.display());
    let (case_data, system, measurements) =
        load_case(case_path).expect("Failed to load case data");
    println!(
        "Loaded: {} buses, {} branches, {} measurements",
        system.n_buses(),
        system.n_branches(),
        measurements.n_equations()
    );

    println!("Building AC model...");
    let model = build_ac_model(&system);

    let state = Arc::new(AppState {
        case_data,
        system,
        model,
        measurements,
        last_result: Mutex::new(None),
    });

    let app = Router::new()
        .route("/api/network", get(get_network))
        .route("/api/true-state", get(get_true_state))
        .route("/api/estimate", post(run_estimate))
        .route("/api/last-result", get(get_last_result))
        .layer(CorsLayer::permissive())
        .with_state(state);

    let addr = "0.0.0.0:3001";
    println!("API server listening on http://{addr}");
    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}
