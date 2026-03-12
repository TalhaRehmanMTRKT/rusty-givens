//! GB Network Case Study — Rust WLS State Estimation
//!
//! Loads the GB network JSON (exported by extract_gb_network.py),
//! runs the selected solver formulation, and outputs JSON for comparison.
//!
//! Usage:
//!   SOLVER_FORMULATION=<name> cargo run --release --bin gb_case_study
//!
//! Formulations: NormalEquations, OrthogonalQR, PetersWilkinson,
//!               EqualityConstrained, FastDecoupled, DcEstimation

use std::path::Path;
use std::time::Instant;

use rusty_givens_core::kernel::{
    EstimationConfig, Factorization, SeSolver, SolverFormulation, WlsSolver,
};
use rusty_givens_core::model::build_ac_model;
use rusty_givens_io::load_case;

fn main() {
    let formulation_str = std::env::var("SOLVER_FORMULATION")
        .unwrap_or_else(|_| "NormalEquations".to_string());

    let formulation = parse_formulation(&formulation_str);
    let case_file = std::env::var("CASE_FILE")
        .unwrap_or_else(|_| "case_study/gb_network.json".to_string());
    let case_path = Path::new(&case_file);

    // ── 1. Load the case data ──
    let t0 = Instant::now();
    let loaded = load_case(case_path).expect("Failed to load case data");
    let load_time = t0.elapsed().as_secs_f64();

    // ── 2. Build AC model ──
    let t0 = Instant::now();
    let model = build_ac_model(&loaded.system);
    let model_time = t0.elapsed().as_secs_f64();

    // ── 3. Run state estimation ──
    let config = EstimationConfig {
        max_iterations: 50,
        tolerance: 1e-4,
        formulation: formulation.clone(),
        ..Default::default()
    };

    let solver = WlsSolver;
    let t0 = Instant::now();
    let result = solver
        .estimate(&loaded.system, &model, &loaded.measurements, &config)
        .expect("State estimation failed");
    let se_time = t0.elapsed().as_secs_f64();

    // ── 4. Compare to true state ──
    let true_vm = &loaded.true_state.voltage_magnitude;
    let true_va = &loaded.true_state.voltage_angle;
    let n = loaded.system.n_buses();

    let vm_mae: f64 = (0..n)
        .map(|i| (result.voltage_magnitude[i] - true_vm[i]).abs())
        .sum::<f64>()
        / n as f64;
    let va_mae: f64 = (0..n)
        .map(|i| (result.voltage_angle[i] - true_va[i]).abs())
        .sum::<f64>()
        / n as f64;
    let vm_max: f64 = (0..n)
        .map(|i| (result.voltage_magnitude[i] - true_vm[i]).abs())
        .fold(0.0f64, f64::max);
    let va_max: f64 = (0..n)
        .map(|i| (result.voltage_angle[i] - true_va[i]).abs())
        .fold(0.0f64, f64::max);

    // ── 5. Save results as JSON ──
    let results = serde_json::json!({
        "formulation": formulation_str,
        "load_time_seconds": load_time,
        "model_build_time_seconds": model_time,
        "se_time_seconds": se_time,
        "iterations": result.iterations,
        "converged": result.converged,
        "vm_mae": vm_mae,
        "va_mae": va_mae,
        "vm_max_error": vm_max,
        "va_max_error": va_max,
        "voltage_magnitude": result.voltage_magnitude,
        "voltage_angle": result.voltage_angle,
    });

    let results_path = "case_study/results_rust.json";
    std::fs::write(results_path, serde_json::to_string_pretty(&results).unwrap())
        .expect("Failed to write results");

    // Print minimal output for subprocess (JSON path)
    eprintln!(
        "[gb_case_study] {} | converged={} iter={} se_time={:.4}s",
        formulation_str, result.converged, result.iterations, se_time
    );
    println!("{}", results_path);
}

fn parse_formulation(s: &str) -> SolverFormulation {
    let fac = match std::env::var("FACTORIZATION").as_deref() {
        Ok("SparseLU") => Factorization::SparseLU,
        Ok("DenseCholesky") => Factorization::DenseCholesky,
        _ => Factorization::SparseCholesky,
    };
    match s {
        "NormalEquations" | "NE" => SolverFormulation::NormalEquations {
            factorization: fac,
        },
        "OrthogonalQR" | "QR" | "Givens" => SolverFormulation::OrthogonalQR,
        "PetersWilkinson" | "PW" => SolverFormulation::PetersWilkinson,
        "EqualityConstrained" | "EC" => SolverFormulation::EqualityConstrained {
            factorization: fac,
            alpha: None,
        },
        "FastDecoupled" | "FD" => SolverFormulation::FastDecoupled,
        "DcEstimation" | "DC" => SolverFormulation::DcEstimation,
        _ => {
            eprintln!("Unknown formulation '{}', using NormalEquations", s);
            SolverFormulation::NormalEquations {
                factorization: fac,
            }
        }
    }
}
