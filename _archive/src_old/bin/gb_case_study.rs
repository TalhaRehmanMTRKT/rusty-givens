//! GB Network Case Study — Rust WLS State Estimation
//!
//! Loads the GB network JSON (exported by extract_gb_network.py),
//! runs the Gauss-Newton WLS state estimation, and outputs timing
//! and accuracy metrics for comparison with JuliaGrid.

use std::path::Path;
use std::time::Instant;

use state_rustimation::ac_model::build_ac_model;
use state_rustimation::io::load_case;
use state_rustimation::solver::{gauss_newton_sparse, GainFactorization, SolverConfig, WlsMethod};

fn main() {
    env_logger::init();

    println!("{}", "=".repeat(60));
    println!("Rust WLS State Estimation — GB Network Case Study");
    println!("{}", "=".repeat(60));

    // ── 1. Load the case data ──
    let case_path = Path::new("case_study/gb_network.json");
    println!("\n[1] Loading GB network from JSON...");
    let t0 = Instant::now();
    let (case_data, system, measurements) = load_case(case_path).expect("Failed to load case data");
    let load_time = t0.elapsed().as_secs_f64();
    println!("    Loaded in {:.3}s", load_time);
    println!("    Buses: {}, Branches: {}", system.n_buses(), system.n_branches());
    println!("    Measurement equations: {}", measurements.n_equations());

    // ── 2. Build AC model ──
    println!("\n[2] Building AC model (Y-bus)...");
    let t0 = Instant::now();
    let model = build_ac_model(&system);
    let model_time = t0.elapsed().as_secs_f64();
    println!("    AC model built in {:.3}s", model_time);

    // ── 3. Run state estimation ──
    let factorization = match std::env::var("GAIN_FACTORIZATION").as_deref() {
        Ok("dense") => GainFactorization::DenseCholesky,
        Ok("sparse_lu") => GainFactorization::SparseLU,
        _ => GainFactorization::SparseCholesky,
    };
    println!("\n[3] Running Gauss-Newton WLS ({:?})...", factorization);
    let config = SolverConfig {
        max_iterations: 50,
        tolerance: 1e-4,
        method: WlsMethod::Normal,
        gain_factorization: factorization,
    };

    let t0 = Instant::now();
    let result = gauss_newton_sparse(&system, &model, &measurements, &config);
    let se_time = t0.elapsed().as_secs_f64();

    println!("    Converged: {} in {} iterations", result.converged, result.iterations);
    println!("    Final max|Δx|: {:.2e}", result.final_increment);
    println!("    SE solve time: {:.3}s", se_time);

    // ── 4. Compare to true state ──
    println!("\n[4] Comparing to true power flow state...");
    let true_vm = &case_data.true_state.voltage_magnitude;
    let true_va = &case_data.true_state.voltage_angle;

    let n = system.n_buses();
    let mut vm_errors = Vec::with_capacity(n);
    let mut va_errors = Vec::with_capacity(n);
    for i in 0..n {
        vm_errors.push((result.voltage_magnitude[i] - true_vm[i]).abs());
        va_errors.push((result.voltage_angle[i] - true_va[i]).abs());
    }

    let vm_mae: f64 = vm_errors.iter().sum::<f64>() / n as f64;
    let va_mae: f64 = va_errors.iter().sum::<f64>() / n as f64;
    let vm_max: f64 = vm_errors.iter().cloned().fold(0.0f64, f64::max);
    let va_max: f64 = va_errors.iter().cloned().fold(0.0f64, f64::max);

    println!("    Voltage magnitude MAE: {:.6} p.u.", vm_mae);
    println!("    Voltage magnitude Max: {:.6} p.u.", vm_max);
    println!("    Voltage angle MAE:     {:.6} rad ({:.4} deg)", va_mae, va_mae.to_degrees());
    println!("    Voltage angle Max:     {:.6} rad ({:.4} deg)", va_max, va_max.to_degrees());

    // ── 5. Summary ──
    println!("\n{}", "=".repeat(60));
    println!("TIMING SUMMARY");
    println!("{}", "-".repeat(60));
    println!("  JSON load time:    {:.3}s", load_time);
    println!("  AC model build:    {:.3}s", model_time);
    println!("  SE solve time:     {:.3}s", se_time);
    println!("  Total:             {:.3}s", load_time + model_time + se_time);
    println!("{}", "=".repeat(60));

    // ── 6. Save results as JSON (including per-bus estimates) ──
    let results = serde_json::json!({
        "solver": "StateRustimation",
        "method": format!("Gauss-Newton ({:?})", factorization),
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
    println!("\nResults saved to {}", results_path);
}
