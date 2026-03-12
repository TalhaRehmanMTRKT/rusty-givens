//! StateRustimation — AC Weighted Least-Squares State Estimation in Rust.
//!
//! A direct translation of JuliaGrid's AC State Estimation solver.
//! This example replicates the 3-bus tutorial system from the JuliaGrid documentation.

use state_rustimation::ac_model::build_ac_model;
use state_rustimation::measurement::*;
use state_rustimation::power_system::*;
use state_rustimation::solver::{gauss_newton, SolverConfig, WlsMethod};

fn main() {
    env_logger::init();

    // ──────────────────────────────────────────────────────────────
    //  1. Build the 3-bus power system (matches JuliaGrid tutorial)
    // ──────────────────────────────────────────────────────────────
    let mut system = PowerSystem::new();

    let mut bus1 = Bus::new(1);
    bus1.bus_type = BusType::Slack;
    bus1.active_demand = 0.5;
    system.add_bus(bus1);

    let mut bus2 = Bus::new(2);
    bus2.reactive_demand = 0.3;
    system.add_bus(bus2);

    let mut bus3 = Bus::new(3);
    bus3.active_demand = 0.5;
    system.add_bus(bus3);

    // Branch defaults: resistance = 0.02, susceptance = 0.04
    let r = 0.02;
    let susc = 0.04;

    let mut br1 = Branch::new(1, 1, 2);
    br1.resistance = r;
    br1.reactance = 0.6;
    br1.susceptance = susc;
    system.add_branch(br1);

    let mut br2 = Branch::new(2, 1, 3);
    br2.resistance = r;
    br2.reactance = 0.7;
    br2.susceptance = susc;
    system.add_branch(br2);

    let mut br3 = Branch::new(3, 2, 3);
    br3.resistance = r;
    br3.reactance = 0.2;
    br3.susceptance = susc;
    system.add_branch(br3);

    // ──────────────────────────────────────────────────────────────
    //  2. Build the AC model (Y-bus)
    // ──────────────────────────────────────────────────────────────
    let model = build_ac_model(&system);

    println!("Power system: {} buses, {} branches", system.n_buses(), system.n_branches());

    // ──────────────────────────────────────────────────────────────
    //  3. Define measurements (matches the WLS tutorial example)
    // ──────────────────────────────────────────────────────────────
    let mut meas = MeasurementSet::new();

    // Wattmeters
    meas.add_bus_wattmeter("Wattmeter 1", 3, -0.5, 1e-3);
    meas.add_branch_wattmeter("Wattmeter 2", 1, BranchEnd::From, 0.2, 1e-4);

    // Varmeters
    meas.add_bus_varmeter("Varmeter 1", 2, -0.3, 1e-3);
    meas.add_branch_varmeter("Varmeter 2", 1, BranchEnd::From, 0.2, 1e-4);

    // PMUs
    meas.add_bus_pmu("PMU 1", 1, 1.0, 0.0, 1e-5, 1e-6, true, false);
    meas.add_bus_pmu("PMU 2", 3, 0.9, -0.2, 1e-5, 1e-5, false, false);

    println!("Measurements: {} equations", meas.n_equations());

    // ──────────────────────────────────────────────────────────────
    //  4. Solve with the conventional (LU) Gauss-Newton method
    // ──────────────────────────────────────────────────────────────
    println!("\n=== Normal (LU) Method ===");
    let config = SolverConfig {
        max_iterations: 20,
        tolerance: 1e-8,
        method: WlsMethod::Normal,
    };

    let result = gauss_newton(&system, &model, &meas, &config);
    print_result(&result, &system);

    // ──────────────────────────────────────────────────────────────
    //  5. Solve with the Orthogonal (QR) method
    // ──────────────────────────────────────────────────────────────
    println!("\n=== Orthogonal (QR) Method ===");
    let config_qr = SolverConfig {
        method: WlsMethod::Orthogonal,
        ..config.clone()
    };

    let result_qr = gauss_newton(&system, &model, &meas, &config_qr);
    print_result(&result_qr, &system);

    // ──────────────────────────────────────────────────────────────
    //  6. Solve with the Peters-Wilkinson method
    // ──────────────────────────────────────────────────────────────
    println!("\n=== Peters-Wilkinson Method ===");
    let config_pw = SolverConfig {
        method: WlsMethod::PetersWilkinson,
        ..config.clone()
    };

    let result_pw = gauss_newton(&system, &model, &meas, &config_pw);
    print_result(&result_pw, &system);
}

fn print_result(
    result: &state_rustimation::solver::AcSeResult,
    system: &PowerSystem,
) {
    println!(
        "Converged: {} in {} iterations (max|Δx| = {:.2e})",
        result.converged, result.iterations, result.final_increment
    );
    println!("Bus  |  V (p.u.)  |  θ (rad)");
    println!("-----|------------|----------");
    for (idx, bus) in system.buses.iter().enumerate() {
        println!(
            "  {}  | {:.10} | {:.10}",
            bus.label, result.voltage_magnitude[idx], result.voltage_angle[idx]
        );
    }
}
