# Quick Start

Build the workspace, run the estimate service, and launch the Angular frontend.

---

## Prerequisites

- **Rust** 1.75+ (2024 edition) — [rustup.rs](https://rustup.rs)
- **Node.js** 18+ and npm — for the Angular frontend
- **protoc** (Protocol Buffers compiler) — for gRPC codegen

---

## Build and Run (Free Edition)

```bash
# Build the workspace (release mode)
cargo build --release

# Run the estimate service
# Loads the GB network, REST on port 3001, gRPC on port 50051
cargo run --release -p estimate-service
```

In another terminal, start the Angular frontend:

```bash
cd frontend && npm install && npx ng serve
```

The frontend proxies `/api` to `http://localhost:3001`. Open [http://localhost:4200](http://localhost:4200) in your browser.

---

## Build and Run (Pro Edition)

```bash
# Build with Pro features enabled
cargo build --release --features pro

# Run with Pro features
cargo run --release -p estimate-service --features pro
```

The Pro build adds Observability Analysis, Redundancy Analysis, and Bad Data Detection endpoints. See the [Pro Edition](pro.md) page for details.

---

## Run an Estimation via REST

### Default (Normal Equations + Sparse Cholesky)

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{"factorization":"SparseCholesky","max_iterations":50,"tolerance":1e-4}'
```

### Choose a Solver Formulation

Use the `formulation` field to select one of six methods:

=== "Normal Equations"

    ```bash
    curl -X POST http://localhost:3001/api/estimate \
      -H 'Content-Type: application/json' \
      -d '{"formulation":"NormalEquations","factorization":"SparseCholesky"}'
    ```

=== "Orthogonal QR (Givens)"

    ```bash
    curl -X POST http://localhost:3001/api/estimate \
      -H 'Content-Type: application/json' \
      -d '{"formulation":"OrthogonalQR","factorization":"SparseCholesky"}'
    ```

=== "Peters-Wilkinson"

    ```bash
    curl -X POST http://localhost:3001/api/estimate \
      -H 'Content-Type: application/json' \
      -d '{"formulation":"PetersWilkinson","factorization":"SparseCholesky"}'
    ```

=== "Equality-Constrained"

    ```bash
    curl -X POST http://localhost:3001/api/estimate \
      -H 'Content-Type: application/json' \
      -d '{"formulation":"EqualityConstrained","factorization":"SparseCholesky"}'
    ```

=== "Fast Decoupled"

    ```bash
    curl -X POST http://localhost:3001/api/estimate \
      -H 'Content-Type: application/json' \
      -d '{"formulation":"FastDecoupled","factorization":"SparseCholesky"}'
    ```

=== "DC Estimation"

    ```bash
    curl -X POST http://localhost:3001/api/estimate \
      -H 'Content-Type: application/json' \
      -d '{"formulation":"DcEstimation","factorization":"SparseCholesky"}'
    ```

### Zero-Injection Configuration

Override automatic zero-injection detection with an explicit bus list:

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{
    "factorization": "SparseCholesky",
    "zi_enabled": true,
    "zi_sigma": 1e-6,
    "zi_violation_threshold": 1e-3,
    "zi_buses": [5, 12, 37, 104]
  }'
```

Or disable zero-injection handling entirely:

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{"factorization":"SparseCholesky","zi_enabled":false}'
```

---

## Use the Kernel as a Library

Add to your `Cargo.toml`:

```toml
[dependencies]
rusty-givens-core = { path = "crates/rusty-givens-core" }
rusty-givens-io   = { path = "crates/rusty-givens-io" }
```

### Minimal Example

```rust
use rusty_givens_core::kernel::{
    WlsSolver, SeSolver, EstimationConfig,
    SolverFormulation, Factorization,
};
use rusty_givens_core::model::build_ac_model;
use rusty_givens_io::load_case;
use std::path::Path;

fn main() {
    let case  = load_case(Path::new("case_study/gb_network.json")).unwrap();
    let model = build_ac_model(&case.system);

    let config = EstimationConfig {
        formulation: SolverFormulation::NormalEquations {
            factorization: Factorization::SparseCholesky,
        },
        ..EstimationConfig::default()
    };

    let result = WlsSolver
        .estimate(&case.system, &model, &case.measurements, &config)
        .expect("estimation failed");

    println!("Converged: {} in {} iterations", result.converged, result.iterations);
    for (i, (vm, va)) in result.voltage_magnitude.iter()
        .zip(result.voltage_angle.iter()).enumerate()
    {
        println!("  Bus {}: |V| = {:.5} p.u., θ = {:.3}°", i, vm, va.to_degrees());
    }
}
```

### Comparing Formulations

```rust
let formulations = vec![
    ("NE/Cholesky", SolverFormulation::NormalEquations {
        factorization: Factorization::SparseCholesky,
    }),
    ("QR/Givens", SolverFormulation::OrthogonalQR),
    ("Peters-Wilkinson", SolverFormulation::PetersWilkinson),
    ("Fast Decoupled", SolverFormulation::FastDecoupled),
    ("DC", SolverFormulation::DcEstimation),
];

for (name, formulation) in formulations {
    let config = EstimationConfig {
        formulation,
        ..EstimationConfig::default()
    };
    match WlsSolver.estimate(&case.system, &model, &case.measurements, &config) {
        Ok(r) => println!("{}: converged={}, iters={}, time={:.3}s",
            name, r.converged, r.iterations, r.diagnostics.last()
                .map(|d| d.total_time_s).unwrap_or(0.0)),
        Err(e) => println!("{}: failed — {}", name, e),
    }
}
```

---

## Run the Case Study Binary

The `gb-case-study` crate provides a standalone binary that runs SE without the HTTP server:

```bash
cargo run --release -p gb-case-study
```

This loads the GB network, runs Normal Equations with Sparse Cholesky, and prints the per-bus results to stdout.

---

## Export Measurements

After running an estimation, retrieve the full measurement table:

```bash
curl http://localhost:3001/api/measurements | python -m json.tool
```

This returns every measurement with its type, label, measured value, standard deviation, estimated value, and residual.
