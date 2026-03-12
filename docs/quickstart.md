# Quick Start

Build the workspace, run the estimate service, and launch the Angular frontend.

## Build and run

```bash
# Build the workspace (release mode)
cargo build --release

# Run the estimate service (loads the GB network, REST on 3001, gRPC on 50051)
cargo run --release -p estimate-service

# In another terminal — run the Angular frontend
cd frontend && npm install && npx ng serve
```

The frontend proxies `/api` to `http://localhost:3001`. Open [http://localhost:4200](http://localhost:4200) in your browser.

## Run an estimation via REST

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{"factorization":"SparseCholesky","max_iterations":50,"tolerance":1e-4}'
```

## Use the kernel as a library

Add to your `Cargo.toml`:

```toml
[dependencies]
rusty-givens-core = { path = "crates/rusty-givens-core" }
rusty-givens-io   = { path = "crates/rusty-givens-io" }
```

Example usage:

```rust
use rusty_givens_core::kernel::{WlsSolver, SeSolver, EstimationConfig, Factorization};
use rusty_givens_core::model::build_ac_model;
use rusty_givens_io::load_case;
use std::path::Path;

fn main() {
    let case  = load_case(Path::new("case_study/gb_network.json")).unwrap();
    let model = build_ac_model(&case.system);
    let cfg   = EstimationConfig {
        factorization: Factorization::SparseCholesky,
        ..EstimationConfig::default()
    };
    let result = WlsSolver.estimate(&case.system, &model, &case.measurements, &cfg)
        .expect("estimation failed");

    println!("Converged in {} iterations", result.iterations);
    println!("Voltage magnitudes: {:?}", result.voltage_magnitude);
}
```
