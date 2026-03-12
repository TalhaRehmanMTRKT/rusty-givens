# StateRustimation

A Rust translation of JuliaGrid's Weighted Least-Squares (WLS) AC State Estimation solver for power systems.

Based on the paper: *"JuliaGrid: An Open-Source Julia-Based Framework for Power System State Estimation"* (Cosovic et al., 2025) and the [JuliaGrid AC SE tutorial](https://mcosovic.github.io/JuliaGrid.jl/dev/tutorials/acStateEstimation/).

## Features

- **AC Power System Model**: Unified π-model branches with transformers, phase-shifters, and shunt elements. Constructs the nodal admittance matrix (Y-bus) from bus/branch data.
- **Full Measurement Suite**: Voltmeters, ammeters (with squared-form option), wattmeters, varmeters, and PMUs in both polar and rectangular coordinate systems with optional error correlation.
- **Three WLS Solver Methods**:
  - **Normal (LU)** — standard Gauss-Newton via gain matrix factorization
  - **Orthogonal (QR)** — improved numerical stability for ill-conditioned systems
  - **Peters-Wilkinson** — LU-based alternative with better conditioning properties
- **Jacobian Computation**: All measurement functions h(x) and their analytical partial derivatives, directly from the JuliaGrid mathematical formulations.

## Dependency Mapping: Julia → Rust

| Purpose | Julia (JuliaGrid) | Rust (StateRustimation) |
|---|---|---|
| Sparse matrices | `SparseArrays` (stdlib) | [`sprs`](https://crates.io/crates/sprs) v0.11 |
| Dense linear algebra | `LinearAlgebra` (stdlib) | [`nalgebra`](https://crates.io/crates/nalgebra) v0.34 |
| Complex numbers | `Base.Complex` (stdlib) | [`num-complex`](https://crates.io/crates/num-complex) v0.4 |
| Serialization | HDF5.jl / custom | [`serde`](https://crates.io/crates/serde) + [`serde_json`](https://crates.io/crates/serde_json) |
| Error handling | Julia exceptions | [`thiserror`](https://crates.io/crates/thiserror) v2 |
| Logging | `@info` / `@debug` macros | [`log`](https://crates.io/crates/log) + [`env_logger`](https://crates.io/crates/env_logger) |
| Optimization (LAV) | JuMP + Ipopt | *(not yet implemented)* |

## Module Structure

```
src/
├── lib.rs            # Library root — public module declarations
├── main.rs           # Example: 3-bus system from JuliaGrid tutorial
├── power_system.rs   # Bus, Branch, PowerSystem types (JuliaGrid's PowerSystem)
├── ac_model.rs       # Y-bus construction, branch parameter precomputation
├── measurement.rs    # All measurement device types (V, I, P, Q, PMU)
├── jacobian.rs       # h(x) evaluation and Jacobian matrix J(x) construction
└── solver.rs         # Gauss-Newton WLS solver (Normal, Orthogonal, PW methods)
```

## Quick Start

```bash
# Build
cargo build --release

# Run the 3-bus example
cargo run --release

# Run with debug logging
RUST_LOG=info cargo run
```

## Example Output

```
Power system: 3 buses, 3 branches
Measurements: 8 equations

=== Normal (LU) Method ===
Converged: true in 4 iterations (max|Δx| = 1.90e-9)
Bus  |  V (p.u.)  |  θ (rad)
-----|------------|----------
  1  | 1.0005824773 | 0.0000000000
  2  | 0.8752366332 | -0.1337572321
  3  | 0.8993422332 | -0.1998178530
```

## Documentation

The full documentation is built with [MkDocs](https://www.mkdocs.org/) and published on [Read the Docs](https://readthedocs.org). It covers the free edition only.

- **Build locally**: `pip install -r docs/requirements.txt && mkdocs serve`
- **Read the Docs**: Import the repo at [readthedocs.org](https://readthedocs.org/dashboard/import/) — builds use `.readthedocs.yaml` and `mkdocs.yml` automatically.

## References

- Cosovic, M. et al. (2025). *JuliaGrid: An Open-Source Julia-Based Framework for Power System State Estimation*. arXiv:2502.18229v2.
- Abur, A. & Expósito, A. (2004). *Power System State Estimation: Theory and Implementation*. Taylor & Francis.
- [JuliaGrid documentation](https://mcosovic.github.io/JuliaGrid.jl/stable/)
