# Architecture

## Module structure

The free edition consists of the following crates:

```
crates/
├── rusty-givens-core    # SE kernel: WLS solver, Jacobian, factorization
├── rusty-givens-io      # I/O: load_case, JSON, format conversion
└── gb-case-study        # GB network case data

services/
└── estimate-service     # HTTP API (REST + gRPC)

frontend/                # Angular web UI
```

## SE Kernel (`rusty-givens-core`)

The kernel is pure computational logic with no I/O or serialization:

- **`model`** — `PowerSystem`, `AcModel`, `MeasurementSet` types
- **`kernel`** — `SeSolver` trait, `WlsSolver`, `EstimationConfig`, `EstimationResult`

### Trait contract

```rust
pub trait SeSolver {
    fn estimate(
        &self,
        system: &PowerSystem,
        model:  &AcModel,
        measurements: &MeasurementSet,
        config: &EstimationConfig,
    ) -> Result<EstimationResult, SolverError>;
}
```

## I/O layer (`rusty-givens-io`)

- `load_case(path)` — Loads network topology, measurements, and true state from JSON
- Format conversion for vis-network and API payloads

## Estimate service

The `estimate-service` binary runs both:

- **REST API** on port 3001 (JSON over HTTP/1.1)
- **gRPC API** on port 50051 (protobuf over HTTP/2)

Both APIs share the same application state and case data.
