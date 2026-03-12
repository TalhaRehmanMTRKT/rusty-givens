# Rusty Givens

A free, open-source **AC Weighted Least-Squares State Estimator** for power systems, written in Rust.

Learn how SE works with the included Angular frontend and dual REST + gRPC APIs.

---

## FOSS Components

The free Rusty Givens library ships with three core pieces:

| Component | Crate | Description |
|-----------|-------|-------------|
| **SE Kernel** | `rusty-givens-core` | Pure Gauss-Newton WLS solver. Zero I/O, zero serde. Sparse Cholesky, sparse LU, or dense Cholesky via `faer`. Exposes the `SeSolver` trait for pluggable backends. |
| **I/O Layer** | `rusty-givens-io` | JSON deserialization, format conversion, and `load_case()`. Loads network topology, measurements, and true state for validation. |
| **Angular Frontend** | `frontend` | Educational web UI: geo-referenced GB transmission grid, configurable factorization, tabular bus results. |

---

## SE Kernel Features

- **Sparse Cholesky (LLT)** — Default factorization via `faer`. Exploits the SPD structure of the gain matrix.
- **Sparse LU Fallback** — For non-SPD or numerically difficult gain matrices.
- **Dense Cholesky** — Small-system path for networks under ~200 buses.
- **Pluggable Backends** — The `SeSolver` trait allows swapping the Rust kernel for other solvers.
- **Structured Diagnostics** — Per-iteration timing returned in the result.
- **Zero-Copy Gain Cache** — Sparsity pattern computed once, values refilled each iteration.

---

## Angular Frontend

- **Geo-referenced Grid** — Interactive map of the GB 400/275 kV transmission network.
- **Configurable Factorization** — Choose Sparse Cholesky, Sparse LU, or Dense Cholesky via the UI.
- **Tabular Results** — Per-bus estimated voltage magnitude (p.u.) and angle (deg).
- **Summary Metrics** — Convergence status, iteration count, solve time, and error metrics.

---

## Two APIs

Rusty Givens exposes the same core functionality over **REST (JSON)** and **gRPC (protobuf)**.

| API | Transport | Endpoints |
|-----|-----------|-----------|
| **REST** | HTTP/1.1 · JSON | `GET /api/network`, `GET /api/true-state`, `POST /api/estimate`, `GET /api/last-result` |
| **gRPC** | HTTP/2 · Protobuf | `EstimateService`: GetNetwork, GetTrueState, RunEstimate, GetLastResult |

---

!!! info "Documentation scope"
    This documentation covers the **free edition** only. Pro features (observability analysis, redundancy analysis, Bad Data Detection) are available under a separate license and are not documented here.
