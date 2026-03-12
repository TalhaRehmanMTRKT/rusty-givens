# Rusty Givens

A modular **AC Weighted Least-Squares State Estimator** for power systems, written in Rust.

Rusty Givens ships in two editions:

- **Free edition** — fully open-source SE kernel with six solver formulations, zero-injection handling, post-estimation evaluation, a dual REST + gRPC API, and an Angular frontend for the Great Britain transmission grid.
- **Pro edition** — adds Observability Analysis, Measurement Redundancy Analysis, and iterative Bad Data Detection with measurement lifecycle management.

---

## Free Edition — Components

| Component | Crate / Directory | Description |
|-----------|-------------------|-------------|
| **SE Kernel** | `rusty-givens-core` | Pure-Rust Gauss-Newton WLS solver. Six solver formulations, zero-injection bus handling, post-estimation power flow evaluation. No I/O or serialization — the kernel is a library with zero side effects. |
| **I/O Layer** | `rusty-givens-io` | JSON deserialization of case files. `load_case()` returns a `PowerSystem`, `MeasurementSet`, `TrueState`, and `BusMetadata` ready for the kernel. |
| **Estimate Service** | `services/estimate-service` | HTTP server exposing both REST (JSON, port 3001) and gRPC (Protobuf, port 50051) APIs. Loads the case at startup and holds shared application state in memory. |
| **Angular Frontend** | `frontend/` | Educational web UI with a geo-referenced Leaflet map of the GB 275/400 kV grid, configurable solver settings, tabular bus results, and summary metrics. |
| **Case Study** | `case_study/` | GB transmission network (2,224 buses, 3,207 branches) and a reduced EHV-only variant (793 buses). Includes Python scripts for extraction from pandapower and comparison tooling. |

---

## SE Kernel — Solver Formulations

Rusty Givens implements six solver formulations, all sharing the same Jacobian evaluator and measurement model:

| Formulation | Method | When to use |
|-------------|--------|-------------|
| **Normal Equations** | \( G\,\Delta x = H^\top W\,\Delta z \), \( G = H^\top W H \) | Default. Good balance of speed and stability for well-conditioned systems. |
| **Orthogonal QR (Givens)** | QR factorization of \( \tilde{H} = W^{1/2} H \) via Givens rotations | Better numerical conditioning than Normal Equations — avoids squaring the condition number. |
| **Peters-Wilkinson** | LU factorization of the gain matrix | More robust than Cholesky when the gain matrix is near-singular. |
| **Equality-Constrained** | Lagrangian KKT system for zero-injection equality constraints | Exact enforcement of zero-injection constraints (vs. virtual measurements). |
| **Fast Decoupled** | Separate P-θ and Q-V sub-problems with constant gain matrices | Fast convergence for well-conditioned grids; useful for online applications. |
| **DC Estimation** | Linear model assuming flat voltages and small angles | Active power only; single-shot solve (no iteration). |

Each formulation (except DC) uses Gauss-Newton iteration with configurable tolerance and maximum iterations.

---

## Measurement Types

| Type | Location | Measured quantity |
|------|----------|-----------------|
| **Voltmeter** | Bus | Voltage magnitude \( |V_i| \) |
| **Ammeter** | Branch terminal | Current magnitude \( |I_{ij}| \) |
| **Wattmeter** | Bus injection or branch flow | Active power \( P \) |
| **Varmeter** | Bus injection or branch flow | Reactive power \( Q \) |
| **PMU** | Bus or branch terminal | Voltage/current magnitude + angle (polar or rectangular) |
| **Current Angle Meter** | Branch terminal | Current phasor angle |

All measurements carry a variance (σ²) for WLS weighting. Measurements can be individually enabled or disabled.

---

## Zero-Injection Bus Handling

Buses with neither generation nor load require special treatment to enforce zero net power injection. Rusty Givens supports two methods:

- **Virtual Measurements** — P = 0 and Q = 0 pseudo-measurements injected with a very small standard deviation (default σ = 10⁻⁶). Compatible with all solver formulations.
- **Equality Constraints** — Exact c(x) = 0 constraints via the Lagrangian KKT system. Only available with the Equality-Constrained formulation.

After estimation, a **violation report** checks whether estimated injections at zero-injection buses exceed a configurable threshold.

Users can also supply an **explicit list of zero-injection bus labels** to override automatic detection, covering the SCADA-operator use case where zero-injection buses are known a priori.

---

## Post-Estimation Evaluation

After a converged estimation, the kernel evaluates dependent quantities:

- **Branch power flows** — \( P_{ij}, Q_{ij}, |I_{ij}| \) at both terminals of every branch
- **Branch losses** — \( \Delta P = P_{ij} + P_{ji} \), \( \Delta Q = Q_{ij} + Q_{ji} \)
- **Bus injections** — net \( P_i, Q_i \) at every bus from the admittance matrix
- **System power balance** — total generation, load, and losses
- **Objective function** — weighted sum of squared residuals \( J(\hat{x}) = r^\top W\,r \) with expected value and degrees of freedom
- **Per-voltage-level statistics** — bus/branch counts and measurement distribution by nominal voltage

---

## Dual API

The estimate service exposes the same core functionality over two transports:

| API | Transport | Port | Protocol |
|-----|-----------|------|----------|
| **REST** | HTTP/1.1 | 3001 | JSON |
| **gRPC** | HTTP/2 | 50051 | Protobuf |

Both APIs share the same application state and produce identical results.

---

## Pro Edition — Capabilities

The Pro edition (compiled with `--features pro`) adds three analysis modules that build on top of the free SE kernel:

| Module | Capability |
|--------|------------|
| **Observability Analysis** | Determines whether the system is fully observable from the current measurement set. Identifies unobservable buses and islands in both the P-θ and Q-V sub-models. Recommends pseudo-measurement placement. |
| **Redundancy Analysis** | Classifies every measurement by redundancy level — Critical, No Redundancy, Simply Redundant, Multiply Redundant. Computes Good/Bad marks per bus and branch. Supports before-and-after-BDD comparison. |
| **Bad Data Detection** | Iterative detection, identification, and elimination of erroneous measurements using Chi-squared and residual tests. Includes measurement lifecycle management with automatic deactivation, release cycles, and historical tracking. |

See the [Pro Edition](pro.md) page for a full capabilities overview.

---

## License

The free edition is released under the **Apache License 2.0**. The Pro edition is available under a separate commercial license.
