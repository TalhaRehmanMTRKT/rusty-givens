# Architecture

## Workspace Layout

```
StateRustimation/
├── crates/
│   ├── rusty-givens-core/      # SE kernel (model + solver)
│   ├── rusty-givens-io/        # I/O: JSON loading, format conversion
│   ├── gb-case-study/          # Standalone binary for the GB case
│   └── pro/                    # Pro edition (feature-gated)
│       ├── rusty-givens-obs/   # Observability Analysis
│       ├── rusty-givens-red/   # Redundancy Analysis
│       └── rusty-givens-bdd/   # Bad Data Detection
├── services/
│   └── estimate-service/       # HTTP API server (REST + gRPC)
├── frontend/                   # Angular web UI
├── proto/                      # Protobuf service definitions
├── case_study/                 # GB network case data + Python scripts
└── docs/                       # This documentation (MkDocs)
```

The free edition includes everything except `crates/pro/`. The Pro crates are compiled conditionally via `--features pro` on the estimate service.

---

## SE Kernel (`rusty-givens-core`)

The kernel is a pure library with **no I/O**, **no serialization**, and **no side effects**. It exposes two modules:

### `model` — Data Structures

| Type | Description |
|------|-------------|
| `PowerSystem` | Network topology: buses (PQ/PV/Slack, demand, shunts, geo-coordinates), branches (π-model with R, X, B, G, tap ratio, phase shift), bus index lookup. |
| `AcModel` | Pre-computed admittance model: complex \( Y_{bus} \) (sparse), \( G_{bus} \), \( B_{bus} \), per-branch parameters (g, b, g_s, b_s, τ, φ). Built once at startup from `PowerSystem`. |
| `MeasurementSet` | All telemetered measurements: voltmeters, ammeters, wattmeters, varmeters, PMUs, and current angle meters. Each measurement has a value, variance (σ²), location, and status flag. |

### `kernel` — Solver and Evaluation

| Type | Description |
|------|-------------|
| `SeSolver` trait | Pluggable solver interface. Single method: `estimate(system, model, measurements, config) → EstimationResult`. |
| `WlsSolver` | The built-in implementation of `SeSolver`. Dispatches to six formulations based on `EstimationConfig.formulation`. |
| `EstimationConfig` | Solver parameters: `formulation`, `max_iterations`, `tolerance`, `zero_injection` config. |
| `EstimationResult` | Output: voltage magnitudes, angles, convergence flag, iteration count, per-iteration diagnostics, solver artifacts, and zero-injection report data. |
| `SolverArtifacts` | Final-iteration matrices retained for post-SE analysis: Jacobian \( H \), residuals \( r \), measurements \( z \), estimated values \( h(\hat{x}) \), precision diagonal \( W \), gain matrix \( G \), measurement map. |

### `SeSolver` Trait Contract

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

Any struct implementing this trait can serve as a drop-in solver backend.

---

## Solver Formulations

All six formulations share the same **Jacobian evaluator** — the measurement function \( h(x) \) and its partial derivatives \( H = \partial h / \partial x \) are computed identically. Only the downstream linear algebra differs.

### Normal Equations (Gauss-Newton)

The standard WLS approach. At each iteration:

1. Evaluate \( h(x) \) and \( H(x) \) for all active measurements.
2. Compute residuals \( r = z - h(x) \).
3. Assemble the gain matrix \( G = H^\top W H \) (sparse CSC).
4. Solve \( G\,\Delta x = H^\top W\,r \) via Cholesky or LU.
5. Update state \( x \leftarrow x + \Delta x \).
6. Check convergence: \( \max|\Delta x| < \varepsilon \).

Three **factorization backends** are available for step 4:

| Backend | Method | Notes |
|---------|--------|-------|
| `SparseCholesky` | Sparse LLT via `faer` | Default. Exploits SPD structure of \( G \). Symbolic factorization cached. |
| `SparseLU` | Sparse LU via `faer` | Fallback for numerically difficult gain matrices. |
| `DenseCholesky` | Dense LLT via `faer` | Small-system path (< ~200 buses). Falls back to dense LU if not SPD. |

The **sparsity pattern is cached**: the first iteration computes the symbolic factorization (fill-reducing ordering); subsequent iterations only refill numerical values.

### Orthogonal QR (Givens Rotations)

Avoids forming \( G = H^\top W H \), which squares the condition number. Instead:

1. Form \( \tilde{H} = W^{1/2} H \) and \( \tilde{z} = W^{1/2} r \).
2. Apply Givens rotations to triangularize \( \tilde{H} = Q\,R \).
3. Solve \( R\,\Delta x = Q_n^\top \tilde{z} \) by back-substitution.

For small networks (\( s < 2000 \) state variables), uses dense column-pivoted QR via `faer`. For large networks, falls back to sparse gain + Cholesky to manage memory.

### Peters-Wilkinson

Uses **LU factorization** of the gain matrix instead of Cholesky. More robust when \( G \) is near-singular or poorly conditioned. The gain matrix assembly is identical to Normal Equations; only the factorization step changes.

### Equality-Constrained WLS

Enforces zero-injection constraints exactly via the **Lagrangian KKT system**:

\[
\begin{bmatrix} G & C^\top \\ C & 0 \end{bmatrix}
\begin{bmatrix} \Delta x \\ \lambda \end{bmatrix}
=
\begin{bmatrix} H^\top W\,r \\ -c(x) \end{bmatrix}
\]

where \( c(x) = 0 \) are the zero-injection equality constraints and \( \lambda \) are Lagrange multipliers. An optional scaling parameter \( \alpha \) improves conditioning.

The augmented matrix is indefinite, so Cholesky cannot be used; this formulation uses **LU factorization**.

### Fast Decoupled

Exploits the P-θ / Q-V decoupling in well-conditioned power systems:

1. Build constant sub-gain matrices \( G_P \) and \( G_Q \) from the flat-start Jacobian (computed once).
2. Alternate between P-θ half-steps (update angles from active power residuals) and Q-V half-steps (update magnitudes from reactive power residuals).

Converges faster than coupled methods when the system is well-conditioned. Does **not** support branch current magnitude measurements (ammeters couple P and Q).

### DC Estimation

A linear approximation assuming flat voltages (\( |V| = 1.0 \) p.u.) and small angle differences:

\[
P_{ij} \approx \frac{\theta_i - \theta_j}{x_{ij}}
\]

Only active power measurements (bus injections and branch flows) are used. The estimation reduces to a single WLS solve — no iteration required. Useful for fast screening, topology processing, and as an initial guess for AC SE.

---

## Zero-Injection Bus Handling

Zero-injection buses are buses with no generator, no load, and no shunt — the net power injection must be exactly zero. Rusty Givens handles them in two ways:

### Virtual Measurements

Artificial measurements \( P_i = 0 \) and \( Q_i = 0 \) are injected with a very small standard deviation (default σ = 10⁻⁶). This creates strong — but not exact — enforcement. Compatible with **all six solver formulations**.

### Equality Constraints

The constraints \( P_i = 0 \) and \( Q_i = 0 \) are added to the KKT system via Lagrange multipliers. This gives mathematically exact enforcement. Available **only** with the Equality-Constrained formulation.

### Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | `true` | Enable/disable zero-injection handling |
| `method` | `VirtualMeasurements` | `VirtualMeasurements` or `EqualityConstraints` |
| `sigma` | `1e-6` | Standard deviation for virtual measurements |
| `violation_threshold_pu` | `1e-3` | Threshold for post-estimation violation reporting |
| `explicit_buses` | `None` | Optional list of bus labels to treat as zero-injection, bypassing automatic detection |

### Automatic Detection

By default, buses are automatically identified as zero-injection if they have:

- No active or reactive demand
- No generation
- No existing P or Q injection measurement
- Bus type is not Slack or PV

### Post-Estimation Violation Report

After estimation, the kernel checks whether the estimated injection at each zero-injection bus exceeds `violation_threshold_pu`. The report includes:

- Total ZI buses and virtual measurement pairs injected
- Per-bus estimated P and Q injections
- Violation flags for buses exceeding the threshold

---

## Post-Estimation Evaluation

When estimation converges, the kernel computes dependent quantities from the estimated state \( \hat{V}, \hat{\theta} \):

| Quantity | Description |
|----------|-------------|
| **Branch power flows** | \( P_{ij}, Q_{ij}, |I_{ij}| \) at the from-terminal and to-terminal of every branch, computed from the π-model. |
| **Branch losses** | \( \Delta P = P_{ij} + P_{ji} \), \( \Delta Q = Q_{ij} + Q_{ji} \) per branch. |
| **Bus injections** | Net \( P_i, Q_i \) at each bus, computed from the admittance matrix. |
| **System power balance** | \( \sum P_{gen} \), \( \sum P_{load} \), \( \sum P_{loss} \) (and corresponding Q). |
| **Objective function** | \( J(\hat{x}) = r^\top W\,r \) with expected value \( E[J] = m - n \) and degrees of freedom. |
| **Per-voltage-level statistics** | Bus counts, branch counts, and measurement distribution grouped by nominal voltage. |

These results are included in the API response and displayed in the Angular frontend.

---

## Per-Iteration Diagnostics

Each Gauss-Newton iteration records:

| Field | Description |
|-------|-------------|
| `iteration` | Iteration number (0-indexed) |
| `max_delta_x` | \( \max|\Delta x| \) — convergence metric |
| `jacobian_time_s` | Time to evaluate \( h(x) \) and \( H(x) \) |
| `gain_time_s` | Time to assemble \( G = H^\top W H \) |
| `solve_time_s` | Time to factorize and solve the linear system |
| `total_time_s` | Wall-clock time for the entire iteration |

---

## Solver Artifacts

The kernel retains the matrices and vectors from the final iteration so that downstream analyses can run **without re-solving**:

| Artifact | Description |
|----------|-------------|
| `jacobian` | \( H \) — sparse Jacobian (CSC) |
| `residuals` | \( r = z - h(\hat{x}) \) |
| `measurement_z` | Original measurement vector \( z \) |
| `measurement_h` | Estimated measurement values \( h(\hat{x}) \) |
| `precision_diag` | Diagonal of the precision matrix \( W = \text{diag}(1/\sigma^2) \) |
| `gain_matrix` | \( G = H^\top W H \) — sparse gain matrix (CSC) |
| `n_states` | Number of state variables \( 2n - 1 \) |
| `slack_index` | Index of the slack bus (reference angle) |
| `measurement_map` | Ordered list of `MeasurementRef` mapping each equation row to its physical measurement |

These artifacts are consumed by the Pro edition's Observability, Redundancy, and Bad Data Detection modules.

---

## I/O Layer (`rusty-givens-io`)

The I/O crate handles JSON deserialization and format conversion:

- `load_case(path)` — Reads a JSON case file and returns:
    - `PowerSystem` — network topology
    - `MeasurementSet` — all measurements with values, variances, and status
    - `TrueState` — reference voltage magnitudes and angles for error comparison
    - `BusMetadata` — bus labels, nominal voltages, types, and geo-coordinates
    - `CaseInfo` — summary statistics (n_buses, n_branches, slack index, base MVA)

---

## Estimate Service

The `estimate-service` binary runs both API transports:

- **REST** on port 3001 (JSON over HTTP/1.1 via `axum`)
- **gRPC** on port 50051 (Protobuf over HTTP/2 via `tonic`)

Both APIs share the same `AppState` (loaded case data, last SE result) and call the same `execute_estimation()` function.

### Pro Feature Gate

When compiled with `--features pro`, the service additionally:

- Imports the OBS, RED, and BDD crates
- Registers additional REST routes (`/api/observability`, `/api/redundancy`, `/api/bdd/*`)
- Registers additional gRPC services (`ObsService`, `RedService`, `BddService`)
- Runs an observability check before each SE run (can be skipped via `skip_obs_check`)

---

## Angular Frontend

The frontend is an Angular 18 single-page application with:

| Component | Description |
|-----------|-------------|
| **Grid View** | Leaflet map showing the GB 275/400 kV transmission grid. Buses are geo-referenced and color-coded by voltage level; after estimation, buses are colored by voltage magnitude error. |
| **Config Panel** | Dropdown for solver formulation and factorization, iteration/tolerance inputs, and a Run button. |
| **Summary Card** | Convergence status, iteration count, solve time, error metrics (MAE, max error), measurement breakdown, and power balance. |
| **Results Table** | Per-bus table with estimated and true voltage magnitudes/angles, and per-bus errors. |
| **Redundancy Panel** | *(Pro only)* Displays redundancy analysis results. |

The frontend proxies `/api` requests to `http://localhost:3001`.

---

## Key Design Decisions

1. **Y_bus and AcModel are built once at startup.** The network topology does not change between estimation runs. Only the state vector changes during iteration.

2. **Sparsity pattern is cached.** The gain matrix \( G = H^\top W H \) has the same nonzero pattern every iteration. The symbolic factorization (fill-reducing ordering) is computed once; only numerical values are refilled.

3. **Six solver formulations share the same Jacobian evaluator.** The measurement function \( h(x) \) and its derivatives are computed identically regardless of which linear algebra method is used downstream.

4. **Dual API transport.** The same `execute_estimation()` function is called by both REST and gRPC handlers, ensuring identical behaviour.

5. **Solver artifacts are retained.** The final-iteration Jacobian, residuals, and gain matrix are stored so that post-estimation analyses (BDD, Observability, Redundancy) can run without re-solving.

6. **Free and Pro are a single binary.** The Pro crates are conditionally compiled via Cargo features. The free binary has zero Pro dependencies.
