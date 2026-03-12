# GB EHV Network (275/400 kV) with Equivalent Injections

Reduced version of the full GB network retaining only the Extra-High-Voltage
(275 kV + 400 kV) transmission grid.  All sub-transmission and distribution
networks (≤132 kV) are replaced by **equivalent constant-power injections**
at each boundary bus.

## Network summary

| Metric | Full GB | EHV reduced |
|--------|---------|-------------|
| Buses | 2 224 | 793 |
| Branches | 3 207 | 1 039 |
| Boundary trafos removed | — | 719 |
| Boundary buses (with equiv. injection) | — | 323 |
| Measurements | ~14 000 | 4 290 |
| State variables | 4 447 | 1 585 |
| Redundancy ratio | ~3.1 | 2.71 |

## Where the equivalent injections are placed

```
  ┌──────────────────────────────────────────────────────┐
  │               EHV TRANSMISSION GRID                   │
  │           (275 kV and 400 kV — retained)              │
  │                                                       │
  │   Bus i ──── Line ──── Bus j ──── 400/275 trafo ──── │
  │     │                    │                             │
  │     │                    │                             │
  └─────┼────────────────────┼─────────────────────────────┘
        │                    │
        │ ← BOUNDARY         │ ← BOUNDARY
        │   TRANSFORMER       │   TRANSFORMER
        │   (removed)         │   (removed)
        │                    │
  ┌─────┼────────────────────┼─────────────────────────────┐
  │     ▼                    ▼                             │
  │   132 kV              33 kV     ← DISTRIBUTION        │
  │   buses               buses       (entirely removed)   │
  │     │                    │                             │
  │   loads, gens,        loads, gens                      │
  │   further trafos      ...                              │
  └──────────────────────────────────────────────────────┘

  After reduction:

  ┌──────────────────────────────────────────────────────┐
  │               EHV TRANSMISSION GRID                   │
  │                                                       │
  │   Bus i ──── Line ──── Bus j ──── 400/275 trafo ──── │
  │     │                    │                             │
  │   [P_eq,Q_eq]          [P_eq,Q_eq]                    │
  │   ▲ equiv.              ▲ equiv.                       │
  │     injection             injection                    │
  └──────────────────────────────────────────────────────┘
```

At each **boundary bus** (an EHV bus that was connected to at least one
transformer feeding a lower-voltage network), we add an equivalent
constant-power injection:

```
P_equiv(bus_i) = Σ  p_hv_mw(trafo_k)     for all removed trafos k at bus i
Q_equiv(bus_i) = Σ  q_hv_mvar(trafo_k)
```

where `p_hv_mw` / `q_hv_mvar` are the power flows at the HV side of each
boundary transformer, obtained from AC power flow on the full network.
These values represent the total active/reactive power that was flowing
from the transmission grid into the distribution network.

The equivalent injection is added to the bus **demand**:

```
p_demand_new = p_demand_orig + P_equiv / base_mva
q_demand_new = q_demand_orig + Q_equiv / base_mva
```

### Interpretation

- **Positive `P_equiv`**: power flows from the EHV bus *down* into
  distribution → appears as additional load at the boundary bus.
- **Negative `P_equiv`**: net generation in the distribution network
  pushes power *up* into the transmission grid → appears as negative
  demand (i.e., additional generation) at the boundary bus.
- Bus 81 (400 kV) is an example of a net-exporting boundary:
  `P_equiv = −40.9 MW` (distribution generation exceeds distribution load).

## SE convergence results

| Formulation | Converged | Iterations | SE time |
|-------------|-----------|------------|---------|
| Normal Equations / Sparse LU | **yes** | 4 | 0.071 s |
| Peters-Wilkinson | **yes** | 4 | 0.069 s |
| Equality-Constrained | **yes** | 6 | 0.864 s |
| Fast Decoupled | **yes** | 6 | 0.029 s |
| DC Estimation | **yes** | 1 | 0.047 s |
| Normal Equations / Sparse Cholesky | **no** (non-positive pivot 522) | — | — |
| Orthogonal QR | **no** (assertion: rectangular matrix) | — | — |

### Root-cause analysis of the two failures

#### 1. Normal Equations / Sparse Cholesky — non-positive pivot

The gain matrix G = H'WH fails Cholesky (LLT) at pivot index **522**,
which corresponds to the voltage angle θ of bus 562 (275 kV, PQ).

**Root cause: 287 radial generator buses (36% of all EHV buses).**

The EHV network retains all 275/400 kV buses, including hundreds of PV
generator buses that are each connected to the grid by a single
high-impedance step-up transformer:

```
  Grid bus ─── x ≈ 0.05–0.11 p.u. ──── PV gen bus (radial, degree 1)
              (step-up trafo)
```

| Bus degree | Count | % of total |
|------------|-------|------------|
| 1 (radial) | 287 | 36.2% |
| 2 (chain) | 214 | 27.0% |
| ≥ 3 | 292 | 36.8% |

These radial generator buses create very small off-diagonal entries in
the gain matrix because:

- The Jacobian ∂P/∂θ for a branch with large reactance x is proportional
  to V²/x, yielding small sensitivity.
- With only a single branch connected, the gain row for that bus angle
  has a near-zero diagonal entry.
- The gain matrix G = H'WH accumulates these near-zero diagonal blocks,
  which eventually cause a non-positive pivot during Cholesky factorization.

This is **not** an observability problem — the system is fully connected
and the redundancy ratio is 2.71.  It is a **numerical conditioning**
issue specific to Cholesky's requirement that all leading sub-matrices
be positive definite.  Sparse LU (which handles indefinite matrices)
converges in the same number of iterations.

#### 2. Orthogonal QR — rectangular matrix assertion

The QR solver uses a **size threshold** to choose between two paths:

```
s = 2 × n_buses = 1586    (state variables)
DENSE_QR_THRESHOLD = 2000
```

Since `s < 2000`, the solver takes the **dense QR** path, which:

1. Builds the weighted Jacobian H̃ = W^{1/2} H of shape (4291 × 1586)
2. Calls `faer::col_piv_qr().solve()`

The `solve()` method in faer 0.24 asserts `nrows == ncols`  — it is
designed for square systems, not least-squares.  The dense QR path would
need `solve_lstsq()` or equivalent for overdetermined systems.

This **does not occur** on the full GB network because `s = 4448 ≥ 2000`
routes to the **sparse QR** path, which assembles G = H'WH and solves
via Cholesky (mathematically equivalent to QR but avoids the rectangular
matrix issue).

**In summary:** the QR failure is a code-path bug (dense QR solve
does not handle rectangular matrices), not a numerical or observability
issue with the reduced network.

### Estimation accuracy (Peters-Wilkinson, 4 iterations)

| Metric | Value |
|--------|-------|
| VM MAE | 1.60 × 10⁻⁴ p.u. |
| VM max error | 3.43 × 10⁻³ p.u. |
| VA MAE | 1.64 × 10⁻⁴ rad (0.0094°) |
| VA max error | 5.44 × 10⁻³ rad (0.31°) |

## Files

| File | Description |
|------|-------------|
| `extract_ehv_network.py` | Python script (uses pandapower) to generate the reduced network |
| `gb_ehv_network.json` | Reduced network JSON for the Rust SE solver |
| `boundary_injections.csv` | Per-bus breakdown of equivalent injections at all 323 boundary buses |

## Regenerating

```bash
python case_study/ehv_network/extract_ehv_network.py
```

## Running SE on the reduced network

```bash
CASE_FILE=case_study/ehv_network/gb_ehv_network.json \
SOLVER_FORMULATION=PetersWilkinson \
cargo run --release --bin gb-case-study
```
