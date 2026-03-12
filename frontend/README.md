# StateRustimation Frontend

Angular web UI for running and visualizing WLS State Estimation on the GB network.

## Features

- **Grid View** — interactive vis-network graph showing the 2224-bus GB network,
  colored by voltage level (400/275/132/33/11 kV). After SE runs, nodes are
  re-colored by estimation error magnitude.
- **Config Panel** — choose the Rust solver's gain factorization method
  (Sparse Cholesky, Sparse LU, or Dense Cholesky), set max iterations and
  convergence tolerance.
- **Summary Card** — key metrics at a glance: convergence status, timing,
  VM/VA MAE and max errors.
- **Results Table** — sortable, filterable, paginated table of per-bus
  estimated vs. true voltage magnitude and angle, with error highlighting.

## Prerequisites

| Dependency | Version |
|---|---|
| Node.js | ≥ 20 |
| Angular CLI | 18 |
| Rust API server | `cargo run --release --bin api_server` (port 3001) |

## Quick Start

```bash
# 1. Start the Rust API server (from the project root)
cargo run --release --bin api_server

# 2. In another terminal, start the Angular dev server
cd frontend
npm install          # first time only
npx ng serve --open  # opens http://localhost:4200
```

The Angular dev server proxies `/api/*` requests to `http://localhost:3001`
(configured in `proxy.conf.json`).

## Build for Production

```bash
npx ng build --configuration=production
```

Output goes to `dist/frontend/`. Serve with any static file server,
pointing API requests to the Rust backend.
