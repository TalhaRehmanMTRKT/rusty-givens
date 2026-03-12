# REST API

The estimate service exposes a REST API on port 3001. All endpoints are prefixed with `/api`.

## Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/api/network` | Topology (buses, branches) for vis-network visualization |
| `GET` | `/api/true-state` | Reference power-flow state (for error comparison) |
| `POST` | `/api/estimate` | Run WLS state estimation with given configuration |
| `GET` | `/api/last-result` | Most recent estimation result |
| `GET` | `/api/measurements` | Measurement export (for debugging/analysis) |

## POST /api/estimate

Request body (JSON):

```json
{
  "factorization": "SparseCholesky",
  "max_iterations": 50,
  "tolerance": 1e-4
}
```

### Parameters

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `factorization` | string | `"SparseCholesky"` | `SparseCholesky`, `SparseLU`, or `DenseCholesky` |
| `max_iterations` | number | 50 | Maximum Gauss-Newton iterations |
| `tolerance` | number | 1e-4 | Convergence tolerance (max \|Δx\|) |

### Response

```json
{
  "converged": true,
  "iterations": 4,
  "solve_time_sec": 0.12,
  "bus_results": [
    {
      "index": 0,
      "label": 1,
      "est_vm": 1.00058,
      "est_va_deg": 0.0,
      "true_vm": 1.0,
      "true_va_deg": 0.0,
      "vm_error": 0.00058,
      "va_error_deg": 0.0
    }
  ]
}
```

## Example

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{"factorization":"SparseCholesky","max_iterations":50,"tolerance":1e-4}'
```
