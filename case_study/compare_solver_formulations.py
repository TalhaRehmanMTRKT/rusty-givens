#!/usr/bin/env python3
"""
Compare all 6 solver formulations in rusty-givens-core on the GB case study.

Formulations:
  1. NormalEquations   — Standard WLS via Gain matrix G = HᵀWH (Ch. 2.6)
  2. OrthogonalQR      — QR factorization via Givens rotations (Ch. 3.2)
  3. PetersWilkinson  — LU of H̃; LᵀL better conditioned (Ch. 3.4)
  4. EqualityConstrained — Zero injections as explicit constraints (Ch. 3.5)
  5. FastDecoupled     — P-θ / Q-V decoupled sub-problems (Ch. 2.7)
  6. DcEstimation     — Linear model, active power only (Ch. 2.8)

Uses the GB network exported by extract_gb_network.py.
Results are written to solver_formulations_report.txt.
"""

import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime

import numpy as np

CASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(CASE_DIR)
JSON_PATH = os.path.join(CASE_DIR, "gb_network.json")
RESULTS_JSON = os.path.join(CASE_DIR, "results_rust.json")
REPORT_PATH = os.path.join(CASE_DIR, "solver_formulations_report.txt")

# All 6 solver formulations in rusty-givens-core
FORMULATIONS = [
    ("NormalEquations", "Normal Equations (NE) — G = HᵀWH, Sparse Cholesky"),
    ("OrthogonalQR", "Orthogonal QR (Givens) — H̃ = W^{1/2}H, no gain matrix"),
    ("PetersWilkinson", "Peters & Wilkinson — LU of H̃, LᵀL solve"),
    ("EqualityConstrained", "Equality-Constrained — Lagrangian KKT, zero injections"),
    ("FastDecoupled", "Fast Decoupled — P-θ / Q-V constant gain sub-problems"),
    ("DcEstimation", "DC Estimation — Linear model, single solve"),
]


def load_case_json():
    """Load the GB network case from JSON."""
    with open(JSON_PATH) as f:
        return json.load(f)


def network_summary(case):
    """Extract network size and structure info."""
    n_buses = case["n_buses"]
    n_branches = case["n_branches"]
    meas = case.get("measurements", {})
    n_voltmeters = len(meas.get("voltmeters", []))
    n_ammeters = len(meas.get("ammeters", []))
    n_wattmeters = len(meas.get("wattmeters", []))
    n_varmeters = len(meas.get("varmeters", []))
    n_pmus = len(meas.get("pmus", []))
    n_equations = n_voltmeters + n_ammeters + n_wattmeters + n_varmeters + 2 * n_pmus

    # Bus type counts
    buses = case.get("buses", [])
    n_slack = sum(1 for b in buses if b.get("bus_type") == 3)
    n_pv = sum(1 for b in buses if b.get("bus_type") == 2)
    n_pq = sum(1 for b in buses if b.get("bus_type") == 1)

    return {
        "n_buses": n_buses,
        "n_branches": n_branches,
        "n_voltmeters": n_voltmeters,
        "n_ammeters": n_ammeters,
        "n_wattmeters": n_wattmeters,
        "n_varmeters": n_varmeters,
        "n_pmus": n_pmus,
        "n_equations": n_equations,
        "n_slack": n_slack,
        "n_pv": n_pv,
        "n_pq": n_pq,
        "base_mva": case.get("base_mva", 100.0),
        "slack_bus_index": case.get("slack_bus_index"),
    }


def run_formulation(formulation_name, env_extra=None):
    """Run the gb_case_study binary with the given formulation."""
    cargo = shutil.which("cargo")
    if cargo is None:
        cargo_env = os.path.expanduser("~/.cargo/env")
        if os.path.exists(cargo_env):
            shell_cmd = f'source "{cargo_env}" && SOLVER_FORMULATION={formulation_name} cargo run --release -p gb-case-study'
        else:
            return None, "cargo not found"
    else:
        shell_cmd = f"SOLVER_FORMULATION={formulation_name} cargo run --release -p gb-case-study"

    env = os.environ.copy()
    env["SOLVER_FORMULATION"] = formulation_name
    if env_extra:
        env.update(env_extra)

    t0 = time.time()
    proc = subprocess.run(
        shell_cmd,
        shell=True,
        capture_output=True,
        text=True,
        cwd=PROJECT_DIR,
        timeout=600,
        env=env,
    )
    wall_time = time.time() - t0

    if proc.returncode != 0:
        stderr = proc.stderr[-800:] if proc.stderr else "(no stderr)"
        return None, f"Binary failed (rc={proc.returncode}): {stderr}"

    if not os.path.exists(RESULTS_JSON):
        return None, "results_rust.json not found"

    with open(RESULTS_JSON) as f:
        res = json.load(f)

    return res, None


def error_metrics(est_vm, est_va, true_vm, true_va):
    """Compute error metrics between estimated and true state."""
    vm_err = np.abs(np.array(est_vm) - np.array(true_vm))
    va_err = np.abs(np.array(est_va) - np.array(true_va))
    va_err_deg = np.rad2deg(va_err)
    return {
        "vm_mae": float(np.mean(vm_err)),
        "vm_max": float(np.max(vm_err)),
        "vm_rmse": float(np.sqrt(np.mean(vm_err**2))),
        "va_mae_rad": float(np.mean(va_err)),
        "va_mae_deg": float(np.mean(va_err_deg)),
        "va_max_rad": float(np.max(va_err)),
        "va_max_deg": float(np.max(va_err_deg)),
        "va_rmse_deg": float(np.sqrt(np.mean(va_err_deg**2))),
    }


def write_report(results, net_summary, case, report_path):
    """Write the .txt report with network info, SE results, and performance."""
    sep = "=" * 78
    thin = "-" * 78
    lines = []

    def w(s=""):
        lines.append(s)

    w(sep)
    w("  RUSTY-GIVENS-CORE — SOLVER FORMULATION COMPARISON REPORT")
    w("  GB Network Case Study")
    w(f"  Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    w(sep)
    w()

    # ── Background and methodology ──
    w("  0. BACKGROUND AND METHODOLOGY")
    w(thin)
    w("    TRUE STATE (reference)")
    w("    The 'true state' is the power flow solution — the voltage magnitude V and")
    w("    angle θ at every bus for a given load/generation condition. It is obtained")
    w("    by running AC power flow (Newton-Raphson) before state estimation. Power flow")
    w("    is NOT part of the SE run; SE only uses measurements and the network model.")
    w("    In this case study, measurements are synthetic: generated from the true state")
    w("    with added noise. Because we know the true state, we can evaluate how well")
    w("    each solver recovers it from noisy measurements.")
    w("    In real operation, the true state is unknown; SE provides the best estimate.")
    w()
    w("    VM MAE (Voltage Magnitude Mean Absolute Error)")
    w("    Average absolute error in voltage magnitude (p.u.) between the estimated")
    w("    state and the true state:  VM MAE = (1/n) × Σ|V_est,i − V_true,i|")
    w("    Lower values indicate higher accuracy. ~6e-4 p.u. ≈ 0.06% error.")
    w()
    w("    VA MAE (Voltage Angle Mean Absolute Error)")
    w("    Average absolute error in voltage angle (degrees) between estimated and")
    w("    true state:  VA MAE = (1/n) × Σ|θ_est,i − θ_true,i|")
    w("    Lower values indicate higher accuracy. ~0.04° is very small for power flow.")
    w()
    w("    FORMULATIONS (6 solver formulations in rusty-givens-core)")
    w("    NormalEquations:  Standard WLS via gain matrix G = HᵀWH, Sparse Cholesky.")
    w("    OrthogonalQR:    QR factorization of weighted Jacobian via Givens rotations.")
    w("    PetersWilkinson: LU of H̃; LᵀL solve, better conditioned than G.")
    w("    EqualityConstrained: Zero injections as explicit Lagrangian constraints.")
    w("    FastDecoupled:   P-θ / Q-V decoupled sub-problems; excludes ammeters.")
    w("    DcEstimation:    Linear DC model, active power only, single solve.")
    w()
    w("    INTERPRETATION")
    w("    • AC formulations (NE, QR, PW, EC) solve the same WLS problem with different")
    w("      linear algebra — they should yield nearly identical accuracy.")
    w("    • DC Estimation uses a linear model (flat voltages, active power only);")
    w("      larger VM/VA MAE is expected — it trades accuracy for speed.")
    w("    • FastDecoupled excludes branch current measurements; a dash (—) indicates")
    w("      numerical issues or that the metric is not applicable.")
    w()

    # ── Network size and structure ──
    w("  1. NETWORK SIZE AND STRUCTURE")
    w(thin)
    w(f"    Case:              {case.get('name', 'GB Network')}")
    w(f"    Description:       {case.get('description', '')[:70]}...")
    w()
    w("    Topology:")
    w(f"      Buses:            {net_summary['n_buses']}")
    w(f"      Branches:         {net_summary['n_branches']}")
    w(f"      Slack buses:      {net_summary['n_slack']}")
    w(f"      PV buses:        {net_summary['n_pv']}")
    w(f"      PQ buses:        {net_summary['n_pq']}")
    w(f"      Base MVA:        {net_summary['base_mva']}")
    w(f"      Slack bus idx:   {net_summary['slack_bus_index']}")
    w()
    w("    Measurement set:")
    w(f"      Voltmeters:       {net_summary['n_voltmeters']}")
    w(f"      Ammeters:        {net_summary['n_ammeters']}")
    w(f"      Wattmeters:      {net_summary['n_wattmeters']}")
    w(f"      Varmeters:       {net_summary['n_varmeters']}")
    w(f"      PMUs:            {net_summary['n_pmus']}")
    w(f"      Total equations: {net_summary['n_equations']}")
    w()

    # ── Solver results summary table ──
    w(sep)
    w("  2. SOLVER RESULTS SUMMARY")
    w(sep)
    w()
    w(f"    {'Formulation':<24} | {'Conv':<5} | {'Iter':<5} | {'SE (s)':<10} | {'VM MAE':<12} | {'VA MAE°':<10}")
    w("    " + thin[:76])
    for r in results:
        conv = "Yes" if r.get("converged", False) else "No"
        iters = str(r.get("iterations", "—"))
        se_t = r.get("se_time_seconds", r.get("se_time", 0))
        vm_mae = r.get("vm_mae")
        va_mae = r.get("va_mae_deg", np.rad2deg(r["va_mae"]) if r.get("va_mae") is not None else None)
        vm_str = f"{vm_mae:.2e}" if vm_mae is not None and np.isfinite(vm_mae) else "—"
        va_str = f"{va_mae:.4f}" if va_mae is not None and np.isfinite(va_mae) else "—"
        w(f"    {r['formulation']:<24} | {conv:<5} | {iters:<5} | {se_t:<10.4f} | {vm_str:<12} | {va_str:<10}")
    w()

    # ── Detailed per-formulation results ──
    true_vm = np.array(case["true_state"]["voltage_magnitude"])
    true_va = np.array(case["true_state"]["voltage_angle"])

    for r in results:
        w(sep)
        w(f"  3.{results.index(r)+1} {r['formulation']}")
        w(sep)
        w(f"    Converged:          {'Yes' if r.get('converged') else 'No'}")
        w(f"    Iterations:          {r.get('iterations', '—')}")
        w(f"    Load time (s):       {r.get('load_time_seconds', 0):.4f}")
        w(f"    Model build (s):     {r.get('model_build_time_seconds', 0):.4f}")
        w(f"    SE solve time (s):   {r.get('se_time_seconds', r.get('se_time', 0)):.4f}")
        w()
        est_vm = r.get("voltage_magnitude") or r.get("est_vm")
        est_va = r.get("voltage_angle") or r.get("est_va")
        if est_vm is not None and est_va is not None and len(est_vm) == len(true_vm) and len(est_va) == len(true_va):
            # Convert None/NaN to np.nan for safe array creation
            def to_finite(x):
                if x is None:
                    return np.nan
                try:
                    return float(x) if np.isfinite(float(x)) else np.nan
                except (TypeError, ValueError):
                    return np.nan

            est_vm = np.array([to_finite(x) for x in est_vm], dtype=float)
            est_va = np.array([to_finite(x) for x in est_va], dtype=float)
            if np.all(np.isfinite(est_vm)) and np.all(np.isfinite(est_va)):
                vm_err = np.abs(est_vm - true_vm)
                va_err_deg = np.rad2deg(np.abs(est_va - true_va))
                w("    State estimation accuracy (vs true power flow state):")
                w(f"      VM MAE (p.u.):      {np.mean(vm_err):.6e}")
                w(f"      VM Max error:       {np.max(vm_err):.6e}")
                w(f"      VM RMSE:            {np.sqrt(np.mean(vm_err**2)):.6e}")
                w(f"      VA MAE (deg):       {np.mean(va_err_deg):.6e}")
                w(f"      VA Max error (deg): {np.max(va_err_deg):.6e}")
                w(f"      VA RMSE (deg):      {np.sqrt(np.mean(va_err_deg**2)):.6e}")
            else:
                w("    (Per-bus estimates contain NaN — numerical issue)")
        else:
            w("    (Per-bus estimates not available)")
        w()

    # ── Performance comparison ──
    w(sep)
    w("  4. COMPUTATIONAL PERFORMANCE")
    w(sep)
    converged_results = [r for r in results if r.get("converged")]
    if converged_results:
        fastest = min(converged_results, key=lambda x: x.get("se_time_seconds", x.get("se_time", float("inf"))))
        slowest = max(converged_results, key=lambda x: x.get("se_time_seconds", x.get("se_time", 0)))
        f_t = fastest.get("se_time_seconds", fastest.get("se_time", 0))
        s_t = slowest.get("se_time_seconds", slowest.get("se_time", 0))
        w(f"    Fastest:  {fastest['formulation']} ({f_t:.4f} s)")
        w(f"    Slowest:  {slowest['formulation']} ({s_t:.4f} s)")
        if f_t > 0:
            w(f"    Speedup:   {slowest['formulation']} is {s_t/f_t:.2f}x slower than {fastest['formulation']}")
    w()

    # ── Failed formulations ──
    failed = [r for r in results if not r.get("converged", True) or r.get("error")]
    if failed:
        w(sep)
        w("  5. FAILURES / NOTES")
        w(sep)
        for r in failed:
            if r.get("error"):
                w(f"    {r['formulation']}: {r['error'][:60]}...")
            elif not r.get("converged"):
                w(f"    {r['formulation']}: Did not converge")
        w()

    w(sep)
    w("  END OF REPORT")
    w(sep)

    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    print("=" * 72)
    print("  Solver Formulation Comparison — GB Network (rusty-givens-core)")
    print("=" * 72)

    if not os.path.exists(JSON_PATH):
        print(f"\nError: {JSON_PATH} not found. Run extract_gb_network.py first.")
        sys.exit(1)

    case = load_case_json()
    net_summary = network_summary(case)
    true_vm = np.array(case["true_state"]["voltage_magnitude"])
    true_va = np.array(case["true_state"]["voltage_angle"])

    print(f"\nNetwork: {net_summary['n_buses']} buses, {net_summary['n_branches']} branches")
    print(f"Measurements: {net_summary['n_equations']} equations")
    print()

    results = []
    for formulation_name, description in FORMULATIONS:
        print(f"  Running {formulation_name}...", end=" ", flush=True)
        res, err = run_formulation(formulation_name)
        if err:
            print(f"FAILED: {err[:50]}...")
            results.append({
                "formulation": formulation_name,
                "converged": False,
                "error": err,
                "se_time_seconds": 0,
                "iterations": None,
                "vm_mae": None,
                "va_mae_deg": None,
            })
            continue
        est_vm = res.get("voltage_magnitude")
        est_va = res.get("voltage_angle")
        # Handle None/NaN in JSON (from Rust f64::NAN serialization)
        if est_vm is not None and est_va is not None and len(est_vm) == len(true_vm) and len(est_va) == len(true_va):
            est_vm = np.array([x if x is not None and not (isinstance(x, float) and (np.isnan(x) or np.isinf(x))) else np.nan for x in est_vm], dtype=float)
            est_va = np.array([x if x is not None and not (isinstance(x, float) and (np.isnan(x) or np.isinf(x))) else np.nan for x in est_va], dtype=float)
            if np.all(np.isfinite(est_vm)) and np.all(np.isfinite(est_va)):
                metrics = error_metrics(est_vm, est_va, true_vm, true_va)
                res.update(metrics)
                res["va_mae_deg"] = np.rad2deg(res["va_mae"]) if res.get("va_mae") is not None else metrics["va_mae_deg"]
            elif res.get("va_mae") is not None and np.isfinite(res["va_mae"]):
                res["va_mae_deg"] = np.rad2deg(res["va_mae"])
        res["formulation"] = formulation_name
        results.append(res)
        conv = "OK" if res.get("converged") else "NO CONV"
        se_t = res.get("se_time_seconds", res.get("se_time", 0))
        print(f"{conv} ({se_t:.3f}s)")

    # Write report
    write_report(results, net_summary, case, REPORT_PATH)
    print(f"\nReport written to {REPORT_PATH}")

    # Summary table to console
    print("\n" + "=" * 72)
    print("  SUMMARY")
    print("=" * 72)
    for r in results:
        c = "✓" if r.get("converged") else "✗"
        t = r.get("se_time_seconds", r.get("se_time", 0))
        vm = r.get("vm_mae")
        va = r.get("va_mae_deg")
        vm_s = f"{vm:.2e}" if vm is not None else "—"
        va_s = f"{va:.4f}°" if va is not None else "—"
        print(f"  {c} {r['formulation']:<22}  {t:.3f}s   VM MAE={vm_s}   VA MAE={va_s}")


if __name__ == "__main__":
    main()
