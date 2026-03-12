#!/usr/bin/env python3
"""
Compare WLS State Estimation results across three implementations:
  1. pandapower  (Python, native)
  2. StateRustimation (Rust, via subprocess)
  3. JuliaGrid   (Julia, via subprocess — optional)

All three use the same GB network and identical measurement data exported
to case_study/gb_network.json by extract_gb_network.py.
"""

import csv
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime

import numpy as np
# Workaround: pandapower SE references APIs removed in numpy 2.x
if not hasattr(np.linalg, "linalg"):
    np.linalg.linalg = np.linalg
if not hasattr(np, "in1d"):
    np.in1d = np.isin
if not hasattr(np, "bool"):
    np.bool = np.bool_
if not hasattr(np, "int"):
    np.int = np.int_
if not hasattr(np, "float"):
    np.float = np.float64
if not hasattr(np, "complex"):
    np.complex = np.complex128

import pandapower as pp
import pandapower.networks as pn

CASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(CASE_DIR)
JSON_PATH = os.path.join(CASE_DIR, "gb_network.json")

SEPARATOR = "=" * 72


# ─────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────

def load_case_json():
    with open(JSON_PATH) as f:
        return json.load(f)


def true_state(case):
    return (
        np.array(case["true_state"]["voltage_magnitude"]),
        np.array(case["true_state"]["voltage_angle"]),
    )


def error_metrics(est_vm, est_va, true_vm, true_va):
    vm_err = np.abs(est_vm - true_vm)
    va_err = np.abs(est_va - true_va)
    return {
        "vm_mae": float(np.mean(vm_err)),
        "vm_max": float(np.max(vm_err)),
        "va_mae_rad": float(np.mean(va_err)),
        "va_max_rad": float(np.max(va_err)),
        "va_mae_deg": float(np.rad2deg(np.mean(va_err))),
        "va_max_deg": float(np.rad2deg(np.max(va_err))),
    }


# ─────────────────────────────────────────────────────────────────────────
#  Output helpers: detailed .txt report and per-bus .csv
# ─────────────────────────────────────────────────────────────────────────

def write_detailed_report(results, true_vm, true_va, case, path):
    """Write a detailed human-readable .txt comparison report."""
    n_buses = len(true_vm)
    n_branches = len(case.get("branches", []))
    n_meas = sum(len(v) for v in case.get("measurements", {}).values())
    sep = "=" * 78
    thin = "-" * 78

    lines = []
    def w(s=""):
        lines.append(s)

    w(sep)
    w("  WLS STATE ESTIMATION — DETAILED COMPARISON REPORT")
    w(f"  Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    w(sep)
    w()
    w("  NETWORK")
    w(thin)
    w(f"    Buses:          {n_buses}")
    w(f"    Branches:       {n_branches}")
    w(f"    Measurements:   {n_meas}")
    w()

    for res in results:
        solver = res["solver"]
        w(sep)
        w(f"  SOLVER: {solver}")
        w(sep)
        w(f"    Converged:          {'Yes' if res.get('converged') else 'No'}")
        w(f"    Iterations:         {res.get('iterations', '—')}")
        w(f"    SE Time (s):        {res.get('se_time', 0):.4f}")
        w()

        est_vm = res.get("est_vm")
        est_va = res.get("est_va")
        if est_vm is not None and est_va is not None:
            vm_err = np.abs(est_vm - true_vm)
            va_err = np.abs(est_va - true_va)
            va_err_deg = np.rad2deg(va_err)

            w("    VOLTAGE MAGNITUDE (p.u.)")
            w(thin)
            w(f"      MAE:              {np.mean(vm_err):.6e}")
            w(f"      Max error:        {np.max(vm_err):.6e}")
            w(f"      RMSE:             {np.sqrt(np.mean(vm_err**2)):.6e}")
            w(f"      Std dev of error: {np.std(vm_err):.6e}")
            w(f"      Min estimated:    {np.min(est_vm):.6f}")
            w(f"      Max estimated:    {np.max(est_vm):.6f}")
            w(f"      Mean estimated:   {np.mean(est_vm):.6f}")
            idx_max_vm = int(np.argmax(vm_err))
            w(f"      Worst bus (idx):  {idx_max_vm}  "
              f"(est={est_vm[idx_max_vm]:.6f}, true={true_vm[idx_max_vm]:.6f}, "
              f"err={vm_err[idx_max_vm]:.6e})")
            w()

            w("    VOLTAGE ANGLE (degrees)")
            w(thin)
            w(f"      MAE:              {np.mean(va_err_deg):.6e}")
            w(f"      Max error:        {np.max(va_err_deg):.6e}")
            w(f"      RMSE:             {np.sqrt(np.mean(va_err_deg**2)):.6e}")
            w(f"      Std dev of error: {np.std(va_err_deg):.6e}")
            w(f"      Min estimated:    {np.rad2deg(np.min(est_va)):.4f}")
            w(f"      Max estimated:    {np.rad2deg(np.max(est_va)):.4f}")
            w(f"      Mean estimated:   {np.rad2deg(np.mean(est_va)):.4f}")
            idx_max_va = int(np.argmax(va_err_deg))
            w(f"      Worst bus (idx):  {idx_max_va}  "
              f"(est={np.rad2deg(est_va[idx_max_va]):.4f}°, "
              f"true={np.rad2deg(true_va[idx_max_va]):.4f}°, "
              f"err={va_err_deg[idx_max_va]:.4f}°)")
            w()

            pct_within = {
                "0.1%": np.mean(vm_err < 0.001) * 100,
                "0.5%": np.mean(vm_err < 0.005) * 100,
                "1.0%": np.mean(vm_err < 0.010) * 100,
            }
            w("    VM ERROR DISTRIBUTION")
            w(thin)
            for thr, pct in pct_within.items():
                w(f"      Buses within {thr} p.u.: {pct:6.2f}%")
            w()
        else:
            w("    (per-bus estimates not available)")
            w()

    # Side-by-side summary if both pandapower and Rust are present
    solvers_with_data = [r for r in results if r.get("est_vm") is not None]
    if len(solvers_with_data) >= 2:
        w(sep)
        w("  HEAD-TO-HEAD COMPARISON")
        w(sep)
        r1, r2 = solvers_with_data[0], solvers_with_data[1]
        w(f"    {'Metric':<28s} | {r1['solver']:<24s} | {r2['solver']:<24s}")
        w("    " + "-" * 28 + "-+-" + "-" * 24 + "-+-" + "-" * 24)

        def row(label, v1, v2, fmt_str="{:.6e}"):
            s1 = fmt_str.format(v1) if v1 is not None else "—"
            s2 = fmt_str.format(v2) if v2 is not None else "—"
            w(f"    {label:<28s} | {s1:<24s} | {s2:<24s}")

        row("SE Time (s)", r1.get("se_time"), r2.get("se_time"), "{:.4f}")
        row("Iterations", r1.get("iterations"), r2.get("iterations"), "{}")
        row("VM MAE (p.u.)", r1.get("vm_mae"), r2.get("vm_mae"))
        row("VM Max Error (p.u.)", r1.get("vm_max"), r2.get("vm_max"))
        row("VA MAE (deg)", r1.get("va_mae_deg"), r2.get("va_mae_deg"))
        row("VA Max Error (deg)", r1.get("va_max_deg"), r2.get("va_max_deg"))
        row("VM RMSE (p.u.)",
            float(np.sqrt(np.mean((r1["est_vm"] - true_vm)**2))),
            float(np.sqrt(np.mean((r2["est_vm"] - true_vm)**2))))
        row("VA RMSE (deg)",
            float(np.rad2deg(np.sqrt(np.mean((r1["est_va"] - true_va)**2)))),
            float(np.rad2deg(np.sqrt(np.mean((r2["est_va"] - true_va)**2)))))

        if r1.get("se_time") and r2.get("se_time"):
            speedup = r1["se_time"] / r2["se_time"]
            if speedup >= 1:
                w(f"\n    => {r2['solver']} is {speedup:.2f}x faster than {r1['solver']}")
            else:
                w(f"\n    => {r1['solver']} is {1/speedup:.2f}x faster than {r2['solver']}")

        # Per-bus difference between the two solvers
        vm_diff = np.abs(r1["est_vm"] - r2["est_vm"])
        va_diff = np.rad2deg(np.abs(r1["est_va"] - r2["est_va"]))
        w()
        w("    INTER-SOLVER AGREEMENT (pandapower vs Rust)")
        w("    " + thin)
        w(f"      VM difference MAE:     {np.mean(vm_diff):.6e} p.u.")
        w(f"      VM difference Max:     {np.max(vm_diff):.6e} p.u.")
        w(f"      VA difference MAE:     {np.mean(va_diff):.6e} deg")
        w(f"      VA difference Max:     {np.max(va_diff):.6e} deg")
        w()

    w(sep)
    w("  END OF REPORT")
    w(sep)

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def write_bus_csv(results, true_vm, true_va, case, path):
    """Write a CSV with per-bus voltage magnitude and angle from each solver."""
    n_buses = len(true_vm)
    bus_labels = list(range(n_buses))
    if "buses" in case and len(case["buses"]) == n_buses:
        bus_labels = [b.get("label", i) for i, b in enumerate(case["buses"])]

    solvers_with_data = [r for r in results if r.get("est_vm") is not None]

    header = ["bus_index", "bus_label",
              "true_vm_pu", "true_va_deg"]
    for r in solvers_with_data:
        tag = r["solver"].replace(" ", "_").replace("(", "").replace(")", "")
        header.extend([
            f"{tag}_vm_pu",
            f"{tag}_va_deg",
            f"{tag}_vm_err_pu",
            f"{tag}_va_err_deg",
        ])

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(n_buses):
            row = [
                i,
                bus_labels[i],
                f"{true_vm[i]:.8f}",
                f"{np.rad2deg(true_va[i]):.8f}",
            ]
            for r in solvers_with_data:
                est_vm_i = r["est_vm"][i]
                est_va_i = r["est_va"][i]
                row.extend([
                    f"{est_vm_i:.8f}",
                    f"{np.rad2deg(est_va_i):.8f}",
                    f"{abs(est_vm_i - true_vm[i]):.8e}",
                    f"{abs(np.rad2deg(est_va_i) - np.rad2deg(true_va[i])):.8e}",
                ])
            writer.writerow(row)


# ─────────────────────────────────────────────────────────────────────────
#  1. pandapower WLS State Estimation
# ─────────────────────────────────────────────────────────────────────────

def run_pandapower_se(case):
    """Run pandapower's built-in WLS estimator on the GB network.

    Measurements are taken from the JSON (injection convention, p.u.) and
    converted to pandapower's expected units (MW/MVAr/kA, load convention
    for bus injections).
    """
    print("\n[pandapower] Loading GB network...")
    t0 = time.time()
    net = pn.GBnetwork()
    pp.runpp(net, algorithm="nr", max_iteration=50, tolerance_mva=1e-6)
    load_time = time.time() - t0
    print(f"  Network loaded + PF in {load_time:.2f}s")

    base_mva = float(net.sn_mva)
    meas = case["measurements"]
    n_lines_total = int(net.line["in_service"].sum())

    # ── Voltmeters (p.u.) ──
    for vm in meas["voltmeters"]:
        pp.create_measurement(net, "v", "bus",
                              value=vm["magnitude"],
                              std_dev=np.sqrt(vm["variance"]),
                              element=vm["bus"])

    # ── Bus injection wattmeters / varmeters ──
    # JSON is in injection convention (p.u.); pandapower uses load convention (MW/MVAr).
    # Convert: negate (injection → load) then scale by base_mva.
    for wm in meas["wattmeters"]:
        loc = wm["location"]
        std = np.sqrt(wm["variance"]) * base_mva
        if "Bus" in loc:
            val_mw = -wm["active"] * base_mva
            pp.create_measurement(net, "p", "bus",
                                  value=val_mw, std_dev=std,
                                  element=loc["Bus"])
        else:
            br = loc["Branch"]
            br_label = br["branch"]
            val_mw = wm["active"] * base_mva
            side = br["end"].lower()
            if br_label <= n_lines_total:
                pp.create_measurement(net, "p", "line",
                                      value=val_mw, std_dev=std,
                                      element=br_label - 1, side=side)
            else:
                trafo_idx = br_label - 1 - n_lines_total
                pp_side = "hv" if side == "from" else "lv"
                pp.create_measurement(net, "p", "trafo",
                                      value=val_mw, std_dev=std,
                                      element=trafo_idx, side=pp_side)

    for vm in meas["varmeters"]:
        loc = vm["location"]
        std = np.sqrt(vm["variance"]) * base_mva
        if "Bus" in loc:
            val_mvar = -vm["reactive"] * base_mva
            pp.create_measurement(net, "q", "bus",
                                  value=val_mvar, std_dev=std,
                                  element=loc["Bus"])
        else:
            br = loc["Branch"]
            br_label = br["branch"]
            val_mvar = vm["reactive"] * base_mva
            side = br["end"].lower()
            if br_label <= n_lines_total:
                pp.create_measurement(net, "q", "line",
                                      value=val_mvar, std_dev=std,
                                      element=br_label - 1, side=side)
            else:
                trafo_idx = br_label - 1 - n_lines_total
                pp_side = "hv" if side == "from" else "lv"
                pp.create_measurement(net, "q", "trafo",
                                      value=val_mvar, std_dev=std,
                                      element=trafo_idx, side=pp_side)

    # ── Ammeters (kA) — only on lines ──
    for am in meas["ammeters"]:
        br_label = am["branch"]
        if br_label > n_lines_total:
            continue
        from_bus = case["branches"][br_label - 1]["from_bus"]
        vn_kv = float(net.bus.at[from_bus, "vn_kv"])
        i_base_ka = base_mva / (np.sqrt(3) * vn_kv)
        val_ka = am["magnitude"] * i_base_ka
        std_ka = np.sqrt(am["variance"]) * i_base_ka
        side = am["end"].lower()
        pp.create_measurement(net, "i", "line",
                              value=val_ka, std_dev=std_ka,
                              element=br_label - 1, side=side)

    # ── PMUs: voltage magnitude + angle at buses ──
    for pmu in meas["pmus"]:
        loc = pmu["location"]
        if "Bus" in loc:
            bus = loc["Bus"]
            pp.create_measurement(net, "v", "bus",
                                  value=pmu["magnitude"],
                                  std_dev=np.sqrt(pmu["variance_magnitude"]),
                                  element=bus)
            pp.create_measurement(net, "va", "bus",
                                  value=np.rad2deg(pmu["angle"]),
                                  std_dev=np.rad2deg(np.sqrt(pmu["variance_angle"])),
                                  element=bus)

    print(f"  Measurements added: {len(net.measurement)}")

    # ── Run SE ──
    print("  Running WLS state estimation...")
    t0 = time.time()
    try:
        result = pp.estimation.estimate(
            net,
            algorithm="wls",
            init="flat",
            tolerance=1e-4,
            maximum_iterations=50,
            zero_injection=None,
        )
        se_time = time.time() - t0
    except Exception as e:
        se_time = time.time() - t0
        print(f"  ERROR: {e}")
        return None

    # pandapower returns either a bool or a dict depending on version
    if isinstance(result, dict):
        success = result.get("success", False)
        iterations = result.get("num_iterations", None)
    else:
        success = bool(result)
        iterations = None

    print(f"  Converged: {success}  |  Iterations: {iterations}  |  SE time: {se_time:.3f}s")

    if not success or "vm_pu" not in net.res_bus_est.columns:
        print("  pandapower SE did not produce results.")
        return None

    est_vm = net.res_bus_est["vm_pu"].values
    est_va = np.deg2rad(net.res_bus_est["va_degree"].values)

    return {
        "solver": "pandapower",
        "se_time": se_time,
        "converged": True,
        "iterations": iterations,
        "est_vm": est_vm,
        "est_va": est_va,
    }


# ─────────────────────────────────────────────────────────────────────────
#  2. Rust (StateRustimation)
# ─────────────────────────────────────────────────────────────────────────

def run_rust_se():
    """Run the Rust gb_case_study binary and read its JSON output."""
    print("\n[Rust] Running gb_case_study binary...")

    cargo = shutil.which("cargo")
    if cargo is None:
        cargo_env = os.path.expanduser("~/.cargo/env")
        if os.path.exists(cargo_env):
            shell_cmd = f'source "{cargo_env}" && cargo run --release --bin gb_case_study'
        else:
            print("  cargo not found — skipping Rust.")
            return None
    else:
        shell_cmd = "cargo run --release --bin gb_case_study"

    t0 = time.time()
    proc = subprocess.run(
        shell_cmd, shell=True, capture_output=True, text=True,
        cwd=PROJECT_DIR, timeout=300,
    )
    wall_time = time.time() - t0

    if proc.returncode != 0:
        print(f"  Rust binary failed (rc={proc.returncode}):")
        print(proc.stderr[-500:] if proc.stderr else "(no stderr)")
        return None

    print(proc.stderr, end="")
    print(proc.stdout)

    results_path = os.path.join(CASE_DIR, "results_rust.json")
    if not os.path.exists(results_path):
        print("  results_rust.json not found.")
        return None

    with open(results_path) as f:
        res = json.load(f)

    rust_result = {
        "solver": "Rust (StateRustimation)",
        "se_time": res.get("se_time_seconds", wall_time),
        "converged": res.get("converged", False),
        "iterations": res.get("iterations", None),
        "vm_mae": res.get("vm_mae"),
        "va_mae": res.get("va_mae"),
        "vm_max": res.get("vm_max_error"),
        "va_max": res.get("va_max_error"),
    }
    if "voltage_magnitude" in res and "voltage_angle" in res:
        rust_result["est_vm"] = np.array(res["voltage_magnitude"])
        rust_result["est_va"] = np.array(res["voltage_angle"])
    return rust_result


# ─────────────────────────────────────────────────────────────────────────
#  3. JuliaGrid (optional)
# ─────────────────────────────────────────────────────────────────────────

def run_julia_se():
    """Run the JuliaGrid script and read its JSON output."""
    julia = shutil.which("julia")
    if julia is None:
        print("\n[Julia] julia not found in PATH — skipping.")
        return None

    script = os.path.join(CASE_DIR, "run_julia_se.jl")
    if not os.path.exists(script):
        print("\n[Julia] run_julia_se.jl not found — skipping.")
        return None

    print("\n[Julia] Running JuliaGrid SE (this may take a while on first run)...")

    t0 = time.time()
    proc = subprocess.run(
        [julia, "--project=.", script],
        capture_output=True, text=True,
        cwd=PROJECT_DIR, timeout=600,
    )
    wall_time = time.time() - t0

    if proc.returncode != 0:
        print(f"  Julia script failed (rc={proc.returncode}):")
        print(proc.stderr[-500:] if proc.stderr else "(no stderr)")
        return None

    print(proc.stdout)

    results_path = os.path.join(CASE_DIR, "results_julia.json")
    if not os.path.exists(results_path):
        print("  results_julia.json not found.")
        return None

    with open(results_path) as f:
        res = json.load(f)

    return {
        "solver": "JuliaGrid",
        "se_time": res.get("se_time_seconds", wall_time),
        "converged": True,
        "vm_mae": res.get("vm_mae"),
        "va_mae": res.get("va_mae"),
        "est_vm": np.array(res["voltage_magnitude"]) if "voltage_magnitude" in res else None,
        "est_va": np.array(res["voltage_angle"]) if "voltage_angle" in res else None,
    }


# ─────────────────────────────────────────────────────────────────────────
#  Main comparison
# ─────────────────────────────────────────────────────────────────────────

def main():
    print(SEPARATOR)
    print("  WLS State Estimation Comparison — GB Network")
    print(f"  Network: 2224 buses, 3207 branches, ~10k measurements")
    print(SEPARATOR)

    case = load_case_json()
    tv, ta = true_state(case)

    results = []

    # ── pandapower ──
    pp_res = run_pandapower_se(case)
    if pp_res is not None and pp_res["est_vm"] is not None:
        m = error_metrics(pp_res["est_vm"], pp_res["est_va"], tv, ta)
        pp_res.update(m)
        results.append(pp_res)

    # ── Rust ──
    rust_res = run_rust_se()
    if rust_res is not None:
        if rust_res.get("est_vm") is not None:
            m = error_metrics(rust_res["est_vm"], rust_res["est_va"], tv, ta)
            rust_res.update(m)
        results.append(rust_res)

    # ── Julia ──
    julia_res = run_julia_se()
    if julia_res is not None:
        if julia_res.get("est_vm") is not None:
            m = error_metrics(julia_res["est_vm"], julia_res["est_va"], tv, ta)
            julia_res.update(m)
        results.append(julia_res)

    # ── Comparison table ──
    if not results:
        print("\nNo results to compare.")
        return

    print(f"\n{SEPARATOR}")
    print("  COMPARISON RESULTS")
    print(SEPARATOR)

    headers = ["Metric", *[r["solver"] for r in results]]
    rows = []

    def fmt(val, unit=""):
        if val is None:
            return "—"
        if isinstance(val, bool):
            return "Yes" if val else "No"
        if isinstance(val, int):
            return str(val)
        if isinstance(val, float):
            if abs(val) < 0.01 and val != 0:
                return f"{val:.2e}{unit}"
            return f"{val:.4f}{unit}"
        return str(val)

    rows.append(["Converged"] + [fmt(r.get("converged")) for r in results])
    rows.append(["SE Time (s)"] + [fmt(r.get("se_time")) for r in results])
    rows.append(["Iterations"] + [fmt(r.get("iterations")) for r in results])
    rows.append(["VM MAE (p.u.)"] + [fmt(r.get("vm_mae")) for r in results])
    rows.append(["VM Max Err (p.u.)"] + [fmt(r.get("vm_max")) for r in results])
    rows.append(["VA MAE (deg)"] + [fmt(r.get("va_mae_deg") or
                  (np.rad2deg(r["va_mae"]) if r.get("va_mae") else None)) for r in results])
    rows.append(["VA Max Err (deg)"] + [fmt(r.get("va_max_deg") or
                  (np.rad2deg(r["va_max"]) if r.get("va_max") else None)) for r in results])

    col_widths = [max(len(str(row[i])) for row in [headers] + rows)
                  for i in range(len(headers))]
    col_widths[0] = max(col_widths[0], 20)
    for i in range(1, len(col_widths)):
        col_widths[i] = max(col_widths[i], 22)

    def print_row(cells, sep=" | "):
        parts = [str(c).ljust(w) for c, w in zip(cells, col_widths)]
        print("  " + sep.join(parts))

    print()
    print_row(headers)
    print("  " + "-+-".join("-" * w for w in col_widths))
    for row in rows:
        print_row(row)

    print(f"\n{SEPARATOR}")

    # ── Save comparison JSON ──
    out = []
    for r in results:
        entry = {k: v for k, v in r.items()
                 if k not in ("est_vm", "est_va") and not isinstance(v, np.ndarray)}
        out.append(entry)

    comp_path = os.path.join(CASE_DIR, "comparison_results.json")
    with open(comp_path, "w") as f:
        json.dump(out, f, indent=2, default=str)
    print(f"  Comparison saved to {comp_path}")

    # ── Detailed .txt report ──
    txt_path = os.path.join(CASE_DIR, "comparison_report.txt")
    write_detailed_report(results, tv, ta, case, txt_path)
    print(f"  Detailed report saved to {txt_path}")

    # ── Per-bus .csv ──
    csv_path = os.path.join(CASE_DIR, "bus_results.csv")
    write_bus_csv(results, tv, ta, case, csv_path)
    print(f"  Per-bus CSV saved to {csv_path}")


if __name__ == "__main__":
    main()
