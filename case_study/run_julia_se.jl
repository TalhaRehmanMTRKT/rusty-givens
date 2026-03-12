#=
  JuliaGrid AC State Estimation on the GB Network
  ──────────────────────────────────────────────────
  This script loads the GB network JSON (exported by extract_gb_network.py),
  builds a PowerSystem + Measurement set in JuliaGrid, and solves the WLS
  state estimation using the Gauss-Newton method.

  Prerequisites:
    using Pkg
    Pkg.add("JuliaGrid")
    Pkg.add("JSON")
=#

using JuliaGrid
using JSON
using Printf
using Statistics

println("=" ^ 60)
println("JuliaGrid AC State Estimation — GB Network")
println("=" ^ 60)

# ── 1. Load the JSON case data ──
println("\n[1] Loading GB network data from JSON...")
data = JSON.parsefile(joinpath(@__DIR__, "gb_network.json"))

n_buses = data["n_buses"]
n_branches = data["n_branches"]
slack_idx = data["slack_bus_index"]
println("    Buses: $n_buses, Branches: $n_branches")

# ── 2. Build the PowerSystem ──
println("\n[2] Building JuliaGrid PowerSystem...")
system = powerSystem()

bus_labels = Int[]
bus_types = Dict{Int, Int}()

for (i, bus) in enumerate(data["buses"])
    label = bus["label"]
    btype = bus["bus_type"]  # 1=PQ, 2=PV, 3=Slack
    push!(bus_labels, label)
    bus_types[label] = btype

    addBus!(system;
        label = label,
        type = btype,
        active = bus["p_demand_pu"],
        reactive = bus["q_demand_pu"],
        conductance = bus["g_shunt_pu"],
        susceptance = bus["b_shunt_pu"],
    )
end

for br in data["branches"]
    if !br["status"]
        continue
    end
    addBranch!(system;
        label = br["label"],
        from = br["from_bus"],
        to = br["to_bus"],
        resistance = br["resistance"],
        reactance = br["reactance"],
        susceptance = br["susceptance"],
        conductance = br["conductance"],
        turnsRatio = br["tap_ratio"],
        shiftAngle = br["shift_angle"],
    )
end

# ── 3. Build AC model ──
println("\n[3] Building AC model...")
t_model = @elapsed acModel!(system)
@printf("    AC model built in %.3f s\n", t_model)

# ── 4. Add measurements ──
println("\n[4] Adding measurements...")
monitoring = measurement(system)

meas = data["measurements"]
n_vm = length(meas["voltmeters"])
n_am = length(meas["ammeters"])
n_wm = length(meas["wattmeters"])
n_var = length(meas["varmeters"])
n_pmu = length(meas["pmus"])

for vm in meas["voltmeters"]
    addVoltmeter!(monitoring;
        bus = vm["bus"],
        magnitude = vm["magnitude"],
        variance = vm["variance"],
    )
end

for am in meas["ammeters"]
    if am["end"] == "From"
        addAmmeter!(monitoring;
            from = am["branch"],
            magnitude = am["magnitude"],
            variance = am["variance"],
        )
    else
        addAmmeter!(monitoring;
            to = am["branch"],
            magnitude = am["magnitude"],
            variance = am["variance"],
        )
    end
end

for wm in meas["wattmeters"]
    loc = wm["location"]
    if haskey(loc, "Bus")
        addWattmeter!(monitoring;
            bus = loc["Bus"],
            active = wm["active"],
            variance = wm["variance"],
        )
    else
        br = loc["Branch"]
        if br["end"] == "From"
            addWattmeter!(monitoring;
                from = br["branch"],
                active = wm["active"],
                variance = wm["variance"],
            )
        else
            addWattmeter!(monitoring;
                to = br["branch"],
                active = wm["active"],
                variance = wm["variance"],
            )
        end
    end
end

for vm in meas["varmeters"]
    loc = vm["location"]
    if haskey(loc, "Bus")
        addVarmeter!(monitoring;
            bus = loc["Bus"],
            reactive = vm["reactive"],
            variance = vm["variance"],
        )
    else
        br = loc["Branch"]
        if br["end"] == "From"
            addVarmeter!(monitoring;
                from = br["branch"],
                reactive = vm["reactive"],
                variance = vm["variance"],
            )
        else
            addVarmeter!(monitoring;
                to = br["branch"],
                reactive = vm["reactive"],
                variance = vm["variance"],
            )
        end
    end
end

for pmu in meas["pmus"]
    loc = pmu["location"]
    is_polar = pmu["coordinate"] == "Polar"
    if haskey(loc, "Bus")
        addPmu!(monitoring;
            bus = loc["Bus"],
            magnitude = pmu["magnitude"],
            angle = pmu["angle"],
            varianceMagnitude = pmu["variance_magnitude"],
            varianceAngle = pmu["variance_angle"],
            polar = is_polar,
            correlated = pmu["correlated"],
        )
    else
        br = loc["Branch"]
        if br["end"] == "From"
            addPmu!(monitoring;
                from = br["branch"],
                magnitude = pmu["magnitude"],
                angle = pmu["angle"],
                varianceMagnitude = pmu["variance_magnitude"],
                varianceAngle = pmu["variance_angle"],
                polar = is_polar,
                correlated = pmu["correlated"],
            )
        else
            addPmu!(monitoring;
                to = br["branch"],
                magnitude = pmu["magnitude"],
                angle = pmu["angle"],
                varianceMagnitude = pmu["variance_magnitude"],
                varianceAngle = pmu["variance_angle"],
                polar = is_polar,
                correlated = pmu["correlated"],
            )
        end
    end
end

println("    Voltmeters: $n_vm, Ammeters: $n_am")
println("    Wattmeters: $n_wm, Varmeters: $n_var, PMUs: $n_pmu")
total_eq = n_vm + n_am + n_wm + n_var + 2 * n_pmu
println("    Total scalar equations: $total_eq")

# ── 5. Run Gauss-Newton WLS State Estimation ──
println("\n[5] Running Gauss-Newton WLS State Estimation...")
analysis = gaussNewton(monitoring)

t_se = @elapsed begin
    for iteration = 0:50
        stopping = increment!(analysis)
        if stopping < 1e-8
            @printf("    Converged at iteration %d (max|Δx| = %.2e)\n", iteration, stopping)
            break
        end
        solve!(analysis)
        if iteration == 50
            @printf("    Did NOT converge after 50 iterations (max|Δx| = %.2e)\n", stopping)
        end
    end
end
@printf("    SE solve time: %.3f s\n", t_se)

# ── 6. Compare to true state ──
println("\n[6] Comparing estimated state to true power flow solution...")
true_vm = Float64.(data["true_state"]["voltage_magnitude"])
true_va = Float64.(data["true_state"]["voltage_angle"])

est_vm = analysis.voltage.magnitude
est_va = analysis.voltage.angle

vm_error = abs.(est_vm .- true_vm)
va_error = abs.(est_va .- true_va)

@printf("    Voltage magnitude MAE: %.6f p.u.\n", mean(vm_error))
@printf("    Voltage magnitude Max: %.6f p.u.\n", maximum(vm_error))
@printf("    Voltage angle MAE:     %.6f rad (%.4f deg)\n", mean(va_error), rad2deg(mean(va_error)))
@printf("    Voltage angle Max:     %.6f rad (%.4f deg)\n", maximum(va_error), rad2deg(maximum(va_error)))

# ── 7. Write results ──
results = Dict(
    "solver" => "JuliaGrid",
    "method" => "Gauss-Newton (LU)",
    "se_time_seconds" => t_se,
    "model_build_time_seconds" => t_model,
    "voltage_magnitude" => collect(est_vm),
    "voltage_angle" => collect(est_va),
    "vm_mae" => mean(vm_error),
    "va_mae" => mean(va_error),
)

open(joinpath(@__DIR__, "results_julia.json"), "w") do f
    JSON.print(f, results, 2)
end

println("\n    Results saved to case_study/results_julia.json")
println("=" ^ 60)
