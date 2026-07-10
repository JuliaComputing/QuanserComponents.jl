# Baseline capture: nominal closed-loop behavior with the controller settings
# from test/runtests.jl. Serves as the behavior-neutrality regression that is
# re-run after every model change: with all new features at their defaults the
# catch time and terminal state must match this file's saved values.

include("common.jl")

model, ssys = build_system()
sol = simulate(ssys, nominal_overrides(ssys); tf = 10.0)
m = metrics(sol, ssys)

@assert m.success "Baseline swingup failed - something is wrong before any changes"
@assert m.max_abs_elbow < 4pi "Elbow angle exceeds quantizer range +-4pi"

df = DataFrame([metrics_row(m; case = "nominal")])
mkpath(resultpath(""))
outfile = resultpath("baseline.csv")

if isfile(outfile)
    ref = CSV.read(outfile, DataFrame)
    dct = abs(m.catch_time - ref.catch_time[1])
    dang = abs(m.final_angle_err - ref.final_angle_err[1])
    ok = dct < 0.05 && dang < 0.02
    @printf("Regression vs saved baseline: Δcatch_time = %.4g s, Δfinal_angle_err = %.4g rad → %s\n",
        dct, dang, ok ? "PASS" : "FAIL")
    ok || error("Behavior-neutrality regression FAILED (saved baseline differs)")
else
    CSV.write(outfile, df)
    println("Saved new baseline to $outfile")
end

@printf("catch_time      = %.3f s\n", m.catch_time)
@printf("first_entry     = %.3f s\n", m.first_entry)
@printf("n_entries       = %d\n", m.n_entries)
@printf("arrival_speed   = %.3f rad/s\n", m.arrival_speed)
@printf("final_angle_err = %.4g rad\n", m.final_angle_err)
@printf("final_speed     = %.4g rad/s\n", m.final_speed)
@printf("max_abs_elbow   = %.3f rad (quantizer range ±%.3f)\n", m.max_abs_elbow, 4pi)
