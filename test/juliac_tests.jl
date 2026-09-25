# The JuliaC target: a program compiled to Julia source, written as an application package and,
# where JuliaC is installed, built into a trimmed binary and run without a device
# (`--dry-run`). The arm64 build for a Raspberry Pi is exercised only where the root file
# system of deploy/arm64/setup.sh exists, since it takes tens of minutes under emulation.

import QuanserComponents as QC
using Test
# This file is normally included from runtests.jl; naming what it uses keeps it runnable on
# its own.
using DelimitedFiles: readdlm

# Parse emitted source the way the application's precompilation will, failing on any error.
function parses(code)
    ex = Meta.parseall(code)
    bad = false
    walk(e) = e isa Expr && (e.head in (:error, :incomplete) ? (bad = true) : foreach(walk, e.args))
    walk(ex)
    return !bad
end

@testset "JuliaC" begin
    Ts = 0.005

    @testset "operator table matches the operators" begin
        # Every entry of `OPERATOR_FFI` has to be the `ccall` the operator makes on the other
        # targets: the same C symbol and the same number of arguments.
        for (op, (sym, lib, nargs)) in QC.OPERATOR_FFI
            f = getfield(QC, op)
            ms = collect(methods(f))
            @test any(m -> m.nargs - 1 == nargs, ms)
            ci = code_lowered(f, NTuple{nargs, Float64})
            @test occursin(":$sym", string(only(ci)))
            @test startswith(lib, "libqube_")
        end
        # ... and every operator the components can call has an entry.
        for op in (:hw_measure, :hw_shoulder, :hw_elbow, :hw_write, :hw_time, :hw_dt,
                   :hw_exec, :hw_count_shoulder, :hw_count_elbow, :hw_realtime_wait,
                   :log_row, :traj_value)
            @test haskey(QC.OPERATOR_FFI, op)
        end
    end

    @testset "programs that cannot be compiled to source" begin
        # The MPC programs call into solver state, and one of them has two clocks.
        @test_throws ArgumentError QC.compile_program_source(QC.FurutaMPCHardware; Ts = 0.01)
        @test_throws ArgumentError QC.compile_program_source(QC.FurutaMPCMultirateHardware)
    end

    src = QC.compile_program_source(QC.FurutaHardware; Ts, log_file = "swingup_log.csv")
    @test src.divisors == (1,) && src.Ts == Ts
    @test issubset([:hw_measure, :hw_write, :log_row], src.operators)
    @test QC.juliac_app_name(src) == "QubeController"

    @testset "application package" begin
        dir = mktempdir()
        res = QC.export_program_juliac(src, dir; Tf = 1.0, gains = (; umax = 7.5))
        app = res.app_dir
        @test res.app_name == "QubeController" && app == joinpath(dir, "QubeController")
        for f in ("Project.toml", "LocalPreferences.toml", "src/controller.jl",
                  "src/hardware_ffi.jl", "src/QubeController.jl", "csrc/qube_hw.c",
                  "csrc/qube_log.c", "csrc/libs.mk", "vendor/SynchJulia/Project.toml")
            @test isfile(joinpath(app, f))
        end
        @test "QubeController/src/controller.jl" in res.files
        @test !any(startswith("QubeController/vendor"), res.files)

        controller = read(joinpath(app, "src", "controller.jl"), String)
        ffi = read(joinpath(app, "src", "hardware_ffi.jl"), String)
        main = read(joinpath(app, "src", "QubeController.jl"), String)
        @test all(parses, (controller, ffi, main))
        # The node is emitted unexpanded, and nothing refers to this package: the application
        # defines the operators itself.
        @test occursin("SynchJulia.@node function top(", controller)
        @test !occursin(r"QuanserComponents\.\w+\(", controller)
        @test !occursin("top_exec", controller)
        # `AutoPars(; ...)` is defined once: precompilation rejects an overwritten method.
        @test !occursin(r"@kwdef mutable struct AutoPars", controller)
        @test occursin("function AutoPars(;", controller)
        for op in src.operators
            @test occursin("\n$op(", ffi) && occursin(string(QC.OPERATOR_FFI[op][1]), ffi)
        end
        # The binary finds the libraries through the bundle, not through a path on this machine.
        @test !occursin(dir, ffi) && !occursin(pkgdir(QC), ffi)
        @test occursin("umax = 7.5", main)
        @test occursin("const EXE = SynchExecutable(top, (Bool, TuningGains, AutoPars))", main)
        @test occursin("compilation_enabled!(false)", main)
        @test occursin("const LOG_FILE = \"swingup_log.csv\"", main)
        @test occursin("dynamic_execution = false",
                       read(joinpath(app, "LocalPreferences.toml"), String))
        project = read(joinpath(app, "Project.toml"), String)
        @test occursin("SynchJulia = {path = \"vendor/SynchJulia\"}", project)
        @test !occursin("d44921f8", project)      # QuanserComponents' UUID
        # An unknown tunable is refused, as for every other target.
        @test_throws ArgumentError QC.export_program_juliac(src, dir; Tf = 1.0,
                                                            gains = (; nonsense = 1.0))
    end

    @testset "a replayed trajectory travels with the program" begin
        trajfile = joinpath(pkgdir(QC), "input_design.csv")
        isrc = QC.compile_program_source(QC.FurutaIdentification; Ts, traj_file = trajfile,
                                         log_file = "replay.csv")
        @test :traj_value in isrc.operators
        dir = mktempdir()
        res = QC.export_program_juliac(isrc, dir; Tf = 1.0)
        @test res.app_name == "QubeIdentification"
        @test isfile(joinpath(dir, basename(trajfile)))
        main = read(joinpath(res.app_dir, "src", "QubeIdentification.jl"), String)
        @test occursin("qube_traj_open(\"$(basename(trajfile))\", Cint(1))", main)
    end

    @testset "which target a JuliaC run goes to" begin
        T(; kw...) = QC.run_target(; kw...)
        pi = "fredrikb@192.168.1.49"
        @test T(run = true, export_c = false, juliac = true, deploy_host = "") === :local_juliac
        @test T(run = true, export_c = true, juliac = true, deploy_host = pi) === :remote_juliac
        @test T(run = false, export_c = false, juliac = true, deploy_host = pi) === :export_juliac
        spec = QC.FurutaSwingupBaseSpec(; juliac = true, export_c = false,
                                        output_dir = "furuta_juliac")
        @test QC.program_log_path(spec, "x.csv") == "x.csv"
        @test QC.local_log_path(spec, "x.csv") == joinpath("furuta_juliac", "x.csv")
        @test_throws ArgumentError QC.juliac_available(; platform = "windows/arm64")
    end

    # The analysis builds the binary with `run = false` too, and the binary runs its whole loop
    # against no device with `--dry-run`, writing the program's log.
    if QC.juliac_available()
        @testset "trimmed binary (this machine)" begin
            DI = QC.DyadInterface
            dir = mktempdir()
            sol = QC.FurutaSwingupExperiment(; output_dir = dir, Ts, run = false, Tf = 1.0,
                                              juliac = true, deploy_host = "")
            exe = joinpath(dir, "bundle", "bin", "QubeController")
            @test isfile(exe) && !sol.hwrun.ran && sol.hwrun.mangled == "QubeController"
            buildlog = read(joinpath(dir, "build.log"), String)
            @test !occursin("Verifier error", buildlog)
            # SynchJulia keeps the C toolchain and the network stack out of the bundle.
            libs = [f for (_, _, fs) in walkdir(joinpath(dir, "bundle")) for f in fs]
            @test !any(f -> occursin(r"clang|libcurl|libssl|libssh2", f), libs)
            @test "libqube_hw.so" in readdir(joinpath(dir, "bundle", "lib"))
            tbl = DI.artifacts(sol, :GeneratedFiles)
            @test "bundle/bin/QubeController" in tbl.file
            # No device on this machine: HIL mode fails cleanly, a dry run completes.
            @test !success(Cmd(`$exe`; dir))
            run(Cmd(`$exe --dry-run`; dir))
            D = readdlm(joinpath(dir, QC.SWINGUP_LOG_FILE), '\t'; header = true)
            @test size(D[1], 1) == round(Int, 1.0 / Ts)
        end
    else
        @info "JuliaC is not installed for this machine; skipping the binary build \
               (see `QuanserComponents.juliac_available`)"
    end

    # A Raspberry Pi binary, built and run under emulation in the arm64 root file system. It
    # takes several minutes, so it runs only when asked for with QUBE_TEST_ARM64=1.
    if get(ENV, "QUBE_TEST_ARM64", "0") == "1" && QC.juliac_available(; platform = "linux/arm64")
        @testset "trimmed binary (linux/arm64)" begin
            dir = mktempdir()
            try
                res = QC.export_program_juliac(src, dir; Tf = 1.0)
                exe = QC.build_program_juliac(res.app_dir; platform = "linux/arm64")
                @test occursin("ARM aarch64", read(`file $exe`, String))
                @test !occursin("Verifier error", read(joinpath(dir, "build.log"), String))
                # The arm64 SDK is installed in the root file system, so HIL mode gets as far as
                # looking for the card; a dry run completes.
                run_sh = joinpath(QC.DEPLOY_DIR, "arm64", "run.sh")
                @test !success(`$run_sh --bind $dir sh -c "cd $dir && $exe"`)
                run(`$run_sh --bind $dir sh -c "cd $dir && $exe --dry-run"`)
                D = readdlm(joinpath(dir, src.log.file), '\t'; header = true)
                @test size(D[1], 1) == round(Int, 1.0 / Ts)
            finally
                rm(dir; recursive = true, force = true)
            end
        end
    end
end
