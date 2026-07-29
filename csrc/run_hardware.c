/* Standalone control loop for the exported swing-up controller.
 *
 * The controller does its own encoder reads and motor writes (through qube_hw.c, the
 * same implementation the Julia backend calls), so this is only timing and logging:
 * tick the clock every Ts and record what the controller reports.
 *
 * Everything that depends on the compiled controller — the mangled node symbols, the
 * serialized parameter blocks, the sample time and the run duration — arrives through
 * run_hardware_config.h, which `QuanserComponents.emit_hardware_harness` generates next
 * to this file. That keeps this a normal C file: editable, clang-format-able, and
 * compilable in place rather than a Julia string. Build with `make`. */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>

#include "qube_hw.h"
#include "top.h"
#include "run_hardware_config.h"

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

/* elapsed seconds a - b for two CLOCK_MONOTONIC timestamps */
static double tsub(struct timespec a, struct timespec b) {
    return (double)(a.tv_sec - b.tv_sec) + (double)(a.tv_nsec - b.tv_nsec) * 1e-9;
}

int main(void) {
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* Opening in HIL mode enables the amplifier, zeroes the motor and records the
     * current encoder counts as homing offsets. Let the pendulum hang straight down
     * (0 = down, pi = up); the arm may be anywhere, as long as QUBE_ARM0 says where.
     * The controller's GoHome then drives the arm to centre. */
#ifdef QUBE_CARD_OPTIONS
    qube_hw_set_card_options(QUBE_CARD_OPTIONS);   /* pin the command-to-torque path */
#endif
    if (qube_hw_open(QUBE_HW_MODE_HIL, QUBE_ARM0) != 0) return 1;
    fprintf(stderr, "run_hardware: card options = %s\n", qube_hw_card_options());

    QUBE_MEM state;
    memset(&state, 0, sizeof(state));
    QUBE_RESET(&state);

    /* Tab-separated log matching test/hardware_swingup.jl's `swingup.csv` layout
     * (writedlm): first four columns are what `plotD` expects (load with
     * `D = readdlm("run_hardware.csv", skipstart=1)'`). Two extra diagnostic columns:
     * `dt` = achieved period since the previous step, `exec` = body (read+step+write)
     * duration — both in seconds, for spotting timing trouble — plus the raw encoder
     * counts, for diagnosing counter glitches (e.g. spurious 2^16 jumps). */
    FILE *logf = fopen("run_hardware.csv", "w");
    if (logf) fprintf(logf, "time\tshoulder_angle\telbow_angle\tcontrol_input\tdt\texec\tcount_shoulder\tcount_elbow\n");

    /* Timing mirrors the Julia @periodically loop: run the body, then sleep the
     * remainder of Ts (relative sleep, no absolute-schedule catch-up — so one slow
     * step just stretches that period instead of compressing the following ones). */
    const long N = (long)(QUBE_TF / QUBE_TS);
    struct timespec t0, start, done;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    struct timespec prev = t0;
    double sum_dt = 0.0, max_dt = 0.0, max_exec = 0.0;
    long periods = 0;

    for (long i = 0; i < N && !stop_flag; ++i) {
        clock_gettime(CLOCK_MONOTONIC, &start);
        double t  = tsub(start, t0);      /* elapsed at step start [s] */
        double dt = tsub(start, prev);    /* achieved period since previous step [s] */
        prev = start;

        /* One call: reads both encoders, runs the state machine, writes the motor
         * voltage, and reports all three back. */
        QUBE_OUT out = QUBE_STEP(true, GAINS_PTR, AUTO_PTR, &state);

        clock_gettime(CLOCK_MONOTONIC, &done);
        double exec = tsub(done, start);  /* body duration [s] */

        if (logf) fprintf(logf, "%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%ld\t%ld\n",
                          t, out.QUBE_OUT_SHOULDER,
                          out.QUBE_OUT_ELBOW, out.QUBE_OUT_U,
                          dt, exec,
                          qube_hw_last_count_shoulder(), qube_hw_last_count_elbow());

        if (i > 0) { sum_dt += dt; if (dt > max_dt) max_dt = dt; periods++; }
        if (exec > max_exec) max_exec = exec;

        double remain = QUBE_TS - exec;
        if (remain > 0.0) {
            struct timespec ts;
            ts.tv_sec  = (time_t)remain;
            ts.tv_nsec = (long)((remain - (double)ts.tv_sec) * 1e9);
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);   /* relative sleep */
        }
    }

    fprintf(stderr, "run_hardware: Ts=%.4f s | mean dt=%.4f s, max dt=%.4f s, "
                    "max exec=%.4f s over %ld periods\n",
            QUBE_TS, periods > 0 ? sum_dt / (double)periods : 0.0, max_dt, max_exec,
            periods);

    /* Zeroes the motor and releases the board. */
    qube_hw_close();
    if (logf) fclose(logf);
    return 0;
}
