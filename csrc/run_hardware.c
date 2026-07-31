/* Standalone control loop for an exported QUBE program.
 *
 * The program does its own encoder reads, motor writes and log rows (through qube_hw.c and
 * qube_log.c, the same implementations the Julia backend calls), so this is only timing:
 * open the device, open the log for the program to write into, tick the clock every Ts.
 *
 * Nothing here knows which program it is running. The node's outputs are never read — the
 * program logs what it wants logged, in the columns its own DataLogger was built with — so
 * the same loop serves the swing-up controller, the friction experiment, or anything else
 * built for this rig. What does depend on the program — the mangled node symbols, the
 * serialized parameter blocks, the sample time, the run duration and the log's identity —
 * arrives through run_hardware_config.h, which
 * `QuanserComponents.emit_hardware_harness` generates next to this file. That keeps this a
 * normal C file: editable, clang-format-able, and compilable in place rather than a Julia
 * string. Build with `make`. */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>

#include "qube_hw.h"
#include "qube_log.h"
#include "qube_traj.h"
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
     * A swing-up program's GoHome then drives the arm to centre. */
#ifdef QUBE_CARD_OPTIONS
    qube_hw_set_card_options(QUBE_CARD_OPTIONS);   /* pin the command-to-torque path */
#endif
    if (qube_hw_open(QUBE_HW_MODE_HIL, QUBE_ARM0) != 0) return 1;
    fprintf(stderr, "run_hardware: card options = %s\n", qube_hw_card_options());

    /* The log the program writes its rows into. Opening it here rather than inside the
     * program is not a split of responsibility: a file name cannot cross a synchronous
     * node's interface (every signal there is a double), so the name travels as a
     * build-time constant and arrives in this header. qube_log.c line-buffers, which is
     * what makes a live plot of a run in progress smooth. */
    if (qube_log_open(QUBE_LOG_FILE, QUBE_LOG_HEADER, QUBE_LOG_NCOLS) != 0) {
        fprintf(stderr, "run_hardware: could not open %s for writing\n", QUBE_LOG_FILE);
        qube_hw_close();
        return 1;
    }

    /* An input sequence to replay, for the programs that have one. Same arrangement as the
     * log: the name cannot cross the node's interface, so it arrives as a build-time constant
     * in this header and the loop hands it to qube_traj.c. */
#ifdef QUBE_TRAJ_FILE
    if (qube_traj_open(QUBE_TRAJ_FILE, QUBE_TRAJ_COLUMN) != 0) {
        fprintf(stderr, "run_hardware: could not read the trajectory %s\n", QUBE_TRAJ_FILE);
        qube_log_close();
        qube_hw_close();
        return 1;
    }
    fprintf(stderr, "run_hardware: replaying %ld samples from %s\n",
            qube_traj_length(), QUBE_TRAJ_FILE);
#endif

    QUBE_MEM state;
    memset(&state, 0, sizeof(state));
    QUBE_RESET(&state);

    /* Timing mirrors the Julia loop: run the body, then sleep the remainder of Ts (relative
     * sleep, no absolute-schedule catch-up — so one slow step just stretches that period
     * instead of compressing the following ones). The dt/exec measured here are for the
     * summary line only; the program logs its own, from inside the tick. */
    const long N = (long)(QUBE_TF / QUBE_TS);
    struct timespec t0, start, done;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    struct timespec prev = t0;
    double sum_dt = 0.0, max_dt = 0.0, max_exec = 0.0;
    long periods = 0;

    for (long i = 0; i < N && !stop_flag; ++i) {
        clock_gettime(CLOCK_MONOTONIC, &start);
        double dt = tsub(start, prev);    /* achieved period since previous step [s] */
        prev = start;

        /* One call: reads both encoders, computes, writes the motor voltage and appends a
         * row to the log. The returned output struct is deliberately unused. */
        QUBE_OUT out = QUBE_STEP(true, GAINS_PTR, AUTO_PTR, &state);
        (void)out;

        clock_gettime(CLOCK_MONOTONIC, &done);
        double exec = tsub(done, start);  /* body duration [s] */

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
                    "max exec=%.4f s over %ld periods | %ld rows in %s\n",
            QUBE_TS, periods > 0 ? sum_dt / (double)periods : 0.0, max_dt, max_exec,
            periods, qube_log_rows(), QUBE_LOG_FILE);
    if (qube_log_error()) fprintf(stderr, "run_hardware: the log was closed by a write error\n");

    /* Zeroes the motor and releases the board. */
    qube_hw_close();
    qube_log_close();
#ifdef QUBE_TRAJ_FILE
    if (qube_traj_error())
        fprintf(stderr, "run_hardware: the run outlived the trajectory (0 V was commanded)\n");
    qube_traj_close();
#endif
    return 0;
}
