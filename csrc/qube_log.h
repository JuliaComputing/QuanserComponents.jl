/* Data logging from *inside* a generated synchronous program.
 *
 * `qube_log_row` is the one entry point the program calls: it appends one
 * tab-separated row per tick. Like csrc/qube_hw.c it is a plain named C function
 * with a `double` signature, which is what lets a single model definition serve
 * every target -- the Julia backend `ccall`s it, the in-process C backend links
 * it, and `export_c` emits `extern double qube_log_row(double, ...);` plus a
 * named call site, so the exported C links against this file with no Julia
 * involved. See QuanserComponents/src/data_log.jl and the `DataLogger` component
 * in dyad/friction.dyad.
 *
 * The file itself is opened and closed by the driver, not by the program: a
 * filename is a string, and strings cannot cross into a synchronous node whose
 * every signal is a `double`. The `DataLogger` component still owns the filename
 * as a structural parameter -- it is a build-time Julia value, so the generator
 * reads it off the model and passes it here (and bakes it into the emitted C
 * config header for the standalone build).
 *
 * With no log open every call is a no-op returning 0, so a model containing a
 * `DataLogger` runs unchanged when nothing wants the data.
 */
#ifndef QUBE_LOG_H
#define QUBE_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Columns `qube_log_row` accepts. Raising this means adding inputs to the
 * `DataLogger` component and arguments to the operator in src/data_log.jl; the
 * three must agree. */
#define QUBE_LOG_MAX_COLS 8

/* ---- called by the program, once per tick -------------------------------- */

/* Append a row of the first `ncols` arguments (as passed to `qube_log_open`) and
 * return the 1-based number of the row just written. Returns 0.0 when no log is
 * open, which is the only failure mode -- a write error closes the log and is
 * reported by `qube_log_error`, so the control loop is never disturbed by one. */
double qube_log_row(double u1, double u2, double u3, double u4,
                    double u5, double u6, double u7, double u8);

/* ---- called by the driver, around the loop ------------------------------- */

/* Open `filename` for writing, replacing any existing file, and log `ncols`
 * columns (clamped to [1, QUBE_LOG_MAX_COLS]). If `header` is non-NULL and
 * non-empty it is written as the first line, verbatim plus a newline -- so it
 * should already be tab-separated. Closes a previously open log first and resets
 * the row counter. Returns 0 on success, negative if the file could not be
 * opened. */
int qube_log_open(const char *filename, const char *header, int ncols);

/* Flush buffered rows to disk without closing, so a viewer tailing the file sees
 * the run in progress. No-op when no log is open. */
void qube_log_flush(void);

/* Flush and close. Safe to call when no log is open. */
void qube_log_close(void);

/* ---- diagnostics --------------------------------------------------------- */

/* Rows written since `qube_log_open`. */
long qube_log_rows(void);

/* Columns being written, or 0 when no log is open. */
int qube_log_cols(void);

/* The file currently open, or "" when none. Never NULL. */
const char *qube_log_filename(void);

/* Nonzero once a write has failed (the log is closed at that point). Cleared by
 * the next `qube_log_open`. */
int qube_log_error(void);

#ifdef __cplusplus
}
#endif

#endif /* QUBE_LOG_H */
