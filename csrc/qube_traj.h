/* A recorded input sequence, read from inside the synchronous program.
 *
 * The mirror image of qube_log.c: that one takes rows out of the program, this one
 * feeds samples in. `qube_traj_value(k)` is a plain named C function with a `double`
 * signature, so one component definition serves every target -- the Julia backend
 * `ccall`s it, the in-process C backend links it, and `export_c` emits
 * `extern double qube_traj_value(double);` plus a named call site. See
 * QuanserComponents/src/traj_source.jl.
 *
 * What cannot travel through the program's interface is the *file name*: every signal
 * crossing a synchronous node's interface is a `double`. So the component owns the name
 * as a build-time (structural) parameter and whoever drives the program opens the file --
 * `open_traj!` from Julia, `qube_traj_open` from the exported C harness. Both are handed
 * the same name the model was built with, so the two cannot disagree.
 *
 * The file is whitespace- or tab-separated text with one sample per line, of which one
 * 1-based `column` is taken; a header line fails to parse and is skipped, so a file
 * written with or without one both work. This is the format examples/input_design.jl
 * writes.
 */
#ifndef QUBE_TRAJ_H
#define QUBE_TRAJ_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- called by the program, once per tick -------------------------------- */

/* Sample number `index` (1-based, i.e. the tick count), or 0.0 when there is no
 * trajectory open or the index is outside it -- a replay that outlives its trajectory
 * stops driving rather than holding the last value or faulting. Reading past the end
 * sets the error flag for `qube_traj_error`. */
double qube_traj_value(double index);

/* ---- called by the driver, around the control loop ---------------------- */

/* Read `column` (1-based) of every parseable line of `filename` into memory. Returns 0
 * on success, or negative: -1 no file name, -2 could not open, -3 out of memory,
 * -4 no numeric rows found. Replaces whatever was loaded before. */
int qube_traj_open(const char *filename, int column);

/* Release the loaded samples. Safe to call when nothing is loaded. */
void qube_traj_close(void);

/* ---- diagnostics -------------------------------------------------------- */

/* Samples loaded, whether a read has fallen outside them, and the file they came
 * from (never NULL). */
long qube_traj_length(void);
int  qube_traj_error(void);
const char *qube_traj_filename(void);

#ifdef __cplusplus
}
#endif

#endif /* QUBE_TRAJ_H */
