/* Hardware I/O for the Quanser QUBE-Servo Furuta pendulum, called from *inside*
 * the generated synchronous controller.
 *
 * The four `qube_hw_measure` / `_shoulder` / `_elbow` / `_write` entry points are
 * the ones the controller node calls. They are plain named C functions with
 * `double` signatures, which is what lets a single controller definition serve
 * every target: the Julia backend `ccall`s them, the in-process C backend links
 * them, and `export_c` emits `extern double qube_hw_measure(double);` plus a
 * named call site, so the exported C links against this file with no Julia
 * involved. See QuanserComponents/src/hardware_io.jl.
 *
 * Two backends, selected once by `qube_hw_open`:
 *
 *   QUBE_HW_MODE_HIL       talk to the board through the Quanser HIL SDK. Only
 *                          compiled when QUBE_HW_HAVE_HIL is defined, so this
 *                          file builds on machines without the SDK.
 *   QUBE_HW_MODE_CALLBACK  call function pointers installed by
 *                          `qube_hw_set_callbacks`. Used to drive a simulator
 *                          (e.g. from Julia via @cfunction) so the controller
 *                          can be exercised in closed loop without hardware.
 */
#ifndef QUBE_HW_H
#define QUBE_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#define QUBE_HW_MODE_CALLBACK 0
#define QUBE_HW_MODE_HIL      1

/* Card-specific options applied after `hil_open` in HIL mode.
 *
 * `deadband_compensation` offsets the motor command to compensate for amplifier
 * deadband, so it is part of the command-to-torque path. The driver applies a
 * default when nothing is set, which makes the effective gain depend on the SDK
 * build rather than on anything we control -- so set it explicitly and get the
 * same behaviour on every host. 0.65 V is the value the QUBE-Servo 3 driver
 * documents as its default. */
#define QUBE_HW_DEFAULT_CARD_OPTIONS "deadband_compensation=0.65"

/* Fills both angles in radians: shoulder 0 at home, elbow 0 hanging down. */
typedef void (*qube_hw_measure_fn)(double *shoulder, double *elbow);
/* Applies a motor voltage, already clamped. */
typedef void (*qube_hw_write_fn)(double u);

/* ---- called by the controller node, once per tick, in this order ---------- */

/* The one effectful read: samples both encoders and caches them. Returns the
 * shoulder angle, which also serves as the dependency token that orders the two
 * accessors below after this call (equation scheduling only respects data
 * dependencies). `dep` is ignored; it exists so the call has an argument. */
double qube_hw_measure(double dep);

/* Cached angles from the most recent `qube_hw_measure`. `dep` is ignored. */
double qube_hw_shoulder(double dep);
double qube_hw_elbow(double dep);

/* Clamps `u` to [-umax, umax], applies it, and returns what was applied. */
double qube_hw_write(double u, double umax);

/* Loop diagnostics of the current tick, so the program can log them itself instead of
 * the surrounding loop keeping a second file. All take an ignored dependency token, for
 * the same scheduling reason as the accessors above.
 *
 *   time  seconds since the first read after `qube_hw_open` (so the first tick is 0)
 *   dt    seconds between this read and the previous one, i.e. the achieved period
 *   exec  seconds from this tick's read to this tick's write
 *
 * `exec` covers the read, the control computation and the write -- everything the node
 * does between the two I/O calls, which is all of it bar the logging call itself. It is
 * therefore slightly smaller than the loop body a surrounding timing loop would measure.
 * `dt` needs a previous tick and is 0 on the first one.
 *
 * The counts are the raw encoder counts of the most recent read, as doubles because
 * every signal inside the program is one; exact up to 2^53, so no count is ever
 * misrepresented. */
double qube_hw_time(double dep);
double qube_hw_dt(double dep);
double qube_hw_exec(double dep);
double qube_hw_count_shoulder(double dep);
double qube_hw_count_elbow(double dep);

/* ---- called by the driver, around the control loop ----------------------- */

/* Open `mode`. Returns 0 on success, negative on failure (including
 * QUBE_HW_MODE_HIL when built without QUBE_HW_HAVE_HIL). Clears the cache and
 * the call counters.
 *
 * In HIL mode this enables the amplifier, zeroes the motor and records the
 * current encoder counts as the homing offsets. `arm_home_rad` says where the arm
 * physically is at that moment, and is added to every shoulder reading, so you do
 * not have to move the arm to its home position first: park it roughly centred,
 * pass how far off it is, and 0 still means centred. Pass 0.0 if the arm really is
 * at home. The pendulum must hang straight down (that reading is always zeroed).
 *
 * `arm_home_rad` is ignored in QUBE_HW_MODE_CALLBACK: the handler installed by
 * `qube_hw_set_callbacks` is expected to return angles that are already
 * calibrated. */
int qube_hw_open(int mode, double arm_home_rad);

/* Zero the motor and release the device. Safe to call when not open. */
void qube_hw_close(void);

/* Override the card-specific options string applied at the next `qube_hw_open` in
 * HIL mode. Pass NULL or "" to skip the call and leave the driver on its own
 * defaults. Truncated to 255 characters. Has no effect in callback mode. */
void qube_hw_set_card_options(const char *options);

/* The options string that will be / was applied. Never NULL. */
const char *qube_hw_card_options(void);

/* Install the QUBE_HW_MODE_CALLBACK handlers. Either may be NULL, in which case
 * that direction becomes a no-op (reads return 0). */
void qube_hw_set_callbacks(qube_hw_measure_fn measure, qube_hw_write_fn write);

/* ---- diagnostics --------------------------------------------------------- */

/* Number of reads/writes since the last `qube_hw_open` or
 * `qube_hw_reset_counters`. The tests assert exactly one of each per tick. */
long qube_hw_n_measure(void);
long qube_hw_n_write(void);
void qube_hw_reset_counters(void);

/* Restart the `time`/`dt`/`exec` measurement, so `time` is 0 at the next read. Call
 * between runs that share an open device. */
void qube_hw_reset_timing(void);

/* Most recent cached values, for logging from the driver. */
double qube_hw_last_shoulder(void);
double qube_hw_last_elbow(void);
double qube_hw_last_u(void);

/* Raw encoder counts of the most recent read (HIL mode; 0 otherwise), for
 * diagnosing counter glitches such as spurious 2^16 jumps. */
long qube_hw_last_count_shoulder(void);
long qube_hw_last_count_elbow(void);

/* Nonzero if this translation unit was built with HIL support. */
int qube_hw_have_hil(void);

#ifdef __cplusplus
}
#endif

#endif /* QUBE_HW_H */
