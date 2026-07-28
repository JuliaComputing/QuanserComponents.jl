/* See qube_hw.h. The HIL path here is the code that used to live in the
 * generated run_hardware.c control loop; moving it behind these four entry
 * points is what lets the controller do its own I/O on every target. */
#include "qube_hw.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifdef QUBE_HW_HAVE_HIL
#include "hil.h"
#include "quanser_messages.h"
#endif

#define QUBE_HW_COUNTS2RAD (2.0 * 3.14159265358979323846 / 2048.0)

static int    s_mode   = -1;    /* -1 = not open */
static double s_shoulder = 0.0;
static double s_elbow    = 0.0;
static double s_u        = 0.0;
static long   s_n_measure = 0;
static long   s_n_write   = 0;
static long   s_count_shoulder = 0;
static long   s_count_elbow    = 0;
/* Added to every shoulder reading: where the arm physically sat at `qube_hw_open`
 * (HIL mode only), so the arm need not be at its home position to calibrate. */
static double s_arm_offset = 0.0;

static qube_hw_measure_fn s_cb_measure = NULL;
static qube_hw_write_fn   s_cb_write   = NULL;

/* Copied rather than aliased: callers (notably Julia) may pass a transient string. */
static char s_card_options[256] = QUBE_HW_DEFAULT_CARD_OPTIONS;

void qube_hw_set_card_options(const char *options) {
    if (options == NULL) { s_card_options[0] = '\0'; return; }
    size_t i = 0;
    while (options[i] != '\0' && i + 1 < sizeof(s_card_options)) {
        s_card_options[i] = options[i];
        i += 1;
    }
    s_card_options[i] = '\0';
}

const char *qube_hw_card_options(void) { return s_card_options; }

#ifdef QUBE_HW_HAVE_HIL
static const char board_type[]       = "qube_servo3_usb";
static const char board_identifier[] = "0";

static t_card   s_board;
static t_int32  s_counts0[2] = {0, 0};   /* homing offsets, subtracted from every read */
static const t_uint32 s_encoder_channels[2] = {0u, 1u};  /* 0 = shoulder, 1 = elbow */
static const t_uint32 s_analog_channel      = 0u;        /* motor voltage */
static const t_uint32 s_digital_channel     = 0u;        /* amplifier enable */

static int qube_hil_start(void) {
    t_double  voltage = 0.0;
    t_boolean enable  = 1;
    t_error   result  = hil_open(board_type, board_identifier, &s_board);
    if (result < 0) {
        char msg[512];
        msg_get_error_message(NULL, result, msg, sizeof(msg));
        fprintf(stderr, "qube_hw: hil_open failed (%d): %s\n", (int)result, msg);
        return -1;
    }
    /* Home in software: the encoder counters are left untouched
     * (hil_set_encoder_counts is deliberately NOT called -- zeroing the counters
     * has been observed to desync the driver's counter extension, producing
     * spurious 2^16-count jumps) and the initial counts become offsets. */
    /* Before touching the motor: pin the command-to-torque path (see qube_hw.h). */
    if (s_card_options[0] != '\0') {
        result = hil_set_card_specific_options(s_board, s_card_options,
                                               strlen(s_card_options));
        if (result < 0) {
            fprintf(stderr, "qube_hw: hil_set_card_specific_options(\"%s\") failed (%d)\n",
                    s_card_options, (int)result);
            hil_close(s_board);
            return -1;
        }
    }
    hil_write_analog(s_board, &s_analog_channel, 1, &voltage);
    hil_write_digital(s_board, &s_digital_channel, 1, &enable);
    hil_read_encoder(s_board, s_encoder_channels, 2, s_counts0);
    return 0;
}

static void qube_hil_stop(void) {
    t_double voltage = 0.0;
    hil_write_analog(s_board, &s_analog_channel, 1, &voltage);
    hil_close(s_board);
}

static void qube_hil_read(double *shoulder, double *elbow) {
    t_int32 counts[2] = {0, 0};
    hil_read_encoder(s_board, s_encoder_channels, 2, counts);
    s_count_shoulder = (long)counts[0];
    s_count_elbow    = (long)counts[1];
    *shoulder = (double)(counts[0] - s_counts0[0]) * QUBE_HW_COUNTS2RAD + s_arm_offset;
    *elbow    = (double)(counts[1] - s_counts0[1]) * QUBE_HW_COUNTS2RAD;
}

static void qube_hil_apply(double u) {
    t_double voltage = (t_double)u;
    hil_write_analog(s_board, &s_analog_channel, 1, &voltage);
}
#endif /* QUBE_HW_HAVE_HIL */

int qube_hw_have_hil(void) {
#ifdef QUBE_HW_HAVE_HIL
    return 1;
#else
    return 0;
#endif
}

int qube_hw_open(int mode, double arm_home_rad) {
    qube_hw_close();
    if (mode == QUBE_HW_MODE_HIL) {
#ifdef QUBE_HW_HAVE_HIL
        if (qube_hil_start() != 0) return -1;
#else
        fprintf(stderr, "qube_hw: built without HIL support "
                        "(compile with -DQUBE_HW_HAVE_HIL)\n");
        return -2;
#endif
    } else if (mode != QUBE_HW_MODE_CALLBACK) {
        fprintf(stderr, "qube_hw: unknown mode %d\n", mode);
        return -3;
    }
    s_mode = mode;
    /* Callback handlers return already-calibrated angles, so the offset is HIL-only. */
    s_arm_offset = (mode == QUBE_HW_MODE_HIL) ? arm_home_rad : 0.0;
    s_shoulder = 0.0;
    s_elbow = 0.0;
    s_u = 0.0;
    s_count_shoulder = 0;
    s_count_elbow = 0;
    qube_hw_reset_counters();
    return 0;
}

void qube_hw_close(void) {
    if (s_mode < 0) return;
#ifdef QUBE_HW_HAVE_HIL
    if (s_mode == QUBE_HW_MODE_HIL) qube_hil_stop();
#endif
    s_mode = -1;
}

void qube_hw_set_callbacks(qube_hw_measure_fn measure, qube_hw_write_fn write) {
    s_cb_measure = measure;
    s_cb_write = write;
}

double qube_hw_measure(double dep) {
    (void)dep;
    s_n_measure += 1;
    switch (s_mode) {
#ifdef QUBE_HW_HAVE_HIL
    case QUBE_HW_MODE_HIL:
        qube_hil_read(&s_shoulder, &s_elbow);
        break;
#endif
    case QUBE_HW_MODE_CALLBACK:
        if (s_cb_measure != NULL) s_cb_measure(&s_shoulder, &s_elbow);
        break;
    default:
        /* Not open: hold the last values rather than fault in a real-time loop. */
        break;
    }
    return s_shoulder;
}

double qube_hw_shoulder(double dep) { (void)dep; return s_shoulder; }
double qube_hw_elbow(double dep)    { (void)dep; return s_elbow; }

double qube_hw_write(double u, double umax) {
    double clamped = u;
    if (umax < 0.0) umax = -umax;
    if (clamped >  umax) clamped =  umax;
    if (clamped < -umax) clamped = -umax;
    /* Never let a NaN reach the amplifier. */
    if (!(clamped == clamped)) clamped = 0.0;

    s_u = clamped;
    s_n_write += 1;
    switch (s_mode) {
#ifdef QUBE_HW_HAVE_HIL
    case QUBE_HW_MODE_HIL:
        qube_hil_apply(clamped);
        break;
#endif
    case QUBE_HW_MODE_CALLBACK:
        if (s_cb_write != NULL) s_cb_write(clamped);
        break;
    default:
        break;
    }
    return clamped;
}

long qube_hw_n_measure(void) { return s_n_measure; }
long qube_hw_n_write(void)   { return s_n_write; }
void qube_hw_reset_counters(void) { s_n_measure = 0; s_n_write = 0; }

double qube_hw_last_shoulder(void) { return s_shoulder; }
double qube_hw_last_elbow(void)    { return s_elbow; }
double qube_hw_last_u(void)        { return s_u; }
long qube_hw_last_count_shoulder(void) { return s_count_shoulder; }
long qube_hw_last_count_elbow(void)    { return s_count_elbow; }
