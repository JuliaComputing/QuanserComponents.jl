/* See qube_log.h for the contract and for why the filename does not come in
 * through the synchronous program's interface. */
#include "qube_log.h"

#include <stdio.h>
#include <string.h>

#define QUBE_LOG_PATH_MAX 512

static FILE *s_file = NULL;
static int   s_cols = 0;
static long  s_rows = 0;
static int   s_error = 0;
static char  s_name[QUBE_LOG_PATH_MAX] = "";

/* Line buffering, so a row reaches the file as soon as it is written. The obvious
 * alternative -- one big buffer flushed rarely -- was measured to release the log in
 * 4 KiB jumps (~55 rows, ~0.28 s at 200 Hz), and a bigger buffer is proportionally
 * worse; anything tailing the file then updates in visible steps. A write syscall per
 * tick costs a couple of microseconds against a 5 ms budget. `qube_log_flush` remains
 * for callers that want to force the issue. */

double qube_log_row(double u1, double u2, double u3, double u4,
                    double u5, double u6, double u7, double u8)
{
    if (s_file == NULL) return 0.0;

    const double v[QUBE_LOG_MAX_COLS] = { u1, u2, u3, u4, u5, u6, u7, u8 };
    for (int i = 0; i < s_cols; ++i) {
        /* Tab-separated, same layout as the swing-up harness writes, so the same
         * readers work: `readdlm(path, skipstart = 1)`. */
        if (fprintf(s_file, i == 0 ? "%.9g" : "\t%.9g", v[i]) < 0) {
            /* Out of space or a bad descriptor. Give up on the log rather than
             * retrying every tick, and let the driver notice through
             * `qube_log_error` -- the control loop must not be held up by it. */
            s_error = 1;
            qube_log_close();
            return 0.0;
        }
    }
    if (fputc('\n', s_file) == EOF) {
        s_error = 1;
        qube_log_close();
        return 0.0;
    }
    return (double)(++s_rows);
}

int qube_log_open(const char *filename, const char *header, int ncols)
{
    qube_log_close();
    s_rows = 0;
    s_error = 0;
    s_name[0] = '\0';

    if (filename == NULL || filename[0] == '\0') return -1;

    s_cols = ncols < 1 ? 1 : (ncols > QUBE_LOG_MAX_COLS ? QUBE_LOG_MAX_COLS : ncols);

    s_file = fopen(filename, "w");
    if (s_file == NULL) {
        s_cols = 0;
        return -2;
    }
    setvbuf(s_file, NULL, _IOLBF, 0);

    strncpy(s_name, filename, sizeof(s_name) - 1);
    s_name[sizeof(s_name) - 1] = '\0';

    if (header != NULL && header[0] != '\0') {
        fprintf(s_file, "%s\n", header);
    }
    return 0;
}

void qube_log_flush(void)
{
    if (s_file != NULL) fflush(s_file);
}

void qube_log_close(void)
{
    if (s_file != NULL) {
        fclose(s_file);
        s_file = NULL;
    }
    s_cols = 0;
}

long qube_log_rows(void) { return s_rows; }
int  qube_log_cols(void) { return s_cols; }
int  qube_log_error(void) { return s_error; }
const char *qube_log_filename(void) { return s_name; }
