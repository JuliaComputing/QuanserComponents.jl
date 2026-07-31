/* See qube_traj.h for the contract and for why the filename does not come in
 * through the synchronous program's interface. */
#include "qube_traj.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define QUBE_TRAJ_PATH_MAX 512

static double *s_samples = NULL;
static long    s_n       = 0;
static int     s_error   = 0;
static char    s_name[QUBE_TRAJ_PATH_MAX] = "";

/* One column of one line, 1-based, whitespace or tab separated. Returns 0 when the
 * line has fewer fields than that (a short or blank line is skipped, not fatal). */
static int parse_field(const char *line, int column, double *out) {
    const char *p = line;
    for (int col = 1;; ++col) {
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '\n' || *p == '\r') return 0;
        char *end = NULL;
        double v = strtod(p, &end);
        if (end == p) return 0;              /* not a number: a header line */
        if (col == column) { *out = v; return 1; }
        p = end;
    }
}

int qube_traj_open(const char *filename, int column) {
    qube_traj_close();
    s_error = 0;
    s_name[0] = '\0';

    if (filename == NULL || filename[0] == '\0') return -1;
    if (column < 1) column = 1;

    FILE *f = fopen(filename, "r");
    if (f == NULL) return -2;

    long cap = 1024;
    s_samples = (double *)malloc((size_t)cap * sizeof(double));
    if (s_samples == NULL) { fclose(f); return -3; }

    char line[512];
    while (fgets(line, sizeof(line), f) != NULL) {
        double v;
        /* A header line simply fails to parse and is skipped, so the same file works
         * with or without one. */
        if (!parse_field(line, column, &v)) continue;
        if (s_n == cap) {
            cap *= 2;
            double *bigger = (double *)realloc(s_samples, (size_t)cap * sizeof(double));
            if (bigger == NULL) { free(s_samples); s_samples = NULL; s_n = 0;
                                  fclose(f); return -3; }
            s_samples = bigger;
        }
        s_samples[s_n++] = v;
    }
    fclose(f);

    if (s_n == 0) { qube_traj_close(); return -4; }
    strncpy(s_name, filename, sizeof(s_name) - 1);
    s_name[sizeof(s_name) - 1] = '\0';
    return 0;
}

double qube_traj_value(double index) {
    /* Off the end is 0, not the last sample and not a fault: a replay that outlives its
     * trajectory should stop driving, and the control loop must not be interrupted to
     * say so. `qube_traj_error` reports it afterwards. */
    if (s_samples == NULL) return 0.0;
    long i = (long)index;                     /* 1-based, as the program counts ticks */
    if (i < 1 || i > s_n) { s_error = 1; return 0.0; }
    return s_samples[i - 1];
}

void qube_traj_close(void) {
    free(s_samples);
    s_samples = NULL;
    s_n = 0;
}

long qube_traj_length(void) { return s_n; }
int  qube_traj_error(void) { return s_error; }
const char *qube_traj_filename(void) { return s_name; }
