/*
 * main.c
 *
 * Swept-frequency impedance analyzer — Goertzel core + VGA Bode display.
 * DE1-SoC HPS. Compile with -DSIM_MODE to run without hardware.
 *
 * VGA primitives borrowed from FPGA Signal Classifier Project
 * https://github.com/niztg/FPGA-Signal-Classifier/tree/main
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <complex.h>
#include <string.h>
#include <stdbool.h>
#include "hw.h"

#define PIXEL_BUF_CTRL   0xFF203020
#define CHAR_BUF_CTRL    0xFF203030

volatile int        pixel_buffer_start;
volatile int       *pixel_ctrl_ptr     = (volatile int *)PIXEL_BUF_CTRL;
volatile char      *character_buffer_start;
volatile int       *character_ctrl_ptr = (volatile int *)CHAR_BUF_CTRL;

#define BACKGROUND_COLOR  0x0000
#define LINE_COLOR        0xFFFF
#define COLOR_MAG         0x07E0
#define COLOR_PHASE       0xF81F
#define COLOR_GRID        0x2104

#define SAMPLE_RATE_HZ   44100.0
#define R_REF_OHMS       1000.0
#define F_START_HZ       10.0
#define F_STOP_HZ        20000.0
#define N_FREQS          50
#define CYCLES_PER_BIN   10

#define LEFT_MARGIN   48
#define RIGHT_MARGIN   8
#define TOP_MARGIN     8
#define BOT_MARGIN    32
#define SEPARATOR      8
#define PANEL_W       (640 - LEFT_MARGIN - RIGHT_MARGIN)
#define PANEL_H       ((480 - TOP_MARGIN - BOT_MARGIN - SEPARATOR) / 2)

static const int MAG_TL_X   = LEFT_MARGIN;
static const int MAG_TL_Y   = TOP_MARGIN;
static const int PHASE_TL_X = LEFT_MARGIN;
static const int PHASE_TL_Y = TOP_MARGIN + PANEL_H + SEPARATOR;

#define ENC0_BASE_BIT  27
#define ENC1_BASE_BIT  17

static double g_f_min, g_f_max, g_z_min, g_z_max;

typedef enum { STATE_IDLE, STATE_SWEEPING, STATE_DONE } AppState;
static AppState app_state = STATE_IDLE;

typedef struct { int x; int y; } point;

static void vga_text(int x, int y, char *text_ptr) {
    int offset = (y << 7) + x;
    while (*text_ptr != '\0') {
        *(character_buffer_start + offset) = *text_ptr;
        ++text_ptr; ++offset;
    }
}

static void plotPixel(point p, short int color) {
    volatile short int *addr =
        (volatile short int *)(pixel_buffer_start + (p.y << 10) + (p.x << 1));
    *addr = color;
}

static void swapXY(point *p) { int t = p->x; p->x = p->y; p->y = t; }
static void swap2Points(point *a, point *b) {
    int t;
    t = a->x; a->x = b->x; b->x = t;
    t = a->y; a->y = b->y; b->y = t;
}

static void drawLine(point p0, point p1, short int color, bool dotted) {
    bool steep = abs(p1.y - p0.y) > abs(p1.x - p0.x);
    int dash = 4;
    if (steep)       { swapXY(&p0); swapXY(&p1); }
    if (p0.x > p1.x)  swap2Points(&p0, &p1);
    int dx = p1.x - p0.x, dy = abs(p1.y - p0.y);
    int err = -dx / 2, y = p0.y, ystep = (p1.y > p0.y) ? 1 : -1;
    for (int x = p0.x; x <= p1.x; x++) {
        short int c = (dotted && dash++ % 4) ? BACKGROUND_COLOR : color;
        plotPixel(steep ? (point){y, x} : (point){x, y}, c);
        if ((err += dy) > 0) { y += ystep; err -= dx; }
    }
}

static void clearRegion(point tl, int w, int h) {
    for (int x = tl.x; x < tl.x + w; x++)
        for (int y = tl.y; y < tl.y + h; y++) {
            volatile short int *addr =
                (volatile short int *)(pixel_buffer_start + (y << 10) + (x << 1));
            *addr = BACKGROUND_COLOR;
        }
    for (int y = tl.y / 4; y < (tl.y + h) / 4 + 1; y++)
        for (int x = tl.x / 4; x < (tl.x + w) / 4 + 1; x++)
            *(character_buffer_start + (y << 7) + x) = ' ';
}

static void drawGraphBoundingBox(point tl, int gh, int gw) {
    point tr = {tl.x + gw, tl.y}, bl = {tl.x, tl.y + gh}, br = {tl.x + gw, tl.y + gh};
    drawLine(tl, tr, LINE_COLOR, false);
    drawLine(tl, bl, LINE_COLOR, false);
    drawLine(br, tr, LINE_COLOR, false);
    drawLine(br, bl, LINE_COLOR, false);
}

static int freq_to_x(double f) {
    double t = log(f / g_f_min) / log(g_f_max / g_f_min);
    if (t < 0.0) t = 0.0; if (t > 1.0) t = 1.0;
    return MAG_TL_X + 1 + (int)(t * (PANEL_W - 2));
}

static int zmag_to_y(double z) {
    double t = log(z / g_z_min) / log(g_z_max / g_z_min);
    if (t < 0.0) t = 0.0; if (t > 1.0) t = 1.0;
    return MAG_TL_Y + PANEL_H - 1 - (int)(t * (PANEL_H - 2));
}

static int phase_to_y(double deg) {
    double t = (deg + 90.0) / 180.0;
    if (t < 0.0) t = 0.0; if (t > 1.0) t = 1.0;
    return PHASE_TL_Y + PANEL_H - 1 - (int)(t * (PANEL_H - 2));
}

static void draw_log_freq_axis(void) {
    static const double sub[3] = { 1.0, 2.0, 5.0 };
    int bot_mag   = MAG_TL_Y   + PANEL_H - 1;
    int bot_phase = PHASE_TL_Y + PANEL_H - 1;
    double decade = pow(10.0, floor(log10(g_f_min)));
    while (decade * 10.0 < g_f_min) decade *= 10.0;
    for (; decade <= g_f_max * 1.01; decade *= 10.0) {
        for (int s = 0; s < 3; s++) {
            double f = decade * sub[s];
            if (f < g_f_min || f > g_f_max) continue;
            int x = freq_to_x(f);
            bool is_decade = (s == 0);
            for (int y = MAG_TL_Y + 1;   y < bot_mag;   y++)
                if (is_decade || y % 2 == 0) plotPixel((point){x, y}, COLOR_GRID);
            for (int y = PHASE_TL_Y + 1; y < bot_phase; y++)
                if (is_decade || y % 2 == 0) plotPixel((point){x, y}, COLOR_GRID);
            if (is_decade) {
                for (int dy = 0; dy < 4; dy++)
                    plotPixel((point){x, bot_phase + 1 + dy}, LINE_COLOR);
                char label[12];
                snprintf(label, sizeof(label),
                         f >= 1000.0 ? "%.0fk" : "%.0f",
                         f >= 1000.0 ? f / 1000.0 : f);
                vga_text(x / 4 - (int)strlen(label) / 2, (bot_phase + 10) / 4, label);
            }
        }
    }
    vga_text((LEFT_MARGIN + PANEL_W / 2) / 4, (480 - 8) / 4, "Hz");
}

static void draw_log_zmag_axis(void) {
    int lx = MAG_TL_X;
    double decade = pow(10.0, ceil(log10(g_z_min)));
    for (; decade <= g_z_max * 1.01; decade *= 10.0) {
        int y = zmag_to_y(decade);
        for (int dx = 0; dx < 4; dx++)
            plotPixel((point){lx - 1 - dx, y}, LINE_COLOR);
        for (int x = lx + 1; x < lx + PANEL_W - 1; x += 4)
            plotPixel((point){x, y}, COLOR_GRID);
        char label[12];
        snprintf(label, sizeof(label),
                 decade >= 1000.0 ? "%.0fk" : "%.0f",
                 decade >= 1000.0 ? decade / 1000.0 : decade);
        vga_text((lx - 4 - (int)strlen(label) * 4) / 4, y / 4, label);
    }
    vga_text(0, (MAG_TL_Y + PANEL_H / 2) / 4, "|Z|");
}

static void draw_phase_axis(void) {
    static const double ticks[] = { -90.0, -45.0, 0.0, 45.0, 90.0 };
    int lx = PHASE_TL_X;
    for (int i = 0; i < 5; i++) {
        int y = phase_to_y(ticks[i]);
        for (int dx = 0; dx < 4; dx++)
            plotPixel((point){lx - 1 - dx, y}, LINE_COLOR);
        for (int x = lx + 1; x < lx + PANEL_W - 1; x += 4)
            plotPixel((point){x, y}, COLOR_GRID);
        char label[8];
        snprintf(label, sizeof(label), "%+.0f", ticks[i]);
        vga_text((lx - 4 - (int)strlen(label) * 4) / 4, y / 4, label);
    }
    vga_text(0, (PHASE_TL_Y + PANEL_H / 2) / 4, "deg");
}

static void draw_status(const char *msg) {
    vga_text((LEFT_MARGIN + PANEL_W - 40) / 4, 0, (char *)msg);
}

static void bode_init(double z_min, double z_max, double f_min, double f_max) {
    g_f_min = f_min; g_f_max = f_max;
    g_z_min = z_min; g_z_max = z_max;
    clearRegion((point){0, 0}, 640, 480);
    drawGraphBoundingBox((point){MAG_TL_X,   MAG_TL_Y},   PANEL_H, PANEL_W);
    drawGraphBoundingBox((point){PHASE_TL_X, PHASE_TL_Y}, PANEL_H, PANEL_W);
    vga_text(LEFT_MARGIN / 4, TOP_MARGIN / 4, "Impedance magnitude");
    vga_text(LEFT_MARGIN / 4, PHASE_TL_Y / 4, "Phase");
    draw_log_freq_axis();
    draw_log_zmag_axis();
    draw_phase_axis();
}

static void bode_plot_point(double f, double z_mag, double z_phase_deg) {
    if (f < g_f_min || f > g_f_max || z_mag <= 0.0) return;
    plotPixel((point){freq_to_x(f), zmag_to_y(z_mag)},        COLOR_MAG);
    plotPixel((point){freq_to_x(f), phase_to_y(z_phase_deg)}, COLOR_PHASE);
}

static int poll_encoder(int base_bit, int idx) {
    static int prev_clk[2] = {1, 1};
    int jp1 = hw_read_jp1();
    int clk = (jp1 >> (base_bit + 0)) & 1;
    int dt  = (jp1 >> (base_bit + 2)) & 1;
    int dir = 0;
    if (clk && !prev_clk[idx]) dir = dt ? -1 : 1;
    prev_clk[idx] = clk;
    return dir;
}

static int poll_encoder_button(int base_bit, int idx) {
    static int prev_btn[2] = {1, 1};
    int btn = (hw_read_jp1() >> (base_bit + 4)) & 1;
    int pressed = (!btn && prev_btn[idx]) ? 1 : 0;
    prev_btn[idx] = btn;
    return pressed;
}

static double complex goertzel(const double *x, int N, double omega) {
    double coeff = 2.0 * cos(omega), w0 = 0.0, w1 = 0.0, w2 = 0.0;
    for (int n = 0; n < N; n++) { w0 = x[n] + coeff * w1 - w2; w2 = w1; w1 = w0; }
    double complex ejw = cos(omega) - I * sin(omega);
    return w1 - ejw * w2;
}

static double complex calc_impedance(const double *ch_dut, const double *ch_ref,
                                     int N, double omega, double r_ref) {
    double complex V_mid = goertzel(ch_dut, N, omega);  // ch0, middle node
    double complex V_top = goertzel(ch_ref, N, omega);  // ch1, top node
    return (V_mid / (V_top - V_mid)) * r_ref;
}

static void logspace(double f0, double f1, int n, double *out) {
    double lf0 = log10(f0), lf1 = log10(f1);
    for (int i = 0; i < n; i++)
        out[i] = pow(10.0, lf0 + (lf1 - lf0) * i / (n - 1));
}

typedef struct { double f; double z_mag; double z_phase_deg; } BinResult;

static void fmt_si(char *buf, size_t len, double val, const char *unit) {
    const char *prefixes[] = { "m", "", "k", "M" };
    double scales[]        = { 1e-3, 1.0, 1e3, 1e6 };
    int idx = 1;
    if      (val >= 1e6) idx = 3;
    else if (val >= 1e3) idx = 2;
    else if (val <  1.0) idx = 0;
    snprintf(buf, len, "%.3g%s%s", val / scales[idx], prefixes[idx], unit);
}

static void extract_and_display(const BinResult *res, int n) {
    double phase_sum = 0.0;
    for (int i = 0; i < n; i++) phase_sum += res[i].z_phase_deg;
    double mean_phase = phase_sum / n;

    typedef enum { MODEL_RC, MODEL_RL, MODEL_R } Model;
    Model model = (mean_phase < -10.0) ? MODEL_RC :
                  (mean_phase >  10.0) ? MODEL_RL : MODEL_R;

    int lo = 0, hi = n;
    if      (model == MODEL_RC) lo = (3 * n) / 4;
    else if (model == MODEL_RL) hi = n / 4;
    double r_sum = 0.0;
    for (int i = lo; i < hi; i++) r_sum += res[i].z_mag;
    double R = r_sum / (hi - lo);

    double target_phase = (model == MODEL_RL) ? 45.0 : -45.0;
    double f_c = 0.0;
    for (int i = 1; i < n; i++) {
        double p0 = res[i-1].z_phase_deg, p1 = res[i].z_phase_deg;
        if ((p0 - target_phase) * (p1 - target_phase) <= 0.0) {
            double t = (target_phase - p0) / (p1 - p0);
            f_c = res[i-1].f + t * (res[i].f - res[i-1].f);
            break;
        }
    }

    double C = 0.0, L = 0.0;
    if (f_c > 0.0) {
        if (model == MODEL_RC) C = 1.0 / (2.0 * M_PI * R * f_c);
        if (model == MODEL_RL) L = R  / (2.0 * M_PI * f_c);
    }

    char r_buf[16], c_buf[16], l_buf[16], fc_buf[16];
    fmt_si(r_buf,  sizeof(r_buf),  R,   "Ohm");
    fmt_si(fc_buf, sizeof(fc_buf), f_c, "Hz");

    int strip_y = PHASE_TL_Y + PANEL_H + 2;
    clearRegion((point){0, strip_y}, 640, 480 - strip_y);

    char line[80];
    switch (model) {
        case MODEL_RC:
            fmt_si(c_buf, sizeof(c_buf), C, "F");
            snprintf(line, sizeof(line), "Model: Series RC    R: %s    C: %s    fc: %s",
                     r_buf, c_buf, fc_buf);
            break;
        case MODEL_RL:
            fmt_si(l_buf, sizeof(l_buf), L, "H");
            snprintf(line, sizeof(line), "Model: Series RL    R: %s    L: %s    fc: %s",
                     r_buf, l_buf, fc_buf);
            break;
        case MODEL_R:
            snprintf(line, sizeof(line), "Model: Resistive    R: %s", r_buf);
            break;
    }
    vga_text(LEFT_MARGIN / 4, strip_y / 4, line);
}

static void run_sweep(void) {
    bode_init(g_z_min, g_z_max, F_START_HZ, F_STOP_HZ);
    draw_status("Sweeping...     ");

    double freqs[N_FREQS];
    logspace(F_START_HZ, F_STOP_HZ, N_FREQS, freqs);

    BinResult results[N_FREQS];

    for (int i = 0; i < N_FREQS; i++) {
        double f     = freqs[i];
        int    N     = (int)round(CYCLES_PER_BIN * SAMPLE_RATE_HZ / f);
        double omega = 2.0 * M_PI * f / SAMPLE_RATE_HZ;

        double *ch_ref = malloc(N * sizeof(double));
        double *ch_dut = malloc(N * sizeof(double));
        if (!ch_ref || !ch_dut) { fprintf(stderr, "malloc failed\n"); return; }

        hw_set_freq(f);
        hw_read_channels(N, SAMPLE_RATE_HZ, f, ch_ref, ch_dut);

        double complex Z = calc_impedance(ch_dut, ch_ref, N, omega, R_REF_OHMS);
        results[i] = (BinResult){ f, cabs(Z), carg(Z) * 180.0 / M_PI };
        bode_plot_point(f, cabs(Z), carg(Z) * 180.0 / M_PI);

#ifdef SIM_MODE
        printf("%.4f, %.6f, %.6f\n", f, cabs(Z), carg(Z) * 180.0 / M_PI);
#endif
        free(ch_ref);
        free(ch_dut);
    }

    extract_and_display(results, N_FREQS);
    draw_status("Done.           ");
}

int main(void) {
    character_buffer_start = (volatile char *)(*character_ctrl_ptr);
    pixel_buffer_start     = *pixel_ctrl_ptr;

    hw_init();

    g_z_min = 1.0;
    g_z_max = 100000.0;

    bode_init(g_z_min, g_z_max, F_START_HZ, F_STOP_HZ);
    draw_status("Press enc0 to sweep");

    while (1) {
        if (poll_encoder_button(ENC0_BASE_BIT, 0)) {
            app_state = STATE_SWEEPING;
            run_sweep();
            app_state = STATE_DONE;
        }

        int d0 = poll_encoder(ENC0_BASE_BIT, 0);
        if (d0 == 1  && g_z_max < 1e7) { g_z_max *= 10.0; bode_init(g_z_min, g_z_max, F_START_HZ, F_STOP_HZ); }
        if (d0 == -1 && g_z_max > 1e2) { g_z_max /= 10.0; bode_init(g_z_min, g_z_max, F_START_HZ, F_STOP_HZ); }

        int d1 = poll_encoder(ENC1_BASE_BIT, 1);
        if (d1 == 1  && g_z_min < g_z_max / 10.0) { g_z_min *= 10.0; bode_init(g_z_min, g_z_max, F_START_HZ, F_STOP_HZ); }
        if (d1 == -1 && g_z_min > 1e-1)            { g_z_min /= 10.0; bode_init(g_z_min, g_z_max, F_START_HZ, F_STOP_HZ); }
    }
}