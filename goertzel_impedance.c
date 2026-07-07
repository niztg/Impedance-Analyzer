/*
 * goertzel_impedance.c
 *
 * Swept-frequency impedance analyzer — Goertzel core for HPS (DE1-SoC).
 * Validates against MATLAB prototype before hardware integration.
 *
 * Build:
 *   gcc -O2 -Wall goertzel_impedance.c -lm -o goertzel_impedance
 *
 * Validation mode (CSV output to stdout, pipe into MATLAB/Python for comparison):
 *   ./goertzel_impedance > results.csv
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <complex.h>   /* C99: double complex, creal(), cimag(), cabs(), carg() */


#define SAMPLE_RATE_HZ   44100.0
#define N_SAMPLES        2048
#define R_REF_OHMS       1000.0

/* Frequency sweep: logarithmically spaced, 10 Hz – 20 kHz */
#define F_START_HZ       10.0
#define F_STOP_HZ        20000.0
#define N_FREQS          50


// Computes Goertzel algorithm on given array of samples (DT signal) x
// returns X(e^{jω}) = (AN / 2) e{j phi}
static double complex goertzel(const double *x, int N, double omega)
{
    double coeff = 2.0 * cos(omega);
    double w0 = 0.0, w1 = 0.0, w2 = 0.0;

    for (int n = 0; n < N; n++) {
        w0 = x[n] + coeff * w1 - w2;
        w2 = w1;
        w1 = w0;
    }

    /* At loop exit: w1 = w[N-1], w2 = w[N-2] */
    double complex ejw = cos(omega) - I * sin(omega);   /* e^{−jω} */
    return w1 - ejw * w2;
}

// Calculates the impedance of the unknown element by using voltage division
// V_{DUT} / V_{ref} = X_{DUT} / X_{REF} = Z(f) / R_{ref}
static double complex impedance(const double *ch_dut,
                                const double *ch_ref,
                                int N,
                                double omega,
                                double r_ref)
{
    double complex X_dut = goertzel(ch_dut, N, omega);
    double complex X_ref = goertzel(ch_ref, N, omega);
    return (X_dut / X_ref) * r_ref;
}

static void logspace(double f0, double f1, int n, double *out)
{
    double log_f0 = log10(f0);
    double log_f1 = log10(f1);
    for (int i = 0; i < n; i++) {
        out[i] = pow(10.0, log_f0 + (log_f1 - log_f0) * i / (n - 1));
    }
}


static void sim_series_RC(double f_hz,
                           double R_s, double C,
                           double r_ref, double fs,
                           int N,
                           double *ch_ref, double *ch_dut)
{
    double omega_s = 2.0 * M_PI * f_hz;
    double complex Z_dut = R_s + 1.0 / (I * omega_s * C);
    double complex denom  = r_ref + Z_dut;

    double complex A_ref = r_ref / denom;
    double complex A_dut = Z_dut / denom;

    for (int n = 0; n < N; n++) {
        double phase = omega_s * (double)n / fs;
        ch_ref[n] = cabs(A_ref) * cos(phase + carg(A_ref));
        ch_dut[n] = cabs(A_dut) * cos(phase + carg(A_dut));
    }
}


int main(void)
{
    const double R_s = 470.0;
    const double C   = 1e-6;
    const double fs  = SAMPLE_RATE_HZ;
    const int    N   = N_SAMPLES;

    double freqs[N_FREQS];
    logspace(F_START_HZ, F_STOP_HZ, N_FREQS, freqs);

    double *ch_ref = malloc(N * sizeof(double));
    double *ch_dut = malloc(N * sizeof(double));
    if (!ch_ref || !ch_dut) {
        fprintf(stderr, "malloc failed\n");
        return 1;
    }

    printf("f_hz,Z_mag_meas,Z_phase_deg_meas,Z_mag_true,Z_phase_deg_true\n");

    for (int i = 0; i < N_FREQS; i++) {
        double f     = freqs[i];
        double omega = 2.0 * M_PI * f / fs;

        sim_series_RC(f, R_s, C, R_REF_OHMS, fs, N, ch_ref, ch_dut);

        double complex Z_meas = impedance(ch_dut, ch_ref, N, omega, R_REF_OHMS);
        double complex Z_true = R_s + 1.0 / (I * 2.0 * M_PI * f * C);

        printf("%.6f,%.8f,%.8f,%.8f,%.8f\n",
               f,
               cabs(Z_meas),  carg(Z_meas) * 180.0 / M_PI,
               cabs(Z_true),  carg(Z_true) * 180.0 / M_PI);
    }

    free(ch_ref);
    free(ch_dut);
    return 0;
}