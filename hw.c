/*
 * hw.c
 *
 * Integration of physical hardware components and circuit with DE1-SoC FPGA
 * 
 */

#include "hw.h"
#include <math.h>
#include <complex.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#ifdef SIM_MODE

#define SIM_R 470.0 // series resistance of DUT
#define SIM_C 1e-6 // series capacitance of device under test
#define SIM_RREF 1000.0 // reference resistor's resistance value

void hw_init(void)    { fprintf(stderr, "[sim] hw_init\n"); }
void hw_cleanup(void) { fprintf(stderr, "[sim] hw_cleanup\n"); }
void hw_set_freq(double f_hz) { (void)f_hz; }

// Simulation of functionality with preset values for Z_{DUT}
void hw_read_channels(int N, double fs, double f_hz, double *ch_ref, double *ch_dut)
{
    double omega_s = 2.0 * M_PI * f_hz;
    double complex Z_dut = SIM_R + 1.0 / (I * omega_s * SIM_C);
    double complex denom  = SIM_RREF + Z_dut;
    double complex A_ref  = SIM_RREF / denom;
    double complex A_dut  = Z_dut    / denom;

    for (int n = 0; n < N; n++) {
        double phase = omega_s * (double)n / fs;
        ch_ref[n] = cabs(A_ref) * cos(phase + carg(A_ref));
        ch_dut[n] = cabs(A_dut) * cos(phase + carg(A_dut));
    }
}

#else