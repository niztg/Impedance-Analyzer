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

#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif

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

#define JP1_BASE                0xFF200060

// pin definitions
#define PIN_SCLK    0 // universal clock
#define PIN_MOSI    1 // Master Out, Slave In: for the FPGA to send a command or frequency word to the AD9833
#define PIN_MISO    2 // Master In, Slave Out: for the MCP3202 to send data back to the FPGA
#define PIN_CS_DAC  3   // active low enable for the AD9833
#define PIN_CS_ADC  4   // active low enable for the MCP3202

#define AD9833_MCLK_HZ  25000000.0

static volatile int *jp1_ptr = (volatile int *)JP1_BASE;
static volatile int *jp1_dir_ptr = (volatile int *)(JP1_BASE + 4); 

static void gpio_set(int pin, int val){
    if (val) *jp1_ptr |=  (1 << pin); // assert bit at `pin` while leaving all other bits identical
    else     *jp1_ptr &= ~(1 << pin); // lower bit at `pin` while leaving all other bits identical
}

static int gpio_get(int pin){
    return (*jp1_ptr >> pin) & 1; // return the value of the bit at `pin`
}

static void spi_delay(void){
    struct timespec ts = {0, 100};
    nanosleep(&ts, NULL);
}

/*
AD9833 (DAC) functions
*/

// Template to write a value into the AD9833
static void ad9833_write(uint16_t word){
    gpio_set(PIN_CS_DAC, 0);
    for (int i = 0; i < 15; i++){ // bit-banging: toggling one bit per iteration
        gpio_set(PIN_SCLK, 0);
        gpio_set(PIN_MOSI, (word >> i) & 1); // to the AD9833: send the i-th bit of the word argument
        spi_delay();
        gpio_set(PIN_SCLK, 1);
        spi_delay();
    }
    gpio_set(PIN_SCLK, 0);
    gpio_set(PIN_CS_DAC, 1);
}

void hw_set_freq(double f_hz){
    uint32_t freq_word = (uint32_t)round(f_hz * (1 << 28) / AD9833_MCLK_HZ); // freq_word = f_hz * 2^28 / f_MCLK

    // in both lsb and msb, bit 14 must be set high
    uint16_t lsb = (uint16_t)( freq_word        & 0x3FFF) | 0x4000; // bottom 14 bits of freq_word
    uint16_t msb = (uint16_t)((freq_word >> 14)  & 0x3FFF) | 0x4000; // top 14 bits of freq_word

    ad9833_write_reg(0x2100); // sets bit 13 (enables a 28 bit address to be written in two consecutive writes) and bit 8 (steadies the AD9833 inbetween these writes)
    ad9833_write_reg(lsb);
    ad9833_write_reg(msb);

    struct timespec ts = {0, (long)(1e9 / f_hz)};
    nanosleep(&ts, NULL);
}

/*
MCP3202 (ADC) functions
*/
static uint16_t mcp3202_read(int channel){
    uint16_t result = 0;
    int odd = channel & 1;

    gpio_set(PIN_CS_ADC, 0); // enable
    spi_delay();

    gpio_set(PIN_MOSI, 1); // start bit, initiates conversion                              
    gpio_set(PIN_SCLK, 1); spi_delay();
    gpio_set(PIN_SCLK, 0); spi_delay();

    gpio_set(PIN_MOSI, 1); // SGL = 1, asserts single-channel mode                            
    gpio_set(PIN_SCLK, 1); spi_delay();
    gpio_set(PIN_SCLK, 0); spi_delay();

    gpio_set(PIN_MOSI, odd); // channel select                         
    gpio_set(PIN_SCLK, 1); spi_delay();
    gpio_set(PIN_SCLK, 0); spi_delay();

    gpio_set(PIN_MOSI, 1); // MSBF = 1, "most significant bit first", arbitrary                                
    gpio_set(PIN_SCLK, 1); spi_delay();
    gpio_set(PIN_SCLK, 0); spi_delay();

    for (int i = 11; i >= 0; i--) {
        gpio_set(PIN_SCLK, 1); spi_delay(); // clocks to put the value of the i-th bit on the MISO pin
        result |= (uint16_t)(gpio_get(PIN_MISO) << i); // stores the value in the i-th bit of the local result variable
        gpio_set(PIN_SCLK, 0); spi_delay();
    }

    gpio_set(PIN_CS_ADC, 1);
    return result;
}

void hw_read_channels(int N, double fs, double f_hz, double* ch_ref, double* ch_dut){
    (void) f_hz; // don't use this variable, the function has to have the same function signature as the sim version
    long period_ns = (long) 1e9 / fs;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    for (int n = 0; n < N; n++){
        // the MCP3202 returns a 12 bit number up to 4095
        // 4096.0 normalizes this to a number between 0 and 1
        // because we always take the ratio of the two channels, the  value normalization factor is irrelevant
        ch_ref[n] = (double)mcp3202_read(0) / 4096.0;
        ch_dut[n] = (double)mcp3202_read(1) / 4096.0;

        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1000000000L){
            next.tv_nsec -= 1000000000L;
            next.tv_nsec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
}

void hw_init(void){
    // Set all SPI pins as outputs except MISO
    *jp1_dir_ptr |=  (1 << PIN_SCLK) | (1 << PIN_MOSI) |
                     (1 << PIN_CS_DAC) | (1 << PIN_CS_ADC);
    *jp1_dir_ptr &= ~(1 << PIN_MISO);

    gpio_set(PIN_CS_DAC, 1);
    gpio_set(PIN_CS_ADC, 1);
    gpio_set(PIN_SCLK,   0);

    ad9833_write_reg(0x2100);  // reset AD9833
}

void hw_cleanup(void) { /* nothing to close */ }

#endif