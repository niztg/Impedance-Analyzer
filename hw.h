#ifndef HW_H
#define HW_H

void hw_init(void);
void hw_cleanup(void);
void hw_set_freq(double f_hz);
void hw_read_channels(int N, double fs, double f_hz, double *ch_ref, double *ch_dut);

int  hw_read_jp1(void);

#endif