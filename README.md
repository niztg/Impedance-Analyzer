# Swept-Frequency Impedance Analyzer

Measures complex impedance Z(f) across the audio band (10 Hz – 20 kHz) using coherent
lock-in detection on a DE1-SoC FPGA board. Produces a live Bode plot on VGA and
estimates component model (series RC, RL, or resistive) with extracted R, C/L, and f_c.

## Hardware

| Component | Purpose |
|---|---|
| AD9833 waveform generator | Sine wave stimulus |
| MCP3202 12-bit SPI ADC | Two-channel voltage sampling |
| OPA2134PA dual op-amp | Unity-gain input buffers |
| 1 kΩ 0.1% thin-film resistor | Reference impedance R_ref |
| DE1-SoC FPGA (RISC-V soft core) | Goertzel DSP + VGA display |

## Circuit Topology

AD9833 drives a series voltage divider: R_ref on top, Z_dut on bottom, referenced to
ground. Two nodes are tapped:

- V_top — between AD9833 and R_ref → ADC ch1 (via OPA2134 buffer)
- V_mid — between R_ref and Z_dut → ADC ch0 (via OPA2134 buffer)

The op-amp buffers prevent the ADC input impedance from loading the divider.

Impedance is recovered from the phasor ratio:

    Z_dut = V_mid / (V_top - V_mid) * R_ref

V_in cancels in the ratio so the AD9833 output amplitude does not need to be calibrated.
The N/2 Goertzel scaling factor is identical on both channels and also cancels.

## Files

| File | Contents |
|---|---|
| `main.c` | Sweep loop, Goertzel, impedance ratio, VGA Bode display, model extraction |
| `hw.c` | Hardware abstraction — AD9833 SPI, MCP3202 SPI, sim path |
| `hw.h` | Four-function interface: `hw_init`, `hw_cleanup`, `hw_set_freq`, `hw_read_channels`, `hw_read_jp1` |

## Build

Edit `USERCCFLAGS` in the Makefile and set:

```makefile
SRCS := main.c hw.c
HDRS := hw.h
```

**Sim build** (runs on board, uses synthetic RC model, no analog hardware needed):
```makefile
USERCCFLAGS := -g -O1 -ffunction-sections -fverbose-asm -fno-inline -gdwarf-2 -DSIM_MODE
```

**Hardware build** (remove `-DSIM_MODE`):
```makefile
USERCCFLAGS := -g -O1 -ffunction-sections -fverbose-asm -fno-inline -gdwarf-2
```

Program the `.sof`, then load and run the `.elf` via GDB — identical flow to the signal classifier project.

## First Run Checklist

1. Verify `PIN_SCLK`, `PIN_MOSI`, `PIN_MISO`, `PIN_CS_DAC`, `PIN_CS_ADC` in `hw.c` match actual wiring to JP1
2. Verify `AD9833_MCLK_HZ` matches the crystal on your AD9833 module (likely 25 MHz)
3. Run sim build first — confirm Bode plot appears and readout shows ~470 Ω, ~1 µF, ~340 Hz
4. Run hardware build with raw ADC prints enabled — confirm MCP3202 returns non-zero values on both channels before running full sweep
5. Run full hardware sweep

## Usage

On boot the Bode plot axes draw and the display shows `Press enc0 to sweep`.

| Control | Action |
|---|---|
| Encoder 0 press | Trigger sweep |
| Encoder 0 turn | Scale Z_max by one decade |
| Encoder 1 turn | Scale Z_min by one decade |

After a sweep completes, the magnitude (green) and phase (magenta) curves appear and
the bottom strip shows the extracted model, e.g.:

    Model: Series RC    R: 470Ohm    C: 1.02uF    fc: 340Hz

## Key Implementation Notes

**Adaptive N per bin** — window length is set to `N = round(CYCLES_PER_BIN * fs / f)`
to capture a whole number of cycles at each frequency, eliminating spectral leakage at
low frequencies.

**Ratiometric measurement** — Z_dut is recovered from a voltage ratio so V_in cancels.
Absolute ADC calibration is not required.

**SOL calibration** — not yet implemented. A short-open-load calibration using a
precision 1 kΩ reference will be added to remove systematic front-end errors and
improve accuracy toward the <0.5% magnitude / <0.5° phase target.

**Bit-banged SPI** — both AD9833 and MCP3202 are driven via bit-banging on JP1 GPIO.
Timing jitter from the OS is manageable at audio-band frequencies given the long
Goertzel integration windows.