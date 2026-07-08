#ifndef UNDERWATERCOMM_CORE_UNDERWATERCOMM_CONFIG_HPP
#define UNDERWATERCOMM_CORE_UNDERWATERCOMM_CONFIG_HPP

/*
 * Build-time role selection.
 *
 * The acoustic front end is treated as half-duplex during bring-up:
 *   - TX firmware drives the 75 kHz power path only.
 *   - RX firmware samples ADC data and runs the frequency analysis only.
 *
 * Keeping these roles separate avoids measuring our own transmit leakage and
 * makes the startup beep identify which firmware image is currently flashed.
 */
#define UNDERWATERCOMM_BUILD_MODE_RX 0
#define UNDERWATERCOMM_BUILD_MODE_TX 1

/* Change this one line when switching between receiver and transmitter tests. */
#ifndef UNDERWATERCOMM_BUILD_MODE
#define UNDERWATERCOMM_BUILD_MODE UNDERWATERCOMM_BUILD_MODE_RX
#endif

/*
 * RX bring-up switch.
 *
 * Enable ADC DMA separately from FFT so HardFaults can be isolated in steps.
 *
 * Test order:
 *   1. DMA=0, FFT=0: startup + buzzer only.
 *   2. DMA=1, FFT=0: ADC raw circular DMA only.
 *   3. DMA=1, FFT=1: ADC raw DMA + frequency analysis.
 */
#ifndef UNDERWATERCOMM_ENABLE_RX_DMA
#define UNDERWATERCOMM_ENABLE_RX_DMA 1
#endif

/*
 * RX FFT switch.
 *
 * 0: keep only raw ADC min/max/average processing.
 * 1: Run CMSIS-DSP 1024-point FFT on each ready DMA half-buffer.
 */
#ifndef UNDERWATERCOMM_ENABLE_RX_FFT
#define UNDERWATERCOMM_ENABLE_RX_FFT 0
#endif

/*
 * RX Goertzel switch.
 *
 * Goertzel is safer for first communication tests than a full FFT because it
 * measures only the frequency bins we care about and does not depend on
 * CMSIS-DSP. Keep this enabled for 75 kHz link bring-up.
 */
#ifndef UNDERWATERCOMM_ENABLE_RX_GOERTZEL
#define UNDERWATERCOMM_ENABLE_RX_GOERTZEL 1
#endif

/*
 * RX Goertzel sweep switch.
 *
 * 0: measure only the exact 75 kHz target bin.
 * 1: also sweep a small band around 75 kHz and report the strongest bin as
 *    dominant_frequency_hz. This gives a simple frequency estimate without
 *    enabling the full FFT path that currently causes HardFault.
 */
#ifndef UNDERWATERCOMM_ENABLE_RX_GOERTZEL_SWEEP
#define UNDERWATERCOMM_ENABLE_RX_GOERTZEL_SWEEP 1
#endif

#endif /* UNDERWATERCOMM_CORE_UNDERWATERCOMM_CONFIG_HPP */
