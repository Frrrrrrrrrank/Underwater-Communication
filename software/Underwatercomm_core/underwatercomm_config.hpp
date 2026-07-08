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
#define UNDERWATERCOMM_ENABLE_RX_DMA 0
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

#endif /* UNDERWATERCOMM_CORE_UNDERWATERCOMM_CONFIG_HPP */
