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

/* Select which TX workflow maintask starts. */
#define UNDERWATERCOMM_TX_TASK_FSK_ENCODER 0
#define UNDERWATERCOMM_TX_TASK_ALTERNATING_TEST 1

#ifndef UNDERWATERCOMM_TX_TASK
#define UNDERWATERCOMM_TX_TASK UNDERWATERCOMM_TX_TASK_FSK_ENCODER
#endif

/*
 * TX bring-up parameters.
 *
 * The RX FSK detector reads the two TX frequency macros below, so keep the
 * transmitter and receiver in the same build from one shared source of truth.
 */
#ifndef UNDERWATERCOMM_TX_DUTY_PERCENT
#define UNDERWATERCOMM_TX_DUTY_PERCENT 50U
#endif

#ifndef UNDERWATERCOMM_TX_DEFAULT_ON_MS
#define UNDERWATERCOMM_TX_DEFAULT_ON_MS 1U
#endif

#ifndef UNDERWATERCOMM_TX_DEFAULT_OFF_MS
#define UNDERWATERCOMM_TX_DEFAULT_OFF_MS 9U
#endif

#ifndef UNDERWATERCOMM_TX_LOW_FREQUENCY_HZ
#define UNDERWATERCOMM_TX_LOW_FREQUENCY_HZ 70000U
#endif

#ifndef UNDERWATERCOMM_TX_HIGH_FREQUENCY_HZ
#define UNDERWATERCOMM_TX_HIGH_FREQUENCY_HZ 80000U
#endif

#ifndef UNDERWATERCOMM_TX_FREQUENCY_SLOT_MS
#define UNDERWATERCOMM_TX_FREQUENCY_SLOT_MS 500U
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

/*
 * RX sweep and smoothing parameters.
 *
 * The sweep must contain the two TX FSK frequencies. With the defaults below,
 * sweep index 4 is 74 kHz and sweep index 7 is 77 kHz.
 */
#ifndef UNDERWATERCOMM_RX_SWEEP_START_HZ
#define UNDERWATERCOMM_RX_SWEEP_START_HZ 70000U
#endif

#ifndef UNDERWATERCOMM_RX_SWEEP_STOP_HZ
#define UNDERWATERCOMM_RX_SWEEP_STOP_HZ 80000U
#endif

#ifndef UNDERWATERCOMM_RX_SWEEP_STEP_HZ
#define UNDERWATERCOMM_RX_SWEEP_STEP_HZ 1000U
#endif

#ifndef UNDERWATERCOMM_RX_SWEEP_FILTER_ALPHA
#define UNDERWATERCOMM_RX_SWEEP_FILTER_ALPHA 0.1F
#endif

/*
 * RX FSK strength calibration.
 *
 * Apply these weights before comparing the configured low/high TX frequency
 * powers. Keep them near 1.0F and only use them to compensate repeatable
 * analog front-end or transducer response differences.
 */
#ifndef UNDERWATERCOMM_RX_FSK_LOW_POWER_WEIGHT
#define UNDERWATERCOMM_RX_FSK_LOW_POWER_WEIGHT 0.85F
#endif

#ifndef UNDERWATERCOMM_RX_FSK_HIGH_POWER_WEIGHT
#define UNDERWATERCOMM_RX_FSK_HIGH_POWER_WEIGHT 1.0F
#endif

/*
 * RX burst detector parameters.
 *
 * The decoder learns the idle noise power and detects a symbol when the
 * combined weighted 74/77 kHz power rises above that noise floor. Separate
 * detect/release ratios provide hysteresis, so one acoustic burst is counted
 * once even when it overlaps two ADC analysis blocks.
 */
#ifndef UNDERWATERCOMM_FSK_RX_DETECT_RATIO
#define UNDERWATERCOMM_FSK_RX_DETECT_RATIO 4.0F
#endif

#ifndef UNDERWATERCOMM_FSK_RX_RELEASE_RATIO
#define UNDERWATERCOMM_FSK_RX_RELEASE_RATIO 1.5F
#endif

#ifndef UNDERWATERCOMM_FSK_RX_MIN_TOTAL_POWER
#define UNDERWATERCOMM_FSK_RX_MIN_TOTAL_POWER 1.0F
#endif

#ifndef UNDERWATERCOMM_FSK_RX_MIN_ABS_SCORE
#define UNDERWATERCOMM_FSK_RX_MIN_ABS_SCORE 0.15F
#endif

#ifndef UNDERWATERCOMM_FSK_RX_NOISE_FILTER_ALPHA
#define UNDERWATERCOMM_FSK_RX_NOISE_FILTER_ALPHA 0.05F
#endif

#ifndef UNDERWATERCOMM_FSK_RX_BIT_TIMEOUT_MS
#define UNDERWATERCOMM_FSK_RX_BIT_TIMEOUT_MS 60U
#endif

/*
 * FSK transmit frame parameters.
 *
 * 0.1 ms/bit would be 10 kbit/s and is too fast for the current 2.048 ms RX
 * analysis frame. Start at 20 ms/bit (50 bit/s), then reduce it after the RX
 * symbol detector and synchronization are working reliably.
 */
#ifndef UNDERWATERCOMM_FSK_BIT_DURATION_MS
#define UNDERWATERCOMM_FSK_BIT_DURATION_MS 20U
#endif

#ifndef UNDERWATERCOMM_FSK_TONE_ON_MS
#define UNDERWATERCOMM_FSK_TONE_ON_MS 1U
#endif

#ifndef UNDERWATERCOMM_FSK_FRAME_GAP_MS
#define UNDERWATERCOMM_FSK_FRAME_GAP_MS 400U
#endif

#ifndef UNDERWATERCOMM_FSK_PREAMBLE_BYTE
#define UNDERWATERCOMM_FSK_PREAMBLE_BYTE 0xAAU
#endif

#ifndef UNDERWATERCOMM_FSK_FIRST_PAYLOAD
#define UNDERWATERCOMM_FSK_FIRST_PAYLOAD 0x00U
#endif

#ifndef UNDERWATERCOMM_FSK_LAST_PAYLOAD
#define UNDERWATERCOMM_FSK_LAST_PAYLOAD 0x04U
#endif

#ifndef UNDERWATERCOMM_FSK_CRC8_POLYNOMIAL
#define UNDERWATERCOMM_FSK_CRC8_POLYNOMIAL 0x07U
#endif

#endif /* UNDERWATERCOMM_CORE_UNDERWATERCOMM_CONFIG_HPP */
