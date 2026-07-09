#ifndef UNDERWATERCOMM_CORE_RX_DRIVER_HPP
#define UNDERWATERCOMM_CORE_RX_DRIVER_HPP

#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "stm32g4xx_hal.h"

// Debug array size for the Goertzel sweep around the 75 kHz carrier.
// Current sweep points are 70, 71, 72, ... 80 kHz.
#define UNDERWATERCOMM_RX_DEBUG_SWEEP_BIN_COUNT 11U

/*
 * Debugger-visible RX state.
 *
 * Watch g_underwatercomm_rx_debug in STM32CubeIDE/Ozone/J-Link while RX raw
 * mode is running. If ADC DMA is working, half/full callback counters should
 * keep increasing and min/max/average should reflect the PB11 ADC input.
 *
 * Frequency analysis fields:
 *   target_*          : exact 75 kHz detector result.
 *   dominant_*        : strongest point in the current Goertzel sweep frame.
 *   filtered_dominant : strongest point after first-order sweep smoothing.
 *   sweep_*[]         : per-frequency strengths from 70 kHz to 80 kHz.
 *   filtered_sweep_*[]: smoothed per-frequency strengths for stable debug.
 */
typedef struct {
  uint32_t started;
  uint32_t sample_rate_hz;
  uint32_t raw_buffer_address;
  uint32_t raw_buffer_length;
  uint32_t half_callback_count;
  uint32_t full_callback_count;
  uint32_t processed_block_count;
  uint16_t first_raw;
  uint16_t last_raw;
  uint16_t min_raw;
  uint16_t max_raw;
  float average_raw;
  float dominant_frequency_hz;
  float dominant_magnitude;
  float dominant_power;
  float filtered_dominant_frequency_hz;
  float filtered_dominant_magnitude;
  float filtered_dominant_power;
  float target_frequency_hz;
  float target_magnitude;
  float target_power;
  uint32_t sweep_bin_count;
  uint32_t sweep_frequency_hz[UNDERWATERCOMM_RX_DEBUG_SWEEP_BIN_COUNT];
  float sweep_magnitude[UNDERWATERCOMM_RX_DEBUG_SWEEP_BIN_COUNT];
  float sweep_power[UNDERWATERCOMM_RX_DEBUG_SWEEP_BIN_COUNT];
  float filtered_sweep_magnitude[UNDERWATERCOMM_RX_DEBUG_SWEEP_BIN_COUNT];
  float filtered_sweep_power[UNDERWATERCOMM_RX_DEBUG_SWEEP_BIN_COUNT];
  uint8_t dma_overrun;
  uint8_t has_result;
} UnderwaterComm_RxDebugData;

extern volatile UnderwaterComm_RxDebugData g_underwatercomm_rx_debug;
}

namespace underwatercomm {

struct RxAnalysisResult {
  uint32_t block_counter;
  uint32_t sample_rate_hz;
  uint32_t sample_count;
  uint16_t min_raw;
  uint16_t max_raw;
  float average_raw;
  float dominant_frequency_hz;
  float dominant_magnitude;
  float target_frequency_hz;
  float target_magnitude;
  bool dma_overrun;
};

class RxDriver final {
 public:
  // Start ADC1 DMA capture.
  //
  // TIM6 is configured by CubeMX as the ADC trigger source. DMA is started
  // first, then TIM6 is started, so the first trigger already has a valid
  // destination buffer.
  static bool Init(ADC_HandleTypeDef *adc, TIM_HandleTypeDef *trigger_timer);

  // Stop TIM6 and ADC DMA before entering TX mode.
  static bool Stop();

  // Analyze one ready DMA half-buffer, if available. Call from the main loop.
  static void Process();

  // Copy out the most recent analysis result for debugger/UI/telemetry use.
  // With FFT disabled this result comes from the Goertzel detector instead.
  static bool GetLatestResult(RxAnalysisResult *result);

  // Raw circular DMA buffer. It is updated continuously by DMA.
  static const volatile uint16_t *GetRawBuffer();
  static size_t GetRawBufferLength();

  // Called from HAL ADC DMA callbacks.
  static void OnAdcHalfComplete(ADC_HandleTypeDef *adc);
  static void OnAdcComplete(ADC_HandleTypeDef *adc);
};

}  // namespace underwatercomm

#endif /* UNDERWATERCOMM_CORE_RX_DRIVER_HPP */
