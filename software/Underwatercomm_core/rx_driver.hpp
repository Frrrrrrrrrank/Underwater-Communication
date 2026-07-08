#ifndef UNDERWATERCOMM_CORE_RX_DRIVER_HPP
#define UNDERWATERCOMM_CORE_RX_DRIVER_HPP

#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "stm32g4xx_hal.h"
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

  // Copy out the most recent FFT result for debugger/UI/telemetry use.
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
