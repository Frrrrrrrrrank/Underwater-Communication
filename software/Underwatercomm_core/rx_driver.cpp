#include "rx_driver.hpp"

#include "underwatercomm_config.hpp"

#include <math.h>

#if UNDERWATERCOMM_ENABLE_RX_FFT
extern "C" {
#include "arm_math.h"
}
#endif

namespace underwatercomm {
namespace {

constexpr uint32_t SAMPLE_RATE_HZ = 500000U;
constexpr uint32_t FFT_SAMPLE_COUNT = 1024U;
constexpr uint32_t ADC_DMA_SAMPLE_COUNT = FFT_SAMPLE_COUNT * 2U;
constexpr uint32_t TARGET_FREQUENCY_HZ = 75000U;
#if UNDERWATERCOMM_ENABLE_RX_FFT
constexpr uint32_t MIN_ANALYSIS_FREQUENCY_HZ = 20000U;
constexpr uint32_t MAX_ANALYSIS_FREQUENCY_HZ = 180000U;
constexpr float PI_F = 3.14159265358979323846F;
#endif

enum class ReadyBlock : uint8_t {
  NONE,
  FIRST_HALF,
  SECOND_HALF,
};

// DMA writes 2048 raw ADC samples in circular mode. Each half-buffer contains
// one complete 1024-point analysis frame, so the main loop can process one half
// while DMA fills the other half.
alignas(4) uint16_t g_adc_raw_buffer[ADC_DMA_SAMPLE_COUNT];

#if UNDERWATERCOMM_ENABLE_RX_FFT
// FFT working buffers are static to keep large arrays off the call stack.
float g_fft_input[FFT_SAMPLE_COUNT];
float g_fft_output[FFT_SAMPLE_COUNT];
float g_window[FFT_SAMPLE_COUNT];
arm_rfft_fast_instance_f32 g_fft_instance;
#endif

ADC_HandleTypeDef *g_adc = nullptr;
TIM_HandleTypeDef *g_trigger_timer = nullptr;
volatile ReadyBlock g_ready_block = ReadyBlock::NONE;
volatile bool g_dma_overrun = false;
bool g_started = false;
#if UNDERWATERCOMM_ENABLE_RX_FFT
bool g_fft_ready = false;
#endif
bool g_has_result = false;
uint32_t g_block_counter = 0U;
RxAnalysisResult g_latest_result = {};

#if UNDERWATERCOMM_ENABLE_RX_FFT
uint32_t FrequencyToBin(uint32_t frequency_hz) {
  // Nearest FFT bin. With 500 kS/s and 1024 samples, each bin is 488.28125 Hz.
  return ((frequency_hz * FFT_SAMPLE_COUNT) + (SAMPLE_RATE_HZ / 2U)) /
         SAMPLE_RATE_HZ;
}

float BinToFrequency(float bin) {
  return (bin * static_cast<float>(SAMPLE_RATE_HZ)) /
         static_cast<float>(FFT_SAMPLE_COUNT);
}

float MagnitudeSquaredForBin(uint32_t bin) {
  // CMSIS real FFT packs bin 0 at output[0] and Nyquist at output[1].
  // Other bins are interleaved real/imag pairs.
  if (bin == 0U) {
    return g_fft_output[0] * g_fft_output[0];
  }

  if (bin >= (FFT_SAMPLE_COUNT / 2U)) {
    return g_fft_output[1] * g_fft_output[1];
  }

  const float real = g_fft_output[2U * bin];
  const float imag = g_fft_output[(2U * bin) + 1U];
  return (real * real) + (imag * imag);
}

float NormalizedMagnitude(uint32_t bin) {
  return sqrtf(MagnitudeSquaredForBin(bin)) /
         static_cast<float>(FFT_SAMPLE_COUNT);
}

float InterpolatePeakBin(uint32_t peak_bin) {
  if ((peak_bin <= 1U) || (peak_bin >= ((FFT_SAMPLE_COUNT / 2U) - 2U))) {
    return static_cast<float>(peak_bin);
  }

  const float left = MagnitudeSquaredForBin(peak_bin - 1U);
  const float center = MagnitudeSquaredForBin(peak_bin);
  const float right = MagnitudeSquaredForBin(peak_bin + 1U);
  const float denominator = left - (2.0F * center) + right;

  if (denominator == 0.0F) {
    return static_cast<float>(peak_bin);
  }

  const float offset = 0.5F * (left - right) / denominator;
  return static_cast<float>(peak_bin) + offset;
}

void BuildHannWindow() {
  // Windowing reduces spectral leakage. This is useful while the incoming
  // acoustic burst is not exactly aligned to the 1024-sample FFT frame.
  for (uint32_t i = 0U; i < FFT_SAMPLE_COUNT; ++i) {
    const float phase =
        (2.0F * PI_F * static_cast<float>(i)) /
        static_cast<float>(FFT_SAMPLE_COUNT - 1U);
    g_window[i] = 0.5F - (0.5F * cosf(phase));
  }
}
#endif

void MarkBlockReady(ADC_HandleTypeDef *adc, ReadyBlock block) {
  // This runs inside the DMA interrupt callback, so do the minimum work here:
  // just record which half is ready. The FFT stays in Process().
  if ((adc != g_adc) || !g_started) {
    return;
  }

  if (g_ready_block != ReadyBlock::NONE) {
    // The main loop did not process the previous half-buffer before another
    // DMA callback arrived. Keep running, but expose this in the result.
    g_dma_overrun = true;
  }

  g_ready_block = block;
}

void AnalyzeBlock(const volatile uint16_t *samples, bool dma_overrun) {
  uint32_t min_raw = 0xFFFFU;
  uint32_t max_raw = 0U;
  uint64_t sum_raw = 0U;

  for (uint32_t i = 0U; i < FFT_SAMPLE_COUNT; ++i) {
    const uint32_t raw = samples[i];
    sum_raw += raw;

    if (raw < min_raw) {
      min_raw = raw;
    }

    if (raw > max_raw) {
      max_raw = raw;
    }
  }

  const float average_raw =
      static_cast<float>(sum_raw) / static_cast<float>(FFT_SAMPLE_COUNT);

  // Raw statistics are always updated. Watch these first when validating ADC DMA.
  g_latest_result.block_counter = ++g_block_counter;
  g_latest_result.sample_rate_hz = SAMPLE_RATE_HZ;
  g_latest_result.sample_count = FFT_SAMPLE_COUNT;
  g_latest_result.min_raw = static_cast<uint16_t>(min_raw);
  g_latest_result.max_raw = static_cast<uint16_t>(max_raw);
  g_latest_result.average_raw = average_raw;

#if UNDERWATERCOMM_ENABLE_RX_FFT
  // Remove DC before FFT. The receiver amplifier biases the ADC signal, and a
  // strong DC bin would otherwise hide the useful nearby bins in debug views.
  for (uint32_t i = 0U; i < FFT_SAMPLE_COUNT; ++i) {
    const float centered_sample = static_cast<float>(samples[i]) - average_raw;
    g_fft_input[i] = centered_sample * g_window[i];
  }

  arm_rfft_fast_f32(&g_fft_instance, g_fft_input, g_fft_output, 0);

  uint32_t start_bin = FrequencyToBin(MIN_ANALYSIS_FREQUENCY_HZ);
  uint32_t stop_bin = FrequencyToBin(MAX_ANALYSIS_FREQUENCY_HZ);
  const uint32_t last_usable_bin = (FFT_SAMPLE_COUNT / 2U) - 1U;

  if (start_bin < 1U) {
    start_bin = 1U;
  }

  if (stop_bin > last_usable_bin) {
    stop_bin = last_usable_bin;
  }

  uint32_t peak_bin = start_bin;
  float peak_magnitude_squared = 0.0F;

  for (uint32_t bin = start_bin; bin <= stop_bin; ++bin) {
    const float magnitude_squared = MagnitudeSquaredForBin(bin);

    if (magnitude_squared > peak_magnitude_squared) {
      peak_magnitude_squared = magnitude_squared;
      peak_bin = bin;
    }
  }

  const uint32_t target_bin = FrequencyToBin(TARGET_FREQUENCY_HZ);
  const float interpolated_peak_bin = InterpolatePeakBin(peak_bin);

  g_latest_result.dominant_frequency_hz =
      BinToFrequency(interpolated_peak_bin);
  g_latest_result.dominant_magnitude =
      sqrtf(peak_magnitude_squared) / static_cast<float>(FFT_SAMPLE_COUNT);
  g_latest_result.target_frequency_hz =
      BinToFrequency(static_cast<float>(target_bin));
  // This is the energy at the nearest 75 kHz bin, not necessarily the global
  // peak. It is useful for Goertzel-style threshold tests later.
  g_latest_result.target_magnitude = NormalizedMagnitude(target_bin);
#else
  // Raw-capture bring-up mode. Leave frequency fields at zero so the debugger
  // can still inspect min/max/average without running CMSIS-DSP yet.
  g_latest_result.dominant_frequency_hz = 0.0F;
  g_latest_result.dominant_magnitude = 0.0F;
  g_latest_result.target_frequency_hz = static_cast<float>(TARGET_FREQUENCY_HZ);
  g_latest_result.target_magnitude = 0.0F;
#endif

  g_latest_result.dma_overrun = dma_overrun;
  g_has_result = true;
}

}  // namespace

bool RxDriver::Init(ADC_HandleTypeDef *adc, TIM_HandleTypeDef *trigger_timer) {
  if ((adc == nullptr) || (trigger_timer == nullptr)) {
    return false;
  }

  if (g_started) {
    return true;
  }

#if UNDERWATERCOMM_ENABLE_RX_FFT
  if (!g_fft_ready) {
    if (arm_rfft_fast_init_f32(&g_fft_instance, FFT_SAMPLE_COUNT) !=
        ARM_MATH_SUCCESS) {
      return false;
    }

    BuildHannWindow();
    g_fft_ready = true;
  }
#endif

  g_adc = adc;
  g_trigger_timer = trigger_timer;
  g_ready_block = ReadyBlock::NONE;
  g_dma_overrun = false;
  g_has_result = false;
  g_block_counter = 0U;

  // Calibrate before starting DMA. If this fails, the ADC is not safe to use yet.
  if (HAL_ADCEx_Calibration_Start(g_adc, ADC_SINGLE_ENDED) != HAL_OK) {
    return false;
  }

  if (HAL_ADC_Start_DMA(g_adc, reinterpret_cast<uint32_t *>(g_adc_raw_buffer),
                        ADC_DMA_SAMPLE_COUNT) != HAL_OK) {
    return false;
  }

  g_started = true;

  // TIM6 TRGO now starts the fixed-rate ADC conversions.
  if (HAL_TIM_Base_Start(g_trigger_timer) != HAL_OK) {
    (void)HAL_ADC_Stop_DMA(g_adc);
    g_started = false;
    return false;
  }

  return true;
}

bool RxDriver::Stop() {
  HAL_StatusTypeDef timer_status = HAL_OK;
  HAL_StatusTypeDef adc_status = HAL_OK;

  if (g_trigger_timer != nullptr) {
    timer_status = HAL_TIM_Base_Stop(g_trigger_timer);
  }

  if (g_adc != nullptr) {
    adc_status = HAL_ADC_Stop_DMA(g_adc);
  }

  __disable_irq();
  g_ready_block = ReadyBlock::NONE;
  g_dma_overrun = false;
  g_started = false;
  __enable_irq();

  return (timer_status == HAL_OK) && (adc_status == HAL_OK);
}

void RxDriver::Process() {
  ReadyBlock block = ReadyBlock::NONE;
  bool dma_overrun = false;

  __disable_irq();
  block = g_ready_block;
  g_ready_block = ReadyBlock::NONE;
  dma_overrun = g_dma_overrun;
  g_dma_overrun = false;
  __enable_irq();

  if (block == ReadyBlock::NONE) {
    return;
  }

  const uint32_t offset =
      (block == ReadyBlock::FIRST_HALF) ? 0U : FFT_SAMPLE_COUNT;
  AnalyzeBlock(&g_adc_raw_buffer[offset], dma_overrun);
}

bool RxDriver::GetLatestResult(RxAnalysisResult *result) {
  if ((result == nullptr) || !g_has_result) {
    return false;
  }

  *result = g_latest_result;
  return true;
}

const volatile uint16_t *RxDriver::GetRawBuffer() {
  return g_adc_raw_buffer;
}

size_t RxDriver::GetRawBufferLength() {
  return ADC_DMA_SAMPLE_COUNT;
}

void RxDriver::OnAdcHalfComplete(ADC_HandleTypeDef *adc) {
  MarkBlockReady(adc, ReadyBlock::FIRST_HALF);
}

void RxDriver::OnAdcComplete(ADC_HandleTypeDef *adc) {
  MarkBlockReady(adc, ReadyBlock::SECOND_HALF);
}

}  // namespace underwatercomm

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  underwatercomm::RxDriver::OnAdcHalfComplete(hadc);
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  underwatercomm::RxDriver::OnAdcComplete(hadc);
}
