#include "tx_driver.hpp"

#include "underwatercomm_config.hpp"

namespace underwatercomm {
namespace {

// HRTIM1 Timer A output TA1 is routed to PA8 in the CubeMX configuration.
constexpr uint32_t TX_TIMER = HRTIM_TIMERID_TIMER_A;
constexpr uint32_t TX_OUTPUT = HRTIM_OUTPUT_TA1;
constexpr uint32_t TX_TIMER_INDEX = HRTIM_TIMERINDEX_TIMER_A;

// CubeMX config uses the 170 MHz HRTIM clock with the HRTIM x16 multiplier.
// Runtime period changes let us test nearby acoustic frequencies without
// changing the IOC file.
constexpr uint32_t HRTIM_BASE_CLOCK_HZ = 170000000U;
constexpr uint32_t HRTIM_CLOCK_MULTIPLIER = 16U;
constexpr uint32_t HRTIM_EFFECTIVE_CLOCK_HZ =
    HRTIM_BASE_CLOCK_HZ * HRTIM_CLOCK_MULTIPLIER;

// Tunable TX parameters live in underwatercomm_config.hpp so the RX detector
// and TX waveform always use the same FSK frequencies.
constexpr uint32_t TX_DUTY_PERCENT = UNDERWATERCOMM_TX_DUTY_PERCENT;
constexpr uint32_t DEFAULT_TX_ON_MS = UNDERWATERCOMM_TX_DEFAULT_ON_MS;
constexpr uint32_t DEFAULT_TX_OFF_MS = UNDERWATERCOMM_TX_DEFAULT_OFF_MS;
constexpr uint32_t LOW_TEST_FREQUENCY_HZ =
    UNDERWATERCOMM_TX_LOW_FREQUENCY_HZ;
constexpr uint32_t HIGH_TEST_FREQUENCY_HZ =
    UNDERWATERCOMM_TX_HIGH_FREQUENCY_HZ;
constexpr uint32_t FREQUENCY_SLOT_MS =
    UNDERWATERCOMM_TX_FREQUENCY_SLOT_MS;

enum class BurstState {
  IDLE,
  TX_ON,
  TX_OFF,
  FAULT,
};

struct BurstControl {
  // Burst timing is deliberately stored here so it can be changed by one call
  // to StartBurstSend() without touching main.c.
  uint32_t tx_on_ms = 1U;
  uint32_t tx_off_ms = 100U;
  uint32_t last_transition_ms = 0U;
  uint32_t pattern_start_ms = 0U;
  uint32_t active_frequency_hz = LOW_TEST_FREQUENCY_HZ;
  bool alternate_frequency = false;
  BurstState state = BurstState::IDLE;
};

BurstControl g_burst;

uint32_t PeriodTicksForFrequency(uint32_t frequency_hz) {
  if (frequency_hz == 0U) {
    return 1U;
  }

  // Rounded integer division keeps the selected test frequencies close to their requested
  // values while staying inside the 16-bit HRTIM period range.
  return (HRTIM_EFFECTIVE_CLOCK_HZ + (frequency_hz / 2U)) / frequency_hz;
}

uint32_t CompareTicksForPeriod(uint32_t period_ticks) {
  uint32_t compare_ticks =
      ((period_ticks * TX_DUTY_PERCENT) + 50U) / 100U;

  if (compare_ticks >= period_ticks) {
    compare_ticks = period_ticks - 1U;
  }

  return compare_ticks;
}

uint32_t FrequencyForCurrentSlot(uint32_t now_ms) {
  if (!g_burst.alternate_frequency) {
    return LOW_TEST_FREQUENCY_HZ;
  }

  const uint32_t elapsed_ms = now_ms - g_burst.pattern_start_ms;
  const uint32_t slot_index = elapsed_ms / FREQUENCY_SLOT_MS;
  return ((slot_index % 2U) == 0U) ? LOW_TEST_FREQUENCY_HZ
                                   : HIGH_TEST_FREQUENCY_HZ;
}

bool ConfigureTone(HRTIM_HandleTypeDef *hrtim, uint32_t frequency_hz) {
  const uint32_t period_ticks = PeriodTicksForFrequency(frequency_hz);

  HRTIM_TimeBaseCfgTypeDef time_base_config = {};
  time_base_config.Period = period_ticks;
  time_base_config.RepetitionCounter = 0x00U;
  time_base_config.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
  time_base_config.Mode = HRTIM_MODE_CONTINUOUS;

  if (HAL_HRTIM_TimeBaseConfig(hrtim, TX_TIMER_INDEX, &time_base_config) !=
      HAL_OK) {
    return false;
  }

  HRTIM_CompareCfgTypeDef compare_config = {};
  compare_config.CompareValue = CompareTicksForPeriod(period_ticks);

  // Override CubeMX's compare value at runtime. This keeps generated Core code untouched.
  if (HAL_HRTIM_WaveformCompareConfig(hrtim, TX_TIMER_INDEX,
                                      HRTIM_COMPAREUNIT_1,
                                      &compare_config) != HAL_OK) {
    return false;
  }

  g_burst.active_frequency_hz = frequency_hz;
  return true;
}

bool HasElapsed(uint32_t now_ms, uint32_t then_ms, uint32_t interval_ms) {
  // Unsigned subtraction keeps the test valid when HAL_GetTick() wraps around.
  return (now_ms - then_ms) >= interval_ms;
}

bool StartBurstSendWithMode(HRTIM_HandleTypeDef *hrtim, uint32_t tx_on_ms,
                            uint32_t tx_off_ms, bool alternate_frequency) {
  if (hrtim == nullptr) {
    return false;
  }

  (void)TxDriver::Stop(hrtim);

  // Avoid a zero interval, which would make the burst state machine toggle too fast.
  g_burst.tx_on_ms = (tx_on_ms > 0U) ? tx_on_ms : 1U;
  g_burst.tx_off_ms = (tx_off_ms > 0U) ? tx_off_ms : 1U;
  g_burst.last_transition_ms = HAL_GetTick();
  g_burst.pattern_start_ms = g_burst.last_transition_ms;
  g_burst.active_frequency_hz = LOW_TEST_FREQUENCY_HZ;
  g_burst.alternate_frequency = alternate_frequency;

  if (!TxDriver::StartContinuousTone(hrtim)) {
    g_burst.state = BurstState::FAULT;
    return false;
  }

  g_burst.state = BurstState::TX_ON;
  return true;
}

}  // namespace

bool TxDriver::StartContinuousTone(HRTIM_HandleTypeDef *hrtim) {
  if (hrtim == nullptr) {
    return false;
  }

  const uint32_t now_ms = HAL_GetTick();
  const uint32_t frequency_hz = FrequencyForCurrentSlot(now_ms);

  return StartToneAtFrequency(hrtim, frequency_hz);
}

bool TxDriver::StartToneAtFrequency(HRTIM_HandleTypeDef *hrtim,
                                    uint32_t frequency_hz) {
  if ((hrtim == nullptr) || (frequency_hz == 0U)) {
    return false;
  }

  if (!ConfigureTone(hrtim, frequency_hz)) {
    return false;
  }

  // Enable the output first, then start the timer counter, matching ST's HRTIM examples.
  if (HAL_HRTIM_WaveformOutputStart(hrtim, TX_OUTPUT) != HAL_OK) {
    return false;
  }

  if (HAL_HRTIM_WaveformCountStart(hrtim, TX_TIMER) != HAL_OK) {
    (void)HAL_HRTIM_WaveformOutputStop(hrtim, TX_OUTPUT);
    return false;
  }

  return true;
}

bool TxDriver::StartDefaultBurst(HRTIM_HandleTypeDef *hrtim) {
  return StartAlternatingFrequencyBurst(hrtim, DEFAULT_TX_ON_MS,
                                        DEFAULT_TX_OFF_MS);
}

bool TxDriver::StartAlternatingFrequencyBurst(HRTIM_HandleTypeDef *hrtim,
                                              uint32_t tx_on_ms,
                                              uint32_t tx_off_ms) {
  return StartBurstSendWithMode(hrtim, tx_on_ms, tx_off_ms, true);
}

bool TxDriver::StartBurstSend(HRTIM_HandleTypeDef *hrtim, uint32_t tx_on_ms,
                              uint32_t tx_off_ms) {
  return StartBurstSendWithMode(hrtim, tx_on_ms, tx_off_ms, false);
}

void TxDriver::Process(HRTIM_HandleTypeDef *hrtim) {
  if (hrtim == nullptr) {
    g_burst.state = BurstState::FAULT;
    return;
  }

  const uint32_t now_ms = HAL_GetTick();

  switch (g_burst.state) {
    case BurstState::TX_ON:
      if (HasElapsed(now_ms, g_burst.last_transition_ms, g_burst.tx_on_ms)) {
        (void)Stop(hrtim);
        g_burst.last_transition_ms = now_ms;
        g_burst.state = BurstState::TX_OFF;
      }
      break;

    case BurstState::TX_OFF:
      if (HasElapsed(now_ms, g_burst.last_transition_ms, g_burst.tx_off_ms)) {
        if (StartContinuousTone(hrtim)) {
          g_burst.last_transition_ms = now_ms;
          g_burst.state = BurstState::TX_ON;
        } else {
          g_burst.state = BurstState::FAULT;
        }
      }
      break;

    case BurstState::IDLE:
    case BurstState::FAULT:
    default:
      break;
  }
}

bool TxDriver::Stop(HRTIM_HandleTypeDef *hrtim) {
  if (hrtim == nullptr) {
    return false;
  }

  const HAL_StatusTypeDef output_status =
      HAL_HRTIM_WaveformOutputStop(hrtim, TX_OUTPUT);
  const HAL_StatusTypeDef counter_status =
      HAL_HRTIM_WaveformCountStop(hrtim, TX_TIMER);

  g_burst.state = BurstState::IDLE;
  return (counter_status == HAL_OK) && (output_status == HAL_OK);
}

}  // namespace underwatercomm
