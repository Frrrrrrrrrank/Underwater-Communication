#include "tx_driver.hpp"

namespace underwatercomm {
namespace {

// HRTIM1 Timer A output TA1 is routed to PA8 in the CubeMX configuration.
constexpr uint32_t TX_TIMER = HRTIM_TIMERID_TIMER_A;
constexpr uint32_t TX_OUTPUT = HRTIM_OUTPUT_TA1;

// Period comes from the IOC setting: 170 MHz HRTIM clock * 16 / 36267 ~= 75 kHz.
constexpr uint32_t TX_PERIOD_TICKS = 36267U;

// Keep the bring-up duty low to reduce stress on the power stage and matching network.
constexpr uint32_t TX_DUTY_PERCENT = 50U;
constexpr uint32_t TX_COMPARE_TICKS =
    ((TX_PERIOD_TICKS * TX_DUTY_PERCENT) + 50U) / 100U;

// Default bring-up burst. Keep this local to the TX driver so the main task
// does not need to know power-stage timing details.
constexpr uint32_t DEFAULT_TX_ON_MS = 1U;
constexpr uint32_t DEFAULT_TX_OFF_MS = 100U;

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
  BurstState state = BurstState::IDLE;
};

BurstControl g_burst;

bool ConfigureDutyCycle(HRTIM_HandleTypeDef *hrtim) {
  HRTIM_CompareCfgTypeDef compare_config = {};
  compare_config.CompareValue = TX_COMPARE_TICKS;

  // Override CubeMX's compare value at runtime. This keeps generated Core code untouched.
  return HAL_HRTIM_WaveformCompareConfig(
             hrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1,
             &compare_config) == HAL_OK;
}

bool HasElapsed(uint32_t now_ms, uint32_t then_ms, uint32_t interval_ms) {
  // Unsigned subtraction keeps the test valid when HAL_GetTick() wraps around.
  return (now_ms - then_ms) >= interval_ms;
}

}  // namespace

bool TxDriver::StartContinuousTone(HRTIM_HandleTypeDef *hrtim) {
  if (hrtim == nullptr) {
    return false;
  }

  if (!ConfigureDutyCycle(hrtim)) {
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
  return StartBurstSend(hrtim, DEFAULT_TX_ON_MS, DEFAULT_TX_OFF_MS);
}

bool TxDriver::StartBurstSend(HRTIM_HandleTypeDef *hrtim, uint32_t tx_on_ms,
                              uint32_t tx_off_ms) {
  if (hrtim == nullptr) {
    return false;
  }

  (void)Stop(hrtim);

  // Avoid a zero interval, which would make the burst state machine toggle too fast.
  g_burst.tx_on_ms = (tx_on_ms > 0U) ? tx_on_ms : 1U;
  g_burst.tx_off_ms = (tx_off_ms > 0U) ? tx_off_ms : 1U;
  g_burst.last_transition_ms = HAL_GetTick();

  if (!StartContinuousTone(hrtim)) {
    g_burst.state = BurstState::FAULT;
    return false;
  }

  g_burst.state = BurstState::TX_ON;
  return true;
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
