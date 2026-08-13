#include "maintask.hpp"

#include "Buzzer.hpp"
#include "fsk_decoder.hpp"
#include "fsk_encoder.hpp"
#include "rx_driver.hpp"
#include "tx_driver.hpp"
#include "underwatercomm_config.hpp"

extern "C" {
#include "adc.h"
#include "hrtim.h"
#include "tim.h"
}

namespace {

enum class MainTaskState {
  STARTING,
  RX_RUNNING,
  TX_RUNNING,
  FAULT,
};

volatile MainTaskState g_state = MainTaskState::STARTING;

#if UNDERWATERCOMM_BUILD_MODE == UNDERWATERCOMM_BUILD_MODE_RX
bool EnterRxMode() {
  // Half-duplex rule: never receive while the 75 kHz power stage is running.
  // The ADC would mostly capture our own transmit leakage and switching noise.
  (void)underwatercomm::TxDriver::Stop(&hhrtim1);

#if UNDERWATERCOMM_ENABLE_RX_DMA
  if (!underwatercomm::RxDriver::Init(&hadc1, &htim6)) {
    g_state = MainTaskState::FAULT;
    return false;
  }

  if (!underwatercomm::FskDecoder::Init()) {
    (void)underwatercomm::RxDriver::Stop();
    g_state = MainTaskState::FAULT;
    return false;
  }
#endif

  Drivers::Buzzer::playRxBootPattern();
  g_state = MainTaskState::RX_RUNNING;
  return true;
}
#endif

#if UNDERWATERCOMM_BUILD_MODE == UNDERWATERCOMM_BUILD_MODE_TX
bool EnterTxMode() {
  // Half-duplex rule: stop ADC DMA before TX. This avoids DMA callbacks and FFT
  // work while the power stage is producing a large local signal.
  underwatercomm::FskDecoder::Stop();
  (void)underwatercomm::RxDriver::Stop();

#if UNDERWATERCOMM_TX_TASK == UNDERWATERCOMM_TX_TASK_FSK_ENCODER
  if (!underwatercomm::FskEncoder::Init(&hhrtim1)) {
    g_state = MainTaskState::FAULT;
    return false;
  }
#elif UNDERWATERCOMM_TX_TASK == UNDERWATERCOMM_TX_TASK_ALTERNATING_TEST
  if (!underwatercomm::TxDriver::StartDefaultBurst(&hhrtim1)) {
    g_state = MainTaskState::FAULT;
    return false;
  }
#else
#error "Unsupported UNDERWATERCOMM_TX_TASK selection"
#endif

  Drivers::Buzzer::playTxBootPattern();
  g_state = MainTaskState::TX_RUNNING;
  return true;
}
#endif

}  // namespace

extern "C" void UnderwaterComm_MainTask_Init(void) {
  Drivers::Buzzer::init();

#if UNDERWATERCOMM_BUILD_MODE == UNDERWATERCOMM_BUILD_MODE_RX
  (void)EnterRxMode();
#elif UNDERWATERCOMM_BUILD_MODE == UNDERWATERCOMM_BUILD_MODE_TX
  (void)EnterTxMode();
#else
#error "UNDERWATERCOMM_BUILD_MODE must be UNDERWATERCOMM_BUILD_MODE_RX or UNDERWATERCOMM_BUILD_MODE_TX"
#endif
}

extern "C" void UnderwaterComm_MainTask_Run(void) {
  // Keep periodic work non-blocking. No HAL_Delay() here, because RX DMA and TX
  // burst timing both rely on the main loop returning quickly.
  Drivers::Buzzer::process();

  switch (g_state) {
    case MainTaskState::RX_RUNNING:
#if UNDERWATERCOMM_ENABLE_RX_DMA
      underwatercomm::RxDriver::Process();
      underwatercomm::FskDecoder::Process();
#endif
      break;

    case MainTaskState::TX_RUNNING:
#if UNDERWATERCOMM_TX_TASK == UNDERWATERCOMM_TX_TASK_FSK_ENCODER
      underwatercomm::FskEncoder::Process();
#elif UNDERWATERCOMM_TX_TASK == UNDERWATERCOMM_TX_TASK_ALTERNATING_TEST
      underwatercomm::TxDriver::Process(&hhrtim1);
#endif
      break;

    case MainTaskState::STARTING:
    case MainTaskState::FAULT:
    default:
      break;
  }
}
