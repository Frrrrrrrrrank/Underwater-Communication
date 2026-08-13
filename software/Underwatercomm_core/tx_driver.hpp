#ifndef UNDERWATERCOMM_CORE_TX_DRIVER_HPP
#define UNDERWATERCOMM_CORE_TX_DRIVER_HPP

#include <stdint.h>

extern "C" {
#include "stm32g4xx_hal.h"
}

namespace underwatercomm {

class TxDriver final {
 public:
  // Start a continuous tone at an explicit frequency. FSK encoding uses this
  // primitive to map each data bit to the configured low or high frequency.
  static bool StartToneAtFrequency(HRTIM_HandleTypeDef *hrtim,
                                   uint32_t frequency_hz);

  // Start a continuous PWM tone on PA8/HRTIM1_CHA1.
  static bool StartContinuousTone(HRTIM_HandleTypeDef *hrtim);

  // Start the default safe bring-up burst configured in tx_driver.cpp.
  static bool StartDefaultBurst(HRTIM_HandleTypeDef *hrtim);

  // Start a burst test that alternates between two TX frequencies.
  static bool StartAlternatingFrequencyBurst(HRTIM_HandleTypeDef *hrtim,
                                             uint32_t tx_on_ms,
                                             uint32_t tx_off_ms);

  // Start repeated burst sending. Both timing arguments are in milliseconds.
  static bool StartBurstSend(HRTIM_HandleTypeDef *hrtim, uint32_t tx_on_ms,
                             uint32_t tx_off_ms);

  // Call this from the main loop. It advances the burst state machine.
  static void Process(HRTIM_HandleTypeDef *hrtim);

  // Stop the HRTIM output and counter.
  static bool Stop(HRTIM_HandleTypeDef *hrtim);
};

}  // namespace underwatercomm

#endif /* UNDERWATERCOMM_CORE_TX_DRIVER_HPP */
