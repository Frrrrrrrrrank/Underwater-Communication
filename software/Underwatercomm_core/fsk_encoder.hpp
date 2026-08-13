#ifndef UNDERWATERCOMM_CORE_FSK_ENCODER_HPP
#define UNDERWATERCOMM_CORE_FSK_ENCODER_HPP

#include <stdint.h>

extern "C" {
#include "stm32g4xx_hal.h"

/*
 * Debugger-visible FSK transmitter state.
 *
 * Watch g_underwatercomm_fsk_tx_debug while TX mode is running. The payload
 * cycles from the configured first value to the last value. current_bit maps
 * to the configured low frequency for 0 and high frequency for 1.
 */
typedef struct {
  uint32_t started;
  uint32_t frame_counter;
  uint32_t current_frequency_hz;
  uint32_t bit_duration_ms;
  uint32_t tone_on_ms;
  uint8_t payload;
  uint8_t crc;
  uint8_t bit_index;
  uint8_t current_bit;
  uint8_t tone_active;
  uint8_t state;
  uint8_t fault;
} UnderwaterComm_FskTxDebugData;

extern volatile UnderwaterComm_FskTxDebugData
    g_underwatercomm_fsk_tx_debug;
}

namespace underwatercomm {

class FskEncoder final {
 public:
  // Initialize the encoder and immediately start transmitting the first frame.
  static bool Init(HRTIM_HandleTypeDef *hrtim);

  // Advance tone, symbol and frame timing. Call continuously from main task.
  static void Process();

  // Stop the encoder and force the HRTIM output inactive.
  static bool Stop();
};

}  // namespace underwatercomm

#endif /* UNDERWATERCOMM_CORE_FSK_ENCODER_HPP */
