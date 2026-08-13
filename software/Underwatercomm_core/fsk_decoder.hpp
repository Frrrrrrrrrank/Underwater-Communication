#ifndef UNDERWATERCOMM_CORE_FSK_DECODER_HPP
#define UNDERWATERCOMM_CORE_FSK_DECODER_HPP

#include <stdint.h>

extern "C" {

/*
 * Debugger-visible FSK receiver state.
 *
 * Useful fields while testing:
 *   current_bit       : most recently detected FSK symbol.
 *   preamble_shift    : rolling 8-bit preamble search value.
 *   payload           : payload currently being assembled.
 *   last_valid_payload: payload from the latest CRC-valid frame.
 *   valid_frame_count : number of complete valid frames received.
 *   crc_error_count   : complete frames rejected by CRC.
 */
typedef struct {
  uint32_t started;
  uint32_t processed_block_count;
  uint32_t detected_bit_count;
  uint32_t sync_count;
  uint32_t valid_frame_count;
  uint32_t crc_error_count;
  uint32_t timeout_count;
  float crc_error_rate;
  float frame_success_rate;
  float noise_power;
  float total_power;
  float detect_threshold;
  float release_threshold;
  float fsk_score;
  uint8_t state;
  uint8_t signal_present;
  uint8_t current_bit;
  uint8_t bit_index;
  uint8_t preamble_shift;
  uint8_t payload;
  uint8_t last_valid_payload;
  uint8_t received_crc;
  uint8_t calculated_crc;
  uint8_t crc_ok;
  uint8_t fault;
} UnderwaterComm_FskRxDebugData;

extern volatile UnderwaterComm_FskRxDebugData
    g_underwatercomm_fsk_rx_debug;
}

namespace underwatercomm {

class FskDecoder final {
 public:
  // Reset the burst detector and frame parser before ADC processing starts.
  static bool Init();

  // Consume the newest RxDriver analysis block, if one is available.
  static void Process();

  // Stop decoding and clear any partially received frame.
  static void Stop();
};

}  // namespace underwatercomm

#endif /* UNDERWATERCOMM_CORE_FSK_DECODER_HPP */
