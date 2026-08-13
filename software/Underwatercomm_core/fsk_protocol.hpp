#ifndef UNDERWATERCOMM_CORE_FSK_PROTOCOL_HPP
#define UNDERWATERCOMM_CORE_FSK_PROTOCOL_HPP

#include <stdint.h>

#include "underwatercomm_config.hpp"

namespace underwatercomm {
namespace fsk_protocol {

constexpr uint8_t PREAMBLE_BYTE = UNDERWATERCOMM_FSK_PREAMBLE_BYTE;
constexpr uint32_t BITS_PER_BYTE = 8U;

// Calculate the CRC byte used by both the transmitter and receiver. The
// current test protocol protects one payload byte with CRC-8 polynomial 0x07.
inline uint8_t CalculateCrc8(uint8_t data) {
  uint8_t crc = data;

  for (uint32_t bit = 0U; bit < BITS_PER_BYTE; ++bit) {
    if ((crc & 0x80U) != 0U) {
      crc = static_cast<uint8_t>(
          (crc << 1U) ^ UNDERWATERCOMM_FSK_CRC8_POLYNOMIAL);
    } else {
      crc = static_cast<uint8_t>(crc << 1U);
    }
  }

  return crc;
}

}  // namespace fsk_protocol
}  // namespace underwatercomm

#endif /* UNDERWATERCOMM_CORE_FSK_PROTOCOL_HPP */
