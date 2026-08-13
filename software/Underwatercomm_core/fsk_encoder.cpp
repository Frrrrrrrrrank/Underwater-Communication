#include "fsk_encoder.hpp"

#include "fsk_protocol.hpp"
#include "tx_driver.hpp"
#include "underwatercomm_config.hpp"

namespace underwatercomm {
namespace {

constexpr uint32_t LOW_FREQUENCY_HZ =
    UNDERWATERCOMM_TX_LOW_FREQUENCY_HZ;
constexpr uint32_t HIGH_FREQUENCY_HZ =
    UNDERWATERCOMM_TX_HIGH_FREQUENCY_HZ;
constexpr uint32_t BIT_DURATION_MS = UNDERWATERCOMM_FSK_BIT_DURATION_MS;
constexpr uint32_t TONE_ON_MS = UNDERWATERCOMM_FSK_TONE_ON_MS;
constexpr uint32_t FRAME_GAP_MS = UNDERWATERCOMM_FSK_FRAME_GAP_MS;
constexpr uint8_t PREAMBLE_BYTE = UNDERWATERCOMM_FSK_PREAMBLE_BYTE;
constexpr uint8_t FIRST_PAYLOAD = UNDERWATERCOMM_FSK_FIRST_PAYLOAD;
constexpr uint8_t LAST_PAYLOAD = UNDERWATERCOMM_FSK_LAST_PAYLOAD;

constexpr uint32_t FRAME_BYTE_COUNT = 3U;
constexpr uint32_t FRAME_BIT_COUNT = FRAME_BYTE_COUNT * 8U;

static_assert(LOW_FREQUENCY_HZ > 0U, "FSK low frequency must be non-zero");
static_assert(HIGH_FREQUENCY_HZ > 0U,
              "FSK high frequency must be non-zero");
static_assert(LOW_FREQUENCY_HZ != HIGH_FREQUENCY_HZ,
              "FSK low and high frequencies must differ");
static_assert(BIT_DURATION_MS > 0U,
              "FSK bit duration must be at least 1 ms");
static_assert((TONE_ON_MS > 0U) && (TONE_ON_MS <= BIT_DURATION_MS),
              "FSK tone-on time must fit inside one bit");
static_assert(FIRST_PAYLOAD <= LAST_PAYLOAD,
              "FSK payload range must be increasing");

enum class EncoderState : uint8_t {
  IDLE = 0U,
  TONE_ON,
  SYMBOL_WAIT,
  FRAME_GAP,
  FAULT,
};

HRTIM_HandleTypeDef *g_hrtim = nullptr;
EncoderState g_state = EncoderState::IDLE;
uint8_t g_frame[FRAME_BYTE_COUNT] = {};
uint8_t g_payload = FIRST_PAYLOAD;
uint32_t g_bit_index = 0U;
uint32_t g_symbol_start_ms = 0U;
uint32_t g_frame_gap_start_ms = 0U;

bool HasElapsed(uint32_t now_ms, uint32_t start_ms, uint32_t duration_ms) {
  return (now_ms - start_ms) >= duration_ms;
}

void UpdateDebugState() {
  g_underwatercomm_fsk_tx_debug.state = static_cast<uint8_t>(g_state);
}

void EnterFault() {
  if (g_hrtim != nullptr) {
    (void)TxDriver::Stop(g_hrtim);
  }

  g_state = EncoderState::FAULT;
  g_underwatercomm_fsk_tx_debug.tone_active = 0U;
  g_underwatercomm_fsk_tx_debug.fault = 1U;
  UpdateDebugState();
}

void PrepareFrame() {
  // Fixed test frame: alternating preamble, one payload byte and its CRC-8.
  g_frame[0] = PREAMBLE_BYTE;
  g_frame[1] = g_payload;
  g_frame[2] = fsk_protocol::CalculateCrc8(g_payload);

  g_underwatercomm_fsk_tx_debug.payload = g_payload;
  g_underwatercomm_fsk_tx_debug.crc = g_frame[2];
}

uint8_t CurrentBit() {
  const uint32_t byte_index = g_bit_index / 8U;
  const uint32_t bit_in_byte = 7U - (g_bit_index % 8U);
  return static_cast<uint8_t>((g_frame[byte_index] >> bit_in_byte) & 0x01U);
}

bool StartCurrentBit(uint32_t now_ms) {
  const uint8_t bit = CurrentBit();
  const uint32_t frequency_hz = (bit == 0U) ? LOW_FREQUENCY_HZ
                                            : HIGH_FREQUENCY_HZ;

  if (!TxDriver::StartToneAtFrequency(g_hrtim, frequency_hz)) {
    EnterFault();
    return false;
  }

  g_symbol_start_ms = now_ms;
  g_state = EncoderState::TONE_ON;
  g_underwatercomm_fsk_tx_debug.bit_index =
      static_cast<uint8_t>(g_bit_index);
  g_underwatercomm_fsk_tx_debug.current_bit = bit;
  g_underwatercomm_fsk_tx_debug.current_frequency_hz = frequency_hz;
  g_underwatercomm_fsk_tx_debug.tone_active = 1U;
  UpdateDebugState();
  return true;
}

void AdvancePayload() {
  if (g_payload >= LAST_PAYLOAD) {
    g_payload = FIRST_PAYLOAD;
  } else {
    ++g_payload;
  }
}

void ResetDebugData() {
  g_underwatercomm_fsk_tx_debug.started = 0U;
  g_underwatercomm_fsk_tx_debug.frame_counter = 0U;
  g_underwatercomm_fsk_tx_debug.current_frequency_hz = 0U;
  g_underwatercomm_fsk_tx_debug.bit_duration_ms = BIT_DURATION_MS;
  g_underwatercomm_fsk_tx_debug.tone_on_ms = TONE_ON_MS;
  g_underwatercomm_fsk_tx_debug.payload = FIRST_PAYLOAD;
  g_underwatercomm_fsk_tx_debug.crc = 0U;
  g_underwatercomm_fsk_tx_debug.bit_index = 0U;
  g_underwatercomm_fsk_tx_debug.current_bit = 0U;
  g_underwatercomm_fsk_tx_debug.tone_active = 0U;
  g_underwatercomm_fsk_tx_debug.state =
      static_cast<uint8_t>(EncoderState::IDLE);
  g_underwatercomm_fsk_tx_debug.fault = 0U;
}

}  // namespace

extern "C" {
volatile UnderwaterComm_FskTxDebugData g_underwatercomm_fsk_tx_debug = {};
}

bool FskEncoder::Init(HRTIM_HandleTypeDef *hrtim) {
  if (hrtim == nullptr) {
    return false;
  }

  g_hrtim = hrtim;
  (void)TxDriver::Stop(g_hrtim);

  g_state = EncoderState::IDLE;
  g_payload = FIRST_PAYLOAD;
  g_bit_index = 0U;
  ResetDebugData();
  PrepareFrame();

  g_underwatercomm_fsk_tx_debug.started = 1U;
  return StartCurrentBit(HAL_GetTick());
}

void FskEncoder::Process() {
  if (g_hrtim == nullptr) {
    EnterFault();
    return;
  }

  const uint32_t now_ms = HAL_GetTick();

  switch (g_state) {
    case EncoderState::TONE_ON:
      if (HasElapsed(now_ms, g_symbol_start_ms, TONE_ON_MS)) {
        if (!TxDriver::Stop(g_hrtim)) {
          EnterFault();
          return;
        }

        g_state = EncoderState::SYMBOL_WAIT;
        g_underwatercomm_fsk_tx_debug.tone_active = 0U;
        UpdateDebugState();
      }
      break;

    case EncoderState::SYMBOL_WAIT:
      if (HasElapsed(now_ms, g_symbol_start_ms, BIT_DURATION_MS)) {
        ++g_bit_index;

        if (g_bit_index < FRAME_BIT_COUNT) {
          (void)StartCurrentBit(now_ms);
        } else {
          ++g_underwatercomm_fsk_tx_debug.frame_counter;
          AdvancePayload();
          g_frame_gap_start_ms = now_ms;
          g_state = EncoderState::FRAME_GAP;
          UpdateDebugState();
        }
      }
      break;

    case EncoderState::FRAME_GAP:
      if (HasElapsed(now_ms, g_frame_gap_start_ms, FRAME_GAP_MS)) {
        g_bit_index = 0U;
        PrepareFrame();
        (void)StartCurrentBit(now_ms);
      }
      break;

    case EncoderState::IDLE:
    case EncoderState::FAULT:
    default:
      break;
  }
}

bool FskEncoder::Stop() {
  bool stopped = true;

  if (g_hrtim != nullptr) {
    stopped = TxDriver::Stop(g_hrtim);
  }

  g_state = EncoderState::IDLE;
  g_underwatercomm_fsk_tx_debug.started = 0U;
  g_underwatercomm_fsk_tx_debug.current_frequency_hz = 0U;
  g_underwatercomm_fsk_tx_debug.tone_active = 0U;
  UpdateDebugState();
  return stopped;
}

}  // namespace underwatercomm
