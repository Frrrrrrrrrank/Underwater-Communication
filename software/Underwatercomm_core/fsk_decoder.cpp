#include "fsk_decoder.hpp"

#include "fsk_protocol.hpp"
#include "rx_driver.hpp"
#include "underwatercomm_config.hpp"

namespace underwatercomm {
namespace {

constexpr float DETECT_RATIO = UNDERWATERCOMM_FSK_RX_DETECT_RATIO;
constexpr float RELEASE_RATIO = UNDERWATERCOMM_FSK_RX_RELEASE_RATIO;
constexpr float MIN_TOTAL_POWER = UNDERWATERCOMM_FSK_RX_MIN_TOTAL_POWER;
constexpr float MIN_ABS_SCORE = UNDERWATERCOMM_FSK_RX_MIN_ABS_SCORE;
constexpr float NOISE_FILTER_ALPHA =
    UNDERWATERCOMM_FSK_RX_NOISE_FILTER_ALPHA;
constexpr uint32_t BIT_DURATION_MS = UNDERWATERCOMM_FSK_BIT_DURATION_MS;
constexpr uint32_t BIT_TIMEOUT_MS = UNDERWATERCOMM_FSK_RX_BIT_TIMEOUT_MS;
constexpr uint32_t MIN_BIT_SPACING_MS = BIT_DURATION_MS / 2U;

static_assert(DETECT_RATIO > RELEASE_RATIO,
              "FSK detect ratio must be greater than release ratio");
static_assert(RELEASE_RATIO > 0.0F,
              "FSK release ratio must be positive");
static_assert((NOISE_FILTER_ALPHA > 0.0F) &&
                  (NOISE_FILTER_ALPHA <= 1.0F),
              "FSK noise filter alpha must be in (0, 1]");
static_assert((MIN_ABS_SCORE >= 0.0F) && (MIN_ABS_SCORE <= 1.0F),
              "FSK minimum score must be in [0, 1]");
static_assert(BIT_TIMEOUT_MS > BIT_DURATION_MS,
              "FSK receive timeout must exceed one bit duration");

enum class DecoderState : uint8_t {
  SEARCH_PREAMBLE = 0U,
  RECEIVE_PAYLOAD,
  RECEIVE_CRC,
};

DecoderState g_state = DecoderState::SEARCH_PREAMBLE;
uint32_t g_last_block_counter = 0U;
uint32_t g_last_bit_ms = 0U;
float g_noise_power = 0.0F;
bool g_noise_has_history = false;
bool g_signal_present = false;
uint8_t g_bit_index = 0U;
uint8_t g_preamble_shift = 0U;
uint8_t g_payload = 0U;
uint8_t g_received_crc = 0U;

float Maximum(float first, float second) {
  return (first > second) ? first : second;
}

float Absolute(float value) {
  return (value >= 0.0F) ? value : -value;
}

bool HasElapsed(uint32_t now_ms, uint32_t start_ms, uint32_t duration_ms) {
  return (now_ms - start_ms) >= duration_ms;
}

void UpdateStateDebug() {
  g_underwatercomm_fsk_rx_debug.state = static_cast<uint8_t>(g_state);
  g_underwatercomm_fsk_rx_debug.signal_present =
      g_signal_present ? 1U : 0U;
  g_underwatercomm_fsk_rx_debug.bit_index = g_bit_index;
  g_underwatercomm_fsk_rx_debug.preamble_shift = g_preamble_shift;

  // Keep the completed frame visible while searching for the next preamble.
  // These fields are cleared when a new preamble starts a new frame.
  if (g_state != DecoderState::SEARCH_PREAMBLE) {
    g_underwatercomm_fsk_rx_debug.payload = g_payload;
    g_underwatercomm_fsk_rx_debug.received_crc = g_received_crc;
  }
}

void ResetFrameParser() {
  g_state = DecoderState::SEARCH_PREAMBLE;
  g_bit_index = 0U;
  g_preamble_shift = 0U;
  g_payload = 0U;
  g_received_crc = 0U;
  UpdateStateDebug();
}

void UpdateFrameSuccessRate() {
  // A synchronized frame has three resolved outcomes: valid, CRC error, or
  // timeout. Treat both error outcomes as failed frames for one comparable
  // link-quality value. Frames whose preamble was never found are not visible
  // to this receiver and therefore cannot be included in this calculation.
  const uint32_t resolved_frame_count =
      g_underwatercomm_fsk_rx_debug.valid_frame_count +
      g_underwatercomm_fsk_rx_debug.crc_error_count +
      g_underwatercomm_fsk_rx_debug.timeout_count;

  if (resolved_frame_count == 0U) {
    g_underwatercomm_fsk_rx_debug.frame_success_rate = 0.0F;
    return;
  }

  g_underwatercomm_fsk_rx_debug.frame_success_rate =
      static_cast<float>(
          g_underwatercomm_fsk_rx_debug.valid_frame_count) /
      static_cast<float>(resolved_frame_count);
}

void ConsumeBit(uint8_t bit, uint32_t now_ms) {
  g_last_bit_ms = now_ms;
  ++g_underwatercomm_fsk_rx_debug.detected_bit_count;
  g_underwatercomm_fsk_rx_debug.current_bit = bit;

  switch (g_state) {
    case DecoderState::SEARCH_PREAMBLE:
      // Shift MSB-first symbols into a rolling byte until 0xAA is found.
      g_preamble_shift =
          static_cast<uint8_t>((g_preamble_shift << 1U) | bit);

      if (g_preamble_shift == fsk_protocol::PREAMBLE_BYTE) {
        g_state = DecoderState::RECEIVE_PAYLOAD;
        g_bit_index = 0U;
        g_payload = 0U;
        g_received_crc = 0U;
        g_underwatercomm_fsk_rx_debug.crc_ok = 0U;
        ++g_underwatercomm_fsk_rx_debug.sync_count;
      }
      break;

    case DecoderState::RECEIVE_PAYLOAD:
      g_payload = static_cast<uint8_t>((g_payload << 1U) | bit);
      ++g_bit_index;

      if (g_bit_index >= fsk_protocol::BITS_PER_BYTE) {
        g_state = DecoderState::RECEIVE_CRC;
        g_bit_index = 0U;
      }
      break;

    case DecoderState::RECEIVE_CRC:
      g_received_crc =
          static_cast<uint8_t>((g_received_crc << 1U) | bit);
      ++g_bit_index;

      if (g_bit_index >= fsk_protocol::BITS_PER_BYTE) {
        const uint8_t calculated_crc = fsk_protocol::CalculateCrc8(g_payload);
        const bool crc_ok = calculated_crc == g_received_crc;

        g_underwatercomm_fsk_rx_debug.payload = g_payload;
        g_underwatercomm_fsk_rx_debug.received_crc = g_received_crc;
        g_underwatercomm_fsk_rx_debug.calculated_crc = calculated_crc;
        g_underwatercomm_fsk_rx_debug.crc_ok = crc_ok ? 1U : 0U;

        if (crc_ok) {
          g_underwatercomm_fsk_rx_debug.last_valid_payload = g_payload;
          ++g_underwatercomm_fsk_rx_debug.valid_frame_count;
        } else {
          ++g_underwatercomm_fsk_rx_debug.crc_error_count;
        }

        // CRC error rate covers complete frames only. A value of 0.10 means
        // 10% of frames that reached the CRC check contained a bit error.
        const uint32_t completed_frame_count =
            g_underwatercomm_fsk_rx_debug.valid_frame_count +
            g_underwatercomm_fsk_rx_debug.crc_error_count;
        g_underwatercomm_fsk_rx_debug.crc_error_rate =
            static_cast<float>(
                g_underwatercomm_fsk_rx_debug.crc_error_count) /
            static_cast<float>(completed_frame_count);
        UpdateFrameSuccessRate();

        ResetFrameParser();
      }
      break;
  }

  UpdateStateDebug();
}

void ResetDebugData() {
  g_underwatercomm_fsk_rx_debug.started = 0U;
  g_underwatercomm_fsk_rx_debug.processed_block_count = 0U;
  g_underwatercomm_fsk_rx_debug.detected_bit_count = 0U;
  g_underwatercomm_fsk_rx_debug.sync_count = 0U;
  g_underwatercomm_fsk_rx_debug.valid_frame_count = 0U;
  g_underwatercomm_fsk_rx_debug.crc_error_count = 0U;
  g_underwatercomm_fsk_rx_debug.timeout_count = 0U;
  g_underwatercomm_fsk_rx_debug.crc_error_rate = 0.0F;
  g_underwatercomm_fsk_rx_debug.frame_success_rate = 0.0F;
  g_underwatercomm_fsk_rx_debug.noise_power = 0.0F;
  g_underwatercomm_fsk_rx_debug.total_power = 0.0F;
  g_underwatercomm_fsk_rx_debug.detect_threshold = MIN_TOTAL_POWER;
  g_underwatercomm_fsk_rx_debug.release_threshold = MIN_TOTAL_POWER;
  g_underwatercomm_fsk_rx_debug.fsk_score = 0.0F;
  g_underwatercomm_fsk_rx_debug.state =
      static_cast<uint8_t>(DecoderState::SEARCH_PREAMBLE);
  g_underwatercomm_fsk_rx_debug.signal_present = 0U;
  g_underwatercomm_fsk_rx_debug.current_bit = 0U;
  g_underwatercomm_fsk_rx_debug.bit_index = 0U;
  g_underwatercomm_fsk_rx_debug.preamble_shift = 0U;
  g_underwatercomm_fsk_rx_debug.payload = 0U;
  g_underwatercomm_fsk_rx_debug.last_valid_payload = 0U;
  g_underwatercomm_fsk_rx_debug.received_crc = 0U;
  g_underwatercomm_fsk_rx_debug.calculated_crc = 0U;
  g_underwatercomm_fsk_rx_debug.crc_ok = 0U;
  g_underwatercomm_fsk_rx_debug.fault = 0U;
}

}  // namespace

extern "C" {
volatile UnderwaterComm_FskRxDebugData g_underwatercomm_fsk_rx_debug = {};
}

bool FskDecoder::Init() {
  g_state = DecoderState::SEARCH_PREAMBLE;
  g_last_block_counter = 0U;
  g_last_bit_ms = 0U;
  g_noise_power = 0.0F;
  g_noise_has_history = false;
  g_signal_present = false;
  g_bit_index = 0U;
  g_preamble_shift = 0U;
  g_payload = 0U;
  g_received_crc = 0U;

  ResetDebugData();
  g_underwatercomm_fsk_rx_debug.started = 1U;
  return true;
}

void FskDecoder::Process() {
  if (g_underwatercomm_fsk_rx_debug.started == 0U) {
    return;
  }

  RxAnalysisResult result = {};
  if (!RxDriver::GetLatestResult(&result) ||
      (result.block_counter == g_last_block_counter)) {
    return;
  }

  g_last_block_counter = result.block_counter;
  g_underwatercomm_fsk_rx_debug.processed_block_count = result.block_counter;

  const float total_power =
      result.weighted_low_power + result.weighted_high_power;
  const uint32_t now_ms = HAL_GetTick();

  // The first analysis block seeds the idle-power estimate. Frame gaps and the
  // 19 ms quiet part of each symbol continuously refine this estimate later.
  if (!g_noise_has_history) {
    g_noise_power = total_power;
    g_noise_has_history = true;
  }

  const float detect_threshold =
      Maximum(g_noise_power * DETECT_RATIO, MIN_TOTAL_POWER);
  const float release_threshold =
      Maximum(g_noise_power * RELEASE_RATIO, MIN_TOTAL_POWER);

  g_underwatercomm_fsk_rx_debug.noise_power = g_noise_power;
  g_underwatercomm_fsk_rx_debug.total_power = total_power;
  g_underwatercomm_fsk_rx_debug.detect_threshold = detect_threshold;
  g_underwatercomm_fsk_rx_debug.release_threshold = release_threshold;
  g_underwatercomm_fsk_rx_debug.fsk_score = result.fsk_score;

  if (g_signal_present) {
    if (total_power <= release_threshold) {
      // Arm the detector for the next symbol only after this burst has ended.
      g_signal_present = false;
    }
  } else {
    const bool spacing_ok =
        (g_underwatercomm_fsk_rx_debug.detected_bit_count == 0U) ||
        HasElapsed(now_ms, g_last_bit_ms, MIN_BIT_SPACING_MS);
    const bool score_is_clear = Absolute(result.fsk_score) >= MIN_ABS_SCORE;

    if ((total_power >= detect_threshold) && score_is_clear && spacing_ok) {
      g_signal_present = true;
      ConsumeBit((result.fsk_score > 0.0F) ? 1U : 0U, now_ms);
    } else {
      // Learn only while no burst is active. Including burst power here would
      // raise the threshold until valid symbols could no longer be detected.
      g_noise_power +=
          NOISE_FILTER_ALPHA * (total_power - g_noise_power);
    }
  }

  if ((g_state != DecoderState::SEARCH_PREAMBLE) &&
      HasElapsed(now_ms, g_last_bit_ms, BIT_TIMEOUT_MS)) {
    ++g_underwatercomm_fsk_rx_debug.timeout_count;
    UpdateFrameSuccessRate();
    ResetFrameParser();
  }

  UpdateStateDebug();
}

void FskDecoder::Stop() {
  g_underwatercomm_fsk_rx_debug.started = 0U;
  g_signal_present = false;
  ResetFrameParser();
}

}  // namespace underwatercomm
