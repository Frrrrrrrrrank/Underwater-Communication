#pragma once

#include "tim.h"

/*
 * Buzzer driver for PC0 / TIM1_CH1.
 *
 * CubeMX config used by this file:
 *   TIM1 clock = 170 MHz
 *   Prescaler  = 3399
 *   Timer tick = 50 kHz
 *
 * The driver is intentionally header-only to match the copied project style.
 * All functions are non-blocking; time-limited tones and boot patterns are
 * advanced by calling process() from the main loop.
 */
namespace Drivers::Buzzer {

namespace detail {

constexpr uint32_t TIMER_TICK_HZ = 50000U;
constexpr uint32_t MIN_FREQUENCY_HZ = 100U;
constexpr uint32_t MAX_FREQUENCY_HZ = 10000U;
constexpr uint32_t DUTY_PERCENT = 50U;

struct BeepStep {
    uint32_t frequencyHz;
    uint32_t durationMs;
    uint32_t gapAfterMs;
};

enum class PatternState {
    IDLE,
    PLAYING,
    GAP,
    DONE,
};

// TX startup prompt: three short equal beeps, "beep beep beep".
constexpr BeepStep TX_BOOT_PATTERN[] = {
    {2500U, 80U, 80U},
    {2500U, 80U, 80U},
    {2500U, 80U, 0U},
};

// RX startup prompt: high then low, "beep boop".
constexpr BeepStep RX_BOOT_PATTERN[] = {
    {2500U, 100U, 80U},
    {1300U, 180U, 0U},
};

inline bool timedToneActive = false;
inline uint32_t toneStartMs = 0U;
inline uint32_t toneDurationMs = 0U;

inline const BeepStep *activePattern = nullptr;
inline uint32_t activePatternCount = 0U;
inline uint32_t activePatternIndex = 0U;
inline uint32_t patternStateStartMs = 0U;
inline PatternState patternState = PatternState::DONE;

inline uint32_t clampFrequency(uint32_t frequencyHz) {
    if (frequencyHz < MIN_FREQUENCY_HZ) {
        return MIN_FREQUENCY_HZ;
    }

    if (frequencyHz > MAX_FREQUENCY_HZ) {
        return MAX_FREQUENCY_HZ;
    }

    return frequencyHz;
}

inline uint32_t calculatePeriodTicks(uint32_t frequencyHz) {
    // Round to the nearest timer period so common tones stay close to request.
    const uint32_t roundedPeriod =
        (TIMER_TICK_HZ + (frequencyHz / 2U)) / frequencyHz;

    // Keep ARR valid and leave at least one high tick and one low tick.
    return (roundedPeriod >= 2U) ? roundedPeriod : 2U;
}

inline bool hasElapsed(uint32_t nowMs, uint32_t startMs, uint32_t durationMs) {
    // Unsigned subtraction keeps the comparison correct after HAL_GetTick wraps.
    return (nowMs - startMs) >= durationMs;
}

inline uint32_t countOf(const BeepStep *pattern, uint32_t count) {
    return (pattern != nullptr) ? count : 0U;
}

}  // namespace detail

/**
 * @brief Stop the acoustic output but keep TIM1 configured.
 *
 * Setting CCR1 to zero is enough to make PC0 stay inactive. We do not deinit
 * TIM1 because the next tone should start quickly and CubeMX owns the setup.
 */
inline void stop() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    detail::timedToneActive = false;
}

/**
 * @brief Prepare PC0 PWM output for later tones.
 *
 * PWM is started once with zero duty. Later play() calls only update ARR/CCR,
 * which avoids repeated GPIO/timer setup in the main loop.
 */
inline void init() {
    stop();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/**
 * @brief Play a continuous tone until stop() or another play() call.
 */
inline void play(uint32_t frequencyHz) {
    if (frequencyHz == 0U) {
        stop();
        return;
    }

    const uint32_t safeFrequencyHz = detail::clampFrequency(frequencyHz);
    const uint32_t periodTicks = detail::calculatePeriodTicks(safeFrequencyHz);
    const uint32_t compareTicks = (periodTicks * detail::DUTY_PERCENT) / 100U;

    // ARR is period - 1 because TIM counts from 0 through ARR.
    __HAL_TIM_SET_AUTORELOAD(&htim1, periodTicks - 1U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compareTicks);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    detail::timedToneActive = false;
}

/**
 * @brief Play one tone for durationMs without blocking the main loop.
 */
inline void play(uint32_t frequencyHz, uint32_t durationMs) {
    if (durationMs == 0U) {
        stop();
        return;
    }

    play(frequencyHz);
    detail::toneStartMs = HAL_GetTick();
    detail::toneDurationMs = durationMs;
    detail::timedToneActive = true;
}

/**
 * @brief Start a non-blocking multi-tone startup pattern.
 */
inline void startPattern(const detail::BeepStep *pattern, uint32_t count) {
    stop();
    detail::activePattern = pattern;
    detail::activePatternCount = detail::countOf(pattern, count);
    detail::activePatternIndex = 0U;
    detail::patternStateStartMs = 0U;
    detail::patternState =
        (detail::activePatternCount > 0U) ? detail::PatternState::IDLE
                                          : detail::PatternState::DONE;
}

inline void playTxBootPattern() {
    startPattern(detail::TX_BOOT_PATTERN,
                 static_cast<uint32_t>(sizeof(detail::TX_BOOT_PATTERN) /
                                       sizeof(detail::TX_BOOT_PATTERN[0])));
}

inline void playRxBootPattern() {
    startPattern(detail::RX_BOOT_PATTERN,
                 static_cast<uint32_t>(sizeof(detail::RX_BOOT_PATTERN) /
                                       sizeof(detail::RX_BOOT_PATTERN[0])));
}

/**
 * @brief Advance timed tones and boot patterns.
 *
 * This must be called often from the main loop. It never calls HAL_Delay(), so
 * it does not block ADC DMA processing or TX burst timing.
 */
inline void process() {
    const uint32_t nowMs = HAL_GetTick();

    if (detail::timedToneActive &&
        detail::hasElapsed(nowMs, detail::toneStartMs, detail::toneDurationMs)) {
        stop();
    }

    if ((detail::activePattern == nullptr) ||
        (detail::patternState == detail::PatternState::DONE)) {
        return;
    }

    const detail::BeepStep &step =
        detail::activePattern[detail::activePatternIndex];

    switch (detail::patternState) {
        case detail::PatternState::IDLE:
            play(step.frequencyHz);
            detail::patternStateStartMs = nowMs;
            detail::patternState = detail::PatternState::PLAYING;
            break;

        case detail::PatternState::PLAYING:
            if (detail::hasElapsed(nowMs, detail::patternStateStartMs,
                                   step.durationMs)) {
                stop();
                detail::patternStateStartMs = nowMs;
                detail::patternState =
                    (step.gapAfterMs > 0U) ? detail::PatternState::GAP
                                           : detail::PatternState::IDLE;

                if (step.gapAfterMs == 0U) {
                    ++detail::activePatternIndex;
                }
            }
            break;

        case detail::PatternState::GAP:
            if (detail::hasElapsed(nowMs, detail::patternStateStartMs,
                                   step.gapAfterMs)) {
                ++detail::activePatternIndex;
                detail::patternState = detail::PatternState::IDLE;
            }
            break;

        case detail::PatternState::DONE:
        default:
            break;
    }

    if (detail::activePatternIndex >= detail::activePatternCount) {
        stop();
        detail::patternState = detail::PatternState::DONE;
    }
}

/**
 * @brief Compatibility name from the copied project.
 */
inline void onTIM1UpdateCallback() {
    stop();
}

}  // namespace Drivers::Buzzer
