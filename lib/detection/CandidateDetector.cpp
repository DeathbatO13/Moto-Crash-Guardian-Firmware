#include "CandidateDetector.h"
#include <algorithm>
#include <cmath>

namespace mcg {

namespace {
constexpr int      kWindowSize       = 40;    // ±100 ms @ 200 Hz
constexpr int      kStillnessWindow  = 200;   // 1 s @ 200 Hz
constexpr uint32_t kLookaheadMs      = 100;   // IMPACT_LOOKAHEAD_MS
constexpr uint32_t kPostTriggerMs    = 3000;  // POST_TRIGGER_MS
constexpr uint32_t kCooldownMs       = 10000; // COOLDOWN_MS
constexpr uint32_t kTiltImpactGapMs  = 5000;  // TILT_IMPACT_GAP_MS
constexpr uint16_t kStillTolMg       = 50;    // TARE_STILLNESS_TOL_MG
constexpr uint16_t kStillGyroDps     = 5;     // TARE_STILLNESS_GYRO_DPS
constexpr uint16_t kAccelSatRaw      = 32767; // MPU6050_FULL_SCALE
constexpr float    kAccelLsbPerG     = 2048.0f;
constexpr float    kGyroLsbPerDps    = 16.4f;
}

CandidateDetector::CandidateDetector() {
    configure(DetectionConfig{});
}

void CandidateDetector::configure(const DetectionConfig& cfg) {
    m_cfg = cfg;
}

void CandidateDetector::arm() {
    if (m_state == protocol::kStateDisarmed ||
        m_state == protocol::kStateCooldown) {
        m_state = protocol::kStateArmed;
        m_tiltActive = false;
    }
}

void CandidateDetector::disarm() {
    if (m_state != protocol::kStateBoot && m_state != protocol::kStateFault) {
        m_state = protocol::kStateDisarmed;
        m_triggerPending = false;
        m_tiltActive = false;
    }
}

void CandidateDetector::enterFault() {
    m_state = protocol::kStateFault;
    m_triggerPending = false;
    m_tiltActive = false;
}

void CandidateDetector::recoverFromFault() {
    // FAULT → BOOT → DISARMED (docs/11 §4)
    m_state = protocol::kStateDisarmed;
}

bool CandidateDetector::armed() const {
    return m_state == protocol::kStateArmed;
}

uint8_t CandidateDetector::state() const {
    return m_state;
}

uint16_t CandidateDetector::allocateEventId() {
    return m_eventId++;
}

void CandidateDetector::pushWindow(uint16_t accelMg, uint16_t gyroDps) {
    m_window[m_windowIdx] = {accelMg, gyroDps};
    m_windowIdx = (m_windowIdx + 1) % kWindowSize;
    if (m_windowCount < kWindowSize) ++m_windowCount;

    m_stillness[m_stillIdx] = {accelMg, gyroDps};
    m_stillIdx = (m_stillIdx + 1) % kStillnessWindow;
    if (m_stillCount < kStillnessWindow) ++m_stillCount;
}

void CandidateDetector::beginTrigger(uint8_t eventType, uint32_t nowMs,
                                       int16_t pitchCdeg, int16_t rollCdeg) {
    m_triggerPending = true;
    m_triggerType = eventType;
    m_triggerMs = nowMs;
    m_triggerLookaheadMs = nowMs + kLookaheadMs;
    m_triggerPitchCdeg = pitchCdeg;
    m_triggerRollCdeg = rollCdeg;
    m_postTriggerStartMs = nowMs;
    if (eventType == protocol::kEventImpact) {
        m_lastImpactMs = nowMs;
        m_hadImpact = true;
    }
    m_state = protocol::kStatePostTrigger;
    m_tiltActive = false;
}

std::optional<CandidateEvent> CandidateDetector::process(
        const protocol::ImuSample& s, int16_t pitchCdeg, int16_t rollCdeg,
        uint32_t nowMs) {

    const float accelMagG = std::sqrt(static_cast<float>(s.ax) * s.ax +
                                      static_cast<float>(s.ay) * s.ay +
                                      static_cast<float>(s.az) * s.az) / kAccelLsbPerG;
    const float gyroMagDps = std::sqrt(static_cast<float>(s.gx) * s.gx +
                                       static_cast<float>(s.gy) * s.gy +
                                       static_cast<float>(s.gz) * s.gz) / kGyroLsbPerDps;
    const uint16_t accelMg = static_cast<uint16_t>(accelMagG * 1000.0f + 0.5f);
    const uint16_t gyroDps = static_cast<uint16_t>(gyroMagDps + 0.5f);
    const bool saturated =
        std::abs(s.ax) >= kAccelSatRaw || std::abs(s.ay) >= kAccelSatRaw ||
        std::abs(s.az) >= kAccelSatRaw || std::abs(s.gx) >= kAccelSatRaw ||
        std::abs(s.gy) >= kAccelSatRaw || std::abs(s.gz) >= kAccelSatRaw;

    m_lastAccelMg = accelMg;
    m_lastGyroDps = gyroDps;
    m_lastSaturated = saturated;
    m_lastPitchCdeg = pitchCdeg;
    m_lastRollCdeg = rollCdeg;
    pushWindow(accelMg, gyroDps);

    // --- POST_TRIGGER: emite el evento a los 100 ms con los picos ---
    if (m_state == protocol::kStatePostTrigger) {
        if (m_triggerPending && nowMs >= m_triggerLookaheadMs) {
            m_triggerPending = false;
            uint16_t peakA = 0, peakG = 0;
            for (int i = 0; i < m_windowCount; ++i) {
                peakA = std::max(peakA, m_window[i].accelMg);
                peakG = std::max(peakG, m_window[i].gyroDps);
            }
            CandidateEvent ev {};
            ev.eventType = m_triggerType;
            ev.eventId = m_eventId++;
            ev.uptimeMs = m_triggerMs;
            ev.peakAccelMg = peakA;
            ev.peakGyroDps = peakG;
            ev.pitchCdeg = m_triggerPitchCdeg;
            ev.rollCdeg = m_triggerRollCdeg;
            ev.eventFlags = saturated ? protocol::kEventFlagSaturated : 0;
            return ev;
        }
        if (nowMs - m_postTriggerStartMs >= kPostTriggerMs) {
            m_state = protocol::kStateCooldown;
            m_cooldownStartMs = nowMs;
        }
        return std::nullopt;
    }

    // --- COOLDOWN: 10 s sin candidatos nuevos ---
    if (m_state == protocol::kStateCooldown) {
        if (nowMs - m_cooldownStartMs >= kCooldownMs) {
            m_state = protocol::kStateArmed;   // docs/11 §4
        }
        return std::nullopt;
    }

    if (m_state != protocol::kStateArmed) {
        return std::nullopt;
    }

    // --- Detección en ARMED ---
    const bool impact = accelMg >= m_cfg.impactThresholdMg ||
                        gyroDps >= m_cfg.gyroThresholdDps;
    if (impact) {
        beginTrigger(protocol::kEventImpact, nowMs, pitchCdeg, rollCdeg);
        return std::nullopt;
    }

    if (m_cfg.tiltEnabled) {
        const int16_t maxTilt =
            std::max(std::abs(pitchCdeg), std::abs(rollCdeg));
        if (maxTilt >= m_cfg.tiltThresholdDeg * 100) {
            if (!m_tiltActive) {
                m_tiltActive = true;
                m_tiltStartMs = nowMs;
            }
            const bool impactRecently =
                m_hadImpact && (nowMs - m_lastImpactMs < kTiltImpactGapMs);
            if (nowMs - m_tiltStartMs >= m_cfg.tiltHoldMs && !impactRecently) {
                beginTrigger(protocol::kEventTilt, nowMs, pitchCdeg, rollCdeg);
                return std::nullopt;
            }
        } else {
            m_tiltActive = false;
        }
    }

    return std::nullopt;
}

bool CandidateDetector::isStill() const {
    if (m_stillCount < kStillnessWindow) return false;
    for (int i = 0; i < kStillnessWindow; ++i) {
        if (std::abs(static_cast<int>(m_stillness[i].accelMg) - 1000) > kStillTolMg) {
            return false;
        }
        if (m_stillness[i].gyroDps > kStillGyroDps) {
            return false;
        }
    }
    return true;
}

uint16_t CandidateDetector::lastAccelMg() const { return m_lastAccelMg; }
uint16_t CandidateDetector::lastGyroDps() const { return m_lastGyroDps; }
bool CandidateDetector::lastSaturated() const { return m_lastSaturated; }

} // namespace mcg
