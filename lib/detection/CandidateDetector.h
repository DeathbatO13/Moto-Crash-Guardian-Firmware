// CandidateDetector — detector de candidatos a accidente (docs/11 §6).
// C++ puro (sin Arduino): se alimenta de muestras crutas y la orientación
// calibrada; emite IMPACT / TILT con los picos de ±100 ms y maneja las
// transiciones POST_TRIGGER (3 s) y COOLDOWN (10 s). Se prueba en native.
#pragma once

#include <cstdint>
#include <optional>

#include "protocol.h"

namespace mcg {

struct DetectionConfig {
    bool     tiltEnabled       = true;
    uint16_t impactThresholdMg = 3500;
    uint16_t gyroThresholdDps  = 300;
    uint8_t  tiltThresholdDeg  = 65;
    uint16_t tiltHoldMs        = 2000;
};

// Evento candidato listo para enviar por BLE (docs/03 §4.3 sin bootCount,
// que asigna la capa de aplicación).
struct CandidateEvent {
    uint8_t  eventType;     // protocol::EventType
    uint16_t eventId;
    uint32_t uptimeMs;
    uint16_t peakAccelMg;
    uint16_t peakGyroDps;
    int16_t  pitchCdeg;
    int16_t  rollCdeg;
    uint8_t  eventFlags;    // bit0 saturado
};

class CandidateDetector {
public:
    CandidateDetector();

    void configure(const DetectionConfig& cfg);
    void arm();
    void disarm();
    void enterFault();        // → FAULT (bus I²C, batería crítica)
    void recoverFromFault();  // FAULT → DISARMED (docs/11 §4)

    bool     armed() const;
    uint8_t  state() const;   // protocol::DeviceState

    // Una muestra @ 200 Hz con la orientación calibrada (cdeg).
    // Devuelve un evento ~100 ms después del disparo (picos en ±100 ms).
    std::optional<CandidateEvent> process(const protocol::ImuSample& s,
                                          int16_t pitchCdeg, int16_t rollCdeg,
                                          uint32_t nowMs);

    // Reserva un eventId sin disparar (para eventos TEST y FAULT).
    uint16_t allocateEventId();

    // Condición de quietud para TARE (docs/03 §4.5): el último segundo
    // dentro de 1 g ± 50 mg y |ω| ≤ 5 °/s.
    bool isStill() const;

    // Magnitudes de la última muestra (telemetría y depuración).
    uint16_t lastAccelMg() const;
    uint16_t lastGyroDps() const;
    bool     lastSaturated() const;

private:
    struct WindowSample {
        uint16_t accelMg;
        uint16_t gyroDps;
    };

    void pushWindow(uint16_t accelMg, uint16_t gyroDps);
    void beginTrigger(uint8_t eventType, uint32_t nowMs,
                      int16_t pitchCdeg, int16_t rollCdeg);

    DetectionConfig m_cfg;
    uint8_t  m_state   = protocol::kStateDisarmed;
    uint16_t m_eventId = 0;

    WindowSample m_window[40] {};   // ±100 ms @ 200 Hz
    uint8_t      m_windowIdx = 0;
    uint8_t      m_windowCount = 0;

    WindowSample m_stillness[200] {};  // 1 s @ 200 Hz (TARE)
    uint16_t     m_stillIdx = 0;
    uint16_t     m_stillCount = 0;

    uint16_t m_lastAccelMg = 1000;
    uint16_t m_lastGyroDps = 0;
    bool     m_lastSaturated = false;
    int16_t  m_lastPitchCdeg = 0;
    int16_t  m_lastRollCdeg = 0;

    uint32_t m_lastImpactMs = 0;
    bool     m_hadImpact = false;

    // Disparo en curso: se esperan 20 muestras para capturar los picos.
    bool     m_triggerPending = false;
    uint8_t  m_triggerType = 0;
    uint32_t m_triggerMs = 0;
    uint32_t m_triggerLookaheadMs = 0;
    int16_t  m_triggerPitchCdeg = 0;
    int16_t  m_triggerRollCdeg = 0;
    uint32_t m_postTriggerStartMs = 0;
    uint32_t m_cooldownStartMs = 0;

    // TILT: tiempo acumulado por encima del umbral.
    uint32_t m_tiltStartMs = 0;
    bool     m_tiltActive = false;
};

} // namespace mcg
