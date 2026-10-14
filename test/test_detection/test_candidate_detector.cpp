// Pruebas del detector de candidatos (docs/09 §2.3,
// specs/features/crash-detection.md). Lógica pura, sin Arduino.
#include <unity.h>

#include "CandidateDetector.h"
#include "protocol.h"

namespace {

using mcg::CandidateDetector;
using mcg::DetectionConfig;
using mcg::protocol::ImuSample;

DetectionConfig defaultCfg() {
    DetectionConfig cfg;
    cfg.tiltEnabled = true;
    cfg.impactThresholdMg = 3500;
    cfg.gyroThresholdDps = 300;
    cfg.tiltThresholdDeg = 65;
    cfg.tiltHoldMs = 2000;
    return cfg;
}

// Muestra en reposo: 1 g en Z (≈2048 LSB), giroscopio en 0
ImuSample atRest() {
    return ImuSample{0, 0, 2048, 0, 0, 0};
}

// Impacto de ~4 g sobre Z (≈4000 mg ≥ 3500 mg)
ImuSample impact4g() {
    return ImuSample{0, 0, 8192, 0, 0, 0};
}

// Giroscopio de ~300 °/s (≈4920 LSB ≥ umbral)
ImuSample gyro300dps() {
    return ImuSample{0, 0, 2048, 4920, 0, 0};
}

void feed(CandidateDetector& d, const ImuSample& s,
             uint32_t fromMs, uint32_t toMs,
             int16_t pitchCdeg = 0, int16_t rollCdeg = 0) {
    for (uint32_t t = fromMs; t < toMs; t += 5) {
        d.process(s, pitchCdeg, rollCdeg, t);
    }
}

void test_reposo_no_dispara() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 0; t < 10000; t += 5) {
        ev = d.process(atRest(), 0, 0, t);
        if (ev) break;
    }
    TEST_ASSERT_FALSE(ev.has_value());
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateArmed, d.state());
}

void test_impac_dispara_evento() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    feed(d, atRest(), 0, 100);

    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 100; t < 2000; t += 5) {
        const ImuSample s = (t < 105) ? impact4g() : atRest();
        ev = d.process(s, 0, 0, t);
        if (ev) break;
    }

    TEST_ASSERT_TRUE(ev.has_value());
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kEventImpact, ev->eventType);
    TEST_ASSERT_EQUAL_UINT16(0, ev->eventId);
    TEST_ASSERT_TRUE(ev->peakAccelMg >= 4000);
    // El evento llega ~100 ms después del disparo
    TEST_ASSERT_TRUE(ev->uptimeMs >= 100 && ev->uptimeMs <= 105);
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStatePostTrigger, d.state());
}

void test_giroscopio_dispara_impacto() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 0; t < 500; t += 5) {
        const ImuSample s = (t < 5) ? gyro300dps() : atRest();
        ev = d.process(s, 0, 0, t);
        if (ev) break;
    }

    TEST_ASSERT_TRUE(ev.has_value());
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kEventImpact, ev->eventType);
    TEST_ASSERT_TRUE(ev->peakGyroDps >= 300);
}

void test_bajo_umbral_no_dispara() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    // ~3499 mg: apenas bajo el umbral de 3500 mg
    const ImuSample below{0, 0, 7165, 0, 0, 0};
    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 0; t < 3000; t += 5) {
        ev = d.process(below, 0, 0, t);
        if (ev) break;
    }
    TEST_ASSERT_FALSE(ev.has_value());
}

void test_tilt_sostenido_dispara() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 0; t < 4000; t += 5) {
        ev = d.process(atRest(), 7000, 0, t);   // 70° ≥ 65°
        if (ev) break;
    }

    TEST_ASSERT_TRUE(ev.has_value());
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kEventTilt, ev->eventType);
    TEST_ASSERT_EQUAL_INT16(7000, ev->pitchCdeg);
}

void test_tilt_breve_no_dispara() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    // 70° solo por 1000 ms (< tiltHoldMs de 2000 ms)
    for (uint32_t t = 0; t < 1000; t += 5) {
        TEST_ASSERT_FALSE(d.process(atRest(), 7000, 0, t).has_value());
    }
    // Vuelve a la vertical
    for (uint32_t t = 1000; t < 4000; t += 5) {
        TEST_ASSERT_FALSE(d.process(atRest(), 0, 0, t).has_value());
    }
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateArmed, d.state());
}

void test_tilt_suprimido_tras_impacto() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    // Impacto en t=0
    d.process(impact4g(), 0, 0, 0);
    // Se desarma y re-arma 1 s después del impacto
    d.disarm();
    d.arm();

    // Inclinación sostenida dentro de la ventana de 5 s
    // del impacto: no debe disparar
    for (uint32_t t = 1000; t < 4995; t += 5) {
        TEST_ASSERT_FALSE(d.process(atRest(), 7000, 0, t).has_value());
    }
    // Pasados 5 s del impacto, la inclinación sostenida sí dispara
    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 4995; t < 6000; t += 5) {
        ev = d.process(atRest(), 7000, 0, t);
        if (ev) break;
    }
    TEST_ASSERT_TRUE(ev.has_value());
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kEventTilt, ev->eventType);
}

void test_transiciones_de_estado() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    d.process(impact4g(), 0, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStatePostTrigger, d.state());

    // POST_TRIGGER dura 3 s
    for (uint32_t t = 5; t < 3000; t += 5) {
        d.process(atRest(), 0, 0, t);
    }
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateCooldown, d.state());

    // COOLDOWN dura 10 s y vuelve a ARMED
    for (uint32_t t = 3000; t < 13000; t += 5) {
        d.process(atRest(), 0, 0, t);
    }
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateArmed, d.state());
}

void test_cooldown_ignora_nuevos_disparos() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    d.process(impact4g(), 0, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStatePostTrigger, d.state());

    // Segundo impacto durante POST_TRIGGER: no genera otro evento
    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 5; t < 3000; t += 5) {
        const ImuSample s = (t == 100) ? impact4g() : atRest();
        ev = d.process(s, 0, 0, t);
    }
    TEST_ASSERT_FALSE(ev.has_value());
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateCooldown, d.state());
}

void test_disarm_impide_deteccion() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();
    d.disarm();

    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 0; t < 2000; t += 5) {
        ev = d.process(impact4g(), 0, 0, t);
        if (ev) break;
    }
    TEST_ASSERT_FALSE(ev.has_value());
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateDisarmed, d.state());
}

void test_tilt_deshabilitado() {
    DetectionConfig cfg = defaultCfg();
    cfg.tiltEnabled = false;
    CandidateDetector d;
    d.configure(cfg);
    d.arm();

    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 0; t < 4000; t += 5) {
        ev = d.process(atRest(), 8000, 8000, t);
        if (ev) break;
    }
    TEST_ASSERT_FALSE(ev.has_value());
}

void test_captura_de_picos() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    // Impacto de 4 g, luego uno mayor (6 g) dentro de ±100 ms
    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 0; t < 300; t += 5) {
        ImuSample s = atRest();
        if (t == 100) s = impact4g();
        if (t == 120) s = ImuSample{0, 0, 12288, 0, 0, 0};   // 6 g
        ev = d.process(s, 0, 0, t);
        if (ev) break;
    }

    TEST_ASSERT_TRUE(ev.has_value());
    TEST_ASSERT_TRUE(ev->peakAccelMg >= 6000);
}

void test_isStill_para_tare() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    // Sin ventana llena todavía: no quietud
    TEST_ASSERT_FALSE(d.isStill());

    // 1 s de reposo: quietud
    for (uint32_t t = 0; t < 1000; t += 5) {
        d.process(atRest(), 0, 0, t);
    }
    TEST_ASSERT_TRUE(d.isStill());

    // Movimiento: pierde la quietud
    for (uint32_t t = 1000; t < 1200; t += 5) {
        d.process(ImuSample{0, 0, 6144, 0, 0, 0}, 0, 0, t);   // 3 g
    }
    TEST_ASSERT_FALSE(d.isStill());

    // Se recupera tras 1 s de reposo (la ventana se renueva)
    for (uint32_t t = 1200; t < 2200; t += 5) {
        d.process(atRest(), 0, 0, t);
    }
    TEST_ASSERT_TRUE(d.isStill());
}

void test_event_id_incrementa() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    d.process(impact4g(), 0, 0, 0);
    // Espera POST_TRIGGER + COOLDOWN (13 s) y dispara de nuevo
    for (uint32_t t = 5; t < 13000; t += 5) {
        d.process(atRest(), 0, 0, t);
    }
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateArmed, d.state());

    d.process(impact4g(), 0, 0, 13000);
    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 13005; t < 13200; t += 5) {
        ev = d.process(atRest(), 0, 0, t);
        if (ev) break;
    }
    TEST_ASSERT_TRUE(ev.has_value());
    TEST_ASSERT_EQUAL_UINT16(1, ev->eventId);
}

void test_fault_y_recuperacion() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    d.enterFault();
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateFault, d.state());

    // En FAULT no se detecta nada
    std::optional<mcg::CandidateEvent> ev;
    for (uint32_t t = 0; t < 2000; t += 5) {
        ev = d.process(impact4g(), 0, 0, t);
        if (ev) break;
    }
    TEST_ASSERT_FALSE(ev.has_value());

    d.recoverFromFault();
    TEST_ASSERT_EQUAL_UINT8(mcg::protocol::kStateDisarmed, d.state());
    d.arm();
    TEST_ASSERT_TRUE(d.armed());
}

void test_tare_busy_en_movimiento() {
    CandidateDetector d;
    d.configure(defaultCfg());
    d.arm();

    // Ventana con movimiento reciente → no apto para TARE
    for (uint32_t t = 0; t < 1000; t += 5) {
        d.process(ImuSample{0, 0, 8192, 0, 0, 0}, 0, 0, t);
    }
    TEST_ASSERT_FALSE(d.isStill());
}

} // namespace

int runUnityTests() {
    UNITY_BEGIN();
    RUN_TEST(test_reposo_no_dispara);
    RUN_TEST(test_impac_dispara_evento);
    RUN_TEST(test_giroscopio_dispara_impacto);
    RUN_TEST(test_bajo_umbral_no_dispara);
    RUN_TEST(test_tilt_sostenido_dispara);
    RUN_TEST(test_tilt_breve_no_dispara);
    RUN_TEST(test_tilt_suprimido_tras_impacto);
    RUN_TEST(test_transiciones_de_estado);
    RUN_TEST(test_cooldown_ignora_nuevos_disparos);
    RUN_TEST(test_disarm_impide_deteccion);
    RUN_TEST(test_tilt_deshabilitado);
    RUN_TEST(test_captura_de_picos);
    RUN_TEST(test_isStill_para_tare);
    RUN_TEST(test_event_id_incrementa);
    RUN_TEST(test_fault_y_recuperacion);
    RUN_TEST(test_tare_busy_en_movimiento);
    return UNITY_END();
}
