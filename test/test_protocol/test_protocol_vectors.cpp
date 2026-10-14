// Vectores de bytes de specs/features/telemetry-streaming.md §5.
// Se usan TAL CUAL aquí y en el parser de la app Android:
// el firmware los empaqueta y la app los parsea exactamente igual.
#include <unity.h>

#include <cstring>

#include "protocol.h"

namespace {

using mcg::protocol;

// TELEMETRY #1: reposo
const uint8_t kTelemetryRest[] = {
    0x01, 0x2A, 0x02, 0x07, 0xD7, 0xFF, 0x78, 0x00,
    0xF2, 0x03, 0x46, 0x05, 0x0C, 0x00, 0x52, 0x00};

// TELEMETRY #2: moto caída y saturación
const uint8_t kTelemetryCrash[] = {
    0x01, 0xFF, 0x03, 0x21, 0x82, 0x05, 0x1C, 0xDF,
    0x80, 0x3E, 0x80, 0x3E, 0xE2, 0x04, 0xFF, 0x00};

// EVENT #1: impacto
const uint8_t kEventImpact[] = {
    0x01, 0x01, 0x03, 0x00, 0x39, 0x00, 0x40, 0xE2,
    0x01, 0x00, 0x6E, 0x14, 0x9C, 0x01, 0xA2, 0xFE,
    0xE4, 0x20, 0x00, 0x00};

// CONFIG: valores por defecto (escritura)
const uint8_t kConfigDefault[] = {
    0x01, 0x01, 0xAC, 0x0D, 0x2C, 0x01, 0x41, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// CONTROL_POINT: respuesta a TARE
const uint8_t kTareResponse[] = {0x01, 0x07, 0x00, 0x7E, 0xFA, 0x26, 0xFD};

void test_packet_sizes() {
    TEST_ASSERT_EQUAL_UINT8(16, sizeof(protocol::DeviceInfoPacket));
    TEST_ASSERT_EQUAL_UINT8(16, sizeof(protocol::TelemetryPacket));
    TEST_ASSERT_EQUAL_UINT8(20, sizeof(protocol::EventPacket));
    TEST_ASSERT_EQUAL_UINT8(16, sizeof(protocol::ConfigPacket));
    TEST_ASSERT_EQUAL_UINT8(16, sizeof(protocol::GpsPacket));
    TEST_ASSERT_EQUAL_UINT8(12, sizeof(protocol::ImuSample));
    TEST_ASSERT_EQUAL_UINT8(13, sizeof(protocol::TraceMetadata));
    TEST_ASSERT_EQUAL_UINT8(6, sizeof(protocol::TraceChunkHeader));
}

void test_telemetry_reposo() {
    protocol::TelemetryPacket p {};
    p.protocolVersion = 1;
    p.seq = 42;
    p.deviceState = protocol::kStateArmed;
    p.flags = 0x07;   // IMU OK · GPS presente · GPS con fix
    p.pitchCdeg = -41;    // −0.41°
    p.rollCdeg = 120;     // +1.20°
    p.accelMagMg = 1010;  // 1.010 g
    p.peakAccelMg = 1350;   // 1.350 g
    p.peakGyroDps = 12;
    p.batteryPct = 82;
    p.reserved = 0;

    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        kTelemetryRest, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void test_telemetry_caida_saturada() {
    protocol::TelemetryPacket p {};
    p.protocolVersion = 1;
    p.seq = 255;
    p.deviceState = protocol::kStatePostTrigger;
    p.flags = 0x21;   // IMU OK · saturado
    p.pitchCdeg = 1410;    // +14.10°
    p.rollCdeg = -8420;    // −84.20°
    p.accelMagMg = 16000;  // 16.000 g
    p.peakAccelMg = 16000;
    p.peakGyroDps = 1250;
    p.batteryPct = 255;    // desconocida
    p.reserved = 0;

    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        kTelemetryCrash, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void test_evento_impacto() {
    protocol::EventPacket p {};
    p.protocolVersion = 1;
    p.eventType = protocol::kEventImpact;
    p.eventId = 3;
    p.bootCount = 57;      // clave 57:3
    p.uptimeMs = 123456;
    p.peakAccelMg = 5230;
    p.peakGyroDps = 412;
    p.pitchCdeg = -350;    // −3.50°
    p.rollCdeg = 8420;     // +84.20°
    p.eventFlags = 0;
    p.faultCode = 0;

    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        kEventImpact, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void test_config_default() {
    protocol::ConfigPacket p {};
    p.protocolVersion = 1;
    p.flags = 0x01;   // detección de inclinación habilitada
    p.impactThresholdMg = 3500;
    p.gyroThresholdDps = 300;
    p.tiltThresholdDeg = 65;
    p.tiltHoldDs = 20;   // 2.0 s
    p.pitchOffsetCdeg = 0;
    p.rollOffsetCdeg = 0;
    p.configRevision = 0;
    p.reserved = 0;

    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        kConfigDefault, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void test_respuesta_tare() {
    // opcode TARE, requestId 7, resultado OK,
    // pitchOffset −14.10°, rollOffset −7.30°
    uint8_t buf[7];
    buf[0] = protocol::kOpTare;
    buf[1] = 7;
    buf[2] = protocol::kResultOk;
    const int16_t pitchOff = -1410;
    const int16_t rollOff = -730;
    memcpy(buf + 3, &pitchOff, 2);
    memcpy(buf + 5, &rollOff, 2);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(kTareResponse, buf, sizeof(buf));
}

} // namespace

int runUnityTests() {
    UNITY_BEGIN();
    RUN_TEST(test_packet_sizes);
    RUN_TEST(test_telemetry_reposo);
    RUN_TEST(test_telemetry_caida_saturada);
    RUN_TEST(test_evento_impacto);
    RUN_TEST(test_config_default);
    RUN_TEST(test_respuesta_tare);
    return UNITY_END();
}
