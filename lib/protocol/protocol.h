// protocol.h — estructuras empaquetadas del protocolo BLE v1 (docs/03 §3–§4)
// C++ puro (sin Arduino): comparte el contrato con las pruebas nativas y
// con el parser de la app Android (specs/features/telemetry-streaming.md §5).
//
// Todas las tramas son little-endian. El ESP32 es little-endian, así que el
// empaquetado/parseo es un memcpy directo del struct.
#pragma once

#include <cstdint>
#include <cstddef>

namespace mcg {
namespace protocol {

constexpr uint8_t kProtocolVersion = 1;

// --- UUIDs (docs/03 §3) ---
inline constexpr const char* kServiceUuid      = "a3c1f000-5b2e-4d7a-9f41-6c2e8b7d0e10";
inline constexpr const char* kDeviceInfoUuid   = "a3c1f001-5b2e-4d7a-9f41-6c2e8b7d0e10";
inline constexpr const char* kTelemetryUuid    = "a3c1f002-5b2e-4d7a-9f41-6c2e8b7d0e10";
inline constexpr const char* kEventUuid        = "a3c1f003-5b2e-4d7a-9f41-6c2e8b7d0e10";
inline constexpr const char* kConfigUuid       = "a3c1f004-5b2e-4d7a-9f41-6c2e8b7d0e10";
inline constexpr const char* kControlPointUuid = "a3c1f005-5b2e-4d7a-9f41-6c2e8b7d0e10";
inline constexpr const char* kGpsUuid          = "a3c1f006-5b2e-4d7a-9f41-6c2e8b7d0e10";
inline constexpr const char* kTraceUuid        = "a3c1f007-5b2e-4d7a-9f41-6c2e8b7d0e10";

inline constexpr uint16_t kBatteryServiceUuid = 0x180F;
inline constexpr uint16_t kBatteryLevelUuid   = 0x2A19;

// --- Estados del dispositivo (docs/03 §5) ---
enum DeviceState : uint8_t {
    kStateBoot        = 0,
    kStateDisarmed    = 1,
    kStateArmed       = 2,
    kStatePostTrigger = 3,
    kStateCooldown    = 4,
    kStateCalibrating = 5,
    kStateFault       = 6,
};

// --- Tipos de evento (docs/03 §4.3) ---
enum EventType : uint8_t {
    kEventImpact = 1,
    kEventTilt   = 2,
    kEventTest   = 3,
    kEventFault  = 4,
};

// --- Flags de TELEMETRY (docs/03 §4.2) ---
enum TelemetryFlags : uint8_t {
    kFlagImuOk      = 1 << 0,
    kFlagGpsPresent = 1 << 1,
    kFlagGpsFix     = 1 << 2,
    kFlagCharging   = 1 << 3,
    kFlagBatteryLow = 1 << 4,
    kFlagSaturated  = 1 << 5,
};

// --- Flags de EVENT (docs/03 §4.3) ---
enum EventFlags : uint8_t {
    kEventFlagSaturated = 1 << 0,
    kEventFlagTrace     = 1 << 1,
    kEventFlagResent    = 1 << 2,
};

// --- Códigos de error de faultCode (docs/03 §4.3) ---
enum FaultCode : uint8_t {
    kFaultImuNoResponse = 1,
    kFaultI2CBus        = 2,
    kFaultBatteryCrit   = 3,
};

// --- Resultados de CONTROL_POINT (docs/03 §4.5) ---
enum ControlResult : uint8_t {
    kResultOk           = 0,
    kResultUnsupported  = 1,
    kResultInvalidParam = 2,
    kResultBusy         = 3,
    kResultFailed       = 4,
    kResultNotFound     = 5,
};

// --- Opcodes de CONTROL_POINT (docs/03 §4.5) ---
enum ControlOpcode : uint8_t {
    kOpTare             = 0x01,
    kOpArm              = 0x02,
    kOpDisarm           = 0x03,
    kOpTriggerTestEvent = 0x04,
    kOpRequestTrace     = 0x05,
    kOpAckEvent         = 0x06,
    kOpReboot           = 0x07,
    kOpFactoryReset     = 0x08,
};

// --- Muestra cruda de la IMU (12 B; también es el formato de la caja negra) ---
struct __attribute__((packed)) ImuSample {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};
static_assert(sizeof(ImuSample) == 12, "ImuSample must be 12 bytes");

// --- DEVICE_INFO (16 B, read) ---
struct __attribute__((packed)) DeviceInfoPacket {
    uint8_t  protocolVersion;
    uint8_t  fwMajor;
    uint8_t  fwMinor;
    uint8_t  fwPatch;
    uint8_t  hwRevision;
    uint8_t  capabilities;   // bit0 GPS · bit1 caja negra · bit2 batería
    uint16_t bootCount;
    uint8_t  deviceId[6];    // MAC Bluetooth
    uint16_t reserved;
};
static_assert(sizeof(DeviceInfoPacket) == 16, "DEVICE_INFO must be 16 bytes");

// --- TELEMETRY (16 B, notify 5 Hz) ---
struct __attribute__((packed)) TelemetryPacket {
    uint8_t  protocolVersion;
    uint8_t  seq;
    uint8_t  deviceState;
    uint8_t  flags;
    int16_t  pitchCdeg;
    int16_t  rollCdeg;
    uint16_t accelMagMg;
    uint16_t peakAccelMg;
    uint16_t peakGyroDps;
    uint8_t  batteryPct;
    uint8_t  reserved;
};
static_assert(sizeof(TelemetryPacket) == 16, "TELEMETRY must be 16 bytes");

// --- EVENT (20 B, indicate) ---
struct __attribute__((packed)) EventPacket {
    uint8_t  protocolVersion;
    uint8_t  eventType;
    uint16_t eventId;
    uint16_t bootCount;
    uint32_t uptimeMs;
    uint16_t peakAccelMg;
    uint16_t peakGyroDps;
    int16_t  pitchCdeg;
    int16_t  rollCdeg;
    uint8_t  eventFlags;
    uint8_t  faultCode;
};
static_assert(sizeof(EventPacket) == 20, "EVENT must be 20 bytes");

// --- CONFIG (16 B, read/write) ---
struct __attribute__((packed)) ConfigPacket {
    uint8_t  protocolVersion;
    uint8_t  flags;             // bit0 detección de inclinación
    uint16_t impactThresholdMg;
    uint16_t gyroThresholdDps;
    uint8_t  tiltThresholdDeg;
    uint8_t  tiltHoldDs;        // décimas de segundo
    int16_t  pitchOffsetCdeg;   // no escribible (solo TARE)
    int16_t  rollOffsetCdeg;    // no escribible (solo TARE)
    uint16_t configRevision;
    uint16_t reserved;
};
static_assert(sizeof(ConfigPacket) == 16, "CONFIG must be 16 bytes");

// --- GPS (16 B, notify 1 Hz) ---
struct __attribute__((packed)) GpsPacket {
    uint8_t  protocolVersion;
    uint8_t  fixType;     // 0 sin fix · 2 2D · 3 3D
    uint8_t  satellites;
    uint8_t  hdopX10;
    int32_t  latE7;
    int32_t  lonE7;
    uint16_t speedKmhX100;
    uint16_t fixAgeS;     // 65535 = nunca
};
static_assert(sizeof(GpsPacket) == 16, "GPS must be 16 bytes");

// --- Metadatos de traza (payload de la respuesta a REQUEST_TRACE) ---
struct __attribute__((packed)) TraceMetadata {
    uint16_t sampleRateHz;       // 200
    uint16_t preTriggerSamples;  // 600
    uint16_t totalSamples;       // 1200 (o menos si el ring no estaba lleno)
    uint16_t accelLsbPerG;       // 2048
    uint16_t gyroLsbPerDpsX10;   // 164
    uint16_t chunkCount;
    uint8_t  samplesPerChunk;    // 19
};
static_assert(sizeof(TraceMetadata) == 13, "TraceMetadata must be 13 bytes");

// --- TRACE chunk (notify): header + muestras crudas (docs/03 §4.7) ---
struct __attribute__((packed)) TraceChunkHeader {
    uint16_t eventId;
    uint16_t chunkIndex;
    uint16_t chunkCount;
};
static_assert(sizeof(TraceChunkHeader) == 6, "TraceChunkHeader must be 6 bytes");

// --- Rangos válidos de CONFIG (docs/03 §4.4) ---
struct ConfigRanges {
    static constexpr uint16_t kImpactMinMg    = 2000;
    static constexpr uint16_t kImpactMaxMg    = 6000;
    static constexpr uint16_t kGyroMinDps     = 150;
    static constexpr uint16_t kGyroMaxDps     = 1000;
    static constexpr uint8_t  kTiltMinDeg     = 50;
    static constexpr uint8_t  kTiltMaxDeg     = 80;
    static constexpr uint8_t  kTiltHoldMinDs  = 5;
    static constexpr uint8_t  kTiltHoldMaxDs  = 50;
    static constexpr int16_t  kOffsetMaxCdeg  = 9000;
};

} // namespace protocol
} // namespace mcg
