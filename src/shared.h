// shared.h — estado compartido entre tareas (docs/11 §3)
#pragma once

#include <Arduino.h>

#include "config.h"
#include "protocol.h"
#include "CandidateDetector.h"   // DetectionConfig

namespace mcg {

constexpr uint16_t kNoConnHandle = 0xFFFF;   // BLE_HS_CONN_HANDLE_NONE

// Configuración del dispositivo (la escribe BLE, la lee imuTask) — bajo g_stateMux
struct DeviceConfig {
    DetectionConfig detection;
    int16_t pitchOffsetCdeg = 0;
    int16_t rollOffsetCdeg = 0;
    uint16_t revision = 0;
};

// Snapshot de telemetría — bajo g_stateMux
struct TelemetrySnapshot {
    uint8_t  deviceState = protocol::kStateBoot;
    uint8_t  flags = 0;
    int16_t  pitchCdeg = 0;
    int16_t  rollCdeg = 0;
    uint16_t accelMagMg = 1000;
    uint16_t peakAccelMg = 0;   // pico del periodo; lo consume bleTask
    uint16_t peakGyroDps = 0;
    uint8_t  batteryPct = 255;
};

// Snapshot del GPS — bajo g_stateMux
struct GpsSnapshot {
    bool     present = false;
    uint8_t  fixType = 0;
    uint8_t  satellites = 0;
    uint8_t  hdopX10 = 0;
    int32_t  latE7 = 0;
    int32_t  lonE7 = 0;
    uint16_t speedKmhX100 = 0;
    uint16_t fixAgeS = 65535;
};

// Caja negra: 2 slots de 1200 muestras (docs/11 §7).
// Los slots congelados son inmutables; g_traceMutex protege la
// captura y la lectura por chunk.
struct TraceSlot {
    bool     valid = false;
    uint16_t eventId = 0;
    uint16_t totalSamples = 0;
    protocol::ImuSample samples[TRACE_TOTAL_SAMPLES];
};

struct TraceStore {
    TraceSlot slots[TRACE_SLOT_COUNT];
    uint8_t  captureSlot = 0;
    bool     capturing = false;
    uint16_t capturedSamples = 0;
};

// Comando interno (BLE → imuTask) — g_cmdQueue
struct DeviceCommand {
    uint8_t  opcode;
    uint8_t  requestId;
    uint16_t param;
};

// Opcodes internos (no son parte del protocolo BLE)
enum InternalOp : uint8_t {
    kInternalBatteryFault = 0xF0,
};

// --- Estado global (definiciones en main.cpp) ---
extern portMUX_TYPE g_stateMux;
extern DeviceConfig g_config;
extern TelemetrySnapshot g_telemetry;
extern GpsSnapshot g_gps;
extern TraceStore g_traces;
extern QueueHandle_t g_eventQueue;
extern QueueHandle_t g_cmdQueue;
extern SemaphoreHandle_t g_traceMutex;
extern volatile uint16_t g_bootCount;
extern volatile bool g_traceRequestPending;
extern volatile uint16_t g_traceRequestEventId;

// Tareas (bleServerTask se declara en ble_server.h)
void imuTask(void* param);
void gpsTask(void* param);
void powerTask(void* param);
void statusLedTask(void* param);

} // namespace mcg
