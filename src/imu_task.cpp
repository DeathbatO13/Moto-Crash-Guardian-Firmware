// imu_task.cpp — muestreo IMU a 200 Hz, filtro complementario,
// detector de candidatos y caja negra (docs/11 §3, §5, §6, §7)
// Tarea en core 1, prioridad 5 (la máxima).
#include "shared.h"
#include "ble_server.h"
#include "settings.h"

#include <Wire.h>
#include <math.h>
#include <Preferences.h>

#include "ComplementaryFilter.h"
#include "CandidateDetector.h"
#include "RingBuffer.h"

namespace mcg {
namespace {

ComplementaryFilter g_filter(IMU_FILTER_ALPHA, 1.0f / IMU_SAMPLE_HZ);
CandidateDetector g_detector;
RingBuffer<protocol::ImuSample, RING_CAPACITY_SAMPLES> g_ring;

int16_t g_pitchOffsetCdeg = 0;
int16_t g_rollOffsetCdeg = 0;

uint32_t g_lastSerialMs = 0;
uint32_t g_lastBusRetryMs = 0;
uint16_t g_activeTraceEventId = 0;

// --- MPU6050 de bajo nivel ---
void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);   // repeated start
    Wire.requestFrom(MPU6050_ADDR, static_cast<uint8_t>(1));
    return Wire.available() ? Wire.read() : 0xFF;
}

// Lectura en ráfaga de 14 B (docs/11 §3)
bool readSample(protocol::ImuSample& s) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);   // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, static_cast<uint8_t>(14));
    if (Wire.available() < 14) return false;
    auto rd = []() -> int16_t {
        return static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    };
    s.ax = rd();
    s.ay = rd();
    s.az = rd();
    (void)rd();   // temperatura
    s.gx = rd();
    s.gy = rd();
    s.gz = rd();
    return true;
}

// ±16 g, ±2000 °/s, DLPF 94 Hz, 200 Hz, reloj gyro X (docs/11 §5.1)
bool initImu() {
    writeRegister(0x6B, 0x80);   // reset
    delay(100);
    if (readRegister(0x75) != 0x68) return false;   // WHO_AM_I
    writeRegister(0x6B, 0x01);   // PWR_MGMT_1: reloj gyro X
    writeRegister(0x1C, 0x18);   // ACCEL_CONFIG: AFS_SEL = ±16 g
    writeRegister(0x1B, 0x18);   // GYRO_CONFIG: FS_SEL = ±2000 °/s
    writeRegister(0x1A, 0x02);   // CONFIG: DLPF 94 Hz
    writeRegister(0x19, 4);      // SMPLRT_DIV: 1 kHz / (1+4) = 200 Hz
    writeRegister(0x6C, 0x00);   // PWR_MGMT_2: todos los ejes
    return true;
}

// --- Caja negra (docs/11 §7) ---
void beginTraceCapture() {
    if (xSemaphoreTake(g_traceMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    TraceSlot& slot = g_traces.slots[g_traces.captureSlot];
    slot.valid = false;
    slot.eventId = 0;
    slot.totalSamples = 0;
    g_traces.capturedSamples =
        static_cast<uint16_t>(g_ring.copyTo(slot.samples, TRACE_TOTAL_SAMPLES));
    g_traces.capturing = true;
    xSemaphoreGive(g_traceMutex);
}

void appendTraceSample(const protocol::ImuSample& s) {
    if (!g_traces.capturing) return;
    if (xSemaphoreTake(g_traceMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    if (g_traces.capturing &&
        g_traces.capturedSamples < TRACE_TOTAL_SAMPLES) {
        TraceSlot& slot = g_traces.slots[g_traces.captureSlot];
        slot.samples[g_traces.capturedSamples++] = s;
    }
    xSemaphoreGive(g_traceMutex);
}

void endTraceCapture() {
    if (xSemaphoreTake(g_traceMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    if (g_traces.capturing) {
        TraceSlot& slot = g_traces.slots[g_traces.captureSlot];
        slot.valid = true;
        slot.eventId = g_activeTraceEventId;
        slot.totalSamples = g_traces.capturedSamples;
        g_traces.capturing = false;
        g_traces.captureSlot =
            (g_traces.captureSlot + 1) % TRACE_SLOT_COUNT;
        g_traces.capturedSamples = 0;
    }
    xSemaphoreGive(g_traceMutex);
}

// --- Telemetría ---
void updateTelemetrySnapshot(const protocol::ImuSample& s,
                                  int16_t pitchCdeg, int16_t rollCdeg) {
    const float accelMagG =
        std::sqrt(static_cast<float>(s.ax) * s.ax +
                      static_cast<float>(s.ay) * s.ay +
                      static_cast<float>(s.az) * s.az) / IMU_ACCEL_LSB_PER_G;
    const float gyroMagDps =
        std::sqrt(static_cast<float>(s.gx) * s.gx +
                      static_cast<float>(s.gy) * s.gy +
                      static_cast<float>(s.gz) * s.gz) / IMU_GYRO_LSB_PER_DPS;
    const uint16_t accelMg =
        static_cast<uint16_t>(accelMagG * 1000.0f + 0.5f);
    const uint16_t gyroDps =
        static_cast<uint16_t>(gyroMagDps + 0.5f);
    const bool saturated =
        std::abs(s.ax) >= MPU6050_FULL_SCALE ||
        std::abs(s.ay) >= MPU6050_FULL_SCALE ||
        std::abs(s.az) >= MPU6050_FULL_SCALE ||
        std::abs(s.gx) >= MPU6050_FULL_SCALE ||
        std::abs(s.gy) >= MPU6050_FULL_SCALE ||
        std::abs(s.gz) >= MPU6050_FULL_SCALE;

    portENTER_CRITICAL(&g_stateMux);
    g_telemetry.accelMagMg = accelMg;
    if (accelMg > g_telemetry.peakAccelMg) {
        g_telemetry.peakAccelMg = accelMg;
    }
    if (gyroDps > g_telemetry.peakGyroDps) {
        g_telemetry.peakGyroDps = gyroDps;
    }
    g_telemetry.pitchCdeg = pitchCdeg;
    g_telemetry.rollCdeg = rollCdeg;
    g_telemetry.deviceState = g_detector.state();
    uint8_t flags = protocol::kFlagImuOk;
    if (g_gps.present) flags |= protocol::kFlagGpsPresent;
    if (g_gps.fixType != 0) flags |= protocol::kFlagGpsFix;
    // bit3 (cargando) = 0: PIN_CHRG ⚠️ no implementada en el prototipo
    if (g_telemetry.batteryPct <= BATTERY_LOW_PCT) {
        flags |= protocol::kFlagBatteryLow;
    }
    if (saturated) flags |= protocol::kFlagSaturated;
    g_telemetry.flags = flags;
    portEXIT_CRITICAL(&g_stateMux);
}

// --- Eventos y comandos ---
void emitFaultEvent(uint8_t faultCode) {
    protocol::EventPacket packet {};
    packet.protocolVersion = protocol::kProtocolVersion;
    packet.eventType = protocol::kEventFault;
    packet.eventId = g_detector.allocateEventId();
    packet.bootCount = g_bootCount;
    packet.uptimeMs = millis();
    packet.peakAccelMg = g_detector.lastAccelMg();
    packet.peakGyroDps = g_detector.lastGyroDps();
    packet.pitchCdeg = 0;
    packet.rollCdeg = 0;
    packet.eventFlags = 0;   // sin traza
    packet.faultCode = faultCode;
    xQueueSend(g_eventQueue, &packet, 0);
}

void saveGyroBiasNow() {
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    g_filter.getBias(&gx, &gy, &gz);
    saveGyroBias(gx, gy, gz);
}

void handleCommand(const DeviceCommand& cmd) {
    switch (cmd.opcode) {
        case protocol::kOpArm:
            g_detector.arm();
            bleSendControlResponse(cmd.opcode, cmd.requestId,
                                     protocol::kResultOk);
            break;

        case protocol::kOpDisarm:
            g_detector.disarm();
            if (g_traces.capturing) {
                // Congela la traza parcial (el evento ya se
                // envió con bit1; la app puede pedirla)
                endTraceCapture();
            }
            bleSendControlResponse(cmd.opcode, cmd.requestId,
                                     protocol::kResultOk);
            break;

        case protocol::kOpTare: {
            if (!g_detector.isStill()) {
                bleSendControlResponse(cmd.opcode, cmd.requestId,
                                         protocol::kResultBusy);
                break;
            }
            const int16_t pitch =
                static_cast<int16_t>(g_filter.pitchDeg() * 100.0f);
            const int16_t roll =
                static_cast<int16_t>(g_filter.rollDeg() * 100.0f);
            g_pitchOffsetCdeg = pitch;
            g_rollOffsetCdeg = roll;
            {
                portENTER_CRITICAL(&g_stateMux);
                g_config.pitchOffsetCdeg = pitch;
                g_config.rollOffsetCdeg = roll;
                portEXIT_CRITICAL(&g_stateMux);
            }
            saveOffsets(pitch, roll);
            const uint8_t payload[4] = {
                static_cast<uint8_t>(pitch & 0xFF),
                static_cast<uint8_t>((pitch >> 8) & 0xFF),
                static_cast<uint8_t>(roll & 0xFF),
                static_cast<uint8_t>((roll >> 8) & 0xFF)};
            bleSendControlResponse(cmd.opcode, cmd.requestId,
                                     protocol::kResultOk, payload,
                                     sizeof(payload));
            break;
        }

        case protocol::kOpTriggerTestEvent: {
            // Evento TEST sin cambiar de estado ni congelar
            // traza (docs/11 §4)
            const int16_t pitch =
                static_cast<int16_t>(g_filter.pitchDeg() * 100.0f)
                    - g_pitchOffsetCdeg;
            const int16_t roll =
                static_cast<int16_t>(g_filter.rollDeg() * 100.0f)
                    - g_rollOffsetCdeg;
            protocol::EventPacket packet {};
            packet.protocolVersion = protocol::kProtocolVersion;
            packet.eventType = protocol::kEventTest;
            packet.eventId = g_detector.allocateEventId();
            packet.bootCount = g_bootCount;
            packet.uptimeMs = millis();
            packet.peakAccelMg = g_detector.lastAccelMg();
            packet.peakGyroDps = g_detector.lastGyroDps();
            packet.pitchCdeg = pitch;
            packet.rollCdeg = roll;
            packet.eventFlags = 0;   // sin traza
            packet.faultCode = 0;
            xQueueSend(g_eventQueue, &packet, 0);
            bleSendControlResponse(
                cmd.opcode, cmd.requestId, protocol::kResultOk,
                reinterpret_cast<const uint8_t*>(&packet.eventId),
                sizeof(packet.eventId));
            break;
        }

        case protocol::kOpReboot:
            bleSendControlResponse(cmd.opcode, cmd.requestId,
                                     protocol::kResultOk);
            delay(20);   // deja salir la indicación
            ESP.restart();
            break;

        case protocol::kOpFactoryReset: {
            if (cmd.param != 0xDEAD) {
                bleSendControlResponse(cmd.opcode, cmd.requestId,
                                         protocol::kResultInvalidParam);
                break;
            }
            {
                Preferences prefs;
                if (prefs.begin("mcg", false)) {
                    prefs.clear();   // config, offsets y sesgo
                    prefs.end();
                }
            }
            bleSendControlResponse(cmd.opcode, cmd.requestId,
                                     protocol::kResultOk);
            delay(20);
            ESP.restart();
            break;
        }

        case kInternalBatteryFault:
            emitFaultEvent(protocol::kFaultBatteryCrit);
            break;

        default:
            bleSendControlResponse(cmd.opcode, cmd.requestId,
                                     protocol::kResultUnsupported);
            break;
    }
}

// Depuración por serial: conserva el formato del prototipo
// (docs/11 §9: solo logs de depuración)
void serialDebug(const protocol::ImuSample& s,
                     int16_t pitchCdeg, int16_t rollCdeg) {
#if SERIAL_TELEMETRY
    const uint32_t now = millis();
    if (now - g_lastSerialMs < 500) return;
    g_lastSerialMs = now;

    portENTER_CRITICAL(&g_stateMux);
    const TelemetrySnapshot tel = g_telemetry;
    const GpsSnapshot gps = g_gps;
    portEXIT_CRITICAL(&g_stateMux);

    Serial.println();
    Serial.println("--------------------------------------");
    Serial.println("GPS:");
    if (gps.fixType != 0) {
        Serial.print("  Latitud:  ");
        Serial.println(gps.latE7 / 1.0e7, 6);
        Serial.print("  Longitud: ");
        Serial.println(gps.lonE7 / 1.0e7, 6);
        Serial.print("  Velocidad: ");
        Serial.print(gps.speedKmhX100 / 100.0, 2);
        Serial.println(" km/h");
        Serial.print("  Satelites: ");
        Serial.println(gps.satellites);
        Serial.print("  HDOP: ");
        Serial.println(gps.hdopX10 / 10.0, 2);
    } else {
        Serial.println("  Ubicacion: SIN FIX");
    }

    Serial.println();
    Serial.println("IMU:");
    Serial.print("  Pitch: ");
    Serial.print(pitchCdeg / 100.0, 1);
    Serial.println(" deg");
    Serial.print("  Roll:  ");
    Serial.print(rollCdeg / 100.0, 1);
    Serial.println(" deg");
    Serial.print("  |a|: ");
    Serial.print(tel.accelMagMg / 1000.0, 2);
    Serial.println(" g");
    Serial.print("  |w|: ");
    Serial.print(tel.peakGyroDps);
    Serial.println(" deg/s");
    Serial.print("  Acel X: ");
    Serial.print(s.ax / IMU_ACCEL_LSB_PER_G, 2);
    Serial.println(" g");
    Serial.print("  Acel Y: ");
    Serial.print(s.ay / IMU_ACCEL_LSB_PER_G, 2);
    Serial.println(" g");
    Serial.print("  Acel Z: ");
    Serial.print(s.az / IMU_ACCEL_LSB_PER_G, 2);
    Serial.println(" g");
    Serial.print("  Giro X: ");
    Serial.print(s.gx / IMU_GYRO_LSB_PER_DPS, 1);
    Serial.println(" deg/s");
    Serial.print("  Giro Y: ");
    Serial.print(s.gy / IMU_GYRO_LSB_PER_DPS, 1);
    Serial.println(" deg/s");
    Serial.print("  Giro Z: ");
    Serial.print(s.gz / IMU_GYRO_LSB_PER_DPS, 1);
    Serial.println(" deg/s");
    Serial.print("  Estado: ");
    Serial.println(tel.deviceState);
    Serial.println("--------------------------------------");
#else
    (void)s;
    (void)pitchCdeg;
    (void)rollCdeg;
#endif
}

} // namespace

void imuTask(void* param) {
    (void)param;

    // Configuración persistente (NVS)
    StoredSettings stored;
    loadSettings(stored);
    g_filter.setBias(stored.gyroBias[0], stored.gyroBias[1],
                         stored.gyroBias[2]);
    {
        portENTER_CRITICAL(&g_stateMux);
        g_config.detection = stored.detection;
        g_config.pitchOffsetCdeg = stored.pitchOffsetCdeg;
        g_config.rollOffsetCdeg = stored.rollOffsetCdeg;
        g_config.revision = stored.configRevision;
        portEXIT_CRITICAL(&g_stateMux);
    }
    g_detector.configure(stored.detection);

    // I²C e IMU
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    Wire.setClock(I2C_FREQ_HZ);
    if (!initImu()) {
        g_detector.enterFault();
        emitFaultEvent(protocol::kFaultImuNoResponse);
    }

    // Calibración de sesgo del giroscopio en las primeras 200
    // muestras en reposo (docs/11 §5.2)
    g_filter.startBiasCalibration();

    portTickType_t lastWake = xTaskGetTickCount();
    uint16_t lastConfigRevision = 0;
    bool biasSaved = false;
    int i2cFailures = 0;

    for (;;) {
        // Aplica configuración nueva desde BLE
        uint16_t revision;
        DetectionConfig detection;
        {
            portENTER_CRITICAL(&g_stateMux);
            revision = g_config.revision;
            detection = g_config.detection;
            g_pitchOffsetCdeg = g_config.pitchOffsetCdeg;
            g_rollOffsetCdeg = g_config.rollOffsetCdeg;
            portEXIT_CRITICAL(&g_stateMux);
        }
        if (revision != lastConfigRevision) {
            lastConfigRevision = revision;
            g_detector.configure(detection);
        }

        protocol::ImuSample sample;
        if (!readSample(sample)) {
            if (++i2cFailures >= 5 &&
                g_detector.state() != protocol::kStateFault) {
                g_detector.enterFault();
                emitFaultEvent(protocol::kFaultImuNoResponse);
            }
            if (g_detector.state() == protocol::kStateFault &&
                millis() - g_lastBusRetryMs >= 1000) {
                g_lastBusRetryMs = millis();
                Wire.resetBus();
                Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
                Wire.setClock(I2C_FREQ_HZ);
                if (initImu()) {
                    g_detector.recoverFromFault();   // → DISARMED
                }
            }
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / IMU_SAMPLE_HZ));
            continue;
        }
        i2cFailures = 0;

        // Unidades físicas
        const float axG = sample.ax / IMU_ACCEL_LSB_PER_G;
        const float ayG = sample.ay / IMU_ACCEL_LSB_PER_G;
        const float azG = sample.az / IMU_ACCEL_LSB_PER_G;
        const float gxDps = sample.gx / IMU_GYRO_LSB_PER_DPS;
        const float gyDps = sample.gy / IMU_GYRO_LSB_PER_DPS;
        const float gzDps = sample.gz / IMU_GYRO_LSB_PER_DPS;

        g_filter.update(axG, ayG, azG, gxDps, gyDps, gzDps);
        if (!biasSaved && !g_filter.isCalibrating()) {
            // Sesgo calibrado en arranque (200 muestras):
            // se persiste una vez (docs/11 §5.2)
            saveGyroBiasNow();
            biasSaved = true;
        }

        const int16_t pitchCdeg =
            static_cast<int16_t>(g_filter.pitchDeg() * 100.0f)
                - g_pitchOffsetCdeg;
        const int16_t rollCdeg =
            static_cast<int16_t>(g_filter.rollDeg() * 100.0f)
                - g_rollOffsetCdeg;

        // Ring buffer siempre activo (docs/11 §7)
        g_ring.push(sample);

        // Detector de candidatos (solo emite en ARMED)
        const uint8_t prevState = g_detector.state();
        auto event = g_detector.process(sample, pitchCdeg, rollCdeg,
                                            millis());
        const uint8_t newState = g_detector.state();

        if (prevState == protocol::kStateArmed &&
            newState == protocol::kStatePostTrigger) {
            beginTraceCapture();
        }
        if (event) {
            g_activeTraceEventId = event->eventId;
            protocol::EventPacket packet {};
            packet.protocolVersion = protocol::kProtocolVersion;
            packet.eventType = event->eventType;
            packet.eventId = event->eventId;
            packet.bootCount = g_bootCount;
            packet.uptimeMs = event->uptimeMs;
            packet.peakAccelMg = event->peakAccelMg;
            packet.peakGyroDps = event->peakGyroDps;
            packet.pitchCdeg = event->pitchCdeg;
            packet.rollCdeg = event->rollCdeg;
            // bit1: la traza se congela al pasar a COOLDOWN (docs/11 §4)
            packet.eventFlags =
                event->eventFlags | protocol::kEventFlagTrace;
            packet.faultCode = 0;
            xQueueSend(g_eventQueue, &packet, 0);
        }
        if (prevState == protocol::kStatePostTrigger &&
            newState == protocol::kStateCooldown) {
            endTraceCapture();
        }

        appendTraceSample(sample);
        updateTelemetrySnapshot(sample, pitchCdeg, rollCdeg);

        // Comandos internos y de BLE
        DeviceCommand cmd;
        while (xQueueReceive(g_cmdQueue, &cmd, 0) == pdTRUE) {
            handleCommand(cmd);
        }

        serialDebug(sample, pitchCdeg, rollCdeg);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / IMU_SAMPLE_HZ));
    }
}

} // namespace mcg
