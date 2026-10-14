// ble_server.cpp — servidor GATT NimBLE, protocolo BLE v1 (docs/03 §3–§7)
//
// Desviación documentada de docs/03 §4.4: NimBLE-Arduino 2.x responde
// éxito al write ANTES del callback onWrite y no expone una API para
// devolver un error ATT (0x80 INVALID_CONFIG). Ante un CONFIG inválido
// se restaura el valor previo y no se aplica nada (atómico); la app lo
// detecta al leer CONFIG de vuelta (docs/03 §6.5).
//
// Flujo de EVENT (indicate): se envía la cola de a uno por vez;
// onStatus (host task) marca la confirmación y bleTask avanza.
// Sin suscripción, los eventos quedan en la cola de hasta 8 sin
// confirmar y se reenvían con bit2 al reconectar (docs/03 §4.3).
#include "ble_server.h"
#include "shared.h"
#include "settings.h"

#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>
#include <NimBLEAdvertising.h>

namespace mcg {
namespace {

// --- Características ---
NimBLECharacteristic* infoChar     = nullptr;
NimBLECharacteristic* telemetryChar = nullptr;
NimBLECharacteristic* eventChar    = nullptr;
NimBLECharacteristic* configChar   = nullptr;
NimBLECharacteristic* controlChar  = nullptr;
NimBLECharacteristic* gpsChar      = nullptr;
NimBLECharacteristic* traceChar    = nullptr;
NimBLECharacteristic* batteryChar  = nullptr;

uint8_t  g_telemetrySeq = 0;
uint16_t g_activeConn = kNoConnHandle;
volatile bool g_eventSubscribed = false;
bool     g_slowAdvertising = false;
uint32_t g_bootTimeMs = 0;
uint8_t  g_lastBatteryPct = 0xFF;
uint32_t g_lastBatteryNotifyMs = 0;

// Flujo de indicaciones (bleTask es el único que manipula la cola)
volatile bool g_indicationConfirmed = false;
volatile bool g_resendRequested = false;
bool     g_inflight = false;

// Cola de eventos sin confirmar (docs/03 §4.3): máximo 8
struct UnackedEvents {
    protocol::EventPacket items[MAX_UNACKED_EVENTS];
    uint8_t count = 0;

    void push(const protocol::EventPacket& ev) {
        if (count >= MAX_UNACKED_EVENTS) {
            for (uint8_t i = 0; i + 1 < MAX_UNACKED_EVENTS; ++i) {
                items[i] = items[i + 1];
            }
            count = MAX_UNACKED_EVENTS - 1;
        }
        items[count++] = ev;
    }

    void remove(uint16_t bootCount, uint16_t eventId) {
        for (uint8_t i = 0; i < count; ++i) {
            if (items[i].bootCount == bootCount &&
                items[i].eventId == eventId) {
                for (uint8_t j = i; j + 1 < count; ++j) {
                    items[j] = items[j + 1];
                }
                --count;
                return;
            }
        }
    }
};
UnackedEvents g_unacked;

// --- Envíos básicos ---
void sendEventPacket(const protocol::EventPacket& ev) {
    eventChar->setValue(reinterpret_cast<const uint8_t*>(&ev), sizeof(ev));
    eventChar->indicate();
}

void flushEventQueue() {
    if (g_inflight || !g_eventSubscribed || g_unacked.count == 0) return;
    g_inflight = true;
    sendEventPacket(g_unacked.items[0]);
}

bool traceExists(uint16_t eventId, uint16_t* totalSamples) {
    if (xSemaphoreTake(g_traceMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }
    bool found = false;
    for (const auto& slot : g_traces.slots) {
        if (slot.valid && slot.eventId == eventId) {
            *totalSamples = slot.totalSamples;
            found = true;
            break;
        }
    }
    xSemaphoreGive(g_traceMutex);
    return found;
}

// Transfiere la traza por chunks (19 muestras/chunk, docs/03 §4.7).
// El mutex se toma por chunk para no bloquear la captura en imuTask.
void streamTrace(uint16_t eventId) {
    const uint16_t chunkSamples = TRACE_SAMPLES_PER_CHUNK;
    const size_t chunkBytes = sizeof(protocol::TraceChunkHeader)
                                + chunkSamples * sizeof(protocol::ImuSample);
    uint8_t buf[6 + chunkSamples * 12];

    uint16_t total = 0;
    if (!traceExists(eventId, &total)) return;
    const uint16_t chunkCount =
        (total + chunkSamples - 1) / chunkSamples;

    for (uint16_t i = 0; i < chunkCount; ++i) {
        const uint16_t first = i * chunkSamples;
        const uint16_t n = (first + chunkSamples <= total)
                               ? chunkSamples
                               : (total - first);

        if (xSemaphoreTake(g_traceMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            return;
        }
        const TraceSlot* slot = nullptr;
        for (const auto& s : g_traces.slots) {
            if (s.valid && s.eventId == eventId) {
                slot = &s;
                break;
            }
        }
        if (slot == nullptr) {
            xSemaphoreGive(g_traceMutex);
            return;   // la traza fue reemplazada: la app reintentará
        }

        protocol::TraceChunkHeader header {};
        header.eventId = eventId;
        header.chunkIndex = i;
        header.chunkCount = chunkCount;
        memcpy(buf, &header, sizeof(header));
        memcpy(buf + sizeof(header), &slot->samples[first],
               n * sizeof(protocol::ImuSample));
        xSemaphoreGive(g_traceMutex);

        traceChar->setValue(buf, sizeof(header) + n * sizeof(protocol::ImuSample));
        traceChar->notify();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void sendTelemetry() {
    portENTER_CRITICAL(&g_stateMux);
    TelemetrySnapshot snap = g_telemetry;
    g_telemetry.peakAccelMg = 0;   // consume el pico del periodo
    g_telemetry.peakGyroDps = 0;
    portEXIT_CRITICAL(&g_stateMux);

    protocol::TelemetryPacket packet {};
    packet.protocolVersion = protocol::kProtocolVersion;
    packet.seq = g_telemetrySeq++;
    packet.deviceState = snap.deviceState;
    packet.flags = snap.flags;
    packet.pitchCdeg = snap.pitchCdeg;
    packet.rollCdeg = snap.rollCdeg;
    packet.accelMagMg = snap.accelMagMg;
    packet.peakAccelMg = snap.peakAccelMg;
    packet.peakGyroDps = snap.peakGyroDps;
    packet.batteryPct = snap.batteryPct;
    packet.reserved = 0;

    telemetryChar->setValue(reinterpret_cast<const uint8_t*>(&packet),
                              sizeof(packet));
    telemetryChar->notify();
}

void sendGps() {
    portENTER_CRITICAL(&g_stateMux);
    const GpsSnapshot snap = g_gps;
    portEXIT_CRITICAL(&g_stateMux);

    // Sin fix se notifica con fixType = 0 (docs/03 §4.6)
    protocol::GpsPacket packet {};
    packet.protocolVersion = protocol::kProtocolVersion;
    packet.fixType = snap.fixType;
    packet.satellites = snap.satellites;
    packet.hdopX10 = snap.hdopX10;
    packet.latE7 = snap.latE7;
    packet.lonE7 = snap.lonE7;
    packet.speedKmhX100 = snap.speedKmhX100;
    packet.fixAgeS = snap.fixAgeS;

    gpsChar->setValue(reinterpret_cast<const uint8_t*>(&packet),
                        sizeof(packet));
    gpsChar->notify();
}

// --- Callbacks del servidor ---
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override {
        g_activeConn = connInfo.getConnHandle();
        // Parámetros de conexión (docs/03 §2): 30–50 ms, latencia 0,
        // supervision timeout 4 s.
        pSrv->updateConnParams(
            g_activeConn,
            CONN_INTERVAL_MIN_MS * 1000 / 1250,
            CONN_INTERVAL_MAX_MS * 1000 / 1250,
            0,
            CONN_SUPERVISION_TIMEOUT_MS / 10);
        // Emparejamiento Just Works con bonding (docs/03 §2)
        NimBLEDevice::startSecurity(g_activeConn);
        g_slowAdvertising = false;
    }

    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo,
                        int reason) override {
        (void)pSrv;
        (void)connInfo;
        (void)reason;
        g_activeConn = kNoConnHandle;
        // advertiseOnDisconnect(true) reanuda el advertising.
    }
};

// --- DEVICE_INFO (read, abierta) ---
class DeviceInfoCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pChar,
                  NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        protocol::DeviceInfoPacket info {};
        info.protocolVersion = protocol::kProtocolVersion;
        info.fwMajor = FW_VERSION_MAJOR;
        info.fwMinor = FW_VERSION_MINOR;
        info.fwPatch = FW_VERSION_PATCH;
        info.hwRevision = 1;   // prototipo MVP
        info.capabilities = 0x07;   // bit0 GPS · bit1 caja negra · bit2 batería
        info.bootCount = g_bootCount;
        const uint8_t* mac = NimBLEDevice::getAddress().getNative();
        for (int i = 0; i < 6; ++i) info.deviceId[i] = mac[i];
        info.reserved = 0;
        pChar->setValue(reinterpret_cast<const uint8_t*>(&info), sizeof(info));
    }
};

// --- CONFIG (read/write cifrada, docs/03 §4.4) ---
class ConfigCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pChar,
                  NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        portENTER_CRITICAL(&g_stateMux);
        const DeviceConfig cfg = g_config;
        portEXIT_CRITICAL(&g_stateMux);

        protocol::ConfigPacket packet {};
        packet.protocolVersion = protocol::kProtocolVersion;
        packet.flags = cfg.detection.tiltEnabled ? 0x01 : 0x00;
        packet.impactThresholdMg = cfg.detection.impactThresholdMg;
        packet.gyroThresholdDps = cfg.detection.gyroThresholdDps;
        packet.tiltThresholdDeg = cfg.detection.tiltThresholdDeg;
        packet.tiltHoldDs = static_cast<uint8_t>(cfg.detection.tiltHoldMs / 100);
        packet.pitchOffsetCdeg = cfg.pitchOffsetCdeg;
        packet.rollOffsetCdeg = cfg.rollOffsetCdeg;
        packet.configRevision = cfg.revision;
        packet.reserved = 0;
        pChar->setValue(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    }

    void onWrite(NimBLECharacteristic* pChar,
                   NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        const auto& value = pChar->getValue();
        if (value.size() < sizeof(protocol::ConfigPacket)) {
            onRead(pChar, connInfo);
            return;
        }

        protocol::ConfigPacket incoming {};
        memcpy(&incoming, value.data(), sizeof(incoming));

        // Validación atómica: si algo está fuera de rango,
        // no se aplica ningún cambio.
        const bool valid =
            incoming.protocolVersion == protocol::kProtocolVersion &&
            (incoming.flags & ~0x01u) == 0 &&
            incoming.impactThresholdMg >= protocol::ConfigRanges::kImpactMinMg &&
            incoming.impactThresholdMg <= protocol::ConfigRanges::kImpactMaxMg &&
            incoming.gyroThresholdDps >= protocol::ConfigRanges::kGyroMinDps &&
            incoming.gyroThresholdDps <= protocol::ConfigRanges::kGyroMaxDps &&
            incoming.tiltThresholdDeg >= protocol::ConfigRanges::kTiltMinDeg &&
            incoming.tiltThresholdDeg <= protocol::ConfigRanges::kTiltMaxDeg &&
            incoming.tiltHoldDs >= protocol::ConfigRanges::kTiltHoldMinDs &&
            incoming.tiltHoldDs <= protocol::ConfigRanges::kTiltHoldMaxDs;

        if (!valid) {
            // Devuelve el valor previo (ver nota sobre el error ATT arriba)
            onRead(pChar, connInfo);
            return;
        }

        portENTER_CRITICAL(&g_stateMux);
        g_config.detection.tiltEnabled = (incoming.flags & 0x01u) != 0;
        g_config.detection.impactThresholdMg = incoming.impactThresholdMg;
        g_config.detection.gyroThresholdDps = incoming.gyroThresholdDps;
        g_config.detection.tiltThresholdDeg = incoming.tiltThresholdDeg;
        g_config.detection.tiltHoldMs =
            static_cast<uint16_t>(incoming.tiltHoldDs) * 100;
        // Los offsets no son escribibles (solo TARE): se ignoran.
        ++g_config.revision;
        const DetectionConfig applied = g_config.detection;
        const uint16_t revision = g_config.revision;
        portEXIT_CRITICAL(&g_stateMux);

        // Lo válido se persiste de inmediato (docs/03 §4.4)
        saveDetectionConfig(applied, revision);
    }
};

// --- CONTROL_POINT (write + indicate, docs/03 §4.5) ---
class ControlPointCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar,
                   NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        const auto& value = pChar->getValue();
        if (value.size() < 2) return;
        const uint8_t* data = value.data();
        const uint8_t opcode = data[0];
        const uint8_t requestId = data[1];

        switch (opcode) {
            case protocol::kOpAckEvent: {
                if (value.size() < 6) {
                    bleSendControlResponse(opcode, requestId,
                                             protocol::kResultInvalidParam);
                    return;
                }
                const uint16_t bootCount =
                    static_cast<uint16_t>(data[2]) | (data[3] << 8);
                const uint16_t eventId =
                    static_cast<uint16_t>(data[4]) | (data[5] << 8);
                // bleTask retira el evento de la cola de sin-confirmar.
                g_unacked.remove(bootCount, eventId);
                bleSendControlResponse(opcode, requestId, protocol::kResultOk);
                return;
            }

            case protocol::kOpRequestTrace: {
                if (value.size() < 4) {
                    bleSendControlResponse(opcode, requestId,
                                             protocol::kResultInvalidParam);
                    return;
                }
                const uint16_t eventId =
                    static_cast<uint16_t>(data[2]) | (data[3] << 8);
                uint16_t totalSamples = 0;
                if (!traceExists(eventId, &totalSamples)) {
                    bleSendControlResponse(opcode, requestId,
                                             protocol::kResultNotFound);
                    return;
                }
                protocol::TraceMetadata meta {};
                meta.sampleRateHz = IMU_SAMPLE_HZ;
                meta.preTriggerSamples = RING_CAPACITY_SAMPLES;
                meta.totalSamples = totalSamples;
                meta.accelLsbPerG = static_cast<uint16_t>(IMU_ACCEL_LSB_PER_G);
                meta.gyroLsbPerDpsX10 =
                    static_cast<uint16_t>(IMU_GYRO_LSB_PER_DPS * 10.0f);
                meta.samplesPerChunk = TRACE_SAMPLES_PER_CHUNK;
                meta.chunkCount =
                    (totalSamples + TRACE_SAMPLES_PER_CHUNK - 1)
                        / TRACE_SAMPLES_PER_CHUNK;
                bleSendControlResponse(opcode, requestId, protocol::kResultOk,
                                         reinterpret_cast<const uint8_t*>(&meta),
                                         sizeof(meta));
                // El streaming lo hace bleTask para no bloquear
                // el stack BLE (64 chunks con pacing).
                g_traceRequestEventId = eventId;
                g_traceRequestPending = true;
                return;
            }

            case protocol::kOpTare:
            case protocol::kOpArm:
            case protocol::kOpDisarm:
            case protocol::kOpTriggerTestEvent:
            case protocol::kOpReboot:
            case protocol::kOpFactoryReset: {
                // Se atienden en imuTask (dueño del detector y la IMU)
                DeviceCommand cmd {};
                cmd.opcode = opcode;
                cmd.requestId = requestId;
                cmd.param = 0;
                if (opcode == protocol::kOpFactoryReset && value.size() >= 4) {
                    cmd.param =
                        static_cast<uint16_t>(data[2]) | (data[3] << 8);
                }
                if (xQueueSend(g_cmdQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
                    bleSendControlResponse(opcode, requestId,
                                             protocol::kResultBusy);
                }
                return;
            }

            default:
                bleSendControlResponse(opcode, requestId,
                                         protocol::kResultUnsupported);
                return;
        }
    }

    void onSubscribe(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo,
                       uint16_t subValue) override {
        (void)pChar;
        (void)connInfo;
        (void)subValue;   // la app habilita indicaciones (docs/03 §6.6)
    }
};

// --- EVENT (indicate, docs/03 §4.3) ---
class EventCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo,
                       uint16_t subValue) override {
        (void)pChar;
        (void)connInfo;
        const bool indicating = (subValue & 0x02u) != 0;
        g_eventSubscribed = indicating;
        if (indicating) {
            // Reenvío de los eventos sin confirmar con bit2
            g_resendRequested = true;
        }
    }

    void onStatus(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo,
                    int code) override {
        (void)pChar;
        (void)connInfo;
        (void)code;   // confirmación (o fallo): avanza la cola
        g_indicationConfirmed = true;
    }
};

// --- Battery Level (read + notify, docs/03 §3) ---
class BatteryCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pChar,
                  NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        portENTER_CRITICAL(&g_stateMux);
        const uint8_t pct = g_telemetry.batteryPct;
        portEXIT_CRITICAL(&g_stateMux);
        pChar->setValue(&pct, 1);
    }
};

} // namespace

void bleSendControlResponse(uint8_t opcode, uint8_t requestId,
                                  uint8_t result,
                                  const uint8_t* payload, size_t payloadLen) {
    uint8_t buf[20];
    const size_t total = 3 + payloadLen;
    if (total > sizeof(buf) || controlChar == nullptr) return;
    buf[0] = opcode;
    buf[1] = requestId;
    buf[2] = result;
    if (payloadLen > 0 && payload != nullptr) {
        memcpy(buf + 3, payload, payloadLen);
    }
    controlChar->setValue(buf, total);
    controlChar->indicate();
}

bool bleIsConnected() {
    return g_activeConn != kNoConnHandle;
}

void bleServerInit() {
    // Nombre en scan response: MCG-XXXX con los 2 últimos bytes
    // de la MAC (docs/03 §1)
    NimBLEDevice::init("MCG");
    char deviceName[16];
    const uint8_t* mac = NimBLEDevice::getAddress().getNative();
    snprintf(deviceName, sizeof(deviceName), "MCG-%02X%02X", mac[4], mac[5]);
    NimBLEDevice::setDeviceName(deviceName);

    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    pServer->advertiseOnDisconnect(true);

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    const uint16_t fastUnits = ADV_INTERVAL_FAST_MS * 1000 / 625;   // 0.625 ms
    adv->setMinInterval(fastUnits);
    adv->setMaxInterval(fastUnits);
    adv->addServiceUUID(protocol::kServiceUuid);

    NimBLEAdvertisementData scanResp;
    scanResp.setName(deviceName);
    adv->setScanResponseData(scanResp);

    NimBLEService* mcgService = pServer->createService(protocol::kServiceUuid);

    infoChar = mcgService->createCharacteristic(
        protocol::kDeviceInfoUuid,
        NIMBLE_PROPERTY::READ,
        sizeof(protocol::DeviceInfoPacket));
    infoChar->setCallbacks(new DeviceInfoCallbacks());

    telemetryChar = mcgService->createCharacteristic(
        protocol::kTelemetryUuid,
        NIMBLE_PROPERTY::NOTIFY,
        sizeof(protocol::TelemetryPacket));

    eventChar = mcgService->createCharacteristic(
        protocol::kEventUuid,
        NIMBLE_PROPERTY::INDICATE,
        sizeof(protocol::EventPacket));
    eventChar->setCallbacks(new EventCallbacks());

    configChar = mcgService->createCharacteristic(
        protocol::kConfigUuid,
        NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC,
        sizeof(protocol::ConfigPacket));
    configChar->setCallbacks(new ConfigCallbacks());

    controlChar = mcgService->createCharacteristic(
        protocol::kControlPointUuid,
        NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::INDICATE,
        20);
    controlChar->setCallbacks(new ControlPointCallbacks());

    gpsChar = mcgService->createCharacteristic(
        protocol::kGpsUuid,
        NIMBLE_PROPERTY::NOTIFY,
        sizeof(protocol::GpsPacket));

    traceChar = mcgService->createCharacteristic(
        protocol::kTraceUuid,
        NIMBLE_PROPERTY::NOTIFY,
        6 + TRACE_SAMPLES_PER_CHUNK * sizeof(protocol::ImuSample));

    // Battery Service estándar (docs/03 §3)
    NimBLEService* batteryService = pServer->createService("180F");
    batteryChar = batteryService->createCharacteristic(
        "2A19",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
        1);
    batteryChar->setCallbacks(new BatteryCallbacks());

    mcgService->start();
    batteryService->start();

    // MTU preferido 247 (docs/03 §2); la app lo solicita al conectar
    NimBLEDevice::setMTU(BLE_MTU_PREFERRED);

    // Seguridad: bonding, LE Secure Connections, Just Works (docs/03 §2)
    NimBLEDevice::setSecurityAuth(ESP_LE_AUTH_REQ_SC_BOND);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_IO);
    NimBLEDevice::setSecurityInitKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    NimBLEDevice::setSecurityRespKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    pServer->start();
    NimBLEDevice::startAdvertising();
}

void bleServerTask(void* param) {
    (void)param;

    g_bootTimeMs = millis();

    uint32_t lastTelemetryMs = 0;
    uint32_t lastGpsMs = 0;
    uint32_t lastBatteryMs = 0;

    for (;;) {
        const uint32_t now = millis();

        // Confirmación de indicación (marcada por el host task)
        if (g_indicationConfirmed) {
            g_indicationConfirmed = false;
            if (g_inflight && g_unacked.count > 0) {
                for (uint8_t i = 0; i + 1 < g_unacked.count; ++i) {
                    g_unacked.items[i] = g_unacked.items[i + 1];
                }
                --g_unacked.count;
            }
            g_inflight = false;
        }

        // Reenvío solicitado por nueva suscripción (docs/03 §4.3)
        if (g_resendRequested) {
            g_resendRequested = false;
            for (uint8_t i = 0; i < g_unacked.count; ++i) {
                g_unacked.items[i].eventFlags |= protocol::kEventFlagResent;
            }
            g_inflight = false;
        }

        // ACK_EVENT retira de la cola (atendido en ControlPointCallbacks)

        // Eventos del detector y de fallos → indicate
        protocol::EventPacket ev;
        while (xQueueReceive(g_eventQueue, &ev, 0) == pdTRUE) {
            g_unacked.push(ev);
        }
        flushEventQueue();

        // TELEMETRY a 5 Hz (docs/03 §4.2)
        if (now - lastTelemetryMs >= 200) {
            lastTelemetryMs = now;
            sendTelemetry();
        }

        // GPS a 1 Hz (docs/03 §4.6)
        if (now - lastGpsMs >= 1000) {
            lastGpsMs = now;
            sendGps();
        }

        // Battery Level: notifica en cambio ≥ 1 % o cada 60 s (docs/03 §3)
        if (now - lastBatteryMs >= 1000) {
            lastBatteryMs = now;
            uint8_t pct;
            portENTER_CRITICAL(&g_stateMux);
            pct = g_telemetry.batteryPct;
            portEXIT_CRITICAL(&g_stateMux);
            const bool changed = (pct != g_lastBatteryPct);
            if (pct != 0xFF &&
                (changed || now - g_lastBatteryNotifyMs >= 60000)) {
                g_lastBatteryPct = pct;
                g_lastBatteryNotifyMs = now;
                batteryChar->setValue(&pct, 1);
                batteryChar->notify();
            }
        }

        // Transferencia de traza solicitada
        if (g_traceRequestPending) {
            g_traceRequestPending = false;
            streamTrace(g_traceRequestEventId);
        }

        // Advertising más lento tras 30 s sin conexión (docs/11 §8)
        if (!g_slowAdvertising && g_activeConn == kNoConnHandle &&
            now - g_bootTimeMs >= ADV_SLOW_AFTER_MS) {
            g_slowAdvertising = true;
            NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
            const uint16_t slowUnits = ADV_INTERVAL_SLOW_MS * 1000 / 625;
            adv->setMinInterval(slowUnits);
            adv->setMaxInterval(slowUnits);
            adv->stop();
            adv->start();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

} // namespace mcg
