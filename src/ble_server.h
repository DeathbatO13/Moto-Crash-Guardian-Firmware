// ble_server.h — servidor GATT NimBLE, protocolo BLE v1 (docs/03)
#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

#include "config.h"
#include "protocol.h"

namespace mcg {

// Crea el servidor GATT y arranca el advertising. Llamar desde setup().
void bleServerInit();

// Tarea principal del BLE (core 0, prio 3): telemetría 5 Hz, GPS 1 Hz,
// Battery Level, eventos por indicate y transferencia de trazas.
void bleServerTask(void* param);

// Respuesta de CONTROL_POINT por indicate (docs/03 §4.5).
void bleSendControlResponse(uint8_t opcode, uint8_t requestId,
                              uint8_t result,
                              const uint8_t* payload = nullptr,
                              size_t payloadLen = 0);

bool bleIsConnected();

} // namespace mcg
