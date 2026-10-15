// status_led.cpp — LEDs de estado. Tarea en core 0, prioridad 1.
//   LED BLE   (GPIO 12): encendido fijo = app conectada;
//                        destello corto cada 2 s = anunciándose, sin conexión.
//   LED ERROR (GPIO 14): encendido fijo = módulo en FAULT (p. ej. IMU sin respuesta).
// Al arrancar ambos se encienden 500 ms para comprobar el cableado.
#include "shared.h"
#include "config.h"
#include "ble_server.h"

namespace mcg {
namespace {

constexpr uint32_t kSelfTestMs = 500;
constexpr uint32_t kAdvertisingPeriodMs = 2000;
constexpr uint32_t kAdvertisingFlashMs = 100;

} // namespace

void statusLedTask(void* param) {
    (void)param;

    pinMode(PIN_LED_BLE, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);

    digitalWrite(PIN_LED_BLE, HIGH);
    digitalWrite(PIN_LED_ERROR, HIGH);
    vTaskDelay(pdMS_TO_TICKS(kSelfTestMs));

    for (;;) {
        const uint32_t now = millis();

        const bool connected = bleIsConnected();
        const bool bleOn =
            connected || (now % kAdvertisingPeriodMs) < kAdvertisingFlashMs;
        digitalWrite(PIN_LED_BLE, bleOn ? HIGH : LOW);

        uint8_t state;
        portENTER_CRITICAL(&g_stateMux);
        state = g_telemetry.deviceState;
        portEXIT_CRITICAL(&g_stateMux);
        digitalWrite(PIN_LED_ERROR,
                     state == protocol::kStateFault ? HIGH : LOW);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

} // namespace mcg
