// main.cpp — arranque y creación de tareas (docs/11 §3)
//
//   Core 1 (APP_CPU): imuTask      prio 5 · 200 Hz
//   Core 0 (PRO_CPU): bleServerTask prio 3 · 200 ms
//                     gpsTask      prio 2 · continua
//                     powerTask    prio 1 · 1 s
#include <Arduino.h>
#include <WiFi.h>   // se apaga para ahorrar energía (docs/11 §1)

#include "config.h"
#include "shared.h"
#include "settings.h"
#include "ble_server.h"

namespace mcg {

// Estado global compartido entre tareas
portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;
DeviceConfig g_config;
TelemetrySnapshot g_telemetry;
GpsSnapshot g_gps;
TraceStore g_traces;
QueueHandle_t g_eventQueue = nullptr;
QueueHandle_t g_cmdQueue = nullptr;
SemaphoreHandle_t g_traceMutex = nullptr;
volatile uint16_t g_bootCount = 0;
volatile bool g_traceRequestPending = false;
volatile uint16_t g_traceRequestEventId = 0;

} // namespace mcg

void setup() {
    Serial.begin(115200);

    // WiFi apagado: ahorra energía y evita coexistencia de radio
    WiFi.mode(WIFI_OFF);
    setCpuFrequencyMhz(CPU_FREQ_MHZ);   // 80 MHz (docs/11 §8)

    // Contador de arranques persistido (docs/03 §4.1)
    g_bootCount = incrementBootCount();

    // Comunicación entre tareas (docs/11 §3)
    g_eventQueue = xQueueCreate(8, sizeof(protocol::EventPacket));
    g_cmdQueue = xQueueCreate(8, sizeof(DeviceCommand));
    g_traceMutex = xSemaphoreCreateMutex();

    // Servidor GATT y advertising
    bleServerInit();

    xTaskCreatePinnedToCore(imuTask,      "imu",  8192, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(bleServerTask, "ble",  8192, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(gpsTask,       "gps",  4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(powerTask,     "pwr",  3072, nullptr, 1, nullptr, 0);

    // La lógica vive en las tareas anteriores
    vTaskDelete(nullptr);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
