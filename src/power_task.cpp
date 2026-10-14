// power_task.cpp — nivel de batería con curva LiPo 1S (docs/11 §8).
// Tarea en core 0, prioridad 1.
#include "shared.h"
#include "config.h"
#include "protocol.h"

namespace mcg {
namespace {

bool g_lowBatteryAnnounced = false;

uint8_t batteryPercent() {
#if BATTERY_SENSING_ENABLED
    uint32_t avg = 0;
    for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; ++i) {
        avg += analogRead(PIN_BATTERY_ADC);
        delay(1);
    }
    avg /= BATTERY_ADC_SAMPLES;

    // ADC_11db: 0–3.3 V → 0–4095; divisor 100k/100k (×2)
    const float voltage =
        (avg * static_cast<float>(BATTERY_ADC_REF_MV)
             / static_cast<float>(BATTERY_ADC_MAX))
        * BATTERY_DIVIDER_RATIO / 1000.0f;

    // Curva por tramos LiPo 1S (docs/11 §8):
    // 4.20 V = 100% · 3.70 V ≈ 50% · 3.30 V = 0%
    if (voltage <= BATTERY_VOLTAGE_EMPTY_V) return 0;
    if (voltage >= BATTERY_VOLTAGE_FULL_V) return 100;
    if (voltage <= BATTERY_VOLTAGE_MID_V) {
        return static_cast<uint8_t>(
            (voltage - BATTERY_VOLTAGE_EMPTY_V)
                / (BATTERY_VOLTAGE_MID_V - BATTERY_VOLTAGE_EMPTY_V)
                * 50.0f);
    }
    return static_cast<uint8_t>(
        50.0f + (voltage - BATTERY_VOLTAGE_MID_V)
                    / (BATTERY_VOLTAGE_FULL_V - BATTERY_VOLTAGE_MID_V)
                    * 50.0f);
#else
    return 255;   // desconocida
#endif
}

} // namespace

void powerTask(void* param) {
    (void)param;

#if BATTERY_SENSING_ENABLED
    analogSetWidth(12);
    analogSetAttenuation(ADC_11db);
#endif

    uint32_t lastReadMs = 0;

    for (;;) {
        const uint32_t now = millis();
        if (now - lastReadMs >= 1000) {
            lastReadMs = now;

            const uint8_t pct = batteryPercent();

            portENTER_CRITICAL(&g_stateMux);
            g_telemetry.batteryPct = pct;
            portEXIT_CRITICAL(&g_stateMux);

            // bit3 (cargando) = 0: PIN_CHRG ⚠️ no implementada

            // Batería < 5%: evento FAULT(3) por flanco;
            // sigue operando hasta el corte del protector (docs/11 §8)
            if (pct != 255 && pct <= BATTERY_LOW_PCT &&
                !g_lowBatteryAnnounced) {
                g_lowBatteryAnnounced = true;
                DeviceCommand cmd {};
                cmd.opcode = kInternalBatteryFault;
                cmd.requestId = 0;
                cmd.param = 0;
                xQueueSend(g_cmdQueue, &cmd, 0);
            }
            // Histéresis para re-anunciar tras recuperación
            if (pct == 255 || pct > BATTERY_LOW_PCT + 2) {
                g_lowBatteryAnnounced = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

} // namespace mcg
