// gps_task.cpp — NMEA del NEO-6M a 9600 baudios, snapshot a 1 Hz
// (docs/11 §3). Tarea en core 0, prioridad 2.
#include "shared.h"
#include "config.h"

#include <algorithm>
#include <cmath>

#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

namespace mcg {
namespace {

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

uint32_t g_lastValidFixMs = 0;

} // namespace

void gpsTask(void* param) {
    (void)param;

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    uint32_t lastPublishMs = 0;

    for (;;) {
        while (gpsSerial.available() > 0) {
            gps.encode(static_cast<char>(gpsSerial.read()));
        }

        const uint32_t now = millis();
        if (now - lastPublishMs >= 1000) {
            lastPublishMs = now;

            GpsSnapshot snap {};
            snap.present = true;

            if (gps.location.isValid() && gps.location.age() < 2000) {
                const uint32_t sats = gps.satellites.value();
                snap.fixType = (sats >= 4) ? 3 : 2;   // 3D / 2D
                snap.satellites = static_cast<uint8_t>(
                    std::min(sats, 255u));
                snap.hdopX10 = gps.hdop.isValid()
                                   ? static_cast<uint8_t>(
                                         std::min(gps.hdop.hdop() * 10.0,
                                                  255.0))
                                   : 255;
                snap.latE7 = static_cast<int32_t>(
                    llround(gps.location.lat() * 1e7));
                snap.lonE7 = static_cast<int32_t>(
                    llround(gps.location.lng() * 1e7));
                snap.speedKmhX100 = static_cast<uint16_t>(
                    std::min(gps.speed.kmph() * 100.0, 65535.0));
                g_lastValidFixMs = now;
            } else {
                snap.fixType = 0;
            }

            snap.fixAgeS = (g_lastValidFixMs == 0)
                               ? 65535
                               : static_cast<uint16_t>(std::min(
                                     (now - g_lastValidFixMs) / 1000,
                                     65535u));

            portENTER_CRITICAL(&g_stateMux);
            g_gps = snap;
            portEXIT_CRITICAL(&g_stateMux);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

} // namespace mcg
