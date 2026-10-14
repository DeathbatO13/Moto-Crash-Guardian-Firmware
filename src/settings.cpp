// settings.cpp — NVS vía Preferences (namespace "mcg")
#include "settings.h"
#include "config.h"

#include <Preferences.h>

namespace mcg {
namespace {
constexpr const char* kNamespace = "mcg";
}

StoredSettings defaultSettings() {
    StoredSettings s {};
    s.bootCount = 0;
    s.detection.tiltEnabled = (DEFAULT_TILT_ENABLED != 0);
    s.detection.impactThresholdMg = DEFAULT_IMPACT_THRESHOLD_MG;
    s.detection.gyroThresholdDps = DEFAULT_GYRO_THRESHOLD_DPS;
    s.detection.tiltThresholdDeg = DEFAULT_TILT_THRESHOLD_DEG;
    s.detection.tiltHoldMs = DEFAULT_TILT_HOLD_MS;
    s.pitchOffsetCdeg = 0;
    s.rollOffsetCdeg = 0;
    s.configRevision = 0;
    s.gyroBias[0] = s.gyroBias[1] = s.gyroBias[2] = 0.0f;
    return s;
}

void loadSettings(StoredSettings& out) {
    out = defaultSettings();

    Preferences prefs;
    if (!prefs.begin(kNamespace, true)) return;

    out.bootCount = prefs.getUShort("bootCount", 0);
    out.detection.tiltEnabled = prefs.getBool("tiltEn",
                                             out.detection.tiltEnabled);
    out.detection.impactThresholdMg = static_cast<uint16_t>(
        prefs.getUShort("impactMg", out.detection.impactThresholdMg));
    out.detection.gyroThresholdDps = static_cast<uint16_t>(
        prefs.getUShort("gyroDps", out.detection.gyroThresholdDps));
    out.detection.tiltThresholdDeg = static_cast<uint8_t>(
        prefs.getUShort("tiltDeg", out.detection.tiltThresholdDeg));
    out.detection.tiltHoldMs = static_cast<uint16_t>(
        prefs.getUShort("tiltHoldMs", out.detection.tiltHoldMs));
    out.pitchOffsetCdeg = static_cast<int16_t>(
        prefs.getInt("pitchOff", out.pitchOffsetCdeg));
    out.rollOffsetCdeg = static_cast<int16_t>(
        prefs.getInt("rollOff", out.rollOffsetCdeg));
    out.configRevision = prefs.getUShort("configRev", 0);
    out.gyroBias[0] = prefs.getFloat("biasX", 0.0f);
    out.gyroBias[1] = prefs.getFloat("biasY", 0.0f);
    out.gyroBias[2] = prefs.getFloat("biasZ", 0.0f);

    prefs.end();
}

void saveDetectionConfig(const DetectionConfig& cfg, uint16_t revision) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, false)) return;
    prefs.putBool("tiltEn", cfg.tiltEnabled);
    prefs.putUShort("impactMg", cfg.impactThresholdMg);
    prefs.putUShort("gyroDps", cfg.gyroThresholdDps);
    prefs.putUShort("tiltDeg", cfg.tiltThresholdDeg);
    prefs.putUShort("tiltHoldMs", cfg.tiltHoldMs);
    prefs.putUShort("configRev", revision);
    prefs.end();
}

void saveOffsets(int16_t pitchCdeg, int16_t rollCdeg) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, false)) return;
    prefs.putInt("pitchOff", pitchCdeg);
    prefs.putInt("rollOff", rollCdeg);
    prefs.end();
}

void saveGyroBias(float gx, float gy, float gz) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, false)) return;
    prefs.putFloat("biasX", gx);
    prefs.putFloat("biasY", gy);
    prefs.putFloat("biasZ", gz);
    prefs.end();
}

uint16_t incrementBootCount() {
    Preferences prefs;
    if (!prefs.begin(kNamespace, false)) return 1;
    const uint16_t count = prefs.getUShort("bootCount", 0) + 1;
    prefs.putUShort("bootCount", count);   // el wrap a 0 es válido:
                                           // la clave es bootCount:eventId
    prefs.end();
    return count;
}

} // namespace mcg
