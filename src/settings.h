// settings.h — persistencia en NVS (docs/11 §1: Preferences)
#pragma once

#include <cstdint>

#include "protocol.h"
#include "CandidateDetector.h"

namespace mcg {

struct StoredSettings {
    uint16_t bootCount;
    DetectionConfig detection;
    int16_t pitchOffsetCdeg;
    int16_t rollOffsetCdeg;
    uint16_t configRevision;
    float gyroBias[3];
};

void loadSettings(StoredSettings& out);
void saveDetectionConfig(const DetectionConfig& cfg, uint16_t revision);
void saveOffsets(int16_t pitchCdeg, int16_t rollCdeg);
void saveGyroBias(float gx, float gy, float gz);
uint16_t incrementBootCount();

} // namespace mcg
