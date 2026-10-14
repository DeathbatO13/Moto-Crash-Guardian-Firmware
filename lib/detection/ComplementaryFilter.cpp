#include "ComplementaryFilter.h"
#include <cmath>
#include <algorithm>

namespace mcg {

ComplementaryFilter::ComplementaryFilter(float alpha, float dtSeconds)
    : m_alpha(alpha), m_dt(dtSeconds) {}

void ComplementaryFilter::setBias(float gx, float gy, float gz) {
    m_biasGx = gx;
    m_biasGy = gy;
    m_biasGz = gz;
}

void ComplementaryFilter::getBias(float* gx, float* gy, float* gz) const {
    if (gx) *gx = m_biasGx;
    if (gy) *gy = m_biasGy;
    if (gz) *gz = m_biasGz;
}

void ComplementaryFilter::startBiasCalibration() {
    m_calibrating = true;
    m_calibSamples = 0;
    m_sumGx = m_sumGy = m_sumGz = 0.0f;
    m_calibStill = true;
}

bool ComplementaryFilter::isCalibrating() const {
    return m_calibrating;
}

void ComplementaryFilter::reset(float pitchDeg, float rollDeg) {
    m_pitchDeg = pitchDeg;
    m_rollDeg = rollDeg;
}

void ComplementaryFilter::update(float axG, float ayG, float azG,
                                 float gxDps, float gyDps, float gzDps) {
    if (m_calibrating) {
        m_sumGx += gxDps;
        m_sumGy += gyDps;
        m_sumGz += gzDps;
        const float accelMag = std::sqrt(axG * axG + ayG * ayG + azG * azG);
        const float gyroMag  = std::sqrt(gxDps * gxDps + gyDps * gyDps + gzDps * gzDps);
        if (std::abs(accelMag - 1.0f) > kStillnessG || gyroMag > kStillnessDps) {
            m_calibStill = false;
        }
        if (++m_calibSamples >= kCalibSamples) {
            m_calibrating = false;
            if (m_calibStill) {
                m_biasGx = m_sumGx / kCalibSamples;
                m_biasGy = m_sumGy / kCalibSamples;
                m_biasGz = m_sumGz / kCalibSamples;
            }
            // si hubo movimiento: se conserva el sesgo anterior (docs/11 §5.2)
        }
    }

    const float bx = gxDps - m_biasGx;
    const float by = gyDps - m_biasGy;
    const float bz = gzDps - m_biasGz;

    // Ángulos por acelerómetro (docs/11 §5.4)
    const float pitchAcc = std::atan2(-axG, std::sqrt(ayG * ayG + azG * azG))
                           * 180.0f / kPi;
    const float rollAcc = std::atan2(ayG, azG) * 180.0f / kPi;

    // Fuera de 1 g ±0.3 g (impacto o aceleración fuerte): solo giroscopio
    const float accelMag = std::sqrt(axG * axG + ayG * ayG + azG * azG);
    const float alpha = (std::abs(accelMag - 1.0f) > kStillnessG) ? 1.0f : m_alpha;

    // pitch integra el giroscopio Y, roll el X (convención de montaje ⚠️
    // POR CONFIRMAR; el TARE compensa el offset estático y la detección
    // usa |pitch|/|roll|, por lo que el signo no altera el producto).
    m_pitchDeg = alpha * (m_pitchDeg + by * m_dt) + (1.0f - alpha) * pitchAcc;
    m_rollDeg  = alpha * (m_rollDeg + bx * m_dt) + (1.0f - alpha) * rollAcc;
}

} // namespace mcg
