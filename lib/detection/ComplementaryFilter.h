// ComplementaryFilter — orientación IMU por filtro complementario,
// C++ puro (docs/11 §5.4). Sin dependencias de Arduino: se prueba en native.
#pragma once

namespace mcg {

class ComplementaryFilter {
public:
    explicit ComplementaryFilter(float alpha = 0.98f, float dtSeconds = 0.005f);

    // Sesgo del giroscopio (docs/11 §5.2): se resta antes de integrar.
    void setBias(float gxDps, float gyDps, float gzDps);
    void getBias(float* gxDps, float* gyDps, float* gzDps) const;

    // Calibración de arranque: llamar con cada muestra; tras 200 muestras
    // en reposo adopta el promedio; si hubo movimiento, conserva el sesgo
    // previo (setBias).
    void startBiasCalibration();
    bool isCalibrating() const;

    void reset(float pitchDeg, float rollDeg);

    // Una muestra: acelerómetro en g, giroscopio en °/s.
    void update(float axG, float ayG, float azG,
                float gxDps, float gyDps, float gzDps);

    float pitchDeg() const { return m_pitchDeg; }
    float rollDeg() const { return m_rollDeg; }

private:
    static constexpr float kPi = 3.14159265358979323846f;
    static constexpr int   kCalibSamples = 200;   // GYRO_BIAS_SAMPLES
    static constexpr float kStillnessG   = 0.3f;  // ±0.3 g alrededor de 1 g
    static constexpr float kStillnessDps = 5.0f;  // |ω| ≤ 5 °/s

    float m_alpha;
    float m_dt;
    float m_pitchDeg = 0.0f;
    float m_rollDeg = 0.0f;

    float m_biasGx = 0.0f, m_biasGy = 0.0f, m_biasGz = 0.0f;

    bool m_calibrating = false;
    int  m_calibSamples = 0;
    float m_sumGx = 0.0f, m_sumGy = 0.0f, m_sumGz = 0.0f;
    bool m_calibStill = true;
};

} // namespace mcg
