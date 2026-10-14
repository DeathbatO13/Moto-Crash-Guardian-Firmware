#pragma once

// ============================================================================
// Moto Crash Guardian — configuración de compilación (docs/11 §2, §5, §8)
// Los pines coinciden con el cableado actual del prototipo.
// Marcados con ⚠️ deben confirmarse con el cableado real.
// ============================================================================

// --- Versión del firmware (se reporta en DEVICE_INFO, docs/03 §4.1) ---
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 0
#define PROTOCOL_VERSION 1u

// --- Pines (docs/11 §2) ⚠️ POR CONFIRMAR con el cableado real ---
#define PIN_IMU_SDA      21   // I²C SDA -> MPU6050
#define PIN_IMU_SCL      22   // I²C SCL -> MPU6050
#define PIN_IMU_INT      19   // MPU6050 INT (data ready) — no usado por defecto
#define PIN_GPS_RX       16   // ESP32 RX <- NEO-6M TX
#define PIN_GPS_TX       17   // ESP32 TX -> NEO-6M RX (opcional)
#define PIN_BATTERY_ADC  34   // ADC1, divisor 100k/100k ⚠️
#define PIN_CHRG         35   // señal CHRG del cargador ⚠️ no implementada
#define PIN_STATUS_LED   2    // LED integrado del DevKit

#define MPU6050_ADDR     0x68
#define I2C_FREQ_HZ      400000u   // docs/11 §2: I²C a 400 kHz
#define MPU6050_FULL_SCALE 32767   // |raw| = full scale → saturado

// --- GPS (NEO-6M 001) ---
#define GPS_BAUD         9600u

// --- IMU (docs/11 §5) ---
#define IMU_SAMPLE_HZ        200u
#define IMU_ACCEL_LSB_PER_G  2048.0f   // ±16 g
#define IMU_GYRO_LSB_PER_DPS 16.4f     // ±2000 °/s
#define IMU_FILTER_ALPHA     0.98f
#define IMU_ANGLE_TRUST_G    0.3f      // fuera de 1 g ±0.3 g → solo giroscopio

// --- Detector (docs/11 §6; defaults de docs/04 §2) ---
#define DEFAULT_IMPACT_THRESHOLD_MG  3500u
#define DEFAULT_GYRO_THRESHOLD_DPS   300u
#define DEFAULT_TILT_THRESHOLD_DEG   65u
#define DEFAULT_TILT_HOLD_MS         2000u
#define DEFAULT_TILT_ENABLED         1

// --- Máquina de estados (docs/11 §4) ---
#define POST_TRIGGER_MS        3000u
#define COOLDOWN_MS            10000u
#define IMPACT_LOOKAHEAD_SAMPLES 20u   // 100 ms @ 200 Hz (picos en ±100 ms)
#define TILT_IMPACT_GAP_MS     5000u // TILT válido sin IMPACT en 5 s
#define TARE_STILLNESS_TOL_MG  50u   // |a| dentro de 1 g ± 50 mg
#define TARE_STILLNESS_GYRO_DPS 5u   // |ω| ≤ 5 °/s
#define TARE_WINDOW_SAMPLES    200u  // 1 s @ 200 Hz
#define GYRO_BIAS_SAMPLES      200u  // calibración de sesgo en arranque

// --- Ring buffer y caja negra (docs/11 §7) ---
#define RING_CAPACITY_SAMPLES  600u   // 3 s @ 200 Hz
#define TRACE_TOTAL_SAMPLES    1200u  // 3 s pre + 3 s post
#define TRACE_SLOT_COUNT       2u
#define TRACE_SAMPLES_PER_CHUNK 19u   // floor((247 − 3 − 6) / 12)
#define TRACE_CHUNK_COUNT      64u    // ceil(1200 / 19), referencia

// --- BLE (docs/03 §1, §2) ---
#define ADV_INTERVAL_FAST_MS   100u
#define ADV_INTERVAL_SLOW_MS   500u
#define ADV_SLOW_AFTER_MS      30000u
#define MAX_UNACKED_EVENTS     8u
#define BLE_MTU_PREFERRED      247u
#define CONN_INTERVAL_MIN_MS   30u   // → 24 unidades de 1.25 ms
#define CONN_INTERVAL_MAX_MS   50u   // → 40
#define CONN_SUPERVISION_TIMEOUT_MS 4000u  // → 400 unidades de 10 ms

// --- Energía (docs/11 §8) ---
#define CPU_FREQ_MHZ           80    // ⚠️ POR CONFIRMAR con medición
#define BATTERY_SENSING_ENABLED 1    // 0 si no está poblado el divisor
#define BATTERY_DIVIDER_RATIO  2.0f  // 100k/100k
#define BATTERY_VOLTAGE_FULL_V  4.20f
#define BATTERY_VOLTAGE_MID_V   3.70f
#define BATTERY_VOLTAGE_EMPTY_V 3.30f
#define BATTERY_LOW_PCT         5u
#define BATTERY_ADC_SAMPLES     16u
#define BATTERY_ADC_REF_MV      3300u   // con ADC_11db
#define BATTERY_ADC_MAX         4095u

// --- Depuración por serial (docs/11 §9: el formato de texto se conserva) ---
#define SERIAL_TELEMETRY 1
