/**
 * @brief Hard-iron (offset) + soft-iron (scale) calibration state for a
 *        3-axis magnetometer. Platform-independent: takes raw int16 samples,
 *        produces float offset/scale, and applies them to new readings.
 */

#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H

#include <stdint.h>
#include <math.h>

#define MAG_CAL_MAX_SAMPLES 300 // Hafızada tutulacak nokta sayısı

/**
 * @brief Tam 3D Elipsoit Uydurma (Hard-Iron + 3x3 Soft-Iron Matris)
 */
typedef struct {
    float offset[3];            // Hard-iron (Merkez kayması)
    float transform[3][3];      // Soft-iron 3x3 Çapraz Düzeltme Matrisi

    float buffer[MAG_CAL_MAX_SAMPLES][3]; // Veri toplama havuzu
    uint16_t sample_count;

    uint8_t is_calibrating;
    uint8_t is_calibrated;
} MagCal_t;

/**
 * @brief  Kalibrasyon oturumunu başlatır.
 */
void MagCal_Start(MagCal_t *cal);

/**
 * @brief  Her döngüde çağrılır. Sensör yeterince hareket etmişse yeni noktayı kaydeder.
 * @retval 1: Buffer doldu (kalibrasyon bitirilebilir), 0: Toplamaya devam.
 */
uint8_t MagCal_Update(MagCal_t *cal, int16_t raw_x, int16_t raw_y, int16_t raw_z);

/**
 * @brief  Toplanan verilerle 3x3 matrisi ve ofsetleri çözer.
 * @retval 0: Başarılı, -1: Yetersiz veri.
 */
int8_t MagCal_Finish(MagCal_t *cal);

/**
 * @brief  3x3 matris çarpımıyla çapraz kirlilikleri temizler ve gerçek açıyı verir.
 */
void MagCal_Apply(const MagCal_t *cal, int16_t raw_x, int16_t raw_y,
                   int16_t raw_z, float *out_x, float *out_y, float *out_z);

/**
 * @brief Hesaplanmış 3x3 elipsoit matrisini ve ofsetleri sisteme manuel yükler
 */
void MagCal_Set_Manual(MagCal_t *cal,
                       float off_x, float off_y, float off_z,
                       float t00, float t01, float t02,
                       float t10, float t11, float t12,
                       float t20, float t21, float t22);
#endif /* MAG_CALIBRATION_H */
