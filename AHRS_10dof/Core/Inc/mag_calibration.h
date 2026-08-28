#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H

#include <stdint.h>

/**
 * @brief Hard-iron (offset) + soft-iron (scale) calibration state for a
 *        3-axis magnetometer. Platform-independent: takes raw int16 samples,
 *        produces float offset/scale, and applies them to new readings.
 */
typedef struct {
    int16_t min_x, max_x;
    int16_t min_y, max_y;
    int16_t min_z, max_z;

    float offset_x, offset_y, offset_z;
    float scale_x, scale_y, scale_z;

    uint8_t is_calibrating;
    uint8_t is_calibrated;
    uint32_t sample_count;
} MagCal_t;

/**
 * @brief  Kalibrasyon oturumunu başlatır. min/max değerlerini sıfırlar.
 *         Bundan sonra kullanıcı sensörü sekiz çizerek her yöne çevirmeli.
 */
void MagCal_Start(MagCal_t *cal);

/**
 * @brief  Kalibrasyon penceresi boyunca her döngüde (her yeni ham okumada)
 *         çağrılır. min/max sınırlarını günceller.
 * @note   MagCal_Start() çağrılmadan bunu çağırmak min/max'ı bozar,
 *         is_calibrating false ise fonksiyon hiçbir şey yapmaz.
 */
void MagCal_Update(MagCal_t *cal, int16_t raw_x, int16_t raw_y, int16_t raw_z);

/**
 * @brief  Toplama penceresini kapatır, offset ve scale değerlerini hesaplar.
 *         is_calibrated = 1 yapar. Yetersiz veri varsa (örn. hiç örnek
 *         toplanmadıysa) is_calibrated 0 kalır ve kalibrasyon uygulanmaz.
 * @retval 0 başarılı, -1 yetersiz veri
 */
int8_t MagCal_Finish(MagCal_t *cal);

/**
 * @brief  Hesaplanmış offset/scale'i ham bir okumaya uygular.
 *         is_calibrated == 0 ise ham veriyi olduğu gibi geri döner
 *         (kalibrasyon henüz yapılmamışsa sistemi kilitlememek için).
 */
void MagCal_Apply(const MagCal_t *cal, int16_t raw_x, int16_t raw_y,
                   int16_t raw_z, float *out_x, float *out_y, float *out_z);

#endif /* MAG_CALIBRATION_H */
