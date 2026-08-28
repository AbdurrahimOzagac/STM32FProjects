#include "mag_calibration.h"

void MagCal_Start(MagCal_t *cal) {
    cal->min_x = cal->min_y = cal->min_z = INT16_MAX;
    cal->max_x = cal->max_y = cal->max_z = INT16_MIN;
    cal->sample_count = 0;
    cal->is_calibrating = 1;
    cal->is_calibrated = 0;
}

void MagCal_Update(MagCal_t *cal, int16_t raw_x, int16_t raw_y, int16_t raw_z) {
    if (!cal->is_calibrating) {
        return;
    }

    if (raw_x < cal->min_x) cal->min_x = raw_x;
    if (raw_x > cal->max_x) cal->max_x = raw_x;

    if (raw_y < cal->min_y) cal->min_y = raw_y;
    if (raw_y > cal->max_y) cal->max_y = raw_y;

    if (raw_z < cal->min_z) cal->min_z = raw_z;
    if (raw_z > cal->max_z) cal->max_z = raw_z;

    cal->sample_count++;
}

int8_t MagCal_Finish(MagCal_t *cal) {
    cal->is_calibrating = 0;

    /* Hiç örnek toplanmadıysa ya da min/max hiç güncellenmediyse
     * (örneğin Update hiç çağrılmadıysa) kalibrasyonu uygulama. */
    if (cal->sample_count < 10) {
        cal->is_calibrated = 0;
        return -1;
    }

    /* Hard-iron: küre/elipsin kaydığı merkez */
    cal->offset_x = (cal->max_x + cal->min_x) / 2.0f;
    cal->offset_y = (cal->max_y + cal->min_y) / 2.0f;
    cal->offset_z = (cal->max_z + cal->min_z) / 2.0f;

    /* Soft-iron: her eksenin yarı-genliği, ortalamaya göre normalize */
    float scale_x = (cal->max_x - cal->min_x) / 2.0f;
    float scale_y = (cal->max_y - cal->min_y) / 2.0f;
    float scale_z = (cal->max_z - cal->min_z) / 2.0f;

    /* Sıfıra bölmeyi önle: bir eksen hiç hareket etmediyse (örn. tamamen
     * düzlemsel bir sekiz çizildiyse) o eksende scale uygulama. */
    float avg_scale = (scale_x + scale_y + scale_z) / 3.0f;

    cal->scale_x = (scale_x > 1.0f) ? (avg_scale / scale_x) : 1.0f;
    cal->scale_y = (scale_y > 1.0f) ? (avg_scale / scale_y) : 1.0f;
    cal->scale_z = (scale_z > 1.0f) ? (avg_scale / scale_z) : 1.0f;

    cal->is_calibrated = 1;
    return 0;
}

void MagCal_Apply(const MagCal_t *cal, int16_t raw_x, int16_t raw_y,
                   int16_t raw_z, float *out_x, float *out_y, float *out_z) {
    if (!cal->is_calibrated) {
        *out_x = (float) raw_x;
        *out_y = (float) raw_y;
        *out_z = (float) raw_z;
        return;
    }

    *out_x = ((float) raw_x - cal->offset_x) * cal->scale_x;
    *out_y = ((float) raw_y - cal->offset_y) * cal->scale_y;
    *out_z = ((float) raw_z - cal->offset_z) * cal->scale_z;
}
