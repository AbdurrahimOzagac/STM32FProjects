#include "mag_calibration.h"

// --- 3x3 Matris Yardımcı Fonksiyonları (Sadece bu dosyada kullanılır) ---
static void Mat3x3_Mul(float A[3][3], float B[3][3], float out[3][3]) {
    float temp[3][3] = {0};
    for(int r = 0; r < 3; r++) {
        for(int c = 0; c < 3; c++) {
            temp[r][c] = A[r][0]*B[0][c] + A[r][1]*B[1][c] + A[r][2]*B[2][c];
        }
    }
    for(int r = 0; r < 3; r++) {
        for(int c = 0; c < 3; c++) {
            out[r][c] = temp[r][c];
        }
    }
}

static void Mat3x3_Scale(float A[3][3], float scalar) {
    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            A[r][c] *= scalar;
}

static void Mat3x3_Identity(float A[3][3]) {
    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            A[r][c] = (r == c) ? 1.0f : 0.0f;
}

static void Mat3x3_Sub(float A[3][3], float B[3][3], float out[3][3]) {
    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            out[r][c] = A[r][c] - B[r][c];
}
// ----------------------------------------------------------------------

void MagCal_Start(MagCal_t *cal) {
    cal->sample_count = 0;
    cal->is_calibrating = 1;
    cal->is_calibrated = 0;
}

uint8_t MagCal_Update(MagCal_t *cal, int16_t raw_x, int16_t raw_y, int16_t raw_z) {
    if (!cal->is_calibrating) return 0;

    // Yeterli veri toplandıysa kullanıcıya "Bitti" sinyali ver
    if (cal->sample_count >= MAG_CAL_MAX_SAMPLES) return 1;

    // Sadece hareket varsa veriyi kaydet (Aynı verinin 300 kere kaydedilmesini önler)
    if (cal->sample_count > 0) {
        float dx = raw_x - cal->buffer[cal->sample_count - 1][0];
        float dy = raw_y - cal->buffer[cal->sample_count - 1][1];
        float dz = raw_z - cal->buffer[cal->sample_count - 1][2];

        // Mesafe çok küçükse (titreme vs.) kaydetme. Değer sensör gürültüne göre ayarlanabilir.
        if ((dx*dx + dy*dy + dz*dz) < 10000.0f) {
            return 0;
        }
    }

    cal->buffer[cal->sample_count][0] = (float)raw_x;
    cal->buffer[cal->sample_count][1] = (float)raw_y;
    cal->buffer[cal->sample_count][2] = (float)raw_z;
    cal->sample_count++;

    return (cal->sample_count >= MAG_CAL_MAX_SAMPLES) ? 1 : 0;
}

int8_t MagCal_Finish(MagCal_t *cal) {
    cal->is_calibrating = 0;

    int N = cal->sample_count;
    if (N < 50) { // En az 50 farklı nokta toplanmalı
        cal->is_calibrated = 0;
        return -1;
    }

    // 1. Hard-Iron (Merkez/Ofset) Hesabı
    cal->offset[0] = cal->offset[1] = cal->offset[2] = 0.0f;
    for(int i = 0; i < N; i++) {
        cal->offset[0] += cal->buffer[i][0];
        cal->offset[1] += cal->buffer[i][1];
        cal->offset[2] += cal->buffer[i][2];
    }
    cal->offset[0] /= N;
    cal->offset[1] /= N;
    cal->offset[2] /= N;

    // 2. Kovaryans Matrisi Oluşturma (Elipsin Şekli)
    float cov[3][3] = {0};
    for(int i = 0; i < N; i++) {
        float dx = cal->buffer[i][0] - cal->offset[0];
        float dy = cal->buffer[i][1] - cal->offset[1];
        float dz = cal->buffer[i][2] - cal->offset[2];

        cov[0][0] += dx*dx; cov[0][1] += dx*dy; cov[0][2] += dx*dz;
        cov[1][0] += dy*dx; cov[1][1] += dy*dy; cov[1][2] += dy*dz;
        cov[2][0] += dz*dx; cov[2][1] += dz*dy; cov[2][2] += dz*dz;
    }
    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            cov[r][c] /= N;

    // 3. Newton-Schulz için Matrisi Normalize Etme
    float trace = cov[0][0] + cov[1][1] + cov[2][2];
    if (trace == 0.0f) return -1;

    float k = 1.0f / trace;
    float C_norm[3][3];
    for(int r = 0; r < 3; r++)
        for(int c = 0; c < 3; c++)
            C_norm[r][c] = cov[r][c] * k;

    // 4. Newton-Schulz İterasyonu (Kovaryans Matrisinin Ters Karekökünü Alma)
    float X[3][3];
    Mat3x3_Identity(X);

    float temp1[3][3], temp2[3][3], temp3[3][3];

    for(int iter = 0; iter < 15; iter++) {
        Mat3x3_Mul(X, X, temp1);            // temp1 = X_n * X_n
        Mat3x3_Mul(C_norm, temp1, temp2);   // temp2 = C_norm * X_n * X_n

        Mat3x3_Identity(temp3);
        Mat3x3_Scale(temp3, 3.0f);
        Mat3x3_Sub(temp3, temp2, temp3);    // temp3 = 3I - temp2

        Mat3x3_Mul(X, temp3, temp1);        // temp1 = X_n * (3I - temp2)
        Mat3x3_Scale(temp1, 0.5f);          // X_{n+1} = 0.5 * temp1

        for(int r = 0; r < 3; r++)
            for(int c = 0; c < 3; c++)
                X[r][c] = temp1[r][c];
    }

    // 5. Ölçeği (Manyetik Şiddeti) Orijinal Seviyeye Geri Döndürme
    float target_radius = 0.0f;
    for(int i = 0; i < N; i++) {
        float dx = cal->buffer[i][0] - cal->offset[0];
        float dy = cal->buffer[i][1] - cal->offset[1];
        float dz = cal->buffer[i][2] - cal->offset[2];
        target_radius += sqrtf(dx*dx + dy*dy + dz*dz);
    }
    target_radius /= N;

    float final_scale = sqrtf(k) * target_radius;
    for(int r = 0; r < 3; r++) {
        for(int c = 0; c < 3; c++) {
            cal->transform[r][c] = X[r][c] * final_scale;
        }
    }

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

    // 1. Hard-Iron düzeltmesi (Merkeze çekme)
    float dx = (float)raw_x - cal->offset[0];
    float dy = (float)raw_y - cal->offset[1];
    float dz = (float)raw_z - cal->offset[2];

    // 2. Soft-Iron düzeltmesi (3x3 Matris çarpımı ile çapraz kirliliklerin temizlenmesi)
    *out_x = cal->transform[0][0]*dx + cal->transform[0][1]*dy + cal->transform[0][2]*dz;
    *out_y = cal->transform[1][0]*dx + cal->transform[1][1]*dy + cal->transform[1][2]*dz;
    *out_z = cal->transform[2][0]*dx + cal->transform[2][1]*dy + cal->transform[2][2]*dz;
}

void MagCal_Set_Manual(MagCal_t *cal,
                       float off_x, float off_y, float off_z,
                       float t00, float t01, float t02,
                       float t10, float t11, float t12,
                       float t20, float t21, float t22) {

    cal->offset[0] = off_x;
    cal->offset[1] = off_y;
    cal->offset[2] = off_z;

    cal->transform[0][0] = t00; cal->transform[0][1] = t01; cal->transform[0][2] = t02;
    cal->transform[1][0] = t10; cal->transform[1][1] = t11; cal->transform[1][2] = t12;
    cal->transform[2][0] = t20; cal->transform[2][1] = t21; cal->transform[2][2] = t22;

    cal->is_calibrating = 0;
    cal->is_calibrated = 1;
}
