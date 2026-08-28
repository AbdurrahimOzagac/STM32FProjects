#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include <stdint.h>

/* Euler angles structure for Mahony filter */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Mahony_EulerAngles_t;

/* Quaternion structure for Mahony filter */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Mahony_Quaternion_t;

/* Holds the Mahony filter state: current orientation + PI gains and integral error */
typedef struct {
    Mahony_Quaternion_t q;
    float Kp;
    float Ki;
    float eInt[3];
} Mahony_t;

void Mahony_Init(Mahony_t *ahrs, float Kp_val, float Ki_val);

void Mahony_Process_Raw_Data(
    Mahony_t *ahrs,
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz,
    int16_t mx, int16_t my, int16_t mz,
    float dt);

Mahony_EulerAngles_t Mahony_QuaternionToEulerAngle(Mahony_Quaternion_t quaternion);

#endif /* MAHONY_AHRS_H */
