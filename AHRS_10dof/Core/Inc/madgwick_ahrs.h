#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <stdint.h>

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

/* Holds the filter state: current orientation quaternion + gain (beta).*/

typedef struct {
    Quaternion_t q;
    float beta;
} Madgwick_t;

void Madgwick_Init(Madgwick_t *ahrs, float beta_val);

void Madgwick_Process_Raw_Data(
    Madgwick_t *ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float dt);

void MadgwickAHRSupdate(Madgwick_t *ahrs, float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz, float dt);

void MadgwickAHRSupdate6dof(Madgwick_t *ahrs, float gx, float gy, float gz,
                                  float ax, float ay, float az, float dt);


EulerAngles_t QuaternionToEulerAngle(Quaternion_t quaternion);

#endif /* MADGWICK_AHRS_H */
