#include "madgwick_ahrs.h"
#include <math.h>
#include <string.h>

/* MPU6050 +-2g & +-250dps sensitivity (see datasheet) */
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f
#define DEG_TO_RAD 0.01745329251f
#define RAD_TO_DEG 57.2957795131f

/* Fast inverse square root (Quake III algorithm).
 * Uses memcpy instead of pointer-casting to avoid strict-aliasing
 * undefined behavior; compilers optimize this away to the same
 * bit-reinterpretation, so there is no runtime cost. */
static float invSqrt(float x) {
    float y = x;
    int32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (0.5f * x * y * y));
    return y;
}

void Madgwick_Init(Madgwick_t *ahrs, float beta_val) {
    ahrs->beta = beta_val;
    ahrs->q.q0 = 1.0f;
    ahrs->q.q1 = 0.0f;
    ahrs->q.q2 = 0.0f;
    ahrs->q.q3 = 0.0f;
}

/* Core 9-DOF Madgwick update. Operates directly on the quaternion
 * stored in the Madgwick_t instance, using its own beta gain
 * instead of relying on any global/shared state. */
void MadgwickAHRSupdate(Madgwick_t *ahrs, float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz, float dt) {

    float q0 = ahrs->q.q0;
    float q1 = ahrs->q.q1;
    float q2 = ahrs->q.q2;
    float q3 = ahrs->q.q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    /* Bail out on invalid (all-zero) sensor readings rather than
     * dividing by zero in the normalization steps below. */
    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) return;
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) return;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    /* Use this instance's own beta gain, not a shared global. */
    qDot1 -= ahrs->beta * s0;
    qDot2 -= ahrs->beta * s1;
    qDot3 -= ahrs->beta * s2;
    qDot4 -= ahrs->beta * s3;

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    ahrs->q.q0 = q0 * recipNorm;
    ahrs->q.q1 = q1 * recipNorm;
    ahrs->q.q2 = q2 * recipNorm;
    ahrs->q.q3 = q3 * recipNorm;
}

void MadgwickAHRSupdate6dof(Madgwick_t *ahrs, float gx, float gy, float gz,
                                  float ax, float ay, float az, float dt) {
    float q0 = ahrs->q.q0;
    float q1 = ahrs->q.q1;
    float q2 = ahrs->q.q2;
    float q3 = ahrs->q.q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;

    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) return;

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    /* Normalise accelerometer measurement */
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* Auxiliary variables to avoid repeated arithmetic */
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;

    /* Gradient decent algorithm corrective step (Gravity vector only) */
    s0 = _4q0 * q2 * q2 + _2q2 * ax + _4q0 * q1 * q1 - _2q1 * ay;
    s1 = _4q1 * q3 * q3 - _2q3 * ax + 4.0f * q0 * q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1 * q1 + _8q1 * q2 * q2 + _4q1 * az;
    s2 = 4.0f * q0 * q0 * q2 + _2q0 * ax + _4q2 * q3 * q3 - _2q3 * ay - _4q2 + _8q2 * q1 * q1 + _8q2 * q2 * q2 + _4q2 * az;
    s3 = 4.0f * q1 * q1 * q3 - _2q1 * ax + 4.0f * q2 * q2 * q3 - _2q2 * ay;

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    /* Apply feedback step */
    qDot1 -= ahrs->beta * s0;
    qDot2 -= ahrs->beta * s1;
    qDot3 -= ahrs->beta * s2;
    qDot4 -= ahrs->beta * s3;

    /* Integrate rate of change of quaternion to yield quaternion */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* Normalise quaternion */
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    ahrs->q.q0 = q0 * recipNorm;
    ahrs->q.q1 = q1 * recipNorm;
    ahrs->q.q2 = q2 * recipNorm;
    ahrs->q.q3 = q3 * recipNorm;
}

void Madgwick_Process_Raw_Data(
    Madgwick_t *ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float dt)
{
    float f_ax = ax / ACCEL_SCALE;
    float f_ay = ay / ACCEL_SCALE;
    float f_az = az / ACCEL_SCALE;

    float f_gx = (gx / GYRO_SCALE) * DEG_TO_RAD;
    float f_gy = (gy / GYRO_SCALE) * DEG_TO_RAD;
    float f_gz = (gz / GYRO_SCALE) * DEG_TO_RAD;

    /* Magnetometer values are normalized inside MadgwickAHRSupdate,
     * so raw units (not converted to gauss) are fine as-is. */
    float f_mx = mx;
    float f_my = my;
    float f_mz = mz;

    MadgwickAHRSupdate(ahrs, f_gx, f_gy, f_gz, f_ax, f_ay, f_az, f_mx, f_my, f_mz, dt);
//    MadgwickAHRSupdate6dof(ahrs, f_gx, f_gy, f_gz, f_ax, f_ay, f_az, dt);

}

EulerAngles_t QuaternionToEulerAngle(Quaternion_t quaternion) {
    EulerAngles_t angles;
    float q0 = quaternion.q0;
    float q1 = quaternion.q1;
    float q2 = quaternion.q2;
    float q3 = quaternion.q3;

    angles.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    angles.pitch = asinf(sinp) * RAD_TO_DEG;

    angles.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
    if (angles.yaw < 0.0f) {
        angles.yaw += 360.0f;
    }

    return angles;
}
