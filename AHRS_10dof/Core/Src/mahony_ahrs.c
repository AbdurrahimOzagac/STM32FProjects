#include "mahony_ahrs.h"
#include <math.h>
#include <string.h>

/* MPU6050 +-2g & +-250dps sensitivity */
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f
#define DEG_TO_RAD  0.01745329251f
#define RAD_TO_DEG  57.2957795131f

/* Fast inverse square root (Quake III algorithm) */
static float invSqrt(float x) {
    float y = x;
    int32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (0.5f * x * y * y));
    return y;
}

void Mahony_Init(Mahony_t *ahrs, float Kp_val, float Ki_val) {
    ahrs->Kp = Kp_val;
    ahrs->Ki = Ki_val;
    ahrs->q.q0 = 1.0f;
    ahrs->q.q1 = 0.0f;
    ahrs->q.q2 = 0.0f;
    ahrs->q.q3 = 0.0f;
    ahrs->eInt[0] = 0.0f;
    ahrs->eInt[1] = 0.0f;
    ahrs->eInt[2] = 0.0f;
}

/* Core 9-DOF Mahony update algorithm */
static void MahonyAHRSupdate(Mahony_t *ahrs, float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz, float dt) {

    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float q0 = ahrs->q.q0;
    float q1 = ahrs->q.q1;
    float q2 = ahrs->q.q2;
    float q3 = ahrs->q.q3;

    /* Compute feedback only if accelerometer measurement valid */
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* Normalize accelerometer measurement */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Normalize magnetometer measurement */
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
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

        /* Reference direction of Earth's magnetic field */
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        /* Estimated direction of gravity and magnetic field */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        /* Error is sum of cross product between estimated direction and measured direction */
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        /* Compute and apply integral feedback if enabled */
        if(ahrs->Ki > 0.0f) {
            ahrs->eInt[0] += ahrs->Ki * halfex * dt;
            ahrs->eInt[1] += ahrs->Ki * halfey * dt;
            ahrs->eInt[2] += ahrs->Ki * halfez * dt;
            gx += ahrs->eInt[0];
            gy += ahrs->eInt[1];
            gz += ahrs->eInt[2];
        } else {
            ahrs->eInt[0] = 0.0f;
            ahrs->eInt[1] = 0.0f;
            ahrs->eInt[2] = 0.0f;
        }

        /* Apply proportional feedback */
        gx += ahrs->Kp * halfex;
        gy += ahrs->Kp * halfey;
        gz += ahrs->Kp * halfez;
    }

    /* Integrate rate of change of quaternion */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalize quaternion */
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    ahrs->q.q0 = q0 * recipNorm;
    ahrs->q.q1 = q1 * recipNorm;
    ahrs->q.q2 = q2 * recipNorm;
    ahrs->q.q3 = q3 * recipNorm;
}

void Mahony_Process_Raw_Data(
    Mahony_t *ahrs,
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz,
    int16_t mx, int16_t my, int16_t mz,
    float dt)
{
    float f_ax = (float)ax / ACCEL_SCALE;
    float f_ay = (float)ay / ACCEL_SCALE;
    float f_az = (float)az / ACCEL_SCALE;

    float f_gx = ((float)gx / GYRO_SCALE) * DEG_TO_RAD;
    float f_gy = ((float)gy / GYRO_SCALE) * DEG_TO_RAD;
    float f_gz = ((float)gz / GYRO_SCALE) * DEG_TO_RAD;

    float f_mx = (float)mx;
    float f_my = (float)my;
    float f_mz = (float)mz;

    MahonyAHRSupdate(ahrs, f_gx, f_gy, f_gz, f_ax, f_ay, f_az, f_mx, f_my, f_mz, dt);
}

Mahony_EulerAngles_t Mahony_QuaternionToEulerAngle(Mahony_Quaternion_t quaternion) {
    Mahony_EulerAngles_t angles;
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
