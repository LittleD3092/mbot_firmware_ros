#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    *theta += wz * dt;

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z,
                                float *x, float *y, float *theta)
{
    (void)wz;  // Heading comes from the IMU gyro.
    if (!x || !y || !theta || dt <= 0.0f) return;

    // Midpoint heading for more accurate position integration.
    float th = *theta + 0.5f * gyro_z * dt;
    float c = cosf(th), s = sinf(th);

    *x += (vx * c - vy * s) * dt;
    *y += (vx * s + vy * c) * dt;
    *theta += gyro_z * dt;

    // Normalize heading to (-pi, pi].
    while (*theta > PI)   *theta -= 2.0f * PI;
    while (*theta <= -PI) *theta += 2.0f * PI;
}