#include "heading.h"

#include <math.h>

#include "angle.h"
#include "config.h"

#define STILL_GYRO_THRESH_RAD_S   DEG2RAD(0.5f)
#define STILL_ACCEL_THRESH_M_S2   0.3f
#define STILL_TICKS               50
#define IMU_BIAS_ALPHA            0.004f
#define GRAVITY_M_S2              9.80665f

static float    s_psi;
static float    s_rate;
static float    s_bias;
static float    s_calibrationStddev;
static uint64_t s_previousUs;
static uint32_t s_stillTicks;
static bool     s_valid;

bool heading_calibrate(float *mean_rad_s, float *stddev_rad_s)
{
    float mean = 0.0f;
    float stddev = 0.0f;
    if (!imu_calibrate_bias(&mean, &stddev))
        return false;

    s_bias = mean;
    s_calibrationStddev = stddev;
    imu_set_yaw_bias(s_bias);
    s_valid = true;
    s_stillTicks = 0;
    s_previousUs = 0;
    if (mean_rad_s)
        *mean_rad_s = mean;
    if (stddev_rad_s)
        *stddev_rad_s = stddev;
    return true;
}

void heading_init(void)
{
    s_psi = 0.0f;
    s_rate = 0.0f;
    s_bias = 0.0f;
    s_calibrationStddev = 0.0f;
    s_previousUs = 0;
    s_stillTicks = 0;
    s_valid = false;
    imu_set_yaw_bias(0.0f);
    (void)heading_calibrate(NULL, NULL);
}

void heading_update(const imu_sample_t *sample, const encoder_sample_t *encoders,
                    bool procedure_active)
{
    if (!sample || !encoders || !sample->fresh)
        return;

    if (s_previousUs != 0) {
        const float dt = (float)(sample->t_us - s_previousUs) * 1e-6f;
        if (dt > 0.0f && dt < 0.1f)
            s_psi = wrap_pi(s_psi + sample->yaw_rate * dt);
    }
    s_previousUs = sample->t_us;
    s_rate = sample->yaw_rate;

    // The driver has already subtracted s_bias, so add it back to perform the
    // stillness test and update against the uncorrected signed rate.
    const float raw_rate = sample->yaw_rate + s_bias;
    const bool accel_still = fabsf(fabsf(sample->az) - GRAVITY_M_S2)
                           < STILL_ACCEL_THRESH_M_S2;
    const bool still = !procedure_active
                    && encoders->left_delta == 0
                    && encoders->right_delta == 0
                    && fabsf(raw_rate - s_bias) < STILL_GYRO_THRESH_RAD_S
                    && accel_still;
    if (!still) {
        s_stillTicks = 0;
        return;
    }

    if (s_stillTicks < STILL_TICKS) {
        ++s_stillTicks;
        return;
    }

    s_bias += IMU_BIAS_ALPHA * (raw_rate - s_bias);
    imu_set_yaw_bias(s_bias);
    s_valid = true;
}

float heading_get(void) { return s_psi; }
float heading_rate(void) { return s_rate; }
float heading_bias(void) { return s_bias; }
float heading_calibration_stddev(void) { return s_calibrationStddev; }
void heading_zero(void) { s_psi = 0.0f; }
bool heading_valid(void) { return s_valid; }
