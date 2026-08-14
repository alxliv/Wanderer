#include <math.h>
#include <stdio.h>

#include "angle.h"
#include "heading.h"

static int failures;
#define CHECK(condition, message) do { \
    if (!(condition)) { printf("FAIL line %d: %s\n", __LINE__, message); ++failures; } \
} while (0)

static bool s_calibrationSucceeds = true;
static float s_calibrationMean;
static float s_calibrationStddev;
static float s_driverBias;

extern "C" void imu_set_yaw_bias(float bias_rad_s) { s_driverBias = bias_rad_s; }
extern "C" float imu_yaw_bias(void) { return s_driverBias; }
extern "C" bool imu_calibrate_bias(float *mean_rad_s, float *stddev_rad_s)
{
    if (!s_calibrationSucceeds)
        return false;
    *mean_rad_s = s_calibrationMean;
    *stddev_rad_s = s_calibrationStddev;
    return true;
}

static imu_sample_t sample(float rate, uint64_t t_us)
{
    imu_sample_t s = {rate, 9.80665f, t_us, true};
    return s;
}

static encoder_sample_t still_encoders(void)
{
    encoder_sample_t e = {};
    return e;
}

static void reset_heading(bool calibrated, float bias)
{
    s_calibrationSucceeds = calibrated;
    s_calibrationMean = bias;
    s_calibrationStddev = 0.01f;
    s_driverBias = 0.0f;
    heading_init();
}

static void test_wrap_pi(void)
{
    const float pi = 3.14159265358979323846f;
    CHECK(fabsf(wrap_pi(-pi) - pi) < 1e-6f, "-pi wraps to +pi");
    CHECK(fabsf(wrap_pi(3.0f * pi) - pi) < 1e-6f, "+3pi wraps to +pi");
    CHECK(fabsf(wrap_pi(-3.0f * pi) - pi) < 1e-6f, "-3pi wraps to +pi");
    CHECK(fabsf(wrap_pi(0.25f) - 0.25f) < 1e-6f, "interior angle unchanged");
}

static void test_constant_rate_integration(void)
{
    reset_heading(true, 0.0f);
    encoder_sample_t e = still_encoders();
    for (unsigned i = 0; i <= 100; ++i)
        heading_update(&sample(1.0f, UINT64_C(1000000) + i * 10000u), &e, false);
    CHECK(fabsf(heading_get() - 1.0f) < 0.001f,
          "constant rate integrates using measured timestamps");
}

static void test_bias_convergence_and_gates(void)
{
    const float true_bias = 3.14159265358979323846f / 180.0f;
    reset_heading(true, true_bias * 0.8f);
    encoder_sample_t e = still_encoders();
    for (unsigned i = 0; i < 1100; ++i) {
        // The driver removes its current bias before exposing the sample.
        imu_sample_t s = sample(true_bias - s_driverBias,
                                UINT64_C(2000000) + i * 10000u);
        heading_update(&s, &e, false);
    }
    CHECK(heading_valid(), "boot calibration produces a valid bias");
    CHECK(fabsf(heading_bias() - true_bias) < 0.0009f,
          "stillness re-zero converges a one degree/s bias");

    const float held_bias = heading_bias();
    imu_sample_t s = sample(true_bias - s_driverBias, UINT64_C(14000000));
    heading_update(&s, &e, true);
    CHECK(fabsf(heading_bias() - held_bias) < 1e-7f,
          "bias is frozen during a procedure");

    e.left_delta = 1;
    s.t_us += 10000;
    heading_update(&s, &e, false);
    CHECK(fabsf(heading_bias() - held_bias) < 1e-7f,
          "encoder motion blocks bias updates");
}

int main(void)
{
    test_wrap_pi();
    test_constant_rate_integration();
    test_bias_convergence_and_gates();
    if (failures == 0)
        printf("test_heading: all checks passed\n");
    return failures != 0;
}
