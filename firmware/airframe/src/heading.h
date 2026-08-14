#ifndef WANDERER_HEADING_H
#define WANDERER_HEADING_H

#include <stdbool.h>

#include "encoders.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

void  heading_init(void);
void  heading_update(const imu_sample_t *sample, const encoder_sample_t *encoders,
                     bool procedure_active);
float heading_get(void);        /* rad, right-turn-positive, wrapped (-pi, pi] */
float heading_rate(void);       /* rad/s, bias-corrected */
float heading_bias(void);       /* rad/s, current estimate */
float heading_calibration_stddev(void); /* rad/s, latest static calibration */
void  heading_zero(void);       /* make the current heading the new zero */
bool  heading_valid(void);      /* false until a bias estimate lands */

/* Static 512-sample bias calibration, for boot and `imu cal`. */
bool heading_calibrate(float *mean_rad_s, float *stddev_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* WANDERER_HEADING_H */
