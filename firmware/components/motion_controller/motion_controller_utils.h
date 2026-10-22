#ifndef MOTION_CONTROLLER_UTILS_H
#define MOTION_CONTROLLER_UTILS_H

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdint.h>
#include <math.h>

#include "motion_controller.h"

#define MOTION_CTRL_FULL_ROTATION_DEGREES (360)

/*
 * Normalize an angle to one full revolution range.
 *
 * @param angle The angle to normalize.
 * @return The normalized angle in the range [0, 360) degrees.
 */
static float motion_ctrl_utils_norm_360(const float angle)
{
    float norm = fmodf(angle, MOTION_CTRL_FULL_ROTATION_DEGREES);
    return (norm < 0.0) ? norm + MOTION_CTRL_FULL_ROTATION_DEGREES : norm;
}

/*
 * Clamp a value between a minimum and maximum.
 *
 * @param value The value to clamp.
 * @param min The minimum value.
 * @param max The maximum value.
 */
static void motion_ctrl_utils_clamp(float* value, const float min, const float max)
{
    *value = fmaxf(min, fminf(*value, max));
}

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_UTILS_H */