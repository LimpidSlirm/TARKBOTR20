#include "turn_control.h"
#include <float.h>
#include <stddef.h>

/* Portable to C90/Keil AC5; reject both NaN and infinities without libm. */
static int finite_float(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

static int positive_finite(float value)
{
    return finite_float(value) && value > 0.0f;
}

static TurnStatus make_target(const TurnConfig *config, float forward_mps,
                              float strength_percent, int direction,
                              TurnTarget *target)
{
    double speed;
    double yaw;
    double no_reverse_limit;
    double outer_wheel;
    double scale;
    TurnStatus status;

    if (target == NULL) {
        return TURN_INVALID_ARGUMENT;
    }
    target->forward_mps = 0.0f;
    target->yaw_rate_radps = 0.0f;
    if (config == NULL ||
        !positive_finite(config->effective_track_m) ||
        !positive_finite(config->max_forward_mps) ||
        !positive_finite(config->max_yaw_rate_radps) ||
        !positive_finite(config->max_wheel_mps) ||
        !finite_float(forward_mps) || forward_mps < 0.0f ||
        !finite_float(strength_percent) ||
        strength_percent < 0.0f || strength_percent > 100.0f) {
        return TURN_INVALID_ARGUMENT;
    }

    status = TURN_OK;
    speed = forward_mps;
    if (speed > config->max_forward_mps) {
        speed = config->max_forward_mps;
        status = TURN_LIMITED;
    }
    /* Double intermediates avoid overflow for finite float inputs. */
    yaw = (double)config->max_yaw_rate_radps * strength_percent / 100.0;
    no_reverse_limit = 2.0 * speed / config->effective_track_m;
    if (yaw > no_reverse_limit) {
        yaw = no_reverse_limit;
        status = TURN_LIMITED;
    }
    outer_wheel = speed + yaw * config->effective_track_m / 2.0;
    if (outer_wheel > config->max_wheel_mps) {
        /* Scale both sides equally, preserving the requested turn radius. */
        scale = config->max_wheel_mps / outer_wheel;
        speed *= scale;
        yaw *= scale;
        status = TURN_LIMITED;
    }

    target->forward_mps = (float)speed;
    target->yaw_rate_radps = (float)(direction * yaw);
    return status;
}

TurnStatus Turn_Left(const TurnConfig *config, float forward_mps,
                     float strength_percent, TurnTarget *target)
{
    return make_target(config, forward_mps, strength_percent, 1, target);
}

TurnStatus Turn_Right(const TurnConfig *config, float forward_mps,
                      float strength_percent, TurnTarget *target)
{
    return make_target(config, forward_mps, strength_percent, -1, target);
}
