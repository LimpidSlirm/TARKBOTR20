#include "ax_motion_sequence.h"
#include "ax_chassis_config.h"
#include "ax_ramp.h"
#include <math.h>
#include <float.h>

#define QUARTER_TURN_RAD 1.570796327f
#define ANGLE_TOL_RAD    0.017453293f
#define GYRO_STILL_RADPS 0.052359878f

static AX_MotionOutput output;
static float phase_time, settled_time;
static float cruise_speed, yaw_limit, turn_timeout;
static int direction;

static int finite_value(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }

static void enter_phase(AX_MotionPhase phase)
{
    output.phase = phase;
    phase_time = settled_time = 0.0f;
}

static void fail(uint8_t code)
{
    AX_RAMP_EmergencyStop();
    output.fault = code;
    output.speed_mps = output.yaw_radps = 0.0f;
    enter_phase(AX_MOTION_FAULT);
}

void AX_MOTION_Init(float speed_limit_mps, float yaw_limit_radps)
{
    output.angle_rad = 0.0f;
    output.speed_mps = output.yaw_radps = 0.0f;
    output.fault = 0u;
    direction = 1;
    enter_phase(AX_MOTION_IDLE);
    AX_RAMP_Init();
    if (!finite_value(speed_limit_mps) || speed_limit_mps <= 0.0f ||
        !finite_value(yaw_limit_radps) || yaw_limit_radps < 0.01f) {
        fail(5u);
        return;
    }
    cruise_speed = speed_limit_mps;
    yaw_limit = yaw_limit_radps;
    turn_timeout = QUARTER_TURN_RAD / yaw_limit + 10.0f;
    if (AX_RAMP_Config(cruise_speed, AX_DEMO_ACCEL_MPS2,
                       AX_DEMO_DECEL_MPS2, AX_DEMO_JERK_MPS3) != AX_RAMP_OK) {
        fail(5u);
    }
}

const AX_MotionOutput *AX_MOTION_Step(int request, float speed_mps,
                                    float gyro_radps, float dt, uint8_t enabled)
{
    float error, yaw;
    if (output.phase == AX_MOTION_FAULT) return &output;
    if (!enabled) {
        AX_RAMP_EmergencyStop();
        output.speed_mps = output.yaw_radps = 0.0f;
        enter_phase(AX_MOTION_IDLE);
        return &output;
    }
    if (!finite_value(dt) || dt <= 0.0f || dt > 0.04001f ||
        !finite_value(speed_mps) || !finite_value(gyro_radps)) {
        fail(5u);
        return &output;
    }
    if (output.phase == AX_MOTION_IDLE) {
        output.speed_mps = output.yaw_radps = 0.0f;
        /* No queued restart while rolling after an interrupted action. */
        if ((request == 1 || request == -1) && fabs(speed_mps) < 0.03f &&
            fabs(gyro_radps) < GYRO_STILL_RADPS) {
            direction = request;
            output.angle_rad = 0.0f;
            AX_RAMP_EmergencyStop();
            AX_RAMP_To(cruise_speed);
            enter_phase(AX_MOTION_ACCEL);
        } else {
            return &output;
        }
    }

    phase_time += dt;
    output.yaw_radps = 0.0f;
    if (output.phase == AX_MOTION_TURN || output.phase == AX_MOTION_DECEL) {
        output.angle_rad += gyro_radps * dt;
    }
    AX_RAMP_Update(dt);
    output.speed_mps = AX_RAMP_GetSpeed();

    switch (output.phase) {
    case AX_MOTION_ACCEL:
        if (AX_RAMP_IsSettled() &&
            fabs(speed_mps - cruise_speed) <= 0.01f) {
            settled_time += dt;
            if (settled_time >= 0.20f) {
                output.angle_rad = 0.0f;
                enter_phase(AX_MOTION_TURN);
            }
        } else settled_time = 0.0f;
        if (phase_time > 5.0f) fail(6u);
        break;
    case AX_MOTION_TURN:
        error = direction * QUARTER_TURN_RAD - output.angle_rad;
        yaw = 2.0f * error;
        if (yaw > yaw_limit) yaw = yaw_limit;
        if (yaw < -yaw_limit) yaw = -yaw_limit;
        if (fabs(error) <= ANGLE_TOL_RAD) {
            yaw = 0.0f;
            if (fabs(gyro_radps) < GYRO_STILL_RADPS) settled_time += dt;
            else settled_time = 0.0f;
            if (settled_time >= 0.20f) {
                AX_RAMP_Stop();
                enter_phase(AX_MOTION_DECEL);
            }
        } else settled_time = 0.0f;
        output.yaw_radps = yaw;
        if (phase_time > turn_timeout) fail(7u);
        break;
    case AX_MOTION_DECEL:
        /* Straight braking in the new heading, with no retained turn command. */
        if (AX_RAMP_IsStopped() && fabs(speed_mps) < 0.03f &&
            fabs(gyro_radps) < GYRO_STILL_RADPS) {
            settled_time += dt;
            if (settled_time >= 0.20f) {
                output.speed_mps = 0.0f;
                enter_phase(AX_MOTION_IDLE);
            }
        } else settled_time = 0.0f;
        if (phase_time > 5.0f) fail(8u);
        break;
    default: break;
    }
    return &output;
}
