#include "ax_turn.h"
#include "ax_robot.h"

/* Defined in ax_robot.c; do not add another definition or force it enabled. */
extern uint8_t ax_robot_move_enable;

#if (ROBOT_TYPE == ROBOT_MEC) || (ROBOT_TYPE == ROBOT_FWD)
static double active_track(void)
{
#if (ROBOT_TYPE == ROBOT_MEC)
    return MEC_WHEEL_BASE + MEC_ACLE_BASE;
#else
    return FWD_WHEEL_BASE * FWD_WB_SCALE;
#endif
}

static AX_TurnStatus apply_turn(float forward_mps, float strength, int left)
{
    TurnConfig config;
    TurnTarget target;
    TurnStatus result;
    AX_TurnStatus status;
    double speed;
    double yaw;
    double bound;
    short vx;
    short vw;

    config.effective_track_m = (float)active_track();
    config.max_forward_mps = AX_TURN_MAX_FORWARD_MPS;
    config.max_yaw_rate_radps = AX_TURN_MAX_YAW_RADPS;
    config.max_wheel_mps = AX_TURN_MAX_WHEEL_MPS;
    if (left) {
        result = Turn_Left(&config, forward_mps, strength, &target);
    } else {
        result = Turn_Right(&config, forward_mps, strength, &target);
    }
    status = (AX_TurnStatus)result;
    speed = (double)target.forward_mps * 1000.0;
    yaw = (double)target.yaw_rate_radps * (left ? 1000.0 : -1000.0);

    /* Clamp before conversion to the signed 16-bit Robot integer interface. */
    bound = R_VX_LIMIT < 32767 ? R_VX_LIMIT : 32767;
    if (speed > bound) {
        speed = bound;
        status = AX_TURN_LIMITED;
    }
    vx = (short)speed;
    bound = R_VW_LIMIT < 32767 ? R_VW_LIMIT : 32767;
    /* Recheck no-reverse after forward-speed quantization to whole mm/s. */
    if (bound > 2.0 * vx / active_track()) {
        bound = 2.0 * vx / active_track();
    }
    if (yaw > bound) {
        yaw = bound;
        status = AX_TURN_LIMITED;
    }
    vw = (short)yaw;
    if (!left) {
        vw = (short)-vw;
    }

    /* Task context only: a receiver interrupt cannot see a half-written target.
     * Other external command sources still need application-level arbitration.
     */
    taskENTER_CRITICAL();
    if (ax_control_mode != CTL_ROS) {
        status = AX_TURN_BUSY;
    } else if (ax_robot_move_enable == 0) {
        R_Vel.TG_IX = 0;
        R_Vel.TG_IY = 0;
        R_Vel.TG_IW = 0;
        status = AX_TURN_DISABLED;
    } else {
        R_Vel.TG_IX = vx;
        R_Vel.TG_IY = 0;
        R_Vel.TG_IW = vw;
    }
    taskEXIT_CRITICAL();
    return status;
}
#endif

AX_TurnStatus AX_TURN_Left(float forward_mps, float strength_percent)
{
#if (ROBOT_TYPE == ROBOT_MEC) || (ROBOT_TYPE == ROBOT_FWD)
    return apply_turn(forward_mps, strength_percent, 1);
#else
    (void)forward_mps;
    (void)strength_percent;
    return AX_TURN_UNSUPPORTED;
#endif
}

AX_TurnStatus AX_TURN_Right(float forward_mps, float strength_percent)
{
#if (ROBOT_TYPE == ROBOT_MEC) || (ROBOT_TYPE == ROBOT_FWD)
    return apply_turn(forward_mps, strength_percent, 0);
#else
    (void)forward_mps;
    (void)strength_percent;
    return AX_TURN_UNSUPPORTED;
#endif
}
