#ifndef AX_TURN_H
#define AX_TURN_H

#include "turn_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring-up limits, not measured chassis performance. Override per project. */
#ifndef AX_TURN_MAX_FORWARD_MPS
#define AX_TURN_MAX_FORWARD_MPS 0.10f
#endif
#ifndef AX_TURN_MAX_YAW_RADPS
#define AX_TURN_MAX_YAW_RADPS 0.60f
#endif
#ifndef AX_TURN_MAX_WHEEL_MPS
#define AX_TURN_MAX_WHEEL_MPS 0.20f
#endif

typedef enum {
    AX_TURN_OK = 0,
    AX_TURN_LIMITED = 1,
    AX_TURN_INVALID_ARGUMENT = -1,
    AX_TURN_BUSY = -2,          /* Manual/other mode: original targets untouched. */
    AX_TURN_DISABLED = -3,      /* Motion disabled: no automatic enabling. */
    AX_TURN_UNSUPPORTED = -4    /* Only existing MEC and FWD models supported. */
} AX_TurnStatus;

/* Call in task context, CTL_ROS mode, before the next kinematics update.
 * Updates R_Vel.TG_IX/IY/IW atomically; does not run PID or write motor PWM.
 * Speed: m/s; strength: [0,100] percent, not degrees. No auto-start or timeout.
 */
AX_TurnStatus AX_TURN_Left(float forward_mps, float strength_percent);
AX_TurnStatus AX_TURN_Right(float forward_mps, float strength_percent);

#ifdef __cplusplus
}
#endif

#endif
