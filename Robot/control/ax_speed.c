#include "ax_speed.h"
#include "ax_robot.h"
#include <float.h>

/* Keep the original incremental controller gains, retain fractional PWM,
 * and clamp before conversion. Stop/reset now clears all four controllers.
 */
typedef struct { float pwm, last_error; } SpeedState;
static SpeedState states[4];
static int16_t speed_step(unsigned wheel, float target, float measured)
{
    SpeedState *state=&states[wheel];
    float error, next;
    if (!(target>=-FLT_MAX && target<=FLT_MAX &&
          measured>=-FLT_MAX && measured<=FLT_MAX)) {
        state->pwm=state->last_error=0.0f; return 0;
    }
#if APP_KEY_TURN_TEST && (ROBOT_TYPE == ROBOT_FWD)
    /* Withhold drive torque at the measured wheel cap; the hard trip is separate. */
    if (measured >= AX_FWD_DRIVE_CUTOFF_MPS || measured <= -AX_FWD_DRIVE_CUTOFF_MPS) {
        state->pwm = 0.0f;
        state->last_error = target - measured;
        return 0;
    }
#endif
    error=target-measured;
    next=state->pwm+ax_motor_kp*error+ax_motor_kd*(error-state->last_error);
#if APP_KEY_TURN_TEST && (ROBOT_TYPE == ROBOT_FWD)
    /* Reduce accumulated drive smoothly above the normal wheel target range.
     * At the drive cutoff the separate zero-output branch still applies.
     */
    {
        const float speed = measured < 0.0f ? -measured : measured;
        if (speed > AX_FWD_MAX_WHEEL_MPS &&
            AX_FWD_DRIVE_CUTOFF_MPS > AX_FWD_MAX_WHEEL_MPS) {
            const float remaining = (AX_FWD_DRIVE_CUTOFF_MPS - speed) /
                                    (AX_FWD_DRIVE_CUTOFF_MPS - AX_FWD_MAX_WHEEL_MPS);
            const float previous = state->pwm < 0.0f ? -state->pwm : state->pwm;
            const float limit = previous * remaining;
            if (next > limit) next = limit;
            if (next < -limit) next = -limit;
        }
    }
    /* A forward-only sequence never commands a wheel to reverse under PID. */
    if ((target >= 0.0f && next < 0.0f) || (target <= 0.0f && next > 0.0f)) next = 0.0f;
#endif
    if (next>4200.0f) next=4200.0f;
    if (next< -4200.0f) next= -4200.0f;
    state->pwm=next; state->last_error=error;
    return (int16_t)next;
}
void AX_SPEED_Reset(void)
{
    unsigned i;
    for (i=0;i<4u;++i) { states[i].pwm=0.0f; states[i].last_error=0.0f; }
}
int16_t AX_SPEED_PidCtlA(float target,float measured) { return speed_step(0u,target,measured); }
int16_t AX_SPEED_PidCtlB(float target,float measured) { return speed_step(1u,target,measured); }
int16_t AX_SPEED_PidCtlC(float target,float measured) { return speed_step(2u,target,measured); }
int16_t AX_SPEED_PidCtlD(float target,float measured) { return speed_step(3u,target,measured); }
