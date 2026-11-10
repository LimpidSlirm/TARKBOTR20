#ifndef TURN_CONTROL_H
#define TURN_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* All limits must be finite and strictly positive. No hardware defaults. */
typedef struct {
    float effective_track_m;       /* Calibrated effective left/right track. */
    float max_forward_mps;
    float max_yaw_rate_radps;
    float max_wheel_mps;
} TurnConfig;

typedef struct {
    float forward_mps;             /* Forward >= 0, unit: m/s. */
    float yaw_rate_radps;          /* Left positive, right negative, rad/s. */
} TurnTarget;

typedef enum {
    TURN_OK = 0,
    TURN_LIMITED = 1,              /* Usable target; one or more limits hit. */
    TURN_INVALID_ARGUMENT = -1     /* Non-null output is cleared to zero. */
} TurnStatus;

/* strength_percent: [0,100], relative to max_yaw_rate_radps, not degrees.
 * Zero strength means straight; zero speed means stopped (no pivot turn).
 * Nonblocking, stateless calculation only: no PID, PWM or hardware writes.
 * Call from the chosen control owner and apply the returned target there.
 * Input config and output target must be distinct objects.
 */
TurnStatus Turn_Left(const TurnConfig *config, float forward_mps,
                     float strength_percent, TurnTarget *target);
TurnStatus Turn_Right(const TurnConfig *config, float forward_mps,
                      float strength_percent, TurnTarget *target);

#ifdef __cplusplus
}
#endif

#endif
