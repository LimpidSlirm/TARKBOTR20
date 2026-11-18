#ifndef AX_MOTION_SEQUENCE_H
#define AX_MOTION_SEQUENCE_H

#include <stdint.h>

typedef enum {
    AX_MOTION_IDLE = 0,
    AX_MOTION_ACCEL,
    AX_MOTION_TURN,
    AX_MOTION_DECEL,
    AX_MOTION_FAULT
} AX_MotionPhase;

typedef struct {
    AX_MotionPhase phase;
    float speed_mps;
    float yaw_radps;       /* ROS: positive = left. */
    float angle_rad;       /* Integrated gyro since turn started. */
    uint8_t fault;         /* 5: input/config; 6: acceleration timeout;
                           * 7: turn timeout; 8: stopping timeout. */
} AX_MotionOutput;

/* Limits are supplied by main from the unmodified chassis/turn caps. */
void AX_MOTION_Init(float speed_limit_mps, float yaw_limit_radps);
/* One owner, once per control cycle. request: +1=A/left, -1=B/right, 0=none.
 * Busy requests are consumed. Inputs are measured body speed and gyro Z.
 * enabled=0 cancels immediately; a new request is needed after re-enabling.
 */
const AX_MotionOutput *AX_MOTION_Step(int request, float speed_mps,
                                    float gyro_radps, float dt, uint8_t enabled);

#endif
