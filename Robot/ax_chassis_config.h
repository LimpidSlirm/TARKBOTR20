#ifndef AX_CHASSIS_CONFIG_H
#define AX_CHASSIS_CONFIG_H

/* Competition training manual: MC520P30, magnetic encoder, 65 mm tires.
 * Track and skid factor are document baselines; calibrate on the real car.
 */
#define AX_COMP_WHEEL_DIAMETER_M  0.065
#define AX_COMP_ENCODER_COUNTS   122880.0
#define AX_COMP_TRACK_M          0.200
#define AX_COMP_TRACK_FACTOR     1.75

/* P30 wiring signs adjusted to the observed directions on this chassis. */
#define AX_FWD_ENCODER_SIGN_A  (-1)
#define AX_FWD_ENCODER_SIGN_B  ( 1) /* Forward rotation gives positive raw counts on this chassis. */
#define AX_FWD_ENCODER_SIGN_C  (-1) /* Forward rotation gives negative raw counts on this chassis. */
#define AX_FWD_ENCODER_SIGN_D  ( 1)
#define AX_FWD_MOTOR_SIGN_A    ( 1)
#define AX_FWD_MOTOR_SIGN_B    (-1)
#define AX_FWD_MOTOR_SIGN_C    ( 1)
#define AX_FWD_MOTOR_SIGN_D    (-1)

/* Initial verification limits, not a claim of measured tracking accuracy.
 * FWD speed uses free-running timer deltas over the measured control interval.
 */
#define AX_DEMO_ACCEL_MPS2     0.60f
#define AX_DEMO_DECEL_MPS2     0.90f
#define AX_DEMO_JERK_MPS3      3.00f
#define AX_DEMO_REQUEST_SPEED_MPS 0.095f
#define AX_DEMO_OUTER_WHEEL_MPS   0.115f /* Leave margin below the measured-speed cutoff. */
#define AX_DEMO_GYRO_Z_SIGN    1.0f /* Verify: physical left turn gives +Z. */
#define AX_FWD_MAX_WHEEL_MPS   0.12f /* Maximum requested wheel speed. */
#define AX_FWD_OVERSPEED_MPS    0.25f /* Latch a stop when measured speed exceeds this. */
#define AX_FWD_DRIVE_CUTOFF_MPS AX_FWD_OVERSPEED_MPS /* Cut drive at the threshold. */
#define AX_CONTROL_MAX_GAP_MS  40u

#endif
