#ifndef AX_CHASSIS_CONFIG_H
#define AX_CHASSIS_CONFIG_H

/* Competition training manual: MC520P30, magnetic encoder, 65 mm tires.
 * Track and skid factor are document baselines; calibrate on the real car.
 */
#define AX_COMP_WHEEL_DIAMETER_M  0.065
#define AX_COMP_ENCODER_COUNTS   122880.0
#define AX_COMP_TRACK_M          0.200
#define AX_COMP_TRACK_FACTOR     1.75

/* Training manual section 9.2, P30 wiring. Separate feedback and PWM signs. */
#define AX_FWD_ENCODER_SIGN_A  ( 1)
#define AX_FWD_ENCODER_SIGN_B  (-1)
#define AX_FWD_ENCODER_SIGN_C  ( 1)
#define AX_FWD_ENCODER_SIGN_D  (-1)
#define AX_FWD_MOTOR_SIGN_A    (-1)
#define AX_FWD_MOTOR_SIGN_B    (-1)
#define AX_FWD_MOTOR_SIGN_C    ( 1)
#define AX_FWD_MOTOR_SIGN_D    ( 1)

/* Initial verification limits, not a claim of measured tracking accuracy.
 * Encoder feedback uses 32-bit reads, including TIM3/TIM4 rollover extension.
 */
#define AX_DEMO_ACCEL_MPS2     0.60f
#define AX_DEMO_DECEL_MPS2     0.90f
#define AX_DEMO_JERK_MPS3      3.00f
#define AX_DEMO_REQUEST_SPEED_MPS 0.50f /* Clamped to existing source limits. */
#define AX_DEMO_GYRO_Z_SIGN    1.0f /* Verify: physical left turn gives +Z. */
#define AX_FWD_MAX_WHEEL_MPS   0.12f
#define AX_CONTROL_MAX_GAP_MS  40u

#endif
