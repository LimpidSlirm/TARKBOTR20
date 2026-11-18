#ifndef AX_BOARD_CONFIG_H
#define AX_BOARD_CONFIG_H

/* 1: H60 V3.7 (PE1/PE4 keys, PB0/PB1 LEDs, PE0 buzzer).
 * 0: legacy H60 V3.2 (PE0 key, PD3/PD4 LEDs, PE1 buzzer).
 * Mechanical parameters come from the competition training manual.
 * IMU type is detected from its chip ID, not inferred from either manual.
 */
#ifndef AX_BOARD_H60_V37
#define AX_BOARD_H60_V37 1
#endif

#endif
