#ifndef AX_IMU_BUS_H
#define AX_IMU_BUS_H
#include <stdint.h>
void AX_IMU_BusInit(void);
void AX_IMU_DelayUs(uint32_t us);
uint8_t AX_IMU_BusRead(uint8_t address, uint8_t reg, uint8_t *data, uint8_t size);
uint8_t AX_IMU_BusWrite(uint8_t address, uint8_t reg, uint8_t value);
#endif
