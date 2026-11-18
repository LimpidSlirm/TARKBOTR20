#ifndef AX_MPU6050_COMPAT_H
#define AX_MPU6050_COMPAT_H
#include <stdint.h>

/* Compatibility API: QMI8658 and MPU6050 are identified by their chip IDs.
 * Output ranges keep the existing host protocol: acc range and 250/500/... dps.
 */
#define AX_ACC_RANGE_2G 0u
#define AX_ACC_RANGE_4G 1u
#define AX_ACC_RANGE_8G 2u
#define AX_ACC_RANGE_16G 3u
#define AX_GYRO_RANGE_250 0u
#define AX_GYRO_RANGE_500 1u
#define AX_GYRO_RANGE_1000 2u
#define AX_GYRO_RANGE_2000 3u
#define AX_DLPF_ACC184_GYRO188 1u
#define AX_DLPF_ACC94_GYRO98 2u
#define AX_DLPF_ACC44_GYRO42 3u
#define AX_DLPF_ACC21_GYRO20 4u
#define AX_DLPF_ACC10_GYRO10 5u
#define AX_DLPF_ACC5_GYRO5 6u
typedef enum { AX_IMU_NONE=0, AX_IMU_MPU6050=1, AX_IMU_QMI8658=2 } AX_IMU_Kind;
void AX_MPU6050_Init(void);
void AX_MPU6050_SetAccRange(uint8_t range);
void AX_MPU6050_SetGyroRange(uint8_t range);
void AX_MPU6050_SetGyroSmplRate(uint16_t rate);
void AX_MPU6050_SetDLPF(uint8_t bandwidth);
float AX_MPU6050_GetTemp(void);
void AX_MPU6050_GetAccData(int16_t *data);
void AX_MPU6050_GetGyroData(int16_t *data);
uint8_t AX_MPU6050_IsReady(void);
AX_IMU_Kind AX_MPU6050_GetKind(void);
#endif
