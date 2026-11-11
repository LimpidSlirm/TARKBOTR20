#include "ax_mpu6050.h"
#include "ax_imu_bus.h"

static AX_IMU_Kind kind;
static uint8_t address, ready, acc_range, gyro_range;
static uint8_t qmi_odr = 5u;

static uint8_t write_reg(uint8_t reg, uint8_t value)
{
    if (!ready || !AX_IMU_BusWrite(address, reg, value)) { ready=0u; return 0u; }
    return 1u;
}
static uint8_t read_reg(uint8_t reg, uint8_t *data, uint8_t size)
{
    if (!ready || !AX_IMU_BusRead(address, reg, data, size)) { ready=0u; return 0u; }
    return 1u;
}
static uint8_t probe(uint8_t addr, uint8_t reg, uint8_t expected)
{
    uint8_t id=0u;
    return AX_IMU_BusRead(addr, reg, &id, 1u) && id==expected;
}
static void qmi_config(void)
{
    if (!write_reg(8u,0u)) return;
    if (!write_reg(2u,0x60u)) return; /* auto-increment, sensor little-endian */
    if (!write_reg(3u,(uint8_t)(0x80u | (acc_range<<4) | qmi_odr))) return;
    if (!write_reg(4u,(uint8_t)(0x80u | ((gyro_range+4u)<<4) | qmi_odr))) return;
    if (!write_reg(6u,0x77u)) return;
    (void)write_reg(8u,3u);
}
void AX_MPU6050_Init(void)
{
    uint8_t addr;
    ready=0u; kind=AX_IMU_NONE; acc_range=0u; gyro_range=1u; qmi_odr=5u;
    AX_IMU_BusInit();
    for (addr=0x6au; addr<=0x6bu; ++addr) {
        if (probe(addr,0u,5u)) { address=addr; kind=AX_IMU_QMI8658; break; }
    }
    if (kind==AX_IMU_QMI8658) {
        ready=1u;
        if (!write_reg(96u,0xb0u)) return;
        AX_IMU_DelayUs(150000u);
        if (!probe(address,0u,5u)) { ready=0u; return; }
        qmi_config(); AX_IMU_DelayUs(20000u);
        return;
    }
    for (addr=0x68u; addr<=0x69u; ++addr) {
        if (probe(addr,0x75u,0x68u)) { address=addr; kind=AX_IMU_MPU6050; break; }
    }
    if (kind!=AX_IMU_MPU6050) return;
    ready=1u;
    if (!write_reg(0x6bu,0x80u)) return;
    AX_IMU_DelayUs(100000u);
    if (!write_reg(0x6bu,1u) || !write_reg(0x6cu,0u) ||
        !write_reg(0x1bu,8u) || !write_reg(0x1cu,0u) ||
        !write_reg(0x1au,2u) || !write_reg(0x19u,4u) ||
        !write_reg(0x38u,0u) || !write_reg(0x23u,0u)) return;
    AX_IMU_DelayUs(20000u);
}
void AX_MPU6050_SetAccRange(uint8_t range)
{
    if (range>3u || !ready) return;
    acc_range=range;
    if (kind==AX_IMU_QMI8658) qmi_config();
    else (void)write_reg(0x1cu,(uint8_t)(range<<3));
}
void AX_MPU6050_SetGyroRange(uint8_t range)
{
    if (range>3u || !ready) return;
    gyro_range=range;
    if (kind==AX_IMU_QMI8658) qmi_config();
    else (void)write_reg(0x1bu,(uint8_t)(range<<3));
}
void AX_MPU6050_SetGyroSmplRate(uint16_t rate)
{
    if (!ready || rate==0u) return;
    if (kind==AX_IMU_QMI8658) {
        qmi_odr = rate>250u ? 4u : rate>125u ? 5u : rate>62u ? 6u : 7u;
        qmi_config();
    } else {
        if (rate>1000u) rate=1000u;
        if (rate<4u) rate=4u;
        (void)write_reg(0x19u,(uint8_t)(1000u/rate-1u));
    }
}
void AX_MPU6050_SetDLPF(uint8_t bandwidth)
{
    if (!ready || bandwidth<1u || bandwidth>6u) return;
    if (kind==AX_IMU_MPU6050) (void)write_reg(0x1au,bandwidth);
    /* QMI uses the enabled LPF mode configured in qmi_config, not MPU registers. */
}
static void read_axes(uint8_t reg, int16_t *data, uint8_t gyro)
{
    uint8_t raw[6]={0u};
    unsigned i;
    int32_t value;
    if (!data) return;
    data[0]=data[1]=data[2]=0;
    if (!read_reg(reg,raw,6u)) return;
    for (i=0u;i<3u;++i) {
        if (kind==AX_IMU_QMI8658) {
            value=(int16_t)((uint16_t)raw[2u*i] | ((uint16_t)raw[2u*i+1u]<<8));
            /* Native +/-512 dps -> the existing +/-500 dps protocol, etc. */
            if (gyro) value=value*128/125;
        } else {
            value=(int16_t)(((uint16_t)raw[2u*i]<<8) | raw[2u*i+1u]);
        }
        if (value>32767) value=32767;
        if (value< -32768) value= -32768;
        data[i]=(int16_t)value;
    }
}
void AX_MPU6050_GetAccData(int16_t *data)
{
    read_axes(kind==AX_IMU_QMI8658 ? 53u : 0x3bu,data,0u);
}
void AX_MPU6050_GetGyroData(int16_t *data)
{
    read_axes(kind==AX_IMU_QMI8658 ? 59u : 0x43u,data,1u);
}
float AX_MPU6050_GetTemp(void)
{
    uint8_t raw[2]={0u};
    int16_t value;
    if (!read_reg(kind==AX_IMU_QMI8658 ? 51u : 0x41u,raw,2u)) return 0.0f;
    if (kind==AX_IMU_QMI8658) {
        value=(int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1]<<8));
        return value/256.0f;
    }
    value=(int16_t)(((uint16_t)raw[0]<<8) | raw[1]);
    return 36.53f+value/340.0f;
}
uint8_t AX_MPU6050_IsReady(void) { return ready; }
AX_IMU_Kind AX_MPU6050_GetKind(void) { return kind; }
