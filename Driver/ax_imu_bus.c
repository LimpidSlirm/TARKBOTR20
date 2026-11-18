#include "ax_imu_bus.h"
#include "ax_sys.h"
#include "ax_delay.h"

/* Both documented IMUs use PB6/PB7. Open-drain avoids driving against a slave. */
#define SCL(v) GPIO_WriteBit(GPIOB, GPIO_Pin_6, (v) ? Bit_SET : Bit_RESET)
#define SDA(v) GPIO_WriteBit(GPIOB, GPIO_Pin_7, (v) ? Bit_SET : Bit_RESET)
#define READ_SDA() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)

void AX_IMU_DelayUs(uint32_t us) { AX_Delayus(us); }
static void pause_bus(void) { AX_Delayus(3u); }
static uint8_t clock_high(void)
{
    unsigned i;
    SCL(1);
    for (i = 0; i < 100u; ++i) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)) { pause_bus(); return 1u; }
        AX_Delayus(1u);
    }
    return 0u;
}
static void stop_bus(void)
{
    SCL(0); SDA(0); pause_bus(); (void)clock_high(); SDA(1); pause_bus();
}
static uint8_t start_bus(void)
{
    SDA(1);
    if (!clock_high() || !READ_SDA()) { stop_bus(); return 0u; }
    SDA(0); pause_bus(); SCL(0); return 1u;
}
static uint8_t send_byte(uint8_t value)
{
    unsigned i;
    uint8_t ack;
    for (i = 0; i < 8u; ++i) {
        SDA(value & 0x80u); pause_bus();
        if (!clock_high()) { SCL(0); return 0u; }
        SCL(0); value <<= 1;
    }
    SDA(1); pause_bus();
    if (!clock_high()) { SCL(0); return 0u; }
    ack = (READ_SDA() == 0u); SCL(0); return ack;
}
static uint8_t receive_byte(uint8_t *value, uint8_t ack)
{
    unsigned i;
    uint8_t result = 0u;
    SDA(1);
    for (i = 0; i < 8u; ++i) {
        pause_bus();
        if (!clock_high()) { SCL(0); return 0u; }
        result = (uint8_t)((result << 1) | (READ_SDA() ? 1u : 0u)); SCL(0);
    }
    SDA(!ack); pause_bus();
    if (!clock_high()) { SCL(0); SDA(1); return 0u; }
    SCL(0); SDA(1); *value = result; return 1u;
}
void AX_IMU_BusInit(void)
{
    GPIO_InitTypeDef gpio;
    unsigned i;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    SDA(1);
    for (i = 0; i < 9u; ++i) { SCL(0); pause_bus(); (void)clock_high(); }
    stop_bus();
}
uint8_t AX_IMU_BusWrite(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t ok = start_bus() && send_byte((uint8_t)(address << 1)) &&
                 send_byte(reg) && send_byte(value);
    stop_bus(); return ok;
}
uint8_t AX_IMU_BusRead(uint8_t address, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t i;
    if (!data || size == 0u) { return 0u; }
    if (!start_bus() || !send_byte((uint8_t)(address << 1)) || !send_byte(reg) ||
        !start_bus() || !send_byte((uint8_t)((address << 1) | 1u))) {
        stop_bus(); return 0u;
    }
    for (i = 0u; i < size; ++i) {
        if (!receive_byte(&data[i], i + 1u < size)) { stop_bus(); return 0u; }
    }
    stop_bus(); return 1u;
}
