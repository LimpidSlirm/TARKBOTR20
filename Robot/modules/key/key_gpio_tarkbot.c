#include "main.h"
#include "ax_board_config.h"
#include "key_gpio_tarkbot.h"

#if AX_BOARD_H60_V37
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define H60_KEY_A_PIN GPIO_Pin_1
#define H60_KEY_B_PIN GPIO_Pin_4

void key_gpio_tarkbot_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = H60_KEY_A_PIN | H60_KEY_B_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &gpio);
}

uint8_t key_gpio_tarkbot_both_released(void)
{
    return GPIO_ReadInputDataBit(GPIOE, H60_KEY_A_PIN) != 0u &&
           GPIO_ReadInputDataBit(GPIOE, H60_KEY_B_PIN) != 0u;
}

uint8_t gpio_get_level(uint8_t pin)
{
    /* 通用模块的通道 0 和 1 分别对应 H60 V3.7 的 KA 和 KB。 */
    switch (pin) {
    case 0u:
        return GPIO_ReadInputDataBit(GPIOE, H60_KEY_A_PIN);
    case 1u:
        return GPIO_ReadInputDataBit(GPIOE, H60_KEY_B_PIN);
    default:
        return 1u;
    }
}
#else
#include "ax_key.h"
void key_gpio_tarkbot_init(void) { AX_KEY_Init(); }
uint8_t key_gpio_tarkbot_both_released(void) { return AX_KEY_Scan()==0u; }

uint8_t gpio_get_level(uint8_t pin)
{
    if (pin == 0u && AX_KEY_Scan() != 0u) {
        return 0u;
    }
    return 1u;
}
#endif
