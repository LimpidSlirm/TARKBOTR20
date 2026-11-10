#include "ax_key.h"

uint8_t gpio_get_level(uint8_t pin)
{
    /* 通道 0 对应 PE0，原驱动以 1 表示按下。 */
    if (pin == 0u && AX_KEY_Scan() != 0u) {
        return 0u;
    }

    /* 其余通道暂未接线，返回松开电平。 */
    return 1u;
}
