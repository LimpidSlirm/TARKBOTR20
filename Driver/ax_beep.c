#include "ax_beep.h"
void AX_BEEP_Init(void) {
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    GPIO_StructInit(&gpio); gpio.GPIO_Pin=AX_BEEP_PIN;
    gpio.GPIO_Mode=GPIO_Mode_OUT; gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_Speed=GPIO_Speed_50MHz; gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
    AX_BEEP_Off(); GPIO_Init(GPIOE,&gpio);
}
