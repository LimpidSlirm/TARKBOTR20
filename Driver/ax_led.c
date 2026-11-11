#include "ax_led.h"
void AX_LED_Init(void) {
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(AX_LED_RCC,ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin=AX_LED_RED|AX_LED_GREEN;
    gpio.GPIO_Mode=GPIO_Mode_OUT; gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_Speed=GPIO_Speed_50MHz; gpio.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_SetBits(AX_LED_PORT,gpio.GPIO_Pin); GPIO_Init(AX_LED_PORT,&gpio);
}
