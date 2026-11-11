#ifndef __AX_LED_H
#define __AX_LED_H
#include "stm32f4xx.h"
#include "ax_board_config.h"
#if AX_BOARD_H60_V37
#define AX_LED_PORT GPIOB
#define AX_LED_RCC RCC_AHB1Periph_GPIOB
#define AX_LED_RED GPIO_Pin_0
#define AX_LED_GREEN GPIO_Pin_1
#else
#define AX_LED_PORT GPIOD
#define AX_LED_RCC RCC_AHB1Periph_GPIOD
#define AX_LED_RED GPIO_Pin_3
#define AX_LED_GREEN GPIO_Pin_4
#endif
void AX_LED_Init(void);
#define AX_LED_Red_Off() GPIO_SetBits(AX_LED_PORT,AX_LED_RED)
#define AX_LED_Red_On() GPIO_ResetBits(AX_LED_PORT,AX_LED_RED)
#define AX_LED_Red_Toggle() GPIO_ToggleBits(AX_LED_PORT,AX_LED_RED)
#define AX_LED_Green_Off() GPIO_SetBits(AX_LED_PORT,AX_LED_GREEN)
#define AX_LED_Green_On() GPIO_ResetBits(AX_LED_PORT,AX_LED_GREEN)
#define AX_LED_Green_Toggle() GPIO_ToggleBits(AX_LED_PORT,AX_LED_GREEN)
#endif
