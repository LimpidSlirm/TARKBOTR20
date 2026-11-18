#ifndef __AX_BEEP_H
#define __AX_BEEP_H
#include "stm32f4xx.h"
#include "ax_board_config.h"
#if AX_BOARD_H60_V37
#define AX_BEEP_PIN GPIO_Pin_0
#else
#define AX_BEEP_PIN GPIO_Pin_1
#endif
void AX_BEEP_Init(void);
#define AX_BEEP_Off() GPIO_ResetBits(GPIOE,AX_BEEP_PIN)
#define AX_BEEP_On() GPIO_SetBits(GPIOE,AX_BEEP_PIN)
#define AX_BEEP_Toggle() GPIO_ToggleBits(GPIOE,AX_BEEP_PIN)
#endif
