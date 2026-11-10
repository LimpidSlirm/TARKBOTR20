/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @内  容  LED灯控制
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_LED_H
#define __AX_LED_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//接口函数
void AX_LED_Init(void);
void AX_LED_Blink(uint8_t times);   //LED红绿交替闪烁，用于开机自检/状态提示
void AX_LED_Test(void);             //LED测试序列，非阻塞，每调用一次切换一相

#define AX_LED_Red_Off()        GPIO_SetBits(GPIOD, GPIO_Pin_3)      //LEDR红色熄灭
#define AX_LED_Red_On()  	    GPIO_ResetBits(GPIOD, GPIO_Pin_3)    //LEDR红色点亮
#define AX_LED_Red_Toggle()     GPIO_WriteBit(GPIOD, GPIO_Pin_3, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)))	//LEDR红色状态翻转

#define AX_LED_Green_Off()  	GPIO_SetBits(GPIOD, GPIO_Pin_4)      //LEDG绿色熄灭
#define AX_LED_Green_On()		GPIO_ResetBits(GPIOD, GPIO_Pin_4)    //LEDG绿色点亮
#define AX_LED_Green_Toggle()   GPIO_WriteBit(GPIOD, GPIO_Pin_4, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)))	//LEDG绿色状态翻转


#endif 

/******************* (C) 版权 2023 XTARK **************************************/
