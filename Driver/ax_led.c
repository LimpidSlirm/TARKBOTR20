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
  * @说  明
  *
  * 
  ******************************************************************************
  */

#include "ax_led.h"
#include "ax_delay.h"

/**
  * @简  述  LED 初始化
  * @参  数  无
  * @返回值  无
  */
void AX_LED_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIO配置 */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE); //启动GPIO时钟 
	
	//配置红色LED指示灯GPIO  推挽输出模式 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD,  GPIO_Pin_3);	

	//配置绿色LED指示灯GPIO  推挽输出模式  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD,  GPIO_Pin_4);	
}

/**
  * @简  述  LED红绿交替闪烁
  * @参  数  times：闪烁次数
  * @返回值  无
  */
void AX_LED_Blink(uint8_t times)
{
	uint8_t i;

	for(i = 0; i < times; i++)
	{
		//红灯亮，绿灯灭
		AX_LED_Red_On();
		AX_LED_Green_Off();
		AX_Delayms(150);

		//红灯灭，绿灯亮
		AX_LED_Red_Off();
		AX_LED_Green_On();
		AX_Delayms(150);
	}

	//结束后恢复默认状态：红灯灭，绿灯常亮
	AX_LED_Red_Off();
	AX_LED_Green_On();
}

/**
  * @简  述  LED测试序列（非阻塞，每调用一次切换一相）
  * @参  数  无
  * @返回值  无
  */
void AX_LED_Test(void)
{
	static uint8_t step = 0;

	switch(step)
	{
		case 0:  AX_LED_Red_On();  AX_LED_Green_Off(); break;   //仅红灯亮
		case 1:  AX_LED_Red_Off(); AX_LED_Green_On();  break;   //仅绿灯亮
		case 2:  AX_LED_Red_On();  AX_LED_Green_On();  break;   //红绿同时亮
		default: AX_LED_Red_Off(); AX_LED_Green_Off(); break;   //全灭
	}

	if(++step > 3) step = 0;
}

/******************* (C) 版权 2023 XTARK **************************************/
