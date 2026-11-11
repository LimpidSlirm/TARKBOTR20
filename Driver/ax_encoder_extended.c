#include "ax_encoder_extended.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

/* TIM2/TIM5 are native 32-bit; extend TIM3/TIM4 with update interrupts.
 * The original uint16_t API truncated every channel, including TIM2/TIM5.
 */
static volatile int32_t wraps[2];
static TIM_TypeDef *timer_for(unsigned channel)
{
    switch(channel) {case 0u:return TIM2;case 1u:return TIM3;
                    case 2u:return TIM5;default:return TIM4;}
}
static unsigned slot_for(unsigned channel) {return channel==1u ? 0u : 1u;}
static void update_wrap(TIM_TypeDef *timer,unsigned slot)
{
    if (TIM_GetITStatus(timer,TIM_IT_Update)!=RESET) {
        wraps[slot] += timer->CNT<0x8000u ? 65536 : -65536;
        TIM_ClearITPendingBit(timer,TIM_IT_Update);
    }
}
void TIM3_IRQHandler(void) {update_wrap(TIM3,0u);}
void TIM4_IRQHandler(void) {update_wrap(TIM4,1u);}

void AX_ENCODER_ExtendedInit(unsigned channel)
{
    TIM_TypeDef *timer=timer_for(channel);
    NVIC_InitTypeDef irq;
    if(channel==0u || channel==2u) {
        TIM_SetAutoreload(timer,0xffffffffu);
    } else {
        wraps[slot_for(channel)]=0;
        irq.NVIC_IRQChannel=channel==1u ? TIM3_IRQn : TIM4_IRQn;
        irq.NVIC_IRQChannelPreemptionPriority=1u;
        irq.NVIC_IRQChannelSubPriority=1u;
        irq.NVIC_IRQChannelCmd=ENABLE;
        NVIC_Init(&irq);
        TIM_ClearITPendingBit(timer,TIM_IT_Update);
        TIM_ITConfig(timer,TIM_IT_Update,ENABLE);
    }
    AX_ENCODER_SetExtended(channel,0u);
}
int32_t AX_ENCODER_ReadExtended(unsigned channel)
{
    TIM_TypeDef *timer=timer_for(channel);
    uint32_t before,after,low;
    int32_t high,result;
    if(channel==0u || channel==2u) return (int32_t)timer->CNT;
    taskENTER_CRITICAL();
    high=wraps[slot_for(channel)]; before=timer->SR;
    low=timer->CNT & 0xffffu; after=timer->SR;
    /* Re-read if overflow occurred between the first flag read and CNT read. */
    if(!(before & TIM_IT_Update) && (after & TIM_IT_Update)) low=timer->CNT & 0xffffu;
    if(after & TIM_IT_Update) high += low<0x8000u ? 65536 : -65536;
    result=high+(int32_t)low;
    taskEXIT_CRITICAL(); return result;
}
void AX_ENCODER_SetExtended(unsigned channel,uint16_t value)
{
    TIM_TypeDef *timer=timer_for(channel);
    taskENTER_CRITICAL();
    if(channel==1u || channel==3u) wraps[slot_for(channel)]=0;
    timer->CNT=value;
    TIM_ClearITPendingBit(timer,TIM_IT_Update);
    taskEXIT_CRITICAL();
}
