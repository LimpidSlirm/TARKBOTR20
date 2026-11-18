#ifndef HOST_PLATFORM_H
#define HOST_PLATFORM_H
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef uint32_t portTickType;
typedef void *TaskHandle_t;
typedef struct { unsigned unused; } JOYSTICK_TypeDef;
#define configTICK_RATE_HZ 1000u
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *previous, TickType_t increment);
void ROBOT_IMUHandle(void);
void ROBOT_SendDataToRos(void);
int32_t AX_ENCODER_A_GetCounter(void);
int32_t AX_ENCODER_B_GetCounter(void);
int32_t AX_ENCODER_C_GetCounter(void);
int32_t AX_ENCODER_D_GetCounter(void);
void AX_ENCODER_A_SetCounter(uint16_t v);
void AX_ENCODER_B_SetCounter(uint16_t v);
void AX_ENCODER_C_SetCounter(uint16_t v);
void AX_ENCODER_D_SetCounter(uint16_t v);
void AX_MOTOR_A_SetSpeed(int16_t v);
void AX_MOTOR_B_SetSpeed(int16_t v);
void AX_MOTOR_C_SetSpeed(int16_t v);
void AX_MOTOR_D_SetSpeed(int16_t v);
typedef struct { volatile uint32_t CNT,SR,ARR; } TIM_TypeDef;
extern TIM_TypeDef host_timers[4];
#define TIM2 (&host_timers[0])
#define TIM3 (&host_timers[1])
#define TIM5 (&host_timers[2])
#define TIM4 (&host_timers[3])
#define TIM_IT_Update 1u
#define RESET 0u
#define ENABLE 1u
#define TIM3_IRQn 3u
#define TIM4_IRQn 4u
typedef struct { unsigned NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,
    NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd; } NVIC_InitTypeDef;
unsigned TIM_GetITStatus(TIM_TypeDef *timer,unsigned interrupt);
void TIM_ClearITPendingBit(TIM_TypeDef *timer,unsigned interrupt);
void TIM_SetAutoreload(TIM_TypeDef *timer,uint32_t value);
void TIM_ITConfig(TIM_TypeDef *timer,unsigned interrupt,unsigned enabled);
void NVIC_Init(NVIC_InitTypeDef *irq);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
#endif
