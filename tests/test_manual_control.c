#include "host_platform_manual.h"
#include "ax_robot.h"
#include "ax_speed.h"
#include "ax_turn.h"
#include "ax_kinematics.h"
#include "ax_mpu6050.h"
#include "ax_encoder_extended.h"
#include "ax_motion_sequence.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

ROBOT_Velocity R_Vel;
ROBOT_Imu R_Imu;
ROBOT_Wheel R_Wheel_A,R_Wheel_B,R_Wheel_C,R_Wheel_D;
int16_t ax_motor_kp=800,ax_motor_kd=1000;
uint8_t ax_robot_move_enable=1u,ax_control_mode=CTL_ROS;
volatile uint8_t ax_motion_fault;
volatile uint8_t ax_feedback_fault_mask;
float ax_control_dt_s=0.02f;
TIM_TypeDef host_timers[4];
static int16_t pwm[4];
static TickType_t ticks;
static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); return 1; } } while(0)
#define IO(w,i) int32_t AX_ENCODER_##w##_GetCounter(void){return AX_ENCODER_ReadExtended(i);} \
void AX_ENCODER_##w##_SetCounter(uint16_t v){AX_ENCODER_SetExtended(i,v);} \
void AX_MOTOR_##w##_SetSpeed(int16_t v){pwm[i]=v;}
IO(A,0) IO(B,1) IO(C,2) IO(D,3)
unsigned TIM_GetITStatus(TIM_TypeDef *timer,unsigned interrupt){return timer->SR & interrupt;}
void TIM_ClearITPendingBit(TIM_TypeDef *timer,unsigned interrupt){timer->SR &= ~interrupt;}
void TIM_SetAutoreload(TIM_TypeDef *timer,uint32_t value){timer->ARR=value;}
void TIM_ITConfig(TIM_TypeDef *timer,unsigned interrupt,unsigned enabled){(void)timer;(void)interrupt;(void)enabled;}
void NVIC_Init(NVIC_InitTypeDef *irq){(void)irq;}
static void feed(int32_t a,int32_t b,int32_t c,int32_t d)
{
    int32_t values[4];unsigned i;
    values[0]=a;values[1]=b;values[2]=c;values[3]=d;
    for(i=0u;i<4u;++i) {
        uint32_t next=host_timers[i].CNT+(uint32_t)values[i];
        host_timers[i].CNT=(i==1u || i==3u) ? (uint16_t)next : next;
    }
}
TickType_t xTaskGetTickCount(void){return ticks;}
static jmp_buf finish_task;
static unsigned cycle;
static uint32_t gaps[2];
void vTaskDelayUntil(TickType_t *previous,TickType_t increment)
{
    (void)increment;
    if(cycle>=2u) longjmp(finish_task,1);
    ticks+=gaps[cycle++]; *previous=ticks;
}
void ROBOT_SendDataToRos(void){}
void AX_CTL_Ps2(void){}
void AX_CTL_App(void){}
void AX_CTL_RemoteSbus(void){}
void ROBOT_IMUHandle(void)
{
    int16_t data[3]; AX_MPU6050_GetAccData(data); AX_MPU6050_GetGyroData(data);
}

/* Sensor bus model: register transactions only; no hardware access. */
static uint8_t regs[256],sensor_address,sensor_id_reg,fail_bus;
void AX_IMU_BusInit(void){}
void AX_IMU_DelayUs(uint32_t us){(void)us;}
uint8_t AX_IMU_BusRead(uint8_t address,uint8_t reg,uint8_t *data,uint8_t size)
{
    if(fail_bus || address!=sensor_address || !data) return 0u;
    memcpy(data,&regs[reg],size); return 1u;
}
uint8_t AX_IMU_BusWrite(uint8_t address,uint8_t reg,uint8_t value)
{
    if(fail_bus || address!=sensor_address) return 0u;
    if(reg!=sensor_id_reg) regs[reg]=value;
    return 1u;
}
static void sensor(uint8_t address,uint8_t reg,uint8_t id)
{
    memset(regs,0,sizeof(regs));fail_bus=0u;
    sensor_address=address;sensor_id_reg=reg;regs[reg]=id;
}
/* Extracted verbatim from the current production ax_robot.c by run_host_tests.py. */
#include "production_steps.inc"

static int test_sensors(void)
{
    int16_t data[3];
    sensor(0x6au,0u,5u);AX_MPU6050_Init();
    CHECK(AX_MPU6050_IsReady() && AX_MPU6050_GetKind()==AX_IMU_QMI8658);
    CHECK(regs[3]==0x85u && regs[4]==0xd5u && regs[8]==3u);
    regs[53]=0u;regs[54]=0x40u;
    AX_MPU6050_GetAccData(data);CHECK(data[0]==16384 && data[1]==0);
    regs[59]=0xd4u;regs[60]=0x30u; /* 12500 native counts */
    AX_MPU6050_GetGyroData(data);CHECK(data[0]==12800);
    regs[59]=0xffu;regs[60]=0x7fu;
    regs[61]=0u;regs[62]=0x80u;
    AX_MPU6050_GetGyroData(data);CHECK(data[0]==32767 && data[1]==-32768);
    AX_MPU6050_SetGyroSmplRate(200u);CHECK((regs[4]&15u)==5u);
    fail_bus=1u;data[0]=123;
    AX_MPU6050_GetAccData(data);CHECK(!AX_MPU6050_IsReady() && data[0]==0);
    fail_bus=0u;AX_MPU6050_GetAccData(data);CHECK(!AX_MPU6050_IsReady());
    sensor(0x6bu,0u,5u);AX_MPU6050_Init();CHECK(AX_MPU6050_IsReady());
    sensor(0x68u,0x75u,0x68u);AX_MPU6050_Init();
    CHECK(AX_MPU6050_IsReady() && AX_MPU6050_GetKind()==AX_IMU_MPU6050);
    CHECK(regs[0x1b]==8u && regs[0x1c]==0u);
    regs[0x3b]=0x40u;regs[0x3c]=0u;
    AX_MPU6050_GetAccData(data);CHECK(data[0]==16384);
    regs[0x43]=0xffu;regs[0x44]=0x9cu;
    AX_MPU6050_GetGyroData(data);CHECK(data[0]==-100);
    sensor(0x69u,0x75u,0x68u);AX_MPU6050_Init();CHECK(AX_MPU6050_IsReady());
    sensor(0x6au,0u,6u);AX_MPU6050_Init();CHECK(!AX_MPU6050_IsReady());
    return 0;
}
static int test_timer_deltas(void)
{
    TIM3->CNT=65530u;
    TIM4->CNT=10u;
    TIM2->CNT=UINT32_MAX-2u;
    TIM5->CNT=2u;
    AX_ENCODER_SyncDeltas();
    TIM3->CNT=14u;
    TIM4->CNT=65526u;
    TIM2->CNT=4u;
    TIM5->CNT=UINT32_MAX-3u;
    /* The velocity path ignores UIF and any legacy high-word state. */
    TIM3->SR=TIM4->SR=TIM_IT_Update;
    CHECK(AX_ENCODER_ReadDelta(0u)==7 && AX_ENCODER_ReadDelta(2u)==-6);
    CHECK(AX_ENCODER_ReadDelta(1u)==20 && AX_ENCODER_ReadDelta(3u)==-20);
    TIM3_IRQHandler(); TIM4_IRQHandler();
    CHECK(AX_ENCODER_ReadDelta(1u)==0 && AX_ENCODER_ReadDelta(3u)==0);
    CHECK(TIM3->CNT==14u && TIM4->CNT==65526u);
    TIM4->CNT=64526u;
    CHECK(AX_ENCODER_ReadDelta(3u)==-1000);
    AX_ROBOT_Stop();
    return 0;
}

static int test_drive(void)
{
    int v,s,i;
    uint16_t before_count;
    const int n=(int)(0.08/FWD_WHEEL_SCALE+0.5);
    CHECK(ROBOT_TYPE==ROBOT_FWD);
    CHECK(FWD_WHEEL_DIAMETER==0.065 && FWD_WHEEL_RESOLUTION==122880.0);
    AX_ROBOT_Stop();
    CHECK(AX_TURN_Left(0.08f,35.0f)==AX_TURN_OK);
    AX_ROBOT_Kinematics();
    CHECK(R_Wheel_B.TG>R_Wheel_A.TG && R_Wheel_D.TG>R_Wheel_C.TG);
    CHECK(pwm[0]>0 && pwm[1]<0 && pwm[2]>0 && pwm[3]<0);
    AX_ROBOT_Stop();
    CHECK(AX_SPEED_PidCtlA(0.0f,0.0f)==0);
    before_count=(uint16_t)TIM4->CNT;
    feed(-n,n,-n,n);
    R_Vel.TG_IX=80;AX_ROBOT_Kinematics();
    CHECK(fabs(R_Vel.RT_IX-80)<=1 && R_Vel.RT_IW==0);
    CHECK(AX_ENCODER_ReadDelta(0u)==0 && AX_ENCODER_ReadDelta(3u)==0);
    CHECK(TIM4->CNT==(uint16_t)(before_count+n));
    ax_control_dt_s=0.04f;
    feed(-2*n,2*n,-2*n,2*n);
    AX_ROBOT_Kinematics();CHECK(fabs(R_Vel.RT_IX-80)<=1);
    ax_control_dt_s=0.02f;
    AX_ROBOT_Stop();
    for(v=0;v<=120;v+=2) for(s=0;s<=100;s+=2) {
        (void)AX_TURN_Left(v/1000.0f,(float)s);AX_ROBOT_Kinematics();
        CHECK(R_Wheel_A.TG>=-0.000001f && R_Wheel_B.TG<=0.120001f);
        (void)AX_TURN_Right(v/1000.0f,(float)s);AX_ROBOT_Kinematics();
        CHECK(R_Wheel_B.TG>=-0.000001f && R_Wheel_A.TG<=0.120001f);
    }
    R_Vel.TG_IX=120;R_Vel.TG_IW=600;AX_ROBOT_Kinematics();
    CHECK(fabs(R_Wheel_B.TG)<=0.120001 && fabs(R_Wheel_A.TG)<=0.120001);
    AX_ROBOT_Stop();
    ax_robot_move_enable=0u;CHECK(AX_TURN_Left(0.08f,35.0f)==AX_TURN_DISABLED);
    ax_robot_move_enable=1u;ax_control_mode=CTL_PS2;
    CHECK(AX_TURN_Left(0.08f,35.0f)==AX_TURN_BUSY);ax_control_mode=CTL_ROS;
    feed(120000,0,0,0); R_Vel.TG_IX=80;AX_ROBOT_Kinematics();
    CHECK(ax_motion_fault==3u && ax_robot_move_enable==0u);
    for(i=0;i<4;i++) CHECK(pwm[i]==0);
    ax_motion_fault=0u;ax_robot_move_enable=1u;
    return 0;
}
static int test_task(void)
{
    int i;
    ticks=1000u; ax_motion_fault=0u; ax_robot_move_enable=1u;
    ax_control_mode=CTL_ROS; R_Vel.TG_IX=80;
    Key_Motion_Init();
    sensor(0x6au,0u,5u); AX_MPU6050_Init();
    gaps[0]=20u; gaps[1]=60u; cycle=0u;
    if(setjmp(finish_task)==0) Robot_Task(NULL);
    CHECK(ax_motion_fault==2u && ax_robot_move_enable==0u);
    for(i=0;i<4;i++) CHECK(pwm[i]==0);
    /* The first fault remains available even after another subsystem fails. */
    AX_ROBOT_LatchFault(1u); CHECK(ax_motion_fault==2u);
    ax_motion_fault=0u; ax_robot_move_enable=1u; ax_control_dt_s=0.02f;
    return 0;
}

static int test_limits_and_reported_failure(void)
{
    int v,y,i;
    ROBOT_Wheel *w[4]={&R_Wheel_A,&R_Wheel_B,&R_Wheel_C,&R_Wheel_D};
    ax_motion_fault=0u; ax_robot_move_enable=1u; ax_control_dt_s=0.02f;
    for(v=-1500;v<=1500;v+=100) for(y=-6280;y<=6280;y+=400) {
        feed(0,0,0,0); R_Vel.TG_IX=(short)v; R_Vel.TG_IW=(short)y;
        AX_ROBOT_Kinematics();
        for(i=0;i<4;i++) CHECK(fabs(w[i]->TG)<=AX_FWD_MAX_WHEEL_MPS+0.000001f);
    }
    AX_ROBOT_Stop();
    CHECK(AX_SPEED_PidCtlA(100000.0f,0)==4200);
    AX_ROBOT_Stop(); CHECK(AX_SPEED_PidCtlA(0,0)==0);
    CHECK(R_Vel.TG_IX==0 && R_Vel.TG_IW==0);
    /* Reproduce the observed negative B feedback without disabling protection. */
    feed(-1353,-3512,-1232,422); R_Vel.TG_IX=100;
    AX_ROBOT_Kinematics();
    CHECK(ax_motion_fault==3u && !ax_robot_move_enable);
    CHECK(R_Wheel_B.RT < -AX_FWD_OVERSPEED_MPS);
    for(i=0;i<4;i++) CHECK(pwm[i]==0);
    R_Vel.TG_IX=1500; AX_ROBOT_Kinematics();
    for(i=0;i<4;i++) CHECK(pwm[i]==0 && w[i]->TG==0);
    ax_motion_fault=0u; ax_robot_move_enable=1u;
    feed(-40000,0,0,0); R_Vel.TG_IX=80; AX_ROBOT_Kinematics();
    CHECK(R_Wheel_A.RT>0 && ax_motion_fault==3u);
    ax_motion_fault=0u; ax_robot_move_enable=1u;
    R_Vel.TG_IX=80; AX_ROBOT_Kinematics();
    /* C moved forward on this chassis while its raw encoder count decreased. */
    feed(0,0,-720,0); AX_ROBOT_Kinematics();
    feed(0,0,-720,0); AX_ROBOT_Kinematics();
    CHECK(ax_motion_fault==0 && R_Wheel_C.RT>0.05 && R_Wheel_C.PWM>=0);
    feed(0,400,0,0); AX_ROBOT_Kinematics();
    feed(0,400,0,0); AX_ROBOT_Kinematics();
    CHECK(ax_motion_fault==0 && R_Wheel_B.RT>0.025 && R_Wheel_B.PWM>=0);
    feed(0,-400,0,0); AX_ROBOT_Kinematics();
    CHECK(ax_motion_fault==0);
    feed(0,-400,0,0); AX_ROBOT_Kinematics();
    CHECK(ax_motion_fault==9 && ax_feedback_fault_mask==2 && !ax_robot_move_enable);
    for(i=0;i<4;i++) CHECK(pwm[i]==0);
    ax_motion_fault=0u; ax_robot_move_enable=1u;
    AX_ROBOT_Stop();
    for(i=0;i<5;i++) {
        AX_SPEED_PidCtlB(0.10f,0.0f);
        AX_SPEED_PidCtlD(-0.10f,0.0f);
    }
    /* A speed above the old 0.12 cutoff may still retain drive. */
    CHECK(AX_SPEED_PidCtlB(0.10f,0.15f)>0);
    CHECK(AX_SPEED_PidCtlD(-0.10f,-0.15f)<0);
    AX_SPEED_Reset();
    CHECK(AX_SPEED_PidCtlB(0.11f,0.0f)>0);
    for(i=0;i<4;i++) AX_SPEED_PidCtlB(0.11f,0.0f);
    {
        const int16_t steady_drive=AX_SPEED_PidCtlB(0.11f,0.0f);
        const int16_t reduced_drive=AX_SPEED_PidCtlB(0.11f,0.19f);
        CHECK(reduced_drive>0 && reduced_drive<steady_drive/2);
        CHECK(AX_SPEED_PidCtlB(0.11f,0.19f)<reduced_drive);
    }
    CHECK(AX_SPEED_PidCtlB(0.10f,0.25f)==0);
    CHECK(AX_SPEED_PidCtlD(-0.10f,-0.25f)==0);
    CHECK(ax_motion_fault==0);
    AX_ROBOT_Stop();
    CHECK(AX_StartupGyroOffset(327670)==-32767);
    CHECK(AX_StartupGyroOffset(-327680)==32767);
    CHECK(AX_StartupGyroOffset(0)==0);
    return 0;
}

static int test_action(int direction)
{
    float left=0.0f, right=0.0f, angle=0.0f;
    int turning=0, decelerating=0;
    unsigned i;
    ax_motion_fault=0u; ax_robot_move_enable=1u; ax_control_mode=CTL_ROS;
    ax_control_dt_s=0.02f;
    AX_ROBOT_Stop();
    R_Vel.RT_IX=0;
    R_Imu.GYRO_Z=0;
    Key_Motion_Init();
    key_motion_request=direction;
    for(i=0;i<2400;++i) {
        int32_t a=(int32_t)(left/FWD_WHEEL_SCALE), b=(int32_t)(right/FWD_WHEEL_SCALE);
        float gyro=(float)((right-left)/(FWD_WHEEL_BASE*FWD_WB_SCALE));
        R_Imu.GYRO_Z=(short)(gyro*32768.0f*180.0f/(500.0f*3.141592654f));
        feed(-a,b,-a,b);
        if(i==40) key_motion_request=-direction;
        Key_Motion_Step();
        CHECK(ax_motion_fault==0 && ax_robot_move_enable);
        CHECK(R_Wheel_A.TG>=0 && R_Wheel_A.TG<=AX_DEMO_OUTER_WHEEL_MPS+0.000001f);
        CHECK(R_Wheel_B.TG>=0 && R_Wheel_B.TG<=AX_DEMO_OUTER_WHEEL_MPS+0.000001f);
        CHECK(R_Wheel_C.TG==R_Wheel_A.TG && R_Wheel_D.TG==R_Wheel_B.TG);
        CHECK(pwm[0]>=0 && pwm[1]<=0 && pwm[2]>=0 && pwm[3]<=0);
        if(direction*R_Vel.TG_IW>0) turning=1;
        if(turning && R_Vel.TG_IX>0 &&
           R_Vel.TG_IX/1000.0f<AX_DEMO_REQUEST_SPEED_MPS-0.005f) decelerating=1;
        left+=(R_Wheel_A.TG-left)*0.15f;
        right+=(R_Wheel_B.TG-right)*0.15f;
        angle+=gyro*0.02f;
    }
    CHECK(turning && decelerating && fabs(angle-direction*1.570796327f)<0.035f);
    CHECK(R_Vel.TG_IX==0 && R_Vel.TG_IW==0);
    printf("PASS: simulated A/B direction %+d, angle %.2f deg\n",direction,(double)(angle*180.0f/3.141592654f));
    return 0;
}
int main(void)
{
    unsigned i;
    for(i=0u;i<4u;++i) AX_ENCODER_ExtendedInit(i);
    CHECK(TIM2->ARR==0xffffffffu && TIM5->ARR==0xffffffffu);
    TIM3->CNT=3u;TIM3->SR=TIM_IT_Update;
    CHECK(AX_ENCODER_ReadExtended(1u)==65539);
    TIM3_IRQHandler();CHECK(AX_ENCODER_ReadExtended(1u)==65539);
    AX_ENCODER_SetExtended(1u,0u);
    TIM3->CNT=65535u;TIM3->SR=TIM_IT_Update;
    CHECK(AX_ENCODER_ReadExtended(1u)==-1);
    TIM3_IRQHandler();CHECK(AX_ENCODER_ReadExtended(1u)==-1);
    TIM3->CNT=65535u;TIM3->SR=TIM_IT_Update;TIM3_IRQHandler();
    TIM3->CNT=31072u;CHECK(AX_ENCODER_ReadExtended(1u)==-100000);
    TIM4->CNT=0u;TIM4->SR=TIM_IT_Update;TIM4_IRQHandler();
    TIM4->CNT=34464u;CHECK(AX_ENCODER_ReadExtended(3u)==100000);
    AX_ROBOT_Stop();
    if(test_timer_deltas() || test_sensors() || test_drive() || test_task() || test_limits_and_reported_failure() ||
       test_action(1) || test_action(-1)) return 1;
    printf("PASS: %u checks, real FWD/turn/PID/IMU code, negative B overspeed, startup and production task; no hardware\n",checks);
    return 0;
}
