/**
  ******************************************************************************
  * @内  容  速度斜坡发生器：平滑加速 / 平滑减速（加加速度受限 S 曲线）
  *
  *          解决的问题：
  *            - 让速度指令从任意当前值平滑过渡到任意目标值；
  *            - 目标可以是 0（平滑停车），也可以随时中途改（限速通道、
  *              路障、行人、弯道降速），改的时候同样平滑；
  *            - 目标的正负号只表示方向：正=前进，负=后退，0=停。
  *              换向自动经过 0 速度，不会一步跳过去。
  *
  *          输出到哪里：
  *            R_Vel.TG_IX（mm/s），交给原有 AX_ROBOT_Kinematics() 逆解成
  *            四轮目标速度，再由四轮 PID 跟。本模块不碰电机、不碰转向。
  *
  *          不碰 R_Vel.TG_IW：转向仍由转向模块负责，两者互不干扰。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_RAMP_H
#define __AX_RAMP_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 与 Robot_Task 的 20ms 周期一致 */
#define AX_RAMP_DT        0.02f

/* 返回值 */
#define AX_RAMP_OK        0
#define AX_RAMP_ERR_PARAM 1

/* 状态结构体 ----------------------------------------------------------------*/
typedef struct
{
    float    v_target;     /* 目标速度 m/s，正=前进，负=后退 */
    float    v_now;        /* 当前输出速度 m/s（已平滑）        */
    float    a_now;        /* 当前加速度 m/s^2                */

    float    v_max;        /* 速度上限 m/s                      */
    float    a_max;        /* 最大加速度 m/s^2（加速段）        */
    float    d_max;        /* 最大减速度 m/s^2（收速、刹车段）  */
    float    j_max;        /* 最大加加速度 m/s^3（"柔"的程度）  */
    float    v_min;        /* 速度死区 m/s                      */

    uint16_t timeout_ms;   /* 指令超时 ms，0=关闭               */
    uint16_t idle_ms;      /* 自上次指令起的计时                */

    uint8_t  active;       /* 1=接管 TG_IX，0=不干预            */
} AX_RAMP_Ctx;


/* 接口 ----------------------------------------------------------------------*/

/* 初始化（编译期默认参数），上电调用一次 */
void AX_RAMP_Init(void);

/* 设置参数，返回 AX_RAMP_OK / AX_RAMP_ERR_PARAM（非法值保持原参数） */
uint8_t AX_RAMP_Config(float v_max, float a_max, float d_max, float j_max);

/* 设置速度死区，低于此速度视为 0 */
void AX_RAMP_SetDeadband(float v_min);

/* 设置指令超时，0=关闭。视觉/上位机控制时建议 200~300ms */
void AX_RAMP_SetTimeout(uint16_t timeout_ms);

/**
  * 核心接口：平滑过渡到目标速度
  *   v_mps = 0.30  前进 0.30 m/s
  *   v_mps = 0     平滑停车
  *   v_mps = -0.20 后退 0.20 m/s
  * 立即返回，实际加速过程由 AX_RAMP_Update() 每次 20ms 推进。
  */
void AX_RAMP_To(float v_mps);

/* 平滑减速到 0（等价于 AX_RAMP_To(0)） */
void AX_RAMP_Stop(void);

/* 急停：立即清零，不走斜坡（仅故障保护用） */
void AX_RAMP_EmergencyStop(void);

/* 交还控制权，恢复原有手柄/串口控制 */
void AX_RAMP_Release(void);

/* 每 20ms 调用一次，推进斜坡曲线（dt 单位 s） */
void AX_RAMP_Update(float dt);

/* 把平滑后的速度写回 R_Vel.TG_IX，须在 AX_ROBOT_Kinematics() 之前调用 */
void AX_RAMP_Apply(void);

/* 状态查询 */
float   AX_RAMP_GetSpeed(void);     /* 当前输出速度 m/s          */
float   AX_RAMP_GetTarget(void);    /* 当前目标速度 m/s          */
float   AX_RAMP_GetAccel(void);     /* 当前加速度 m/s^2          */
uint8_t AX_RAMP_IsActive(void);     /* 是否接管中                */
uint8_t AX_RAMP_IsSettled(void);    /* 是否已到目标速度          */
uint8_t AX_RAMP_IsStopped(void);    /* 是否已停稳                */

/* 调试用：读取内部状态 */
const AX_RAMP_Ctx *AX_RAMP_GetCtx(void);

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) 版权 2026 ********************************************/
