/**
  ******************************************************************************
  * @内  容  速度斜坡发生器：平滑加速 / 平滑减速（加加速度受限 S 曲线）
  *
  *          为什么需要它：
  *            直接把 R_Vel.TG_IX 从 0 改成 300（mm/s），得到的是一个阶跃。
  *            阶跃的加速度是无穷大（冲击、轮胎打滑、轮速 PID 积分饱和、
  *            车头点头），线性斜坡的加速度是阶跃（start/stop 时"顿"一下）。
  *            本模块让"加速度"也连续变化，即限制 jerk（加加速度），
  *            速度曲线因此有两个圆角，起步和收车都柔。
  *
  *          数学形式（每 20ms 执行一次）：
  *            1) e  = v_target - v_now                     剩余速度差
  *            2) ac = 加速段 ? a_max : d_max                本步加速度上限
  *            3) ar = min(ac, sqrt(2·d·|e|)) * sign(e)      收口曲线，天然不过冲
  *            4) a += clamp(ar - a, ±j·dt)                  加速度限斜率（jerk）
  *            5) v += a·dt
  *            6) 到点/过冲时把 v 钉在目标上，a 继续按 jerk 收敛到 0
  *
  *          边界：
  *            - 只写 R_Vel.TG_IX（纵向），不写 R_Vel.TG_IW（转向）
  *            - TG_IX 单位 mm/s，输出前四舍五入并限幅到 R_VX_LIMIT
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_ramp.h"

#include <math.h>

#ifndef AX_RAMP_NO_HAL
#include "ax_robot.h"
#endif

/* 默认参数 ------------------------------------------------------------------*/
#define AX_RAMP_V_MAX_DEF     0.40f    /* m/s    线速度上限                  */
#define AX_RAMP_A_MAX_DEF     0.60f    /* m/s^2  加速度                      */
#define AX_RAMP_D_MAX_DEF     0.90f    /* m/s^2  减速度（可略大于加速度）    */
#define AX_RAMP_J_MAX_DEF     3.00f    /* m/s^3  加加速度                    */
#define AX_RAMP_V_MIN_DEF     0.005f   /* m/s    速度死区                    */
#define AX_RAMP_TIMEOUT_DEF   0u       /* ms     默认关闭，视觉场景建议 200  */

/* 内部变量 ------------------------------------------------------------------*/
static AX_RAMP_Ctx ax_ramp;

/* 内部函数 ------------------------------------------------------------------*/
static float ax_ramp_fabs(float x)
{
    return (x < 0.0f) ? (-x) : x;
}

static float ax_ramp_clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

/* 公共函数 ------------------------------------------------------------------*/

/**
  * @简  述  初始化，上电调用一次
  */
void AX_RAMP_Init(void)
{
    ax_ramp.v_target   = 0.0f;
    ax_ramp.v_now      = 0.0f;
    ax_ramp.a_now      = 0.0f;

    ax_ramp.v_max      = AX_RAMP_V_MAX_DEF;
    ax_ramp.a_max      = AX_RAMP_A_MAX_DEF;
    ax_ramp.d_max      = AX_RAMP_D_MAX_DEF;
    ax_ramp.j_max      = AX_RAMP_J_MAX_DEF;
    ax_ramp.v_min      = AX_RAMP_V_MIN_DEF;

    ax_ramp.timeout_ms = AX_RAMP_TIMEOUT_DEF;
    ax_ramp.idle_ms    = 0u;

    ax_ramp.active     = 0u;   /* 默认不接管，避免影响原有控制方式 */
}

/**
  * @简  述  设置速度与加速度参数
  * @返回值  AX_RAMP_OK 成功；AX_RAMP_ERR_PARAM 参数非法
  */
uint8_t AX_RAMP_Config(float v_max, float a_max, float d_max, float j_max)
{
    /* NaN 检测：x != x 当且仅当 x 是 NaN；到这一步的 NaN 也不能放过 */
    if ((v_max != v_max) || (a_max != a_max) || (d_max != d_max) || (j_max != j_max))
    {
        return AX_RAMP_ERR_PARAM;
    }

    if ((v_max <= 0.0f) || (a_max <= 0.0f) || (d_max <= 0.0f) || (j_max <= 0.0f))
    {
        return AX_RAMP_ERR_PARAM;
    }

    ax_ramp.v_max = v_max;
    ax_ramp.a_max = a_max;
    ax_ramp.d_max = d_max;
    ax_ramp.j_max = j_max;

    ax_ramp.v_now    = ax_ramp_clampf(ax_ramp.v_now,    -v_max, v_max);
    ax_ramp.v_target = ax_ramp_clampf(ax_ramp.v_target, -v_max, v_max);

    return AX_RAMP_OK;
}

/**
  * @简  述  设置速度死区
  */
void AX_RAMP_SetDeadband(float v_min)
{
    if ((v_min != v_min) || (v_min < 0.0f))
    {
        return;
    }
    ax_ramp.v_min = v_min;
}

/**
  * @简  述  设置指令超时时间
  */
void AX_RAMP_SetTimeout(uint16_t timeout_ms)
{
    ax_ramp.timeout_ms = timeout_ms;
    ax_ramp.idle_ms    = 0u;
}

/**
  * @简  述  平滑过渡到目标速度（核心接口）
  * @参  数  v_mps 目标速度 m/s，正=前进，负=后退，0=停车
  */
void AX_RAMP_To(float v_mps)
{
    /* 视觉/串口传来 NaN 时按停车处理，避免电机失控 */
    if (v_mps != v_mps)
    {
        v_mps = 0.0f;
    }

    ax_ramp.v_target = ax_ramp_clampf(v_mps, -ax_ramp.v_max, ax_ramp.v_max);
    ax_ramp.idle_ms  = 0u;
    ax_ramp.active   = 1u;
}

/**
  * @简  述  平滑减速到 0
  */
void AX_RAMP_Stop(void)
{
    AX_RAMP_To(0.0f);
}

/**
  * @简  述  急停，不经斜坡
  */
void AX_RAMP_EmergencyStop(void)
{
    ax_ramp.v_target = 0.0f;
    ax_ramp.v_now    = 0.0f;
    ax_ramp.a_now    = 0.0f;
    ax_ramp.idle_ms  = 0u;
    ax_ramp.active   = 1u;
}

/**
  * @简  述  交还控制权
  */
void AX_RAMP_Release(void)
{
    ax_ramp.v_target = 0.0f;
    ax_ramp.v_now    = 0.0f;
    ax_ramp.a_now    = 0.0f;
    ax_ramp.active   = 0u;
}

/**
  * @简  述  推进一步斜坡曲线，20ms 调用一次
  * @参  数  dt 单位 s
  */
void AX_RAMP_Update(float dt)
{
    float e;           /* 剩余速度差        */
    float dir;         /* 追赶方向          */
    float a_cap;       /* 本步加速度上限    */
    float a_req;       /* 本步期望加速度    */
    float da;
    float da_max;

    if (dt <= 0.0f)
    {
        return;
    }

    /* 任务被阻塞时 dt 会异常大，截断防止一步跳到目标 */
    if (dt > 0.10f)
    {
        dt = 0.10f;
    }

    /* ---- 指令超时：视觉丢帧 / 上位机卡死时平滑停车 ---- */
    if ((ax_ramp.timeout_ms != 0u) && (ax_ramp.active != 0u))
    {
        if (ax_ramp.idle_ms < ax_ramp.timeout_ms)
        {
            ax_ramp.idle_ms = (uint16_t)(ax_ramp.idle_ms + (uint16_t)(dt * 1000.0f + 0.5f));
            if (ax_ramp.idle_ms >= ax_ramp.timeout_ms)
            {
                ax_ramp.idle_ms  = ax_ramp.timeout_ms;
                ax_ramp.v_target = 0.0f;
            }
        }
    }

    /* ---- 已在目标且加速度收敛，直接保持 ---- */
    e = ax_ramp.v_target - ax_ramp.v_now;
    if ((ax_ramp_fabs(e) <= 1e-6f) && (ax_ramp_fabs(ax_ramp.a_now) <= 1e-6f))
    {
        ax_ramp.v_now = ax_ramp.v_target;
        ax_ramp.a_now = 0.0f;
        return;
    }

    dir = (e >= 0.0f) ? 1.0f : (-1.0f);

    /* ---- 本步属于"加速"还是"减速" ----
       |v| 正在向目标变大  -> 加速段，用 a_max
       其余（含过零换向）  -> 减速段，用 d_max                        */
    if ((ax_ramp.v_now == 0.0f) ||
        ((ax_ramp.v_now * ax_ramp.v_target >= 0.0f) &&
         (ax_ramp_fabs(ax_ramp.v_now) < ax_ramp_fabs(ax_ramp.v_target))))
    {
        a_cap = ax_ramp.a_max;
    }
    else
    {
        a_cap = ax_ramp.d_max;
    }

    /* ---- 收口曲线 sqrt(2·d·|e|)：刚好能用 d_max 停在目标上的加速度 ----
       离目标越近允许的加速度越小，所以不会冲过头                      */
    a_req = (float)sqrt((double)(2.0f * ax_ramp.d_max * ax_ramp_fabs(e)));
    if (a_req > a_cap)
    {
        a_req = a_cap;
    }
    a_req *= dir;

    /* ---- 加速度限斜率：jerk 有界，"柔"就来自这一步 ---- */
    da     = a_req - ax_ramp.a_now;
    da_max = ax_ramp.j_max * dt;
    if (da >  da_max) da =  da_max;
    if (da < -da_max) da = -da_max;
    ax_ramp.a_now += da;

    /* ---- 积分得到新速度 ---- */
    ax_ramp.v_now += ax_ramp.a_now * dt;

    /* ---- 到点 / 过冲：只把速度钉在目标上，不动加速度 ----
       加速度继续按 j·dt 收敛到 0，因此 jerk 始终有界；
       若在这里把 a_now 直接清零，会人为制造一次加速度阶跃。      */
    if (((ax_ramp.v_target - ax_ramp.v_now) * e) <= 0.0f)
    {
        ax_ramp.v_now = ax_ramp.v_target;
    }

    /* ---- 死区：目标和当前都很小时直接归零，避免电机嗡嗡抖 ---- */
    if ((ax_ramp_fabs(ax_ramp.v_target) < ax_ramp.v_min) &&
        (ax_ramp_fabs(ax_ramp.v_now)    < ax_ramp.v_min))
    {
        ax_ramp.v_now    = 0.0f;
        ax_ramp.v_target = 0.0f;
        /* 这里同样不能把 a_now 清零：清零就是一次加速度阶跃，
           会破坏 jerk 指标。让它在后面几个周期里自行收敛。 */
    }
}

/**
  * @简  述  把平滑后的速度写回 R_Vel.TG_IX
  * @说  明  必须在 AX_ROBOT_Kinematics() 之前调用；active=0 时不做任何事
  */
void AX_RAMP_Apply(void)
{
#ifndef AX_RAMP_NO_HAL
    int32_t mm;

    if (ax_ramp.active == 0u)
    {
        return;
    }

    /* m/s -> mm/s，四舍五入 */
    if (ax_ramp.v_now >= 0.0f)
    {
        mm = (int32_t)(ax_ramp.v_now * 1000.0f + 0.5f);
    }
    else
    {
        mm = (int32_t)(ax_ramp.v_now * 1000.0f - 0.5f);
    }

    if (mm >  R_VX_LIMIT) mm =  R_VX_LIMIT;
    if (mm < -R_VX_LIMIT) mm = -R_VX_LIMIT;

    R_Vel.TG_IX = (short)mm;
#endif
}

/**
  * @简  述  当前输出速度 m/s
  */
float AX_RAMP_GetSpeed(void)
{
    return ax_ramp.v_now;
}

/**
  * @简  述  当前目标速度 m/s
  */
float AX_RAMP_GetTarget(void)
{
    return ax_ramp.v_target;
}

/**
  * @简  述  当前规划加速度 m/s^2
  */
float AX_RAMP_GetAccel(void)
{
    return ax_ramp.a_now;
}

/**
  * @简  述  是否接管中
  */
uint8_t AX_RAMP_IsActive(void)
{
    return ax_ramp.active;
}

/**
  * @简  述  是否已到目标速度
  * @说  明  只看速度；到点后内部加速度还会在几十毫秒内收敛到 0
  */
uint8_t AX_RAMP_IsSettled(void)
{
    if (ax_ramp_fabs(ax_ramp.v_target - ax_ramp.v_now) < ax_ramp.v_min)
    {
        return 1u;
    }
    return 0u;
}

/**
  * @简  述  是否已停稳
  */
uint8_t AX_RAMP_IsStopped(void)
{
    if ((ax_ramp_fabs(ax_ramp.v_target) < ax_ramp.v_min) &&
        (ax_ramp_fabs(ax_ramp.v_now)    < ax_ramp.v_min))
    {
        return 1u;
    }
    return 0u;
}

/**
  * @简  述  读取内部状态（调试用）
  */
const AX_RAMP_Ctx *AX_RAMP_GetCtx(void)
{
    return &ax_ramp;
}

/******************* (C) 版权 2026 ********************************************/
