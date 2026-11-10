# TARKBOT R20 机器人底盘控制源码

塔克创新（XTARK）**TARKBOT R20 系列**机器人底盘的应用层源码，运行在 **OpenCTR H60 V3.2** 控制器上，固件版本 **V3.02（2024-09-09）**。

## 硬件平台

| 项目 | 参数 |
| --- | --- |
| 控制器 | OpenCTR H60 V3.2 |
| 主控 | STM32F407VET6（Cortex-M4F，168MHz，512K Flash / 192K SRAM） |
| 操作系统 | FreeRTOS |
| 电机 | 4 路直流减速电机，TIM1 / TIM9 / TIM12 输出 20kHz PWM |
| 编码器 | 4 路正交编码器，TIM2 / TIM3 / TIM4 / TIM5，支持 13 线霍尔与 1024 线磁编码器 |
| 传感器 | MPU6050（三轴加速度 + 三轴陀螺仪）、VIN 电压检测 |
| 外设 | WS2812 灯带、OLED、PS2 手柄、SBUS 航模遥控、蓝牙 APP、CAN、I2C EEPROM |

## 支持的底盘类型

通过 `Robot/ax_robot.h` 中的 `#define ROBOT_TYPE` 选择，默认 `ROBOT_MEC`：

| 宏 | 底盘 | 电机数量 |
| --- | --- | --- |
| `ROBOT_MEC` | 麦克纳姆轮 | 4 |
| `ROBOT_FWD` | 四轮差速 | 4 |
| `ROBOT_AKM` | 阿克曼 | 2（+ 舵机转向） |
| `ROBOT_TWD` | 两轮差速 | 2 |
| `ROBOT_TNK` | 履带 | 2 |
| `ROBOT_OMT` | 三轮全向 | 3 |

电机编号规则（手册）：**A = 左前，B = 右前，C = 左后，D = 右后**，插错接口会导致电机控制异常。

## 目录结构

```
Robot/                        应用层源码
├─ app/                       应用入口与任务编排
│  ├─ main.c/.h               主函数：外设初始化、开机自检、创建 FreeRTOS 任务
│  └─ ax_robot.c/.h           全局变量与任务（机器人控制 / 按键 / 灯效 / 显示 / PS2 / 琐事）
├─ control/                   运动控制
│  ├─ ax_kinematics.c/.h      运动学正逆解（按 ROBOT_TYPE 条件编译）
│  ├─ ax_speed.c/.h           四路轮子 PID 速度闭环
│  ├─ ax_control.c/.h         PS2 手柄、蓝牙 APP、SBUS 遥控指令解析
│  └─ ax_akm.c/.h             阿克曼转向角解算
├─ modules/                   功能模块
│  ├─ light/ ax_light.c/.h                        RGB 灯效（单色 / 呼吸 / 彩色 / 警灯 / 眼睛 / 跑马灯）
│  ├─ ramp/  ax_ramp.c/.h                         加减速斜坡
│  ├─ turn/  ax_turn.c/.h, turn_control.c/.h      转向控制
│  └─ key/   key_module.c/.h, key_gpio_tarkbot.c  按键模块
└─ platform/                  芯片与系统配置
   ├─ FreeRTOSConfig.h        FreeRTOS 配置
   ├─ stm32f4xx_conf.h        STM32 标准外设库配置
   ├─ stm32f4xx_it.c/.h       中断服务函数
   └─ system_stm32f4xx.c      系统时钟配置

Project/                       Keil 工程与构建脚本
├─ build.bat                  无头编译
├─ flash.bat                  无头烧录（CMSIS-DAP / DAPLink）
└─ keilkilll.bat              清理 Keil 编译中间文件

tests/                         主机侧单元测试（不参与固件编译）
├─ run_tests.py                编译并运行 turn_control.c 的算法断言
├─ test_turn_control.c         测试用例
└─ robot_integration_results.json

文件结构规范.md                 目录组织、命名、工程维护等约定
按键接入说明.md                 按键模块接入文档
转向接入说明.md                 转向模块接入文档
```

详细的目录职责、命名约定与工程维护规则见 [文件结构规范.md](./文件结构规范.md)。

## 任务划分（FreeRTOS）

| 任务 | 周期 | 作用 |
| --- | --- | --- |
| `Robot_Task` | 20ms（50Hz） | 遥控指令解析、运动学解算、PID 输出、IMU 处理、数据上报 |
| `Key_Task` | 事件驱动 | 按键短按切灯效 / 中按 AKM 校准 / 长按恢复出厂 |
| `Light_Task` | 30ms | RGB 灯带刷新 |
| `Disp_Task` | 100ms | OLED 状态显示 |
| `Ps2_Task` | 20ms | PS2 手柄数据读取 |
| `Trivia_Task` | 500ms | 电池电量管理、IMU 校准、蜂鸣器与 LED 心跳 |

## 通信协议摘要

| 方向 | 帧头 | 含义 |
| --- | --- | --- |
| 串口上行 | `0x10` | 机器人综合数据 |
| 串口下行 | `0x50` | 目标速度 |
| 串口下行 | `0x51` | IMU 零偏校准 |
| 串口下行 | `0x52` / `0x53` | 灯光控制 / 灯光参数保存 |
| 串口下行 | `0x54` | 蜂鸣器鸣叫 |
| 串口下行 | `0x5A` | 机器人型号选择 |
| 蓝牙 APP | `0x30` ~ `0x42` | 连接、摇杆 / 手柄 / 重力 / 坦克 / 机械臂模式、灯光 |
| CAN | `0x181` | 接收速度数据 |
| CAN | `0x121` ~ `0x123` | 发送机器人数据 |

## 编译与烧录

> 仓库已包含完整可编译工程：`Robot/`（应用层）、`Driver/`（板级驱动）、`FreeRTOS/`（内核与移植层）、`Libraries/`（CMSIS + STM32F4xx 标准外设库）以及 Keil 工程文件。clone 后可直接用 Keil 打开 `Project/xproject.uvprojx` 编译。

开发环境：Keil MDK 5.x（脚本按 `D:\Keil_v5` 编写，请按实际安装路径修改脚本首行的 `UV4`）。

```bat
Project\build.bat            :: 增量编译
Project\build.bat rebuild    :: 全量重新编译
Project\flash.bat            :: 通过 CMSIS-DAP / DAPLink 烧录并复位运行
```

两个脚本都是静默执行，日志分别写入 `Project\build.log` 与 `Project\flash.log`。

退出码：`0` 无错误无警告，`1` 有警告，`2` 编译或下载失败。

### 烧录常见问题

- `Internal DLL Error` / `Target DLL has been cancelled`：把 DAPLink 拔插一次再重试。
- `Erase Done. Programming Failed! RDDI-DAP Error`：SWD 时钟过高，把 `xproject.uvoptx` 中的 `<DbgClock>` 从 `10000000` 改为 `1000000` 即可。