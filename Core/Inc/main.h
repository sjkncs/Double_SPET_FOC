/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_ll_cordic.h"

#define LED_ON   WRITE_REG(GPIOC->BSRR, LL_GPIO_PIN_13)
#define LED_OFF  WRITE_REG(GPIOC->BRR, LL_GPIO_PIN_13)
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* 单电机两相线电流 Q1.15 格式
 * 左对齐12-bit ADC原始值减去零电流偏移 → 有符号Q15
 * 范围: [-32768, +32752], 有效精度12-bit (低4位始终为0) */
typedef struct {
    int16_t Ia;   /* A相线电流 (M*_1引脚), Q1.15 */
    int16_t Ib;   /* B相线电流 (M*_2引脚), Q1.15 */
} PhaseCurrentQ15_t;

/* 电机控制模式 (M1/M2 共用, Live Watch 切换 g_M1_CtrlMode / g_M2_CtrlMode) */
typedef enum {
    MODE_OPEN_LOOP = 0,      /* 电流闭环 + 角度开环 (Id=mA, 步进) */
    MODE_SPEED,              /* 速度闭环 (编码器角度, 速度PI→Iq) */
    MODE_POSITION,           /* 位置闭环 (PD→mA, 直出电流环) */
    MODE_STEP_ANGLE,         /* 步距角闭环 (电角度PI→Iq + Id磁弹簧, 抗齿槽) */
} MotorCtrlMode_t;

/* 调试用: 物理电流值 (单位: mA) */
typedef struct {
    int16_t M1_Ia_mA;   /* 电机1 A相 实际电流 mA */
    int16_t M1_Ib_mA;   /* 电机1 B相 实际电流 mA */
    int16_t M2_Ia_mA;   /* 电机2 A相 实际电流 mA */
    int16_t M2_Ib_mA;   /* 电机2 B相 实际电流 mA */
} DebugCurrent_t;

/* DQ 坐标系电流/电压 Q1.15 格式 */
typedef struct {
    int16_t d;   /* d轴, Q1.15 */
    int16_t q;   /* q轴, Q1.15 */
} DQ_Q15_t;

/* CORDIC sin/cos 缓存 (每电机每ISR周期计算一次, Park+InvPark共享) */
typedef struct {
    int16_t cos_val;   /* cos(θ), Q1.15 */
    int16_t sin_val;   /* sin(θ), Q1.15 */
} SinCos_Q15_t;

/* αβ 坐标系电压 Q1.15 (逆Park输出, 2相步进: α=PhaseA, β=PhaseB) */
typedef struct {
    int16_t alpha;   /* α轴 = Phase A 电压, Q1.15 */
    int16_t beta;    /* β轴 = Phase B 电压, Q1.15 */
} AlphaBeta_Q15_t;

/* PI 控制器 (电流环专用, Q15定点, kp/ki 支持 >32767)
 * 积分采用 Ki 预乘: integral += Ki × error, 输出 I项 = integral >> 15
 * kp/ki 为 int32_t: 突破 int16_t 上限, 支持高带宽电流环
 * 内部用 int64_t 中间量防溢出 (Cortex-M4 SMULL 1 cycle) */
typedef struct {
    int32_t kp;           /* 比例增益 (原 Q15, 现可 >32767) */
    int32_t ki;           /* 积分增益 (原 Q15, 现可 >32767) */
    int32_t integral;     /* 积分累积 (内含Ki因子, 运行态) */
    int32_t integ_limit;  /* 积分限幅 (正值, 对称) */
    int16_t out_limit;    /* 输出限幅 Q15 (正值, 对称) */
} PI_Q15_t;

/* 速度环 PI (浮点, 单位: RPM→mA, 1kHz 运行, 硬件FPU ≈100ns/次) */
typedef struct {
    float kp;             /* 比例增益: mA per RPM_error */
    float ki;             /* 积分增益: mA per (RPM_error × ms) */
    float integral;       /* 积分累积 (mA, 运行态) */
    float integ_limit;    /* 积分限幅 (mA, 独立于输出限幅, 防堵转飙积分) */
    float out_limit;      /* 输出限幅 (mA, 正值对称) */
    float output;         /* 最终输出 (mA, 限幅后, 只读诊断) */
} SpeedPI_Float_t;

/* 位置环 PID (浮点, 单位: counts→mA, 1kHz 运行)
 * P: 位置误差 × kp → mA (弹簧)
 * I: 误差积分 × ki → mA (消除稳态误差)
 * D: 速度反馈 × kd → mA (阻尼, 抑制振荡) */
typedef struct {
    float kp;             /* mA per count_error */
    float ki;             /* mA per (count_error × ms) */
    float kd;             /* mA per RPM (阻尼系数) */
    float integral;       /* 积分累积 (mA, 运行态) */
    float integ_limit;    /* 积分限幅 (mA) */
    float out_limit;      /* 输出限幅 (mA) */
    float output;         /* 最终输出 (mA, 限幅后, 只读诊断) */
} PosPID_Float_t;

/* VOFA+ JustFloat 发送帧 (10通道 + 帧尾)
 * I0~I5: 模式相关, I6: Id_Eff(mA), I7: Id_fbk(mA), I8: Iq_fbk(mA), I9: VqSat报警
 * 帧尾 0x7F800000 = IEEE754 +Inf, 小端存储 {0x00,0x00,0x80,0x7F} */
#define VOFA_CH_COUNT  10U

typedef struct {
    float    ch[VOFA_CH_COUNT];
    uint32_t tail;    /* 初始化时固定写入 0x7F800000, 运行时不修改 */
} VofaFrame_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* ---- HRTIM PWM参数 (Timer A/B/C/D 共用, UP_DOWN模式) ---- */
#define PWM_PERIOD      10000U                        /* 计数周期 (UP_DOWN半周期) */
#define PWM_MIN         150U                          /* CMP下限 (保护ADC采样窗口) */
#define PWM_Max         (PWM_PERIOD - PWM_MIN)        /* CMP上限 = 9850 */
#define PWM_ADC_OffSet  (PWM_PERIOD - PWM_MIN)        /* Master CMP1 = ADC触发点 */
#define PWM_CENTER      (PWM_PERIOD / 2U)             /* H桥中点 = 零电压 = 5000 */
#define PWM_HALF_RANGE  (PWM_CENTER - PWM_MIN)        /* Q15满量程对应的PWM半幅 = 4850 */

/* 电机 → HRTIM定时器映射 (每电机2相, 每相1个Timer控制H桥)
 * sTimerxRegs[] 数组索引: A=0, B=1, C=2, D=3, E=4, F=5
 *   HRTIMA(0) → M1 Phase A → 电流采样 PA2 (ADC1_JDR1)
 *   HRTIMB(1) → M1 Phase B → 电流采样 PA0 (ADC2_JDR1)
 *   HRTIMC(2) → M2 Phase A → 电流采样 PA1 (ADC2_JDR2)
 *   HRTIMD(3) → M2 Phase B → 电流采样 PA3 (ADC1_JDR2) */
#define M1_TIMER_PHASE_A  0U   /* Timer A — PA2 */
#define M1_TIMER_PHASE_B  1U   /* Timer B — PA0 */
#define M2_TIMER_PHASE_A  2U   /* Timer C — PA1 */
#define M2_TIMER_PHASE_B  3U   /* Timer D — PA3 */

/* ---- 电机参数 (切换电机时修改此区块) ----
 * 规格书与代码引用: 见 docs/MOTOR_8HA0205-10_SPEC.md
 * 当前: RB 8HA0205-10 (2相PM步进电机, 20mm, NEMA08)
 *   步距角     1.8° (50极对)
 *   额定电压   DC 7.4V
 *   额定电流   DC 0.4A/相
 *   相电阻 Rs  18.5 Ω ±10%  (20℃)
 *   相电感 Ls  5.3 mH ±20%  (1kHz; 阶跃测试实测 ~28mH, PM步进低频电感远高于AC额定)
 *   保持转矩   ≥20 mN·m
 *   转动惯量   1.7 g·cm²
 *
 * 电气时间常数 τ_e = Ls/Rs = 5.3mH/18.5Ω = 0.286ms (1kHz额定)
 * 阶跃实测等效 τ_e ≈ 28mH/18.5Ω ≈ 1.5ms (低频有效值, 用于PI调参)
 *
 * PI极零对消比: ki/kp = Rs / (Ls_eff × fs) ≈ 18.5 / (28mH × 17000) ≈ 0.039
 *   (注: 当前 ki/kp=0.205 是按 Ls=5.3mH 标定, 阶跃响应为一阶无超调, 可用) */
#define M1_POLE_PAIRS     12U   /* 0.9° 步距角: 400步/圈, 100极对 */
#define M2_POLE_PAIRS     50U    /* 1.8° 步距角: 200步/圈, 50极对 */
#define M1_RAMP_DIV       2U     /* M1 开环斜坡分频: delta 每 N 个 ISR 才 ±1 (高电感需慢加速) */
#define M1_ENC_DIR        (1)    /* M1 编码器方向: +1=同向, -1=反向 (已通过交换接线修正) */
#define M2_RS_MOHM        18500U  /* 相电阻 mΩ (仅供注释参考, 不参与运算) */
#define M2_LS_UH          5300U   /* 相电感 µH @1kHz (仅供注释参考) */

/* 电流检测硬件参数 (INA240A1 + 100mΩ 采样电阻) */
#define SHUNT_R_MOHM       100U    /* 采样电阻 mΩ */
#define AMP_GAIN           20U     /* INA240A1 放大倍数 */
#define ADC_VREF_MV        3300U   /* ADC参考电压 mV */

/* 满量程电流 (mA): Q15 = ±32768 时对应的物理电流
 * = Vref_mV × 1000 / (2 × Gain × R_mΩ)
 * = 3300 × 1000 / (2 × 20 × 100) = 825 mA */
#define I_FULLSCALE_MA     (ADC_VREF_MV * 1000U / (2U * AMP_GAIN * SHUNT_R_MOHM))

/* VBUS 分压电阻检测 (TLV9061 跟随器, 22kΩ / 4.7kΩ 分压) */
#define VBUS_R_HIGH_OHM   22000U
#define VBUS_R_LOW_OHM    4700U
/* 定点乘数: Vref_mV × (Rhigh+Rlow)/Rlow × 2^SHIFT / 4096
 * = 3300 × 26700/4700 × 8192/4096 = 37494
 * 用法: g_Vbus_mV = ((g_VbusRaw >> 4) * VBUS_MV_SCALE) >> VBUS_MV_SHIFT
 * 溢出校验: 4095 × 37494 = 153M < UINT32_MAX ✓ */
#define VBUS_MV_SCALE     37494U
#define VBUS_MV_SHIFT     13U

/* PI自适应: 基准电压 + 各电机出厂标定增益 (Flash 擦除后回退基准) */
#define VBUS_NOMINAL_MV   12000U
#define PI_M1_KP_NOM      21333
#define PI_M1_KI_NOM      1130
#define PI_M2_KP_NOM      80000
#define PI_M2_KI_NOM      16400

/* M2 速度环安全限幅 */
#define M2_RPM_CMD_MAX     1000   /* 最大允许指令 RPM */
#define M2_IQ_LIMIT_MAX_MA 600    /* 最大允许力矩 mA */
#define M2_RPM_RAMP_RATE   2      /* 斜坡限速: RPM/ms (1000→满速需1s, 可Live Watch调) */

/* M1 速度环安全限幅 */
#define M1_RPM_CMD_MAX     1000   /* 最大允许指令 RPM */
#define M1_IQ_LIMIT_MAX_MA 600    /* 最大允许力矩 mA */
#define M1_RPM_RAMP_RATE   2      /* 斜坡限速: RPM/ms (1000→满速需1s, 可Live Watch调) */

/* 位置环参数 (单位: 编码器 counts, 65536 = 1 圈 = 360°)
 * 位置 PD → mA 直出电流环, 跳过速度PI, 响应更快
 * P: 位置误差 × kp → mA,  D: 速度反馈 × kd → mA (阻尼) */
#define M2_POS_IQ_LIMIT_MA 300    /* 位置环输出电流上限 mA */

/* 编码器速度计算 */
#define ENC_ISR_FREQ_HZ    17000U   /* = fHRTIM / (2 × PWM_PERIOD) */
#define ENC_RPM_COEFF      (ENC_ISR_FREQ_HZ * 60U / 256U)  /* = 3984 */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* ---- 全局变量 extern 声明 (必须在 inline 函数之前) ---- */

/* FOC 电流采样
 * ADC通道映射 (注入模式, 12-bit左对齐, HRTIM_TRG2触发):
 *   M1_Ia: PA2 → ADC1_INJ_RANK1 (JDR1)  — HRTIMA Phase A
 *   M1_Ib: PA0 → ADC2_INJ_RANK1 (JDR1)  — HRTIMB Phase B
 *   M2_Ia: PA1 → ADC2_INJ_RANK2 (JDR2)  — HRTIMC Phase A
 *   M2_Ib: PA3 → ADC1_INJ_RANK2 (JDR2)  — HRTIMD Phase B
 *   Vbus:  PB1 → ADC3_INJ_RANK1 (JDR1)
 */
extern volatile PhaseCurrentQ15_t g_M1Current;
extern volatile PhaseCurrentQ15_t g_M2Current;
extern volatile uint16_t          g_VbusRaw;
/* 零电流校准偏移 (左对齐原始值, 需在电机不通电时标定) */
extern uint16_t g_Offset_M1Ia;
extern uint16_t g_Offset_M1Ib;
extern uint16_t g_Offset_M2Ia;
extern uint16_t g_Offset_M2Ib;

extern volatile uint8_t g_AdcConvDone;

extern DebugCurrent_t g_DebugCurrent;

/* 开环调试: 电角度 Q15 及步进/电压参数 (Live Watch可调) */
extern volatile int16_t g_M1_ElecAngle_Q15;
extern volatile int16_t g_M2_ElecAngle_Q15;
extern volatile int16_t g_M1_AngleDelta;
extern volatile int16_t g_M2_AngleDelta;
extern int16_t g_M1_AngleDelta_Target;  /* Live Watch设目标, 速度每ISR步进1 */
extern int16_t g_M2_AngleDelta_Target;
extern volatile int16_t g_M2_RPM_Cmd;  /* Live Watch: 设 RPM → 自动转 AngleDelta */
extern int16_t g_M1_OpenLoop_Vq;
extern int16_t g_M2_OpenLoop_Vq;
extern uint32_t g_Vbus_mV;

/* 调试用: Live Watch直接改mA值, while(1)自动转Q15写入Idq_Ref */
extern int16_t g_M1_Iq_Ref_mA;
extern int16_t g_M2_Iq_Ref_mA;
extern int16_t g_M2_Id_Hold_mA;              /* 闭环模式 d 轴保持电流 mA (磁弹簧, 抗齿槽脉动) */

/* CORDIC调试: 验证sin/cos输出是否正确 */
extern volatile SinCos_Q15_t g_DbgSinCos_M2;

/* KTH7111 编码器硬件配置 */
typedef struct {
    SPI_TypeDef  *spi;
    GPIO_TypeDef *cs_port;
    uint32_t      cs_pin;
} KTH7111_Hw_t;

extern const KTH7111_Hw_t g_Enc1_Hw;       /* SPI3 + PA15 */
extern const KTH7111_Hw_t g_Enc2_Hw;       /* SPI1 + PA4  */

extern volatile uint16_t g_Enc1_Angle;      /* 16-bit机械角 */
extern volatile uint8_t  g_Enc1_CrcOk;      /* CRC校验: 1=通过 */
extern volatile uint8_t  g_Enc1_RxBuf[3];   /* Debug用原始数据 */
extern volatile uint16_t g_Enc2_Angle;
extern volatile uint8_t  g_Enc2_CrcOk;
extern volatile uint8_t  g_Enc2_RxBuf[3];

extern volatile int32_t g_Enc1_SpeedRPM;    /* 编码器1转速 (RPM, 原始值) */
extern volatile int32_t g_Enc2_SpeedRPM;    /* 编码器2转速 (RPM, 原始值) */
extern volatile int32_t g_Enc1_SpeedFilt;   /* 编码器1转速 (RPM, IIR低通) */
extern volatile int32_t g_Enc2_SpeedFilt;   /* 编码器2转速 (RPM, IIR低通) */

extern VofaFrame_t g_VofaFrame;             /* VOFA+ JustFloat DMA 发送帧 */
extern volatile uint8_t  g_DoKth71Calib_M2;    /* Live Watch 置 1: 触发 Enc2/M2 ANLC 非线性校准 */
extern volatile uint8_t  g_DoKth71Calib_M1; /* Live Watch 置 1: 触发 Enc1/M1 ANLC 非线性校准 */
extern volatile uint8_t  g_DoZeroCalib_M2;       /* Live Watch 置 1: 触发电角度零点标定 */
extern volatile uint8_t  g_DoCurrLoopCalib_M2;   /* Live Watch 置 1: 电流环自校准 (开环电压注入 DC+AC 辨 R/L → PI) */
extern volatile uint8_t  g_CurrLoopCalibInProgress_M2;  /* 自校准进行中: ISR 累加 Vd/Id */
extern volatile uint8_t  g_CurrLoopCalibResultReady_M2;  /* 校准引擎完成标志 (TIM3 置 1 → main 清 0) */
extern volatile uint8_t  g_CalibVdDirectActive_M2;      /* M2: 1=ISR 跳过 PI, 直接写 g_CalibVdDirect_M2 */
extern volatile int16_t  g_CalibVdDirect_M2;             /* M2: 开环电压注入 D 轴 Q15 值 */
extern volatile int32_t  g_CalibVdAcc_M2;   /* M2: 17kHz ISR 累加 Vd */
extern volatile int32_t  g_CalibIdAcc_M2;   /* M2: 17kHz ISR 累加 Id */
extern volatile uint16_t g_CalibAccCnt_M2;  /* M2: 累加计数 */

/* M1 电流环校准 ISR 支持 */
extern volatile uint8_t  g_DoCurrLoopCalib_M1;        /* Live Watch 置 1: M1 电流环自校准 */
extern volatile uint8_t  g_CurrLoopCalibInProgress_M1; /* M1 校准进行中 */
extern volatile uint8_t  g_CurrLoopCalibResultReady_M1;
extern volatile uint8_t  g_CalibVdDirectActive_M1;    /* M1: 1=ISR 跳过 PI, 直接写 Vd */
extern volatile int16_t  g_CalibVdDirect_M1;          /* M1: 开环电压注入 D 轴 Q15 值 */
extern volatile int32_t  g_CalibVdAcc_M1;             /* M1: 17kHz ISR 累加 Vd */
extern volatile int32_t  g_CalibIdAcc_M1;             /* M1: 17kHz ISR 累加 Id */
extern volatile uint16_t g_CalibAccCnt_M1;            /* M1: 累加计数 */
extern volatile uint8_t  g_DoZeroCalib_M1;            /* Live Watch 置 1: M1 零点标定 */

extern volatile uint16_t g_ZeroCalibAngle_M1; /* M1 编码器 ZERO (C 变量, 与芯片同步) */
extern volatile uint16_t g_ZeroCalibAngle_M2; /* M2 编码器 ZERO (C 变量, 与芯片同步) */

/* Park变换输出: DQ坐标系电流 */
extern volatile DQ_Q15_t g_M1_Idq;
extern volatile DQ_Q15_t g_M2_Idq;

/* PI控制器实例 */
extern PI_Q15_t g_M1_PI_d;
extern PI_Q15_t g_M1_PI_q;
extern PI_Q15_t g_M2_PI_d;
extern PI_Q15_t g_M2_PI_q;

/* PI 基准参数 (Vbus 自适应缩放的基准点)
 * 校准后或 Flash 加载后更新, FOC_UpdatePI_ByVbus() 据此实时缩放
 * kp/ki 是在 vbus_mv 电压下的设计值 */
typedef struct {
    int32_t  kp;
    int32_t  ki;
    uint32_t vbus_mv;    /* 这组增益对应的母线电压 */
} PI_BaseGains_t;

extern PI_BaseGains_t g_M1_PI_Base;
extern PI_BaseGains_t g_M2_PI_Base;

/* Flash 参数手动擦除标志 (Live Watch 置 1 触发) */
extern volatile uint8_t g_FlashParamsErase;
/* Flash 参数加载状态 (boot 时读取结果, 仅 Live Watch 诊断) */
extern volatile uint8_t g_FlashParamsLoaded;  /* 0=未加载/无效, 1=M1有效, 2=M2有效, 3=全有效 */

/* dq 前馈解耦因子 (主循环预计算, ISR 使用)
 * wLs_factor = round(2π × fs × Ls × I_fullscale / (Vbus/2) × 2^20 / 32768)
 * ISR 中: Vd_ff = -(speed_dpp × Iq × wLs_factor) >> 20
 * 仅闭环模式 (编码器角度可靠) 时生效, 开环禁止 (丢步会正反馈) */
extern volatile int16_t g_M2_wLs_factor;

/* M2 速度闭环 */
extern volatile MotorCtrlMode_t g_M2_CtrlMode;   /* Live Watch: 0=开环, 1=速度, 2=位置 */
extern SpeedPI_Float_t g_M2_SpeedPI;           /* 速度PI (浮点, mA 单位, 1kHz) */
extern volatile int16_t g_M2_ElecAngleOffset;  /* 电角度偏移补偿 Q15, Live Watch 可调 */
extern volatile uint8_t g_M2_VqSaturated;      /* 电压圆饱和标志: Vd²+Vq²>Max² (限幅前检测) */
extern int16_t g_M2_RpmRampRate;               /* 斜坡限速 RPM/ms (Live Watch 可调) */

/* 位置环 PID→mA (MODE_POSITION 时生效, 直出电流环) */
extern PosPID_Float_t g_M2_PosPID;            /* 位置PID (浮点, counts→mA, 1kHz) */
extern volatile int32_t g_M2_PosCmd;          /* 位置指令 (counts, Live Watch 可调) */
extern volatile int32_t g_M2_PosFbk;          /* 位置反馈 (counts, 编码器累计) */

/* 位置环调试 (TIM3写→VOFA读, 仅 MODE_POSITION) */
extern volatile float g_Dbg_Pos_iqmA;        /* 位置PD最终输出 mA */
extern volatile float g_Dbg_Pos_pTerm;       /* P项 mA (衰减后) */
extern volatile float g_Dbg_Pos_dTerm;       /* D项 mA */

/* 速度环调试 (TIM3写→VOFA读, 仅 MODE_SPEED) */
extern volatile float g_Dbg_Spd_RpmCmd;      /* 斜坡后实际 RPM 指令 (PI 跟踪目标) */
extern volatile float g_Dbg_Spd_Err;         /* 速度误差 = RpmCmd - SpeedFilt (RPM) */
extern volatile float g_Dbg_Spd_IqOut;       /* 速度PI输出 = Iq指令 (mA) */

/* ---- 步距角闭环 (MODE_STEP_ANGLE) ----
 * 原理: 机械位置指令匀速递增, 位置误差→Iq力矩, 速度→阻尼, Id磁弹簧抗齿槽
 *       控制在机械位置空间 (encoder counts), 避免极对数放大效应
 *       参考: Trinamic TMC4671 position mode + SimpleFOC angle loop */
extern PosPID_Float_t g_M2_StepAnglePD;      /* 步距角PD (复用 PosPID_Float_t, Live Watch 展开) */
extern volatile int32_t g_M2_StepAngle_Ref;  /* 机械位置指令 (encoder counts, 65536=1圈) */
extern volatile int32_t g_M2_StepAngle_Err;  /* 位置误差 = Ref - PosFbk (encoder counts) */
extern volatile int16_t g_M2_StepAngle_LossThresh;  /* 失步阈值 (encoder counts, 默认30°=5461) */

/* ---- Id 自适应三级控制 (Standby / Hold / Boost + 迟滞) ----
 * Standby: 静止 N ms 后降流省电
 * Hold:    正常运行电流
 * Boost:   大位置误差时增强磁弹簧刚度, 抗失步 */
extern int16_t g_M2_Id_Standby_mA;           /* 静止保持电流 mA (IHold, 默认25) */
extern int16_t g_M2_Id_StandbyDelay_ms;      /* 运动→静止的延时 ms (默认 500) */
extern int16_t g_M2_Id_StillDeltaMax;        /* 静止判定: |Δenc|≤此值视为静止 (默认18≈0.1°) */
extern int16_t g_M2_Id_Boost_mA;             /* 大误差增强电流 mA (默认 100) */
extern int16_t g_M2_Id_BoostEnter;           /* 进入Boost阈值 counts (默认500≈2.7°) */
extern int16_t g_M2_Id_BoostExit;            /* 退出Boost阈值 counts (默认250≈1.4°, 迟滞) */
extern volatile int16_t g_M2_Id_Eff_mA;      /* 当前有效Id (只读诊断, ISR写) */

/* ============================================================
 * M1 控制变量 (与 M2 镜像, 共用 MotorCtrlMode_t 枚举)
 * ============================================================ */
extern volatile MotorCtrlMode_t g_M1_CtrlMode;
extern volatile int16_t g_M1_RPM_Cmd;
extern int16_t g_M1_RpmRampRate;

/* M1 速度PI */
extern SpeedPI_Float_t g_M1_SpeedPI;

/* M1 位置环 PI-P */
extern PosPID_Float_t g_M1_PosPID;
extern volatile int32_t g_M1_PosCmd;
extern volatile int32_t g_M1_PosFbk;

/* M1 步距角 PD */
extern PosPID_Float_t g_M1_StepAnglePD;
extern volatile int32_t g_M1_StepAngle_Ref;
extern volatile int32_t g_M1_StepAngle_Err;
extern volatile int16_t g_M1_StepAngle_LossThresh;

/* M1 Id 自适应 */
extern int16_t g_M1_Id_Hold_mA;
extern int16_t g_M1_Id_Standby_mA;
extern int16_t g_M1_Id_StandbyDelay_ms;
extern int16_t g_M1_Id_StillDeltaMax;
extern int16_t g_M1_Id_Boost_mA;
extern int16_t g_M1_Id_BoostEnter;
extern int16_t g_M1_Id_BoostExit;
extern volatile int16_t g_M1_Id_Eff_mA;

/* M1 电角度偏移 + dq 解耦 */
extern volatile int16_t g_M1_ElecAngleOffset;
extern volatile uint8_t g_M1_VqSaturated;      /* 电压圆饱和标志: Vd²+Vq²>Max² (限幅前检测) */
extern volatile int16_t g_M1_wLs_factor;

/* M1 调试 (TIM3写→VOFA读) */
extern volatile float g_Dbg_M1_Spd_RpmCmd;
extern volatile float g_Dbg_M1_Spd_Err;
extern volatile float g_Dbg_M1_Spd_IqOut;
extern volatile float g_Dbg_M1_Pos_iqmA;
extern volatile float g_Dbg_M1_Pos_pTerm;
extern volatile float g_Dbg_M1_Pos_dTerm;

/* ---- 电流环阶跃响应录制 (ISR采集→while回放→VOFA+) ----
 * 流程: g_StepCapTarget_mA + g_StepCapArm_M1/M2=1 → 自动选电机 → Id吸1s → Phase0(d轴)录→回放 → Phase1(q轴)录→回放→暂停
 * VOFA 回放时: ch0=Id_ref, ch1=Iq_ref, ch2=Id_fbk, ch3=Iq_fbk,
 *             ch4=样本序号(0~127, 阶跃发生在 STEP_CAP_PRE=10, 即第10个控制周期),
 *             ch5=phase(0/1)+100×发送翻转(方波, 每半周期=1次发送≈1ms, 便于数周期).
 * 评价PI: 到目标值周期数 ≈ (反馈达90%时 ch4 读数) - STEP_CAP_PRE; 过冲看峰值与稳态差 */
#define STEP_CAP_DEPTH   128U         /* 每轴录制深度, 样本 @17kHz ≈ 58.8µs/点 */
#define STEP_CAP_PRE     10U          /* 阶跃前基线采样数 (阶跃施加在第10点) */

typedef struct {
    int16_t id_ref;   /* d轴参考 Q15 */
    int16_t iq_ref;   /* q轴参考 Q15 */
    int16_t id_fbk;   /* d轴反馈 Q15 */
    int16_t iq_fbk;   /* q轴反馈 Q15 */
} StepCapSample_t;

/* 状态: 0=空闲, 1=录制中, 2=回放中, 3=暂停(两轴均完成) */
extern volatile uint8_t  g_StepCapState;
extern volatile uint8_t  g_StepCapArm_M1;    /* Live Watch 置1: 启动 M1 录制 (自动设 g_StepCapMotor=1) */
extern volatile uint8_t  g_StepCapArm_M2;    /* Live Watch 置1: 启动 M2 录制 (自动设 g_StepCapMotor=0) */
extern volatile uint8_t  g_StepCapMotor;     /* 0=M2, 1=M1 (由 Arm 自动设置, ISR/回放内部使用) */
extern volatile uint8_t  g_StepCapPhase;     /* 0=d轴测试, 1=q轴测试 */
extern volatile uint16_t g_StepCapIdx;       /* 当前写入位置 */
extern volatile int16_t  g_StepCapTarget_mA; /* 阶跃目标电流 mA (Live Watch设置) */
extern StepCapSample_t   g_StepCapBuf[STEP_CAP_DEPTH];

/* 电流环设定值与电压输出 (volatile: 主循环写/ISR读, 或 ISR速度PI写/ISR电流PI读) */
extern volatile DQ_Q15_t g_M1_Idq_Ref;
extern volatile DQ_Q15_t g_M2_Idq_Ref;
extern volatile DQ_Q15_t g_M1_Vdq;
extern volatile DQ_Q15_t g_M2_Vdq;

/* 逆Park输出: αβ电压 */
extern volatile AlphaBeta_Q15_t g_M1_Vab;
extern volatile AlphaBeta_Q15_t g_M2_Vab;

/* ---- ISR 内联函数 ---- */

/* Q15 → mA 转换宏
 * I_mA = (Q15 × I_FULLSCALE_MA) >> 15
 * 溢出校验: 32768 × 825 = 27,033,600 < INT32_MAX ✓ */
#define Q15_TO_MA(q15)  ((int16_t)(((int32_t)(q15) * (int32_t)I_FULLSCALE_MA) >> 15))

/* mA → Q15 转换宏 (调试用: 设置电流参考值)
 * Q15 = (mA × 32768) / I_FULLSCALE_MA
 * 溢出校验: 825 × 32768 = 27,033,600 < INT32_MAX ✓
 * 用法: g_M2_Idq_Ref.q = MA_TO_Q15(100);  → 设置100mA */
#define MA_TO_Q15(ma)   ((int16_t)(((int32_t)(ma) * 32768L) / (int32_t)I_FULLSCALE_MA))
#define MA_TO_Q15_F     (32768.0f / (float)I_FULLSCALE_MA)  /* 浮点版: mA×此值→Q15 */

/* ---- ISR内联函数: always_inline强制展开, 不依赖优化等级 ---- */
#define ALWAYS_INLINE __attribute__((always_inline)) static inline

/** CORDIC计算sin/cos (每电机每ISR周期1次, Park+InvPark共享)
 *  16-bit输出 + NBREAD=1: 单次RDATA读取同时获得cos和sin (Q1.15)
 *  RDATA[15:0] = cos, RDATA[31:16] = sin (RM0440 §18.4.2)
 *  相比32-bit模式: 省1次AHB总线读取 + 省2次>>16移位 */
ALWAYS_INLINE void FOC_CalcSinCos(int16_t theta_q15, SinCos_Q15_t *pSC)
{
    WRITE_REG(CORDIC->WDATA, (0x7FFF0000U | (uint32_t)(uint16_t)theta_q15));
    while (!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)) {}
    uint32_t packed = READ_REG(CORDIC->RDATA);       /* 单次读取: cos+sin */
    pSC->cos_val = (int16_t)(packed & 0xFFFFU);       /* 低16位 = cos, Q1.15 */
    pSC->sin_val = (int16_t)(packed >> 16);            /* 高16位 = sin, Q1.15 */
}

/** Park变换: αβ→DQ (纯算术, 无外设访问) */
ALWAYS_INLINE void FOC_ParkTransform(const SinCos_Q15_t *pSC,
                                     const volatile PhaseCurrentQ15_t *pIab,
                                     volatile DQ_Q15_t *pIdq)
{
    int32_t ia = (int32_t)pIab->Ia;
    int32_t ib = (int32_t)pIab->Ib;
    int16_t c  = pSC->cos_val;
    int16_t s  = pSC->sin_val;
    pIdq->d = (int16_t)((ia * c + ib * s) >> 15);
    pIdq->q = (int16_t)((-ia * s + ib * c) >> 15);
}

/** 逆Park变换: DQ→αβ (纯算术, 无外设访问) */
ALWAYS_INLINE void FOC_InvParkTransform(const SinCos_Q15_t *pSC,
                                        const volatile DQ_Q15_t *pVdq,
                                        volatile AlphaBeta_Q15_t *pVab)
{
    int32_t vd = (int32_t)pVdq->d;
    int32_t vq = (int32_t)pVdq->q;
    int16_t c  = pSC->cos_val;
    int16_t s  = pSC->sin_val;
    pVab->alpha = (int16_t)((vd * c - vq * s) >> 15);
    pVab->beta  = (int16_t)((vd * s + vq * c) >> 15);
}

/** 电流环PI控制器 (Q15定点, kp/ki为int32_t, 反算法anti-windup)
 *  kp/ki 可 >32767, 用 int64_t 中间量防溢出 (M4 SMULL 1 cycle)
 *  ADC 12位限制: kp ~200000 时量化噪声引起 ~2mA 电流纹波,
 *  超过 ~500000 纹波显著增大, 需权衡带宽与噪声 */
ALWAYS_INLINE int16_t FOC_PI_Run(PI_Q15_t *pi, int16_t ref, int16_t fbk)
{
    int16_t error = ref - fbk;

    /* P项: int64_t 防止 kp>32767 时溢出 */
    int32_t p_term = (int32_t)(((int64_t)pi->kp * error) >> 15);

    /* ① 积分累积 + 自身限幅 (int64_t 防止 ki×error + integral 溢出) */
    int64_t integ_64 = (int64_t)pi->integral + (int64_t)pi->ki * error;
    if (integ_64 > (int64_t)pi->integ_limit)       integ_64 = pi->integ_limit;
    else if (integ_64 < -(int64_t)pi->integ_limit)  integ_64 = -pi->integ_limit;
    int32_t integ_new = (int32_t)integ_64;

    int32_t output = p_term + (integ_new >> 15);

    /* ② 反算法 anti-windup: 输出饱和时修正积分
     * 若 P 项已独立超限, 积分归零防 windup; 否则精确回算 */
    if (output > (int32_t)pi->out_limit) {
        output = (int32_t)pi->out_limit;
        int32_t diff = (int32_t)pi->out_limit - p_term;
        integ_new = (diff > 0) ? (diff << 15) : 0;
    } else if (output < -(int32_t)pi->out_limit) {
        output = -(int32_t)pi->out_limit;
        int32_t diff = -(int32_t)pi->out_limit - p_term;
        integ_new = (diff < 0) ? (diff << 15) : 0;
    }

    pi->integral = integ_new;
    return (int16_t)output;
}

/** 圆限幅: 约束电压向量 Vq²+Vd² ≤ MaxModule² (防逆变器过调制)
 *  优先保 Vd (d轴电流控制稳定性), 剩余空间分配给 Vq (力矩)
 *  仅超限时触发 sqrtf → ARM FPU VSQRT.F32 (14 cycles @170MHz ≈ 82ns)
 *  正常运行 (未超限) 仅需 2次乘法 + 1次加法 + 1次比较 */
#define CIRCLE_MAX_MODULE  32000   /* Q15 最大电压向量模 (~97.7% 满量程, 留余量) */
#define CIRCLE_MAX_VD      30400   /* Vd 上限 = 95% × MaxModule */
#define CIRCLE_SQ_MAX      ((uint32_t)CIRCLE_MAX_MODULE * CIRCLE_MAX_MODULE)
#define CIRCLE_SQ_VD_MAX   ((uint32_t)CIRCLE_MAX_VD * CIRCLE_MAX_VD)

ALWAYS_INLINE void FOC_CircleLimitation(volatile DQ_Q15_t *pVdq)
{
    int32_t vd = (int32_t)pVdq->d;
    int32_t vq = (int32_t)pVdq->q;
    uint32_t sq_d = (uint32_t)(vd * vd);   /* |vd|≤32768, sq≤1.07e9, uint32安全 */
    uint32_t sq_q = (uint32_t)(vq * vq);

    if (sq_d + sq_q <= CIRCLE_SQ_MAX) return;   /* 快速路径: 未超限 */

    if (sq_d <= CIRCLE_SQ_VD_MAX) {
        /* Vd 在预算内: 保持 Vd, Vq = sqrt(MaxModule² - Vd²) */
        int16_t new_vq = (int16_t)__builtin_sqrtf((float)(CIRCLE_SQ_MAX - sq_d));
        pVdq->q = (vq >= 0) ? new_vq : -new_vq;
    } else {
        /* Vd 超预算: 限 Vd = ±MaxVd, Vq 取剩余空间 */
        pVdq->d = (vd >= 0) ? (int16_t)CIRCLE_MAX_VD : (int16_t)(-CIRCLE_MAX_VD);
        /* sqrt(MaxModule² - MaxVd²) 为编译期常量, 优化器会折叠 */
        int16_t new_vq = (int16_t)__builtin_sqrtf((float)(CIRCLE_SQ_MAX - CIRCLE_SQ_VD_MAX));
        pVdq->q = (vq >= 0) ? new_vq : -new_vq;
    }
}

/** 电压圆饱和诊断: Vd²+Vq² > MaxModule² 即将被 CircleLimitation 裁剪
 *  在 CircleLimitation 之前调用, 结果写入 g_Mx_VqSaturated */
ALWAYS_INLINE uint8_t FOC_IsVqSaturated(const volatile DQ_Q15_t *v)
{
    uint32_t sq = (uint32_t)((int32_t)v->d * v->d)
               + (uint32_t)((int32_t)v->q * v->q);
    return (sq > CIRCLE_SQ_MAX);
}

/** dq 前馈解耦: Vd -= ωLs·Iq, Vq += ωLs·Id
 *  speed_dpp = 电角速度 (Q15/ISR), wLs_factor = Ls 缩放因子 (主循环预算)
 *  开环模式跳过 (编码器角度无意义, 解耦会注入噪声) */
ALWAYS_INLINE void FOC_DecoupleFF(volatile DQ_Q15_t *vdq,
                                  int16_t speed_dpp,
                                  const volatile DQ_Q15_t *idq,
                                  int16_t wLs_factor,
                                  MotorCtrlMode_t mode)
{
    if (mode == MODE_OPEN_LOOP || wLs_factor == 0) return;
    int32_t vd_ff = -(int32_t)speed_dpp * (int32_t)idq->q;
    int32_t vq_ff = +(int32_t)speed_dpp * (int32_t)idq->d;
    vdq->d = (int16_t)((int32_t)vdq->d + (int16_t)((vd_ff * (int32_t)wLs_factor) >> 20));
    vdq->q = (int16_t)((int32_t)vdq->q + (int16_t)((vq_ff * (int32_t)wLs_factor) >> 20));
}

/** 逆 Park + 延迟补偿: 电角度前推一个 ISR 周期补偿 ADC→PWM 延迟
 *  comp_angle = elec_angle + speed_dpp (约 1 个 PWM 周期的电角度变化) */
ALWAYS_INLINE void FOC_InvParkWithComp(int16_t elec_angle, int16_t speed_dpp,
                                       const volatile DQ_Q15_t *vdq,
                                       volatile AlphaBeta_Q15_t *vab)
{
    int16_t comp_angle = elec_angle + speed_dpp;
    SinCos_Q15_t sc;
    FOC_CalcSinCos(comp_angle, &sc);
    FOC_InvParkTransform(&sc, vdq, vab);
}

/** 速度环 PI (浮点, 输入 RPM, 输出 mA, 1kHz 调用)
 *  反算法 anti-windup 三重保护:
 *  ① 积分限幅 ±integ_limit (堵转/超调保护)
 *  ② 输出饱和时从积分扣除超出量 (一拍退饱和, 无残留积分)
 *  ③ 输出限幅 ±out_limit (最终安全兜底) */
ALWAYS_INLINE float SpeedPI_Run(SpeedPI_Float_t *pi, float ref_rpm, float fbk_rpm)
{
    float error = ref_rpm - fbk_rpm;
    float p_term = pi->kp * error;
    float i_new;
    if (pi->ki == 0.0f) { i_new = 0.0f; pi->integral = 0.0f; }
    else { i_new = pi->integral + pi->ki * error; }

    /* ① 积分自身限幅 */
    if (i_new >  pi->integ_limit) i_new =  pi->integ_limit;
    if (i_new < -pi->integ_limit) i_new = -pi->integ_limit;

    float output = p_term + i_new;

    /* ② 反算法 anti-windup: 输出饱和时从积分扣除超出量
     * 对比"冻结"法: 一拍退饱和, 无残留积分, 速度提升一个数量级 */
    if (output > pi->out_limit) {
        i_new -= (output - pi->out_limit);
        output = pi->out_limit;
    } else if (output < -pi->out_limit) {
        i_new -= (output + pi->out_limit);
        output = -pi->out_limit;
    }
    /* 确保修正后仍在积分限幅内 */
    if (i_new >  pi->integ_limit) i_new =  pi->integ_limit;
    if (i_new < -pi->integ_limit) i_new = -pi->integ_limit;

    pi->integral = i_new;
    pi->output = output;
    return output;  /* 单位: mA */
}

/** αβ电压 → HRTIM PWM占空比 (H桥差分, 含防御性限幅)
 *  直接写 sTimerxRegs[].CMP1xR/CMP3xR, 绕过 LL 层的
 *  POSITION_VAL + 查表 + MODIFY_REG(读-改-写) 开销.
 *  idx_phA/idx_phB: sTimerxRegs[] 数组索引 (A=0,B=1,C=2,D=3) */
ALWAYS_INLINE void FOC_SetPhasePWM(uint32_t idx_phA, uint32_t idx_phB,
                                   const volatile AlphaBeta_Q15_t *pVab)
{
    /* 一次性读取 volatile, 后续在寄存器中计算 */
    int32_t va = (int32_t)pVab->alpha;
    int32_t vb = (int32_t)pVab->beta;

    int32_t delta_a = (va * (int32_t)PWM_HALF_RANGE) >> 15;
    int32_t delta_b = (vb * (int32_t)PWM_HALF_RANGE) >> 15;

    int32_t cmp1_a = (int32_t)PWM_CENTER + delta_a;
    int32_t cmp3_a = (int32_t)PWM_CENTER - delta_a;
    int32_t cmp1_b = (int32_t)PWM_CENTER + delta_b;
    int32_t cmp3_b = (int32_t)PWM_CENTER - delta_b;

    if (cmp1_a < (int32_t)PWM_MIN) cmp1_a = (int32_t)PWM_MIN;
    if (cmp1_a > (int32_t)PWM_Max) cmp1_a = (int32_t)PWM_Max;
    if (cmp3_a < (int32_t)PWM_MIN) cmp3_a = (int32_t)PWM_MIN;
    if (cmp3_a > (int32_t)PWM_Max) cmp3_a = (int32_t)PWM_Max;
    if (cmp1_b < (int32_t)PWM_MIN) cmp1_b = (int32_t)PWM_MIN;
    if (cmp1_b > (int32_t)PWM_Max) cmp1_b = (int32_t)PWM_Max;
    if (cmp3_b < (int32_t)PWM_MIN) cmp3_b = (int32_t)PWM_MIN;
    if (cmp3_b > (int32_t)PWM_Max) cmp3_b = (int32_t)PWM_Max;

    /* 直接写 CMP 寄存器 (无需 read-modify-write) */
    HRTIM1->sTimerxRegs[idx_phA].CMP1xR = (uint32_t)cmp1_a;
    HRTIM1->sTimerxRegs[idx_phA].CMP3xR = (uint32_t)cmp3_a;
    HRTIM1->sTimerxRegs[idx_phB].CMP1xR = (uint32_t)cmp1_b;
    HRTIM1->sTimerxRegs[idx_phB].CMP3xR = (uint32_t)cmp3_b;
}

/** 从ADC JDR读取线电流, 减去偏移 → Q1.15, 同时读取Vbus并换算mV
 *  M1: ADC_M1_2(PA2,ADC1_JDR1)=PhaseA, ADC_M1_1(PA0,ADC2_JDR1)=PhaseB
 *  M2: ADC_M2_1(PA1,ADC2_JDR2)=PhaseA, ADC_M2_2(PA3,ADC1_JDR2)=PhaseB
 *  Vbus: ADC3_JDR1, 12-bit左对齐 → mV (1次乘法+2次移位, ~3 cycles) */
ALWAYS_INLINE void FOC_GetPhaseCurrent(void)
{
    uint16_t raw_m1_ia = (uint16_t)(ADC1->JDR1);  /* PA2, ADC_M1_2, U2/U8 = Phase A */
    uint16_t raw_m1_ib = (uint16_t)(ADC2->JDR1);  /* PA0, ADC_M1_1, U1/U9 = Phase B */
    uint16_t raw_m2_ia = (uint16_t)(ADC2->JDR2);  /* PA1, ADC_M2_1, U3/U10 = Phase A */
    uint16_t raw_m2_ib = (uint16_t)(ADC1->JDR2);  /* PA3, ADC_M2_2, U4/U11 = Phase B */

    g_M1Current.Ia = (int16_t)(raw_m1_ia - g_Offset_M1Ia);
    g_M1Current.Ib = (int16_t)(raw_m1_ib - g_Offset_M1Ib);
    g_M2Current.Ia = (int16_t)(raw_m2_ia - g_Offset_M2Ia);
    g_M2Current.Ib = (int16_t)(raw_m2_ib - g_Offset_M2Ib);
    g_VbusRaw = (uint16_t)(ADC3->JDR1);
}

/** 开环电角度递增 + 速度缓变 (每ISR周期调用, 17kHz同步)
 *  pDelta 每次向 target 靠近1, 然后累加到角度
 *  从0到164需 164/17000 ≈ 9.6ms 平滑启动 */
ALWAYS_INLINE void OpenLoop_IncAngle(volatile int16_t *pAngle,
                                     volatile int16_t *pDelta,
                                     int16_t target)
{
    int16_t cur = *pDelta;
    if (cur < target)       cur++;
    else if (cur > target)  cur--;
    *pDelta = cur;
    *pAngle = (int16_t)((uint16_t)(*pAngle) + (uint16_t)cur);
}

/** 步距角模式: 机械位置指令递增 (17kHz)
 *  将电角度增量 (AngleDelta) 转换为机械位置 (encoder counts) 并累积
 *  使用分数累积器避免整数除法精度丢失:
 *    AngleDelta = 电角度Q15/周期, 机械增量 = AngleDelta / POLE_PAIRS
 *    例: 100RPM → AngleDelta≈321, 321/50=6.42 counts/cycle, 分数 .42 逐周期累积 */
ALWAYS_INLINE void OpenLoop_IncAngle_Ref(void)
{
    static int32_t frac_acc = 0;
    int16_t cur = g_M2_AngleDelta;
    int16_t target = g_M2_AngleDelta_Target;
    if (cur < target)       cur++;
    else if (cur > target)  cur--;
    g_M2_AngleDelta = cur;

    frac_acc += (int32_t)cur;
    int32_t mech_inc = frac_acc / M2_POLE_PAIRS;
    frac_acc -= mech_inc * M2_POLE_PAIRS;
    g_M2_StepAngle_Ref += mech_inc;
}

ALWAYS_INLINE void M1_OpenLoop_IncAngle_Ref(void)
{
    static int32_t frac_acc = 0;
    int16_t cur = g_M1_AngleDelta;
    int16_t target = g_M1_AngleDelta_Target;
    if (cur < target)       cur++;
    else if (cur > target)  cur--;
    g_M1_AngleDelta = cur;

    frac_acc += (int32_t)cur;
    int32_t mech_inc = frac_acc / M1_POLE_PAIRS;
    frac_acc -= mech_inc * M1_POLE_PAIRS;
    g_M1_StepAngle_Ref += mech_inc;
}

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* 函数已拆至各模块头文件:
 *   foc_adapt.h           : ADC_CalibrateOffset / Debug_ReadCurrent_mA / FOC_UpdatePI_ByVbus / RPM_to_AngleDelta
 *   calib_platform_m2.h   : CalibM2_Start / CalibM2_OnDone / CalibM2_OnTIM3_1ms / CalibM2_FillVofa
 *   zero_calib.h           : ZeroCalib_RunBlocking
 */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEST_Pin LL_GPIO_PIN_13
#define TEST_GPIO_Port GPIOC
#define ADC_M1_1_Pin LL_GPIO_PIN_0
#define ADC_M1_1_GPIO_Port GPIOA
#define ADC_M2_1_Pin LL_GPIO_PIN_1
#define ADC_M2_1_GPIO_Port GPIOA
#define ADC_M1_2_Pin LL_GPIO_PIN_2
#define ADC_M1_2_GPIO_Port GPIOA
#define ADC_M2_2_Pin LL_GPIO_PIN_3
#define ADC_M2_2_GPIO_Port GPIOA
#define VBUS_Pin LL_GPIO_PIN_1
#define VBUS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
