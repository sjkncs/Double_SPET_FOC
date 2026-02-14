/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "kth71xx.h"
#include "curr_loop_autocalib.h"
#include "flash_params.h"
#include "zero_calib.h"
#include "calib_platform_m2.h"
#include "calib_platform_m1.h"
#include "foc_adapt.h"
#include "vofa_engine.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---- FOC 电流采样数据 ---- */
volatile PhaseCurrentQ15_t g_M1Current;
volatile PhaseCurrentQ15_t g_M2Current;
volatile uint16_t          g_VbusRaw;
/* 零电流校准偏移量 (左对齐原始值, 由ADC_CalibrateOffset()标定) */
uint16_t g_Offset_M1Ia = 0x8000U;
uint16_t g_Offset_M1Ib = 0x8000U;
uint16_t g_Offset_M2Ia = 0x8000U;
uint16_t g_Offset_M2Ib = 0x8000U;

volatile uint8_t g_AdcConvDone = 0;  /* HRTIM ISR置1, 校准函数清0 */

DebugCurrent_t g_DebugCurrent;       /* 调试用: 物理电流 mA */

/* 开环调试: 电角度 Q15, 步进量, 电压 */
volatile int16_t g_M1_ElecAngle_Q15 = 0;
volatile int16_t g_M2_ElecAngle_Q15 = 0;
volatile int16_t g_M1_AngleDelta = 0;  /* 实际步进值 (ISR内缓变) */
volatile int16_t g_M2_AngleDelta = 0;
int16_t g_M1_AngleDelta_Target = 0;  /* Live Watch设目标: 164≈51RPM, 负值反转 */
int16_t g_M2_AngleDelta_Target = 0;  /* 1.8°步距角(50极对), 321≈100RPM @17kHz */
volatile int16_t g_M2_RPM_Cmd = 0;  /* Live Watch: 设 RPM, while(1)自动→AngleDelta */
int16_t g_M1_OpenLoop_Vq = 0;  /* 开环电压: 2000~5000起步 */
int16_t g_M2_OpenLoop_Vq = 0;
uint32_t g_Vbus_mV;            /* 母线电压 mV */

/* 调试用: Live Watch改mA值, while(1)自动转Q15写入Idq_Ref.q */
int16_t g_M1_Iq_Ref_mA = 100;
int16_t g_M2_Iq_Ref_mA = 100;
int16_t g_M2_Id_Hold_mA = 50;  /* 闭环 d 轴保持电流 mA (磁弹簧抗齿槽, Live Watch 可调) */

/* Park变换输出 */
volatile DQ_Q15_t g_M1_Idq;
volatile DQ_Q15_t g_M2_Idq;

/* CORDIC调试变量: 用Live Watch查看M2的sin/cos是否正确 */
volatile SinCos_Q15_t g_DbgSinCos_M2;

/* ---- 电流环PI控制器 (极零对消, Vbus_nom=12V, Fs=17kHz, Ts≈58.8μs) ----
 * PI标定、不同角度波形差异、自动标定说明: docs/PI_TUNING.md
 *
 * M1 (14HYTF5203Z): R=27Ω, L=30mH
 *   τ = L/R = 1.111ms, kp/ki = L/(R×Ts) = 18.89
 *   kp=21333, ki=1130 → BW ≈ 50Hz
 *
 * M2 (8HA0205-10):  R=18.5Ω, L=5.3mH
 *   τ = L/R = 0.286ms, kp/ki = L/(R×Ts) = 4.87
 *   kp=10000, ki=2053 → BW ≈ 133Hz
 *
 * FOC_UpdatePI_ByVbus() 按实测Vbus等比缩放, 保持带宽恒定 (当前已禁用)
 *
 * M2 PI 调参依据 (RB 8HA0205-10):
 *   阶跃实测: kp=10000→τ≈6.5ms, kp=30000→τ≈2.2ms → K常数≈65
 *   目标200Hz BW → τ=0.8ms → kp=65/0.0008=81000
 *   ki/kp 保持 0.205 (按 Ls=5.3mH 标定, 实测一阶无超调)
 *   ADC 12-bit噪声限制: kp ~200000 时 ~2mA纹波, >500000 噪声显著
 *
 * 溢出安全 (int32_t kp/ki + int64_t中间量):
 *   max kp×error = 200000×32767 = 6.5e9 → int64_t >> 15 = 200000 ✓
 *   max ki×error = 41000×32767 = 1.34e9 < INT32_MAX ✓ */
#define PI_M1_KP_NOM            21333   /* M1 kp @12V (待调) */
#define PI_M1_KI_NOM            1130    /* M1 ki @12V (待调) */
#define PI_M2_KP_NOM            80000   /* M2 kp @12V → 目标BW≈200Hz */
#define PI_M2_KI_NOM            16400   /* M2 ki = kp × 0.205 (极零对消) */
#define PI_DEFAULT_INTEG_LIMIT  ((int32_t)32767 * 32768)
#define PI_DEFAULT_OUT_LIMIT    32767

PI_Q15_t g_M1_PI_d = { .kp = PI_M1_KP_NOM, .ki = PI_M1_KI_NOM, .integral = 0,
                        .integ_limit = PI_DEFAULT_INTEG_LIMIT,
                        .out_limit   = PI_DEFAULT_OUT_LIMIT };
PI_Q15_t g_M1_PI_q = { .kp = PI_M1_KP_NOM, .ki = PI_M1_KI_NOM, .integral = 0,
                        .integ_limit = PI_DEFAULT_INTEG_LIMIT,
                        .out_limit   = PI_DEFAULT_OUT_LIMIT };
PI_Q15_t g_M2_PI_d = { .kp = PI_M2_KP_NOM, .ki = PI_M2_KI_NOM, .integral = 0,
                        .integ_limit = PI_DEFAULT_INTEG_LIMIT,
                        .out_limit   = PI_DEFAULT_OUT_LIMIT };
PI_Q15_t g_M2_PI_q = { .kp = PI_M2_KP_NOM, .ki = PI_M2_KI_NOM, .integral = 0,
                        .integ_limit = PI_DEFAULT_INTEG_LIMIT,
                        .out_limit   = PI_DEFAULT_OUT_LIMIT };

/* ---- PI 基准参数 (Vbus 自适应缩放的基准点) ----
 * Boot 时从 Flash 加载, 或回退到编译时 NOM; 校准后实时更新 */
PI_BaseGains_t g_M1_PI_Base = { .kp = PI_M1_KP_NOM, .ki = PI_M1_KI_NOM, .vbus_mv = VBUS_NOMINAL_MV };
PI_BaseGains_t g_M2_PI_Base = { .kp = PI_M2_KP_NOM, .ki = PI_M2_KI_NOM, .vbus_mv = VBUS_NOMINAL_MV };
volatile uint8_t g_FlashParamsErase  = 0;   /* Live Watch 置 1: 擦除 Flash 参数 */
volatile uint8_t g_FlashParamsLoaded = 0;   /* Boot 诊断: bit0=M1, bit1=M2 */
volatile int16_t g_M2_wLs_factor    = 0;   /* dq 解耦因子 (=0 时不解耦) */

/* ---- M2 控制模式 (Live Watch 改 g_M2_CtrlMode: 0=开环, 1=速度, 2=位置, 3=步距角) ---- */
volatile M2_CtrlMode_t g_M2_CtrlMode = M2_MODE_OPEN_LOOP;

/* ---- 速度环PI (浮点, 1kHz, 单位: RPM→mA) ----
 * kp:          mA per RPM_error
 * ki:          mA per (RPM_error × ms)
 * integ_limit: 积分上限 mA (防堵转暴冲, 应远小于 out_limit)
 * out_limit:   输出上限 mA (最大力矩电流) */
SpeedPI_Float_t g_M2_SpeedPI = { .kp = 0.5f, .ki = 0.01f,
                                  .integral = 0.0f,
                                  .integ_limit = 100.0f,
                                  .out_limit = 300.0f };
volatile int16_t g_M2_ElecAngleOffset = 0;  /* 电角度偏移 Q15, Live Watch 校准 */
volatile uint8_t g_M2_VqSaturated = 0;     /* 电流环q轴电压饱和标志 (仅诊断) */

/* ---- 电流环阶跃响应录制 ---- */
volatile uint8_t  g_StepCapState = 0;      /* 0=空闲 1=录制 2=回放 3=暂停 */
volatile uint8_t  g_StepCapArm   = 0;      /* Live Watch 置1启动 */
volatile uint8_t  g_StepCapPhase = 0;      /* 0=d轴 1=q轴 */
volatile uint16_t g_StepCapIdx   = 0;
volatile int16_t  g_StepCapTarget_mA = 200; /* 阶跃目标电流 mA (默认200) */
StepCapSample_t   g_StepCapBuf[STEP_CAP_DEPTH];
int16_t g_M2_RpmRampRate = M2_RPM_RAMP_RATE;  /* 斜坡限速 RPM/ms, Live Watch 可调 */

/* ---- 步距角闭环: 直接 PD→Iq (mA), 高带宽弹簧-阻尼器 ----
 * kp: mA/count  (弹簧刚度, 0.1 → 182 counts/1° 对应 18.2mA)
 * kd: mA/RPM   (速度阻尼, 用速度误差 speed-RPM_Cmd)
 * ki: mA/count·tick (积分, 消残差, 过零清积分防过冲)
 * out_limit: Iq 输出限幅 mA */
PosPID_Float_t g_M2_StepAnglePD = { .kp = 0.1f,  .kd = 0.8f, .ki = 0.01f,
                                     .integral = 0.0f,
                                     .integ_limit = 50.0f,
                                     .out_limit = 300.0f };
volatile int32_t g_M2_StepAngle_Ref = 0;       /* 机械位置指令 (encoder counts) */
volatile int32_t g_M2_StepAngle_Err = 0;       /* 位置误差 (encoder counts, VOFA 诊断) */
volatile int16_t g_M2_StepAngle_LossThresh = 5461;  /* 失步阈值 ≈ 30° 机械 */

/* ---- Id 自适应降流 (Trinamic IRun/IHold 模式) ---- */
int16_t g_M2_Id_Standby_mA    = 25;            /* 静止保持电流 mA (IHold, 默认=Id_Hold/2) */
int16_t g_M2_Id_StandbyDelay_ms = 500;         /* 运动→静止的降流延时 ms */
int16_t g_M2_Id_StillDeltaMax  = 18;          /* 静止判定阈值 (counts/ms, 18≈0.099°机械) */
int16_t g_M2_Id_Boost_mA      = 100;         /* 大误差增强电流 mA */
int16_t g_M2_Id_BoostEnter    = 500;         /* 进入Boost阈值 counts (500≈2.75°机械) */
int16_t g_M2_Id_BoostExit     = 250;         /* 退出Boost阈值 counts (250≈1.37°, 迟滞) */
volatile int16_t g_M2_Id_Eff_mA = 0;          /* 当前有效Id (ISR写, 只读诊断) */

/* ---- 位置环外环 PI-P 结构 (位置PI → 速度P-only → 电流PI) ----
 * kp:         RPM/count  (比例: 快速响应大偏差, 0.1 → 1°=18.2RPM)
 * ki:         RPM/count·tick (积分: 克服摩擦死区, 过零清积分防过冲)
 * kd:         未使用 (阻尼由速度内环 P 提供)
 * integ_limit: 积分限幅 RPM (防饱和)
 * out_limit:  速度限幅 RPM (velocity_limit, 防大位移飞车) */
PosPID_Float_t g_M2_PosPID = { .kp = 0.1f, .ki = 0.01f, .kd = 0.0f,
                                .integral = 0.0f,
                                .integ_limit = 300.0f,
                                .out_limit = 800.0f };
volatile int32_t g_M2_PosCmd = 0;   /* 位置指令 counts, Live Watch 可调 */
volatile int32_t g_M2_PosFbk = 0;   /* 位置反馈 counts, TIM3 累计 */

/* ============================================================
 * M1 控制变量 (与 M2 镜像结构, 默认参数相同)
 * ============================================================ */
volatile M2_CtrlMode_t g_M1_CtrlMode = M2_MODE_OPEN_LOOP;
volatile int16_t g_M1_RPM_Cmd = 0;
int16_t g_M1_RpmRampRate = M2_RPM_RAMP_RATE;

SpeedPI_Float_t g_M1_SpeedPI = { .kp = 0.5f, .ki = 0.01f,
                                  .integral = 0.0f,
                                  .integ_limit = 100.0f,
                                  .out_limit = 300.0f };

/* M1 步距角闭环: 直接 PD→Iq */
PosPID_Float_t g_M1_StepAnglePD = { .kp = 0.1f,  .kd = 0.8f, .ki = 0.01f,
                                     .integral = 0.0f,
                                     .integ_limit = 50.0f,
                                     .out_limit = 300.0f };
volatile int32_t g_M1_StepAngle_Ref = 0;
volatile int32_t g_M1_StepAngle_Err = 0;
volatile int16_t g_M1_StepAngle_LossThresh = 5461;

/* M1 Id 自适应 */
int16_t g_M1_Id_Hold_mA       = 50;
int16_t g_M1_Id_Standby_mA    = 25;
int16_t g_M1_Id_StandbyDelay_ms = 500;
int16_t g_M1_Id_StillDeltaMax  = 18;
int16_t g_M1_Id_Boost_mA      = 100;
int16_t g_M1_Id_BoostEnter    = 500;
int16_t g_M1_Id_BoostExit     = 250;
volatile int16_t g_M1_Id_Eff_mA = 0;

/* M1 位置环 PI-P */
PosPID_Float_t g_M1_PosPID = { .kp = 0.1f, .ki = 0.01f, .kd = 0.0f,
                                .integral = 0.0f,
                                .integ_limit = 300.0f,
                                .out_limit = 800.0f };
volatile int32_t g_M1_PosCmd = 0;
volatile int32_t g_M1_PosFbk = 0;

/* M1 电角度偏移 + dq 解耦 */
volatile int16_t g_M1_ElecAngleOffset = 0;
volatile uint8_t g_M1_VqSaturated = 0;     /* 电流环q轴电压饱和标志 (仅诊断) */
volatile int16_t g_M1_wLs_factor = 0;

/* M1 调试变量 */
volatile float g_Dbg_M1_Spd_RpmCmd = 0.0f;
volatile float g_Dbg_M1_Spd_Err    = 0.0f;
volatile float g_Dbg_M1_Spd_IqOut  = 0.0f;
volatile float g_Dbg_M1_Pos_iqmA   = 0.0f;
volatile float g_Dbg_M1_Pos_pTerm  = 0.0f;
volatile float g_Dbg_M1_Pos_dTerm  = 0.0f;

/* 电流给定: d轴=0(步进电机不弱磁), q轴=力矩电流
 * 使用 MA_TO_Q15() 宏, 调试时在Live Watch直接改mA值更直观 */
volatile DQ_Q15_t g_M1_Idq_Ref = { .d = 0, .q = 0 };
volatile DQ_Q15_t g_M2_Idq_Ref = { .d = 0, .q = 0 };

/* PI输出: DQ电压指令 (供逆Park变换使用) */
volatile DQ_Q15_t g_M1_Vdq;
volatile DQ_Q15_t g_M2_Vdq;

/* 逆Park输出: αβ电压 (直接对应两相PWM占空比) */
volatile AlphaBeta_Q15_t g_M1_Vab;
volatile AlphaBeta_Q15_t g_M2_Vab;

/* 位置环调试变量 (TIM3 ISR写 → VOFA读, 仅 M2_MODE_POSITION) */
volatile float g_Dbg_Pos_iqmA  = 0.0f;
volatile float g_Dbg_Pos_pTerm = 0.0f;
volatile float g_Dbg_Pos_dTerm = 0.0f;

/* 速度环调试变量 (TIM3 ISR写 → VOFA读, 仅 M2_MODE_SPEED) */
volatile float g_Dbg_Spd_RpmCmd = 0.0f;
volatile float g_Dbg_Spd_Err    = 0.0f;
volatile float g_Dbg_Spd_IqOut  = 0.0f;

/* ---- VOFA+ JustFloat DMA 帧 (TIM4 vofa_engine 自动发送) ---- */
VofaFrame_t g_VofaFrame = { .ch = {0}, .tail = 0x7F800000U };

/* 电流环自校准: 开环电压注入，全状态由 TIM3 1ms 驱动（无阻塞延时） */
volatile uint8_t g_DoCurrLoopCalib = 0;
volatile uint8_t  g_CurrLoopCalibInProgress = 0;
volatile uint8_t  g_CurrLoopCalibResultReady = 0;
volatile uint8_t  g_CalibVdDirectActive = 0;    /* M2: 1=ISR 跳过 PI, 直接写 Vd */
volatile int16_t  g_CalibVdDirect = 0;          /* M2: 开环电压 D 轴 Q15 */

/* M2: 17kHz ISR 累加 Vd/Id → TIM3 每 1ms 读平均值 */
volatile int32_t  g_CalibVdAcc  = 0;
volatile int32_t  g_CalibIdAcc  = 0;
volatile uint16_t g_CalibAccCnt = 0;

/* M1 电流环校准 */
volatile uint8_t  g_DoCurrLoopCalib_M1 = 0;
volatile uint8_t  g_CurrLoopCalibInProgress_M1 = 0;
volatile uint8_t  g_CurrLoopCalibResultReady_M1 = 0;
volatile uint8_t  g_CalibVdDirectActive_M1 = 0;
volatile int16_t  g_CalibVdDirect_M1 = 0;
volatile int32_t  g_CalibVdAcc_M1  = 0;
volatile int32_t  g_CalibIdAcc_M1  = 0;
volatile uint16_t g_CalibAccCnt_M1 = 0;
volatile uint8_t  g_DoZeroCalib_M1 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* 所有 static 函数定义在 USER CODE BEGIN 4, 此处仅前向声明 */
static void Main_RunKth71Calib(void);
static void Main_RunZeroCalib(void);
static void Main_RunZeroCalib_M1(void);
static void FlashParams_LoadToRuntime(void);
static void FlashParams_EraseAndReset(void);
static void StepCap_ArmAndAlign(void);
static void StepCap_Playback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- KTH7111 编码器 ---- */
const KTH7111_Hw_t g_Enc1_Hw = { SPI3, GPIOA, LL_GPIO_PIN_15 };
const KTH7111_Hw_t g_Enc2_Hw = { SPI1, GPIOA, LL_GPIO_PIN_4  };

volatile uint16_t g_Enc1_Angle = 0;
volatile uint16_t g_Enc2_Angle = 0;
volatile uint8_t  g_Enc1_CrcOk = 0;
volatile uint8_t  g_Enc1_RxBuf[3];
volatile uint8_t  g_Enc2_CrcOk = 0;
volatile int32_t  g_Enc1_SpeedRPM = 0;
volatile int32_t  g_Enc2_SpeedRPM = 0;
volatile int32_t  g_Enc1_SpeedFilt = 0;  /* IIR低通: cutoff ≈ 16Hz @1kHz采样 */
volatile int32_t  g_Enc2_SpeedFilt = 0;
volatile uint8_t  g_Enc2_RxBuf[3];
volatile uint8_t  g_DoKth71Calib = 0;   /* Live Watch 置 1：触发 Enc2 非线性校准(全自动开环600rpm) */
volatile uint8_t  g_DoZeroCalib  = 0;   /* Live Watch 置 1：触发 Enc2 电角度零点标定 */
volatile uint16_t g_ZeroCalibAngle_M1 = 0; /* M1 编码器 ZERO (C 变量, 与芯片同步) */
volatile uint16_t g_ZeroCalibAngle_M2 = 0; /* M2 编码器 ZERO (C 变量, 与芯片同步) */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //等待电路完全上电
  HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_CORDIC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* 上电同步: 从芯片 MTP 读回 ZERO, 对齐 g_ZeroCalibAngle_Mx.
   * 掉电后 MCU RAM 归零, 芯片从 MTP 加载; 不同步则下次校准 old_zero = 0 出错 */
  {
      uint8_t zl, zh;
      KTH71_ReadReg(&g_Enc1_Hw, KTH71_REG_ZERO_LO, &zl);
      KTH71_ReadReg(&g_Enc1_Hw, KTH71_REG_ZERO_HI, &zh);
      g_ZeroCalibAngle_M1 = ((uint16_t)zh << 8) | zl;

      KTH71_ReadReg(&g_Enc2_Hw, KTH71_REG_ZERO_LO, &zl);
      KTH71_ReadReg(&g_Enc2_Hw, KTH71_REG_ZERO_HI, &zh);
      g_ZeroCalibAngle_M2 = ((uint16_t)zh << 8) | zl;
  }

  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_MASTER, PWM_ADC_OffSet);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_MASTER);  /* Preload已开, 强制同步到活动寄存器 */
  /* 使能各定时器对 FLT4(PB11) 的响应：拉低 PB11 时 PWM 输出进入故障状态(INACTIVE) */
//  LL_HRTIM_TIM_EnableFault(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_FAULT_4);
//  LL_HRTIM_TIM_EnableFault(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_FAULT_4);
//  LL_HRTIM_TIM_EnableFault(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_FAULT_4);
//  LL_HRTIM_TIM_EnableFault(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_FAULT_4);
//  LL_HRTIM_EnableIT_FLT4(HRTIM1);   /* 故障中断: FLT4(PB11) */


  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, 5000);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_A, 5000);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_B, 5000);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_B, 5000);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, 5000);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_C, 5000);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, 5000);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_D, 5000);

  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATETRIG_REPETITION);
  LL_HRTIM_EnableIT_UPDATE(HRTIM1, LL_HRTIM_TIMER_MASTER);  /* 周期更新 */


  /* 启用HRTIM1的输出通道: TA1&2, TB1&2, TC1&2, TD1&2 */
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 |
                                 LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TB2 |
                                 LL_HRTIM_OUTPUT_TC1 | LL_HRTIM_OUTPUT_TC2 |
                                 LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2);

  LL_ADC_INJ_StartConversion(ADC1);
  LL_ADC_INJ_StartConversion(ADC2);
  LL_ADC_INJ_StartConversion(ADC3);
  /* 启动HRTIM1的定时器计数器: Timer A, B, C, D */
  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_MASTER | LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B | LL_HRTIM_TIMER_C |
                                       LL_HRTIM_TIMER_D );


  /* 零电流偏移校准 (ADC/HRTIM已启动, 电机未通电, 约241ms) */
  ADC_CalibrateOffset();



  /* 校准完成后使能ADC1 JEOS中断 → 电流环由ADC1_2_IRQHandler接管 */
  LL_ADC_EnableIT_JEOS(ADC1);

  /* ---- VOFA+ 诊断: 阻塞发一个'U'(0x55), 示波器应看到方波 ---- */
  /* VOFA+ DMA初始化: 地址 + TC中断 + DMAT使能 */
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&USART1->TDR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&g_VofaFrame);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_USART_EnableDMAReq_TX(USART1);

  /* 从 Flash 加载校准参数 → PI + Ls (无效则保持编译时默认) */
  FlashParams_LoadToRuntime();

  /* 启动TIM3 1kHz中断 → 速度环 / 位置环 */
  LL_TIM_ClearFlag_UPDATE(TIM3);
  LL_TIM_EnableIT_UPDATE(TIM3);
  LL_TIM_EnableCounter(TIM3);

  /* 启动TIM4 1kHz VOFA 引擎 → 自动填帧 + DMA 发送 (优先级10, 最低) */
  Vofa_StartTIM4();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- M1: 按控制模式设定电流参考 ---- */
    /* 电流环自校准期间: Idq_Ref 由校准模块全权写入, while 不覆盖 */
    if (g_M1_CtrlMode == M2_MODE_OPEN_LOOP && !g_CurrLoopCalibInProgress_M1) {
        g_M1_Idq_Ref.d = MA_TO_Q15(0);
        g_M1_Idq_Ref.q = MA_TO_Q15(g_M1_Iq_Ref_mA);
        g_M1_AngleDelta_Target = RPM_to_AngleDelta(g_M1_RPM_Cmd, M1_POLE_PAIRS, ENC_ISR_FREQ_HZ);
    }
    if (g_M1_CtrlMode == M2_MODE_STEP_ANGLE && !g_CurrLoopCalibInProgress_M1) {
        g_M1_AngleDelta_Target = RPM_to_AngleDelta(g_M1_RPM_Cmd, M1_POLE_PAIRS, ENC_ISR_FREQ_HZ);
    }
    /* 速度/位置闭环: Idq_Ref 由 M1_ControlLoop (TIM3) 写入, 主循环不干预 */

    /* ---- M2: 按控制模式设定电流参考 ---- */
    /* 阶跃录制期间(state!=0) 或 电流环自校准期间: Idq_Ref 由 ISR/校准模块全权写入, while 不覆盖 */
    if (g_M2_CtrlMode == M2_MODE_OPEN_LOOP && g_StepCapState == 0 && !g_CurrLoopCalibInProgress) {
        /* 开环: Id=0, Iq=保持力(mA), 角度由 RPM 步进 */
        g_M2_Idq_Ref.d = MA_TO_Q15(0);
        g_M2_Idq_Ref.q = MA_TO_Q15(g_M2_Iq_Ref_mA);
        g_M2_AngleDelta_Target = RPM_to_AngleDelta(g_M2_RPM_Cmd, M2_POLE_PAIRS, ENC_ISR_FREQ_HZ);
    }
    /* 步距角闭环: RPM→AngleDelta 控制递增速度, Idq_Ref 由 TIM3 ISR 写入 */
    if (g_M2_CtrlMode == M2_MODE_STEP_ANGLE && !g_CurrLoopCalibInProgress) {
        g_M2_AngleDelta_Target = RPM_to_AngleDelta(g_M2_RPM_Cmd, M2_POLE_PAIRS, ENC_ISR_FREQ_HZ);
    }
    /* 速度/位置闭环: Idq_Ref 由 ISR 内速度PI写入, 主循环不干预 */

    /* ---- 速度环参数安全保护 (浮点, 防负值/极端值) ---- */
    if (g_M2_SpeedPI.kp < 0.0f) g_M2_SpeedPI.kp = 0.0f;
    if (g_M2_SpeedPI.ki < 0.0f) g_M2_SpeedPI.ki = 0.0f;
    if (g_M2_RPM_Cmd >  (int16_t)M2_RPM_CMD_MAX)  g_M2_RPM_Cmd =  (int16_t)M2_RPM_CMD_MAX;
    if (g_M2_RPM_Cmd < -(int16_t)M2_RPM_CMD_MAX)  g_M2_RPM_Cmd = -(int16_t)M2_RPM_CMD_MAX;
    if (g_M2_SpeedPI.integ_limit < 0.0f) g_M2_SpeedPI.integ_limit = 0.0f;
    if (g_M2_SpeedPI.integ_limit > (float)M2_IQ_LIMIT_MAX_MA)
        g_M2_SpeedPI.integ_limit = (float)M2_IQ_LIMIT_MAX_MA;
    if (g_M2_SpeedPI.out_limit < 0.0f)   g_M2_SpeedPI.out_limit = 0.0f;
    if (g_M2_SpeedPI.out_limit > (float)M2_IQ_LIMIT_MAX_MA)
        g_M2_SpeedPI.out_limit = (float)M2_IQ_LIMIT_MAX_MA;
    if (g_M2_RpmRampRate < 1)    g_M2_RpmRampRate = 1;
    if (g_M2_RpmRampRate > 100)  g_M2_RpmRampRate = 100;  /* 最快100RPM/ms=全程10ms */
    if (g_M2_Id_Hold_mA < 0)    g_M2_Id_Hold_mA = 0;
    if (g_M2_Id_Hold_mA > 300)  g_M2_Id_Hold_mA = 300;   /* 上限 300mA, 防过热 */

    /* 步距角参数保护 (同 PosPID 结构体) */
    if (g_M2_StepAnglePD.kp < 0.0f) g_M2_StepAnglePD.kp = 0.0f;
    if (g_M2_StepAnglePD.kd < 0.0f) g_M2_StepAnglePD.kd = 0.0f;
    if (g_M2_StepAnglePD.ki < 0.0f) g_M2_StepAnglePD.ki = 0.0f;
    if (g_M2_StepAnglePD.integ_limit < 0.0f) g_M2_StepAnglePD.integ_limit = 0.0f;
    if (g_M2_StepAnglePD.integ_limit > (float)M2_IQ_LIMIT_MAX_MA)
        g_M2_StepAnglePD.integ_limit = (float)M2_IQ_LIMIT_MAX_MA;
    if (g_M2_StepAnglePD.out_limit < 0.0f) g_M2_StepAnglePD.out_limit = 0.0f;
    if (g_M2_StepAnglePD.out_limit > (float)M2_IQ_LIMIT_MAX_MA)
        g_M2_StepAnglePD.out_limit = (float)M2_IQ_LIMIT_MAX_MA;

    /* Id Standby 参数保护 */
    if (g_M2_Id_Standby_mA < 0)    g_M2_Id_Standby_mA = 0;
    if (g_M2_Id_Standby_mA > g_M2_Id_Hold_mA)
        g_M2_Id_Standby_mA = g_M2_Id_Hold_mA;  /* Standby ≤ Hold */
    if (g_M2_Id_StandbyDelay_ms < 50)   g_M2_Id_StandbyDelay_ms = 50;
    if (g_M2_Id_StandbyDelay_ms > 5000) g_M2_Id_StandbyDelay_ms = 5000;
    if (g_M2_Id_StillDeltaMax < 1)    g_M2_Id_StillDeltaMax = 1;
    if (g_M2_Id_StillDeltaMax > 200)  g_M2_Id_StillDeltaMax = 200; /* 200≈1.1° */
    if (g_M2_Id_Boost_mA < g_M2_Id_Hold_mA)
        g_M2_Id_Boost_mA = g_M2_Id_Hold_mA;  /* Boost ≥ Hold */
    if (g_M2_Id_Boost_mA > 300) g_M2_Id_Boost_mA = 300;
    if (g_M2_Id_BoostEnter < 50)   g_M2_Id_BoostEnter = 50;
    if (g_M2_Id_BoostEnter > 5000) g_M2_Id_BoostEnter = 5000;
    if (g_M2_Id_BoostExit < 10)    g_M2_Id_BoostExit = 10;
    if (g_M2_Id_BoostExit >= g_M2_Id_BoostEnter)
        g_M2_Id_BoostExit = g_M2_Id_BoostEnter / 2;  /* 迟滞: 退出 < 进入 */

    /* 位置环参数保护 (级联: out_limit = velocity_limit RPM) */
    if (g_M2_PosPID.kp < 0.0f)  g_M2_PosPID.kp = 0.0f;
    if (g_M2_PosPID.kp > 1.0f)  g_M2_PosPID.kp = 1.0f;   /* RPM/count, 1.0已极高 */
    if (g_M2_PosPID.out_limit < 10.0f)   g_M2_PosPID.out_limit = 10.0f;
    if (g_M2_PosPID.out_limit > 1000.0f) g_M2_PosPID.out_limit = 1000.0f;

    /* ---- M1 参数安全保护 (镜像 M2) ---- */
    if (g_M1_SpeedPI.kp < 0.0f) g_M1_SpeedPI.kp = 0.0f;
    if (g_M1_SpeedPI.ki < 0.0f) g_M1_SpeedPI.ki = 0.0f;
    if (g_M1_RPM_Cmd >  (int16_t)M2_RPM_CMD_MAX)  g_M1_RPM_Cmd =  (int16_t)M2_RPM_CMD_MAX;
    if (g_M1_RPM_Cmd < -(int16_t)M2_RPM_CMD_MAX)  g_M1_RPM_Cmd = -(int16_t)M2_RPM_CMD_MAX;
    if (g_M1_SpeedPI.integ_limit < 0.0f) g_M1_SpeedPI.integ_limit = 0.0f;
    if (g_M1_SpeedPI.integ_limit > (float)M2_IQ_LIMIT_MAX_MA)
        g_M1_SpeedPI.integ_limit = (float)M2_IQ_LIMIT_MAX_MA;
    if (g_M1_SpeedPI.out_limit < 0.0f)   g_M1_SpeedPI.out_limit = 0.0f;
    if (g_M1_SpeedPI.out_limit > (float)M2_IQ_LIMIT_MAX_MA)
        g_M1_SpeedPI.out_limit = (float)M2_IQ_LIMIT_MAX_MA;
    if (g_M1_RpmRampRate < 1)    g_M1_RpmRampRate = 1;
    if (g_M1_RpmRampRate > 100)  g_M1_RpmRampRate = 100;
    if (g_M1_Id_Hold_mA < 0)    g_M1_Id_Hold_mA = 0;
    if (g_M1_Id_Hold_mA > 300)  g_M1_Id_Hold_mA = 300;

    if (g_M1_StepAnglePD.kp < 0.0f) g_M1_StepAnglePD.kp = 0.0f;
    if (g_M1_StepAnglePD.kd < 0.0f) g_M1_StepAnglePD.kd = 0.0f;
    if (g_M1_StepAnglePD.ki < 0.0f) g_M1_StepAnglePD.ki = 0.0f;
    if (g_M1_StepAnglePD.integ_limit < 0.0f) g_M1_StepAnglePD.integ_limit = 0.0f;
    if (g_M1_StepAnglePD.integ_limit > (float)M2_IQ_LIMIT_MAX_MA)
        g_M1_StepAnglePD.integ_limit = (float)M2_IQ_LIMIT_MAX_MA;
    if (g_M1_StepAnglePD.out_limit < 0.0f) g_M1_StepAnglePD.out_limit = 0.0f;
    if (g_M1_StepAnglePD.out_limit > (float)M2_IQ_LIMIT_MAX_MA)
        g_M1_StepAnglePD.out_limit = (float)M2_IQ_LIMIT_MAX_MA;

    if (g_M1_Id_Standby_mA < 0)    g_M1_Id_Standby_mA = 0;
    if (g_M1_Id_Standby_mA > g_M1_Id_Hold_mA)
        g_M1_Id_Standby_mA = g_M1_Id_Hold_mA;
    if (g_M1_Id_StandbyDelay_ms < 50)   g_M1_Id_StandbyDelay_ms = 50;
    if (g_M1_Id_StandbyDelay_ms > 5000) g_M1_Id_StandbyDelay_ms = 5000;
    if (g_M1_Id_StillDeltaMax < 1)    g_M1_Id_StillDeltaMax = 1;
    if (g_M1_Id_StillDeltaMax > 200)  g_M1_Id_StillDeltaMax = 200;
    if (g_M1_Id_Boost_mA < g_M1_Id_Hold_mA)
        g_M1_Id_Boost_mA = g_M1_Id_Hold_mA;
    if (g_M1_Id_Boost_mA > 300) g_M1_Id_Boost_mA = 300;
    if (g_M1_Id_BoostEnter < 50)   g_M1_Id_BoostEnter = 50;
    if (g_M1_Id_BoostEnter > 5000) g_M1_Id_BoostEnter = 5000;
    if (g_M1_Id_BoostExit < 10)    g_M1_Id_BoostExit = 10;
    if (g_M1_Id_BoostExit >= g_M1_Id_BoostEnter)
        g_M1_Id_BoostExit = g_M1_Id_BoostEnter / 2;

    if (g_M1_PosPID.kp < 0.0f)  g_M1_PosPID.kp = 0.0f;
    if (g_M1_PosPID.kp > 1.0f)  g_M1_PosPID.kp = 1.0f;
    if (g_M1_PosPID.out_limit < 10.0f)   g_M1_PosPID.out_limit = 10.0f;
    if (g_M1_PosPID.out_limit > 1000.0f) g_M1_PosPID.out_limit = 1000.0f;

    g_Vbus_mV = ((uint32_t)(g_VbusRaw >> 4) * VBUS_MV_SCALE) >> VBUS_MV_SHIFT;
    FOC_UpdatePI_ByVbus();       /* PI增益跟随Vbus自适应 (基于 g_Mx_PI_Base 缩放) */
    Debug_ReadCurrent_mA();      /* g_DebugCurrent 显示mA电流 */

    /* KTH7111 非线性校准：Live Watch 置 g_DoKth71Calib=1 触发 */
    if (g_DoKth71Calib) {
        g_DoKth71Calib = 0;
        Main_RunKth71Calib();
    }

    /* M2 电角度零点标定 */
    if (g_DoZeroCalib) {
        g_DoZeroCalib = 0;
        Main_RunZeroCalib();
    }

    /* M1 电角度零点标定 */
    if (g_DoZeroCalib_M1) {
        g_DoZeroCalib_M1 = 0;
        Main_RunZeroCalib_M1();
    }

    /* M2 电流环自校准 */
    if (g_DoCurrLoopCalib) {
        g_DoCurrLoopCalib = 0;
        CalibM2_Start();
    }
    if (g_CurrLoopCalibResultReady) {
        g_CurrLoopCalibResultReady = 0;
        CalibM2_OnDone();
    }

    /* M1 电流环自校准 */
    if (g_DoCurrLoopCalib_M1) {
        g_DoCurrLoopCalib_M1 = 0;
        CalibM1_Start();
    }
    if (g_CurrLoopCalibResultReady_M1) {
        g_CurrLoopCalibResultReady_M1 = 0;
        CalibM1_OnDone();
    }

    /* Flash 参数手动擦除: Live Watch 置 g_FlashParamsErase=1 */
    if (g_FlashParamsErase) {
        g_FlashParamsErase = 0;
        FlashParams_EraseAndReset();
    }

    /* 电流环阶跃响应测试：Live Watch 设 g_StepCapTarget_mA + 置 g_StepCapArm=1 触发
     * 内部设 g_VofaSrc = VOFA_SRC_STEP_PLAYBACK, TIM4 自动回放
     * state=4 表示一轴播完, 由 StepCap_Playback 处理轴切换 */
    StepCap_ArmAndAlign();
    StepCap_Playback();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA2   ------> ADC1_IN3
  PA3   ------> ADC1_IN4
  */
  GPIO_InitStruct.Pin = ADC_M1_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADC_M1_2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ADC_M2_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADC_M2_2_GPIO_Port, &GPIO_InitStruct);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_HRTIM_TRG2;
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);
  LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);
  LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_4);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */
  /* 对 ADC1 进行校准 → 使能 → 等待 ADRDY */
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
  {
  }
  LL_ADC_Enable(ADC1);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
  {
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC2 GPIO Configuration
  PA0   ------> ADC2_IN1
  PA1   ------> ADC2_IN2
  */
  GPIO_InitStruct.Pin = ADC_M1_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADC_M1_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ADC_M2_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ADC_M2_1_GPIO_Port, &GPIO_InitStruct);

  /* ADC2 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC2, 0);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_HRTIM_TRG2;
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);
  LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
  LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC2);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC2_Init 2 */
  /* 对 ADC2 进行校准 → 使能 → 等待 ADRDY */
  LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0)
  {
  }
  LL_ADC_Enable(ADC2);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0)
  {
  }
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC345);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**ADC3 GPIO Configuration
  PB0   ------> ADC3_IN12
  PB1   ------> ADC3_IN1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = VBUS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(VBUS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC3, &ADC_InitStruct);
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC3, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC3, 0);
  LL_ADC_SetOverSamplingScope(ADC3, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC3), &ADC_CommonInitStruct);
  ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_HRTIM_TRG2;
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC3, &ADC_INJ_InitStruct);
  LL_ADC_INJ_SetQueueMode(ADC3, LL_ADC_INJ_QUEUE_DISABLE);
  LL_ADC_INJ_SetTriggerEdge(ADC3, LL_ADC_INJ_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC3);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC3);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC3, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC3, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_12);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC3_Init 2 */
  /* 对 ADC3 进行校准 → 使能 → 等待 ADRDY */
  LL_ADC_StartCalibration(ADC3, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC3) != 0)
  {
  }
  LL_ADC_Enable(ADC3);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC3) == 0)
  {
  }
  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */

  /* nothing else to be configured */

  /* USER CODE BEGIN CORDIC_Init 2 */

  /* 配置CORDIC为 sin/cos模式 (一次性, 后续直接读写WDATA/RDATA)
   * Function: COSINE → RDATA 单次读取同时获得 cos + sin
   * Precision: 4 iterations → ~16-bit精度, 精确匹配Q1.15 (4 cycles @ 170MHz ≈ 24ns)
   * Input:  16-bit Q1.15, 1次写入 (低16位=角度, 高16位=模值)
   * Output: 16-bit Q1.15, 1次读取 → RDATA[15:0]=cos, RDATA[31:16]=sin (RM0440 §18.4.2) */
  WRITE_REG(CORDIC->CSR,
      LL_CORDIC_FUNCTION_COSINE   |
      LL_CORDIC_PRECISION_4CYCLES |
      LL_CORDIC_SCALE_0           |
      LL_CORDIC_NBWRITE_1         |
      LL_CORDIC_NBREAD_1          |
      LL_CORDIC_INSIZE_16BITS     |
      LL_CORDIC_OUTSIZE_16BITS);  /* 16-bit输出: cos+sin打包在单次RDATA读取 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_HRTIM1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**HRTIM1 GPIO Configuration
  PB11   ------> HRTIM1_FLT4
  PA12   ------> HRTIM1_FLT1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* HRTIM1 interrupt Init */
  NVIC_SetPriority(HRTIM1_Master_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(HRTIM1_Master_IRQn);
  NVIC_SetPriority(HRTIM1_FLT_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(HRTIM1_FLT_IRQn);

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  LL_HRTIM_ConfigDLLCalibration(HRTIM1, LL_HRTIM_DLLCALIBRATION_MODE_CONTINUOUS, LL_HRTIM_DLLCALIBRATION_RATE_0);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_FLT_SetPrescaler(HRTIM1, LL_HRTIM_FLT_PRESCALER_DIV1);
  LL_HRTIM_FLT_SetSrc(HRTIM1, LL_HRTIM_FAULT_1, LL_HRTIM_FLT_SRC_DIGITALINPUT);
  LL_HRTIM_FLT_SetPolarity(HRTIM1, LL_HRTIM_FAULT_1, LL_HRTIM_FLT_POLARITY_HIGH);
  LL_HRTIM_FLT_SetFilter(HRTIM1, LL_HRTIM_FAULT_1, LL_HRTIM_FLT_FILTER_NONE);
  LL_HRTIM_FLT_SetBlankingSrc(HRTIM1, LL_HRTIM_FAULT_1, LL_HRTIM_FLT_BLANKING_RSTALIGNED);
  LL_HRTIM_FLT_SetCounterThreshold(HRTIM1, LL_HRTIM_FAULT_1, 0);
  LL_HRTIM_FLT_SetResetMode(HRTIM1, LL_HRTIM_FAULT_1, LL_HRTIM_FLT_COUNTERRST_UNCONDITIONAL);
  LL_HRTIM_FLT_EnableBlanking(HRTIM1, LL_HRTIM_FAULT_1);
  LL_HRTIM_FLT_Enable(HRTIM1, LL_HRTIM_FAULT_1);
  LL_HRTIM_FLT_SetBlankingSrc(HRTIM1, LL_HRTIM_FAULT_4, LL_HRTIM_FLT_BLANKING_RSTALIGNED);
  LL_HRTIM_FLT_SetCounterThreshold(HRTIM1, LL_HRTIM_FAULT_4, 0);
  LL_HRTIM_FLT_SetResetMode(HRTIM1, LL_HRTIM_FAULT_4, LL_HRTIM_FLT_COUNTERRST_UNCONDITIONAL);
  LL_HRTIM_FLT_EnableBlanking(HRTIM1, LL_HRTIM_FAULT_4);
  LL_HRTIM_FLT_SetSrc(HRTIM1, LL_HRTIM_FAULT_4, LL_HRTIM_FLT_SRC_DIGITALINPUT);
  LL_HRTIM_FLT_SetPolarity(HRTIM1, LL_HRTIM_FAULT_4, LL_HRTIM_FLT_POLARITY_LOW);
  LL_HRTIM_FLT_SetFilter(HRTIM1, LL_HRTIM_FAULT_4, LL_HRTIM_FLT_FILTER_1);
  LL_HRTIM_FLT_Enable(HRTIM1, LL_HRTIM_FAULT_4);
  LL_HRTIM_ConfigADCTrig(HRTIM1, LL_HRTIM_ADCTRIG_2, LL_HRTIM_ADCTRIG_UPDATE_TIMER_A, LL_HRTIM_ADCTRIG_SRC24_MCMP1);
  LL_HRTIM_SetADCPostScaler(HRTIM1, LL_HRTIM_ADCTRIG_2, 0);
  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_MASTER, 20000);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_MASTER, 0x00);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATETRIG_NONE);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_MASTER, 9850);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, 10000);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_A, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_COUNTING_MODE_UP_DOWN);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetComp3Mode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_GTCMP3_EQUAL);
  LL_HRTIM_TIM_SetRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetFaultEventRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetBMRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetADCRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetOutputRollOverMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION|LL_HRTIM_UPDATETRIG_RESET);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisableDeadTime(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, 5000);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_A, 5000);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUTPUTSET_RESYNC);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUTPUTRESET_TIMCMP1);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUTPUTSET_RESYNC);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUTPUTRESET_TIMCMP3);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_B, 10000);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_B, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_COUNTING_MODE_UP_DOWN);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetComp3Mode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_GTCMP3_EQUAL);
  LL_HRTIM_TIM_SetRollOverMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetFaultEventRollOverMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetBMRollOverMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetADCRollOverMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetOutputRollOverMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION|LL_HRTIM_UPDATETRIG_RESET);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_DisableDeadTime(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_B, 0x1388);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_B, 0x1388);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUTPUTSET_RESYNC);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUTPUTRESET_TIMCMP1);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUTPUTSET_RESYNC);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUTPUTRESET_TIMCMP3);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_C, 10000);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_C, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_COUNTING_MODE_UP_DOWN);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetComp3Mode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_GTCMP3_EQUAL);
  LL_HRTIM_TIM_SetRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetFaultEventRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetBMRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetADCRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetOutputRollOverMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION|LL_HRTIM_UPDATETRIG_RESET);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisableDeadTime(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, 0x1388);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_C, 0x1388);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUTPUTSET_RESYNC);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUTPUTRESET_TIMCMP1);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUTPUTSET_RESYNC);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUTPUTRESET_TIMCMP3);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_PRESCALERRATIO_MUL2);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_D, 10000);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_D, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_COUNTING_MODE_UP_DOWN);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetComp3Mode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_GTCMP3_EQUAL);
  LL_HRTIM_TIM_SetRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetFaultEventRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetBMRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetADCRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetOutputRollOverMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_ROLLOVER_MODE_BOTH);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION|LL_HRTIM_UPDATETRIG_RESET);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_DisableDeadTime(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_D, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_D);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, 0x1388);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_D, 0x1388);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUTPUTSET_RESYNC);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUTPUTRESET_TIMCMP1);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TD1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUTPUTSET_RESYNC);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUTPUTRESET_TIMCMP3);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TD2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  /* USER CODE BEGIN HRTIM1_Init 2 */



  /* USER CODE END HRTIM1_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**HRTIM1 GPIO Configuration
  PB12   ------> HRTIM1_CHC1
  PB13   ------> HRTIM1_CHC2
  PB14   ------> HRTIM1_CHD1
  PB15   ------> HRTIM1_CHD2
  PA8   ------> HRTIM1_CHA1
  PA9   ------> HRTIM1_CHA2
  PA10   ------> HRTIM1_CHB1
  PA11   ------> HRTIM1_CHB2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */
  /* LL_SPI_Init(NSS_SOFT)设SSM=1但不设SSI, Master需SSI=1防MODF */
  SET_BIT(SPI1->CR1, SPI_CR1_SSI);
  /* 8-bit模式必须设FRXTH=1, 否则RXNE需2字节才触发 (RM0440 §40.5.9) */
  SET_BIT(SPI1->CR2, SPI_CR2_FRXTH);

  /* PA15(CS)在MX_GPIO_Init中默认低, 拉高=CS空闲 */
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);

  /* 8-bit帧需quarter FIFO阈值, LL_SPI_Init不设此位 */
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI3 GPIO Configuration
  PB3   ------> SPI3_SCK
  PB5   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI3);
  /* USER CODE BEGIN SPI3_Init 2 */
  /* LL_SPI_Init(NSS_SOFT)设SSM=1但不设SSI, Master需SSI=1防MODF */
  SET_BIT(SPI3->CR1, SPI_CR1_SSI);
  /* 8-bit模式必须设FRXTH=1, 否则RXNE需2字节才触发 (RM0440 §40.5.9) */
  SET_BIT(SPI3->CR2, SPI_CR2_FRXTH);

  /* PA4(CS)在MX_GPIO_Init中默认低, 拉高=CS空闲 */
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /* 8-bit帧需quarter FIFO阈值, LL_SPI_Init不设此位 */
  LL_SPI_SetRxFIFOThreshold(SPI3, LL_SPI_RX_FIFO_TH_QUARTER);
  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 169;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */
  /* 覆盖CubeMX优先级: 速度环(TIM3)必须低于电流环(ADC=1,0) */
  NVIC_SetPriority(TIM3_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),10, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 169;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 1152000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(TEST_GPIO_Port, TEST_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);

  /**/
  GPIO_InitStruct.Pin = TEST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(TEST_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ---- 已拆至独立模块的函数 (勿在此重复定义) ----
 * main.h (ALWAYS_INLINE) : FOC_CalcSinCos / FOC_ParkTransform / FOC_InvParkTransform
 *                          FOC_PI_Run / FOC_SetPhasePWM / FOC_GetPhaseCurrent / OpenLoop_IncAngle
 * foc_adapt.c             : ADC_CalibrateOffset / Debug_ReadCurrent_mA / RPM_to_AngleDelta
 *                          FOC_UpdatePI_ByVbus / FOC_UpdateDecoupleFactors / FOC_SetM2Ls
 * zero_calib.c            : ZeroCalib_RunBlocking
 * calib_platform_m2.c     : CalibM2_Start / CalibM2_OnDone / CalibM2_OnTIM3_1ms / CalibM2_FillVofa
 * vofa_engine.c           : Vofa_StartTIM4 / Vofa_OnTIM4_1ms (TIM4 1kHz 自动填帧+DMA发送)
 */

/* ---- 阻塞式校准入口 (主循环触发, 阻塞数秒) ---- */

/**
 * @brief  KTH7111 非线性校准 (ANLC)
 *
 * 切开环 → 设力矩电流 → 等 1s → 阻塞式校准.
 * 全程 VOFA 由 TIM4 vofa_engine 自动发送 (VOFA_SRC_KTH71_CALIB).
 *
 * 前提: 电流环 PI 已标定; g_M2_Iq_Ref_mA 需足以驱动 800RPM 旋转.
 * 校准期间 ISR 暂停编码器 SPI, 主循环被 HAL_Delay 阻塞.
 */
static void Main_RunKth71Calib(void)
{
    /* 切开环, 确保力矩电流足够驱动 800RPM 旋转 */
    g_M2_CtrlMode = M2_MODE_OPEN_LOOP;
    g_M2_Idq_Ref.d = 0;
    g_M2_Idq_Ref.q = MA_TO_Q15(g_M2_Iq_Ref_mA);
    g_VofaSrc = VOFA_SRC_KTH71_CALIB;     /* TIM4 自动发送 KTH71 诊断帧 */
    HAL_Delay(1000);

    KTH71_MotorCtrl_t m2_calib = {
        .p_delta_target = &g_M2_AngleDelta_Target,
        .calib_delta    = KTH71_RPM_TO_DELTA(KTH71_CALIB_RPM, M2_POLE_PAIRS, ENC_ISR_FREQ_HZ),
    };
    (void)KTH71_CalibRunBlocking(&g_Enc2_Hw, &m2_calib, KTH71_CALIB_TIMEOUT_MS);
    /* 校准结束: TIM4 最后一帧自动包含最终 g_CalibDbgStep */
    g_VofaSrc = VOFA_SRC_IDLE;
}

/** 电角度零点标定：切开环 → 等 1s → Id 吸合摆动 → 写 ZERO → 保持开环 */
static void Main_RunZeroCalib(void)
{
    g_M2_CtrlMode = M2_MODE_OPEN_LOOP;
    HAL_Delay(1000);

    ZeroCalib_Motor_t m2_zero = {
        .p_elec_angle        = &g_M2_ElecAngle_Q15,
        .p_delta             = &g_M2_AngleDelta,
        .p_delta_target      = &g_M2_AngleDelta_Target,
        .p_idq_ref           = &g_M2_Idq_Ref,
        .p_zero_calib_angle  = &g_ZeroCalibAngle_M2,
    };
    (void)ZeroCalib_RunBlocking(&g_Enc2_Hw, &m2_zero);
}

/** M1 电角度零点标定 */
static void Main_RunZeroCalib_M1(void)
{
    g_M1_CtrlMode = M2_MODE_OPEN_LOOP;
    HAL_Delay(1000);

    ZeroCalib_Motor_t m1_zero = {
        .p_elec_angle        = &g_M1_ElecAngle_Q15,
        .p_delta             = &g_M1_AngleDelta,
        .p_delta_target      = &g_M1_AngleDelta_Target,
        .p_idq_ref           = &g_M1_Idq_Ref,
        .p_zero_calib_angle  = &g_ZeroCalibAngle_M1,
    };
    (void)ZeroCalib_RunBlocking(&g_Enc1_Hw, &m1_zero);
}

/* ---- Flash → 运行时参数加载 ---- */

/**
 * @brief  从 Flash 加载校准参数到运行时 PI / Ls
 *
 * 上电调用一次. 读取 Flash 中 Magic+CRC 校验过的 M1/M2 校准数据,
 * 写入 PI 控制器 + PI 基准 + Ls, 使上电即用上次校准结果.
 * Flash 无效或首次上电时保持编译时默认 PI_Mx_KP_NOM.
 */
static void FlashParams_LoadToRuntime(void)
{
    FlashCalibData_t flash_data;
    if (!FlashParams_Read(&flash_data)) return;   /* 无效/首次上电 → 保持默认 */

    if (flash_data.m1_valid) {
        g_M1_PI_Base.kp      = flash_data.m1_kp;
        g_M1_PI_Base.ki      = flash_data.m1_ki;
        g_M1_PI_Base.vbus_mv = flash_data.m1_vbus_calib_mv;
        g_M1_PI_d.kp = flash_data.m1_kp;  g_M1_PI_d.ki = flash_data.m1_ki;
        g_M1_PI_q.kp = flash_data.m1_kp;  g_M1_PI_q.ki = flash_data.m1_ki;
        FOC_SetM1Ls(flash_data.m1_ls_henry);
        g_FlashParamsLoaded |= 1U;
    }
    if (flash_data.m2_valid) {
        g_M2_PI_Base.kp      = flash_data.m2_kp;
        g_M2_PI_Base.ki      = flash_data.m2_ki;
        g_M2_PI_Base.vbus_mv = flash_data.m2_vbus_calib_mv;
        g_M2_PI_d.kp = flash_data.m2_kp;  g_M2_PI_d.ki = flash_data.m2_ki;
        g_M2_PI_q.kp = flash_data.m2_kp;  g_M2_PI_q.ki = flash_data.m2_ki;
        FOC_SetM2Ls(flash_data.m2_ls_henry);
        g_FlashParamsLoaded |= 2U;
    }
}

/* ---- HRTIM PWM 全输出位掩码 (关/恢复 PWM, 避免重复展开) ---- */
#define HRTIM_ALL_OUTPUTS  (LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | \
                            LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TB2 | \
                            LL_HRTIM_OUTPUT_TC1 | LL_HRTIM_OUTPUT_TC2 | \
                            LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2)

/* Vofa_DmaSend 已移至 vofa_engine.c (TIM4 自动发送, 无需手动触发) */

/**
 * @brief  Flash 参数擦除 + 运行时回退到编译时默认 PI
 *
 * Live Watch 置 g_FlashParamsErase=1 后由主循环调用.
 * 关 PWM → Flash erase → 恢复 PWM → PI 基准回退.
 */
static void FlashParams_EraseAndReset(void)
{
    LL_HRTIM_DisableOutput(HRTIM1, HRTIM_ALL_OUTPUTS);
    FlashParams_Erase();
    LL_HRTIM_EnableOutput(HRTIM1, HRTIM_ALL_OUTPUTS);

    g_M1_PI_Base = (PI_BaseGains_t){ PI_M1_KP_NOM, PI_M1_KI_NOM, VBUS_NOMINAL_MV };
    g_M2_PI_Base = (PI_BaseGains_t){ PI_M2_KP_NOM, PI_M2_KI_NOM, VBUS_NOMINAL_MV };
    g_FlashParamsLoaded = 0;
}

/* ---- 电流环 dq 双轴阶跃响应测试 ----
 * 流程: g_StepCapArm=1 → Id 吸合 1s 对齐零点 →
 *       Phase0(d轴) 录制→回放 → Phase1(q轴) 录制→回放 → 暂停
 * VOFA 通道: ch0=Id_ref, ch1=Iq_ref, ch2=Id_fbk, ch3=Iq_fbk,
 *           ch4=样本序号(阶跃在第10点), ch5=phase + 100×发送翻转方波 */

/**
 * @brief  阶跃测试启动: Id 吸合对齐 → 清 PI → 启动 ISR 录制
 *
 * 仅在 g_StepCapArm=1 且空闲(0)/暂停(3) 时执行, 否则跳过.
 */
static void StepCap_ArmAndAlign(void)
{
    if (!(g_StepCapArm && (g_StepCapState == 0 || g_StepCapState == 3)))
        return;
    g_StepCapArm = 0;

    /* Id 吸合 1s 对齐零点: 开环 + 电角度 0° + d 轴电流 */
    g_M2_CtrlMode = M2_MODE_OPEN_LOOP;
    g_M2_AngleDelta_Target = 0;
    g_M2_ElecAngle_Q15 = 0;
    g_M2_Idq_Ref.d = MA_TO_Q15(ZERO_CALIB_ID_MA);
    g_M2_Idq_Ref.q = 0;
    HAL_Delay(1000);

    /* 清 PI 积分 + 清 Ref, 保证干净初始状态 */
    g_M2_PI_d.integral = 0;
    g_M2_PI_q.integral = 0;
    g_M2_Idq_Ref.d = 0;
    g_M2_Idq_Ref.q = 0;
    g_M2_Iq_Ref_mA = 0;
    g_StepCapPhase = 0;       /* 从 d 轴开始 */
    g_StepCapIdx   = 0;
    g_VofaSrc = VOFA_SRC_STEP_PLAYBACK;   /* TIM4 自动回放 */
    g_StepCapState = 1;       /* ISR 开始录制 */
}

/**
 * @brief  阶跃测试: 处理 TIM4 回放完成后的轴切换
 *
 * TIM4 vofa_engine 以 1kHz 自动回放 g_StepCapBuf 到 VOFA.
 * 一轴播完后 TIM4 置 g_StepCapState = 4, 本函数处理:
 *   - phase 0 (d轴) 完: 清 PI → 切 q 轴录制 (state=1)
 *   - phase 1 (q轴) 完: 两轴完成 → 暂停 (state=3)
 *
 * PI 复位和 Idq_Ref 清零涉及电机控制, 不宜在低优先级 TIM4 ISR 中做.
 */
static void StepCap_Playback(void)
{
    if (g_StepCapState != 4) return;   /* 4 = TIM4 一轴播完 */

    if (g_StepCapPhase == 0) {
        /* d 轴完 → 自动切 q 轴录制 */
        g_M2_PI_d.integral = 0;
        g_M2_PI_q.integral = 0;
        g_M2_Idq_Ref.d = 0;
        g_M2_Idq_Ref.q = 0;
        g_StepCapPhase = 1;
        g_StepCapIdx   = 0;
        g_StepCapState = 1;   /* ISR 开始 q 轴录制 */
    } else {
        g_StepCapState = 3;   /* 两轴完成 → 暂停 */
        g_VofaSrc = VOFA_SRC_NORMAL;  /* 恢复正常 VOFA */
    }
}

/* Vofa_SendNormal 已移至 vofa_engine.c fill_normal() (TIM4 自动调度) */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
