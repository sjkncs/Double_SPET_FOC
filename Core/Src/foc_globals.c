/**
 * @file    foc_globals.c
 * @brief   双电机 FOC 全局变量集中定义
 *
 * 所有 g_ 前缀全局变量在此文件定义, extern 声明在 main.h.
 * 按信号流分 7 区, 方便阅读和 Live Watch 查找.
 *
 * 编码器相关变量 (g_Enc1/2_*) 定义在 kth71xx.c, 不在此文件.
 */
#include "main.h"

/* ================================================================
 * §1  硬件采样 — ADC 电流/母线 + 零偏校准
 * ================================================================ */
volatile PhaseCurrentQ15_t g_M1Current;
volatile PhaseCurrentQ15_t g_M2Current;
volatile uint16_t          g_VbusRaw;
uint32_t                   g_Vbus_mV;
volatile uint8_t           g_AdcConvDone = 0;

uint16_t g_Offset_M1Ia = 0x8000U;
uint16_t g_Offset_M1Ib = 0x8000U;
uint16_t g_Offset_M2Ia = 0x8000U;
uint16_t g_Offset_M2Ib = 0x8000U;

/* ================================================================
 * §2  FOC 信号链 — 电角度 / dq 电流 / dq 电压 / αβ 输出
 * ================================================================ */
volatile int16_t g_M1_ElecAngle_Q15 = 0;
volatile int16_t g_M2_ElecAngle_Q15 = 0;
volatile int16_t g_M1_AngleDelta = 0;
volatile int16_t g_M2_AngleDelta = 0;
int16_t g_M1_AngleDelta_Target = 0;
int16_t g_M2_AngleDelta_Target = 0;
int16_t g_M1_OpenLoop_Vq = 0;
int16_t g_M2_OpenLoop_Vq = 0;
int16_t g_M1_Iq_Ref_mA = 50;
int16_t g_M2_Iq_Ref_mA = 100;

volatile DQ_Q15_t g_M1_Idq;
volatile DQ_Q15_t g_M2_Idq;
volatile DQ_Q15_t g_M1_Idq_Ref = { .d = 0, .q = 0 };
volatile DQ_Q15_t g_M2_Idq_Ref = { .d = 0, .q = 0 };
volatile DQ_Q15_t g_M1_Vdq;
volatile DQ_Q15_t g_M2_Vdq;
volatile AlphaBeta_Q15_t g_M1_Vab;
volatile AlphaBeta_Q15_t g_M2_Vab;

/* ================================================================
 * §3  电流环 PI (Q15 定点, 17kHz ADC ISR)
 *     极零对消: τ=L/R, kp/ki=L/(R×Ts), 校准后 Flash 持久化
 * ================================================================ */
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

PI_BaseGains_t g_M1_PI_Base = { .kp = PI_M1_KP_NOM, .ki = PI_M1_KI_NOM, .vbus_mv = VBUS_NOMINAL_MV };
PI_BaseGains_t g_M2_PI_Base = { .kp = PI_M2_KP_NOM, .ki = PI_M2_KI_NOM, .vbus_mv = VBUS_NOMINAL_MV };

volatile int16_t g_M1_ElecAngleOffset = 0;
volatile int16_t g_M2_ElecAngleOffset = 0;
volatile int16_t g_M1_wLs_factor = 0;
volatile int16_t g_M2_wLs_factor = 0;
volatile uint8_t g_M1_VqSaturated = 0;
volatile uint8_t g_M2_VqSaturated = 0;

/* ================================================================
 * §4  M2 外环控制 — 速度PI / 位置PI-P / 步距角PD / Id自适应
 * ================================================================ */
volatile MotorCtrlMode_t g_M2_CtrlMode = MODE_OPEN_LOOP;
volatile int16_t g_M2_RPM_Cmd = 0;
int16_t g_M2_RpmRampRate = M2_RPM_RAMP_RATE;
int16_t g_M2_Id_Hold_mA = 50;

SpeedPI_Float_t g_M2_SpeedPI = { .kp = 0.5f, .ki = 0.01f,
                                  .integral = 0.0f,
                                  .integ_limit = 100.0f,
                                  .out_limit = 300.0f };

PosPID_Float_t g_M2_StepAnglePD = { .kp = 0.1f,  .kd = 0.8f, .ki = 0.01f,
                                     .integral = 0.0f,
                                     .integ_limit = 50.0f,
                                     .out_limit = 300.0f };
volatile int32_t g_M2_StepAngle_Ref = 0;
volatile int32_t g_M2_StepAngle_Err = 0;
volatile int16_t g_M2_StepAngle_LossThresh = 5461;

PosPID_Float_t g_M2_PosPID = { .kp = 0.1f, .ki = 0.01f, .kd = 0.0f,
                                .integral = 0.0f,
                                .integ_limit = 300.0f,
                                .out_limit = 800.0f };
volatile int32_t g_M2_PosCmd = 0;
volatile int32_t g_M2_PosFbk = 0;

int16_t g_M2_Id_Standby_mA      = 25;
int16_t g_M2_Id_StandbyDelay_ms = 500;
int16_t g_M2_Id_StillDeltaMax   = 18;
int16_t g_M2_Id_Boost_mA        = 100;
int16_t g_M2_Id_BoostEnter      = 500;
int16_t g_M2_Id_BoostExit       = 250;
volatile int16_t g_M2_Id_Eff_mA = 0;

/* ================================================================
 * §5  M1 外环控制 (与 M2 镜像)
 * ================================================================ */
volatile MotorCtrlMode_t g_M1_CtrlMode = MODE_OPEN_LOOP;
volatile int16_t g_M1_RPM_Cmd = 400;
int16_t g_M1_RpmRampRate = M1_RPM_RAMP_RATE;
int16_t g_M1_Id_Hold_mA = 50;

SpeedPI_Float_t g_M1_SpeedPI = { .kp = 0.5f, .ki = 0.01f,
                                  .integral = 0.0f,
                                  .integ_limit = 100.0f,
                                  .out_limit = 300.0f };

PosPID_Float_t g_M1_StepAnglePD = { .kp = 0.1f,  .kd = 0.8f, .ki = 0.01f,
                                     .integral = 0.0f,
                                     .integ_limit = 50.0f,
                                     .out_limit = 300.0f };
volatile int32_t g_M1_StepAngle_Ref = 0;
volatile int32_t g_M1_StepAngle_Err = 0;
volatile int16_t g_M1_StepAngle_LossThresh = 5461;

PosPID_Float_t g_M1_PosPID = { .kp = 0.1f, .ki = 0.01f, .kd = 0.0f,
                                .integral = 0.0f,
                                .integ_limit = 300.0f,
                                .out_limit = 800.0f };
volatile int32_t g_M1_PosCmd = 0;
volatile int32_t g_M1_PosFbk = 0;

int16_t g_M1_Id_Standby_mA      = 25;
int16_t g_M1_Id_StandbyDelay_ms = 500;
int16_t g_M1_Id_StillDeltaMax   = 18;
int16_t g_M1_Id_Boost_mA        = 100;
int16_t g_M1_Id_BoostEnter      = 500;
int16_t g_M1_Id_BoostExit       = 250;
volatile int16_t g_M1_Id_Eff_mA = 0;

/* ================================================================
 * §6  校准 — 电流环自整定 + 电角度零点标定
 * ================================================================ */
volatile uint8_t  g_DoCurrLoopCalib_M2 = 0;
volatile uint8_t  g_CurrLoopCalibInProgress_M2 = 0;
volatile uint8_t  g_CurrLoopCalibResultReady_M2 = 0;
volatile uint8_t  g_CalibVdDirectActive_M2 = 0;
volatile int16_t  g_CalibVdDirect_M2 = 0;
volatile int32_t  g_CalibVdAcc_M2  = 0;
volatile int32_t  g_CalibIdAcc_M2  = 0;
volatile uint16_t g_CalibAccCnt_M2 = 0;

volatile uint8_t  g_DoCurrLoopCalib_M1 = 0;
volatile uint8_t  g_CurrLoopCalibInProgress_M1 = 0;
volatile uint8_t  g_CurrLoopCalibResultReady_M1 = 0;
volatile uint8_t  g_CalibVdDirectActive_M1 = 0;
volatile int16_t  g_CalibVdDirect_M1 = 0;
volatile int32_t  g_CalibVdAcc_M1  = 0;
volatile int32_t  g_CalibIdAcc_M1  = 0;
volatile uint16_t g_CalibAccCnt_M1 = 0;
volatile uint8_t  g_DoZeroCalib_M1 = 0;

volatile uint8_t  g_FlashParamsErase  = 0;
volatile uint8_t  g_FlashParamsLoaded = 0;

/* ================================================================
 * §7  调试 & VOFA — 阶跃录制 / ISR→VOFA 中间变量 / DMA 帧
 * ================================================================ */
volatile uint8_t  g_StepCapState = 0;
volatile uint8_t  g_StepCapArm_M1 = 0;
volatile uint8_t  g_StepCapArm_M2 = 0;
volatile uint8_t  g_StepCapMotor  = 1;
volatile uint8_t  g_StepCapPhase = 0;
volatile uint16_t g_StepCapIdx   = 0;
volatile int16_t  g_StepCapTarget_mA = 200;
StepCapSample_t   g_StepCapBuf[STEP_CAP_DEPTH];

DebugCurrent_t g_DebugCurrent;
volatile SinCos_Q15_t g_DbgSinCos_M2;

volatile float g_Dbg_Spd_RpmCmd = 0.0f;
volatile float g_Dbg_Spd_Err    = 0.0f;
volatile float g_Dbg_Spd_IqOut  = 0.0f;
volatile float g_Dbg_Pos_iqmA   = 0.0f;
volatile float g_Dbg_Pos_pTerm  = 0.0f;
volatile float g_Dbg_Pos_dTerm  = 0.0f;

volatile float g_Dbg_M1_Spd_RpmCmd = 0.0f;
volatile float g_Dbg_M1_Spd_Err    = 0.0f;
volatile float g_Dbg_M1_Spd_IqOut  = 0.0f;
volatile float g_Dbg_M1_Pos_iqmA   = 0.0f;
volatile float g_Dbg_M1_Pos_pTerm  = 0.0f;
volatile float g_Dbg_M1_Pos_dTerm  = 0.0f;

VofaFrame_t g_VofaFrame = { .ch = {0}, .tail = 0x7F800000U };
