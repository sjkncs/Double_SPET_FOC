/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kth71xx.h"              /* g_Kth71CalibActive: 校准期间暂停编码器 SPI */
#include "calib_platform_m2.h"    /* CalibM2_OnTIM3_1ms: M2 电流环自校准 1ms 驱动 */
#include "calib_platform_m1.h"    /* CalibM1_OnTIM3_1ms: M1 电流环自校准 1ms 驱动 */
#include "vofa_engine.h"          /* Vofa_OnTIM4_1ms: VOFA 1kHz 自动发送 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 前向声明: 定义在 USER CODE BEGIN 1 (文件末尾), 被 ADC1_2_IRQHandler 调用 */
static inline void M1_UpdateElecAngle(void);
static inline void M2_UpdateElecAngle(void);
static inline int16_t CalcSpeedDpp(int16_t *prev, int16_t current);
static inline void StepCap_Record(void);
static void M1_ControlLoop(int16_t d1);
static void M2_ControlLoop(int16_t d2);

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  if (LL_DMA_IsActiveFlag_TC1(DMA1))
  {
      LL_DMA_ClearFlag_TC1(DMA1);
      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  }
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	LED_ON;
  /* USER CODE END ADC1_2_IRQn 0 */
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* ADC1 JEOS触发进入, 等待ADC2和ADC3也完成
   * 三者由同一HRTIM_TRG2触发、相同采样+转换时间, 应几乎同时完成
   * 超时保护: 100次空循环 ≈ 几百ns, 正常情况下0~几次即可通过 */
  uint32_t wait_count = 0;
  while (!((ADC1->ISR & ADC_ISR_JEOS) &&(ADC2->ISR & ADC_ISR_JEOS) &&(ADC3->ISR & ADC_ISR_JEOS))){
      if (++wait_count > 100)
      {
          break;  /* 硬件异常保护, 正常不应触发 */
      }
  }
  /* ---- FOC电流环 (采样 → Park → PI → 逆Park → PWM) ---- */
  FOC_GetPhaseCurrent();

  /* ---- 编码器 SPI (校准期间暂停, 避免 SPI 总线竞争) ---- */
  if (!g_Kth71CalibActive)
  {
      KTH7111_ReadDual(&g_Enc1_Hw, &g_Enc2_Hw);
  }

  /* 读编码器耗时8.5us  - */

  /* ---- 电角度更新 (校准/开环/步距角/闭环 四路分发) ---- */
  M1_UpdateElecAngle();
  M2_UpdateElecAngle();

  /* 电角速度估算: 逆Park延迟补偿 + dq解耦 (int16 自然回绕) */
  static int16_t m1_elec_prev = 0, m2_elec_prev = 0;
  int16_t m1_speed_dpp = CalcSpeedDpp(&m1_elec_prev, g_M1_ElecAngle_Q15);
  int16_t m2_speed_dpp = CalcSpeedDpp(&m2_elec_prev, g_M2_ElecAngle_Q15);


  /* CORDIC: Park 变换用当前电角度 */
  SinCos_Q15_t sc_m1, sc_m2;
  FOC_CalcSinCos(g_M1_ElecAngle_Q15, &sc_m1);
  FOC_CalcSinCos(g_M2_ElecAngle_Q15, &sc_m2);
//  g_DbgSinCos_M2 = sc_m2;  /* DEBUG: Live Watch查看cos/sin是否正确 */


  /* Park变换: αβ→DQ (2相步进电机无需Clarke, Iα=Ia, Iβ=Ib) */
  FOC_ParkTransform(&sc_m1, &g_M1Current, &g_M1_Idq);
  FOC_ParkTransform(&sc_m2, &g_M2Current, &g_M2_Idq);


  /* 电流环阶跃响应录制 (state==1 时采集, 录满后 state→2 触发回放) */
  StepCap_Record();

  /* ---- 电流闭环 ---- */
  /* M1: 校准时开环电压注入（跳过 PI），正常时 PI 闭环 */
  if (g_CalibVdDirectActive_M1) {
      g_M1_Vdq.d = g_CalibVdDirect_M1;
      g_M1_Vdq.q = 0;
  } else {
      g_M1_Vdq.d = FOC_PI_Run(&g_M1_PI_d, g_M1_Idq_Ref.d, g_M1_Idq.d);
      g_M1_Vdq.q = FOC_PI_Run(&g_M1_PI_q, g_M1_Idq_Ref.q, g_M1_Idq.q);
  }
  /* M2: 校准时开环电压注入（跳过 PI），正常时 PI 闭环 */
  if (g_CalibVdDirectActive_M2) {
      g_M2_Vdq.d = g_CalibVdDirect_M2;
      g_M2_Vdq.q = 0;
  } else {
      g_M2_Vdq.d = FOC_PI_Run(&g_M2_PI_d, g_M2_Idq_Ref.d, g_M2_Idq.d);
      g_M2_Vdq.q = FOC_PI_Run(&g_M2_PI_q, g_M2_Idq_Ref.q, g_M2_Idq.q);
  }


  /* dq 前馈解耦 (开环自动跳过) */
  FOC_DecoupleFF(&g_M1_Vdq, m1_speed_dpp, &g_M1_Idq, g_M1_wLs_factor, g_M1_CtrlMode);
  FOC_DecoupleFF(&g_M2_Vdq, m2_speed_dpp, &g_M2_Idq, g_M2_wLs_factor, g_M2_CtrlMode);

  /* 电压圆饱和诊断 (限幅前检测, VOFA I9 报警) */
  g_M1_VqSaturated = FOC_IsVqSaturated(&g_M1_Vdq);
  g_M2_VqSaturated = FOC_IsVqSaturated(&g_M2_Vdq);

  /* 圆限幅: Vq²+Vd² ≤ MaxModule² (防逆变器过调制, 防逆Park int16溢出) */
  FOC_CircleLimitation(&g_M1_Vdq);
  FOC_CircleLimitation(&g_M2_Vdq);

  /* 电流环自校准: 累加 Vd/Id（TIM3 每 1ms 读平均并清零） */
  if (g_CurrLoopCalibInProgress_M2) {
      g_CalibVdAcc_M2 += (int32_t)g_M2_Vdq.d;
      g_CalibIdAcc_M2 += (int32_t)g_M2_Idq.d;
      g_CalibAccCnt_M2++;
  }
  if (g_CurrLoopCalibInProgress_M1) {
      g_CalibVdAcc_M1 += (int32_t)g_M1_Vdq.d;
      g_CalibIdAcc_M1 += (int32_t)g_M1_Idq.d;
      g_CalibAccCnt_M1++;
  }


  /* 逆Park + 延迟补偿: 电角度前推 1 个 ISR 周期补偿 ADC→PWM 延迟 */
  FOC_InvParkWithComp(g_M1_ElecAngle_Q15, m1_speed_dpp, &g_M1_Vdq, &g_M1_Vab);
  FOC_InvParkWithComp(g_M2_ElecAngle_Q15, m2_speed_dpp, &g_M2_Vdq, &g_M2_Vab);


  /* αβ电压 → PWM占空比 */
  FOC_SetPhasePWM(M1_TIMER_PHASE_A, M1_TIMER_PHASE_B, &g_M1_Vab);
  FOC_SetPhasePWM(M2_TIMER_PHASE_A, M2_TIMER_PHASE_B, &g_M2_Vab);

  /* 清除3个ADC的JEOS标志 (必须在读取JDR之后) */
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_ClearFlag_JEOS(ADC2);
  LL_ADC_ClearFlag_JEOS(ADC3);
  LED_OFF;
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  LL_TIM_ClearFlag_UPDATE(TIM3);

  /* 电流环自校准：每 1ms 执行一步 */
  CalibM2_OnTIM3_1ms();       /* M2 */
  CalibM1_OnTIM3_1ms();       /* M1 */

  /* ---- 速度环 @1kHz (速度计算 + IIR滤波 + PI控制) ---- */
  static uint16_t enc1_prev = 0, enc2_prev = 0;
  static int32_t  filt_acc1 = 0, filt_acc2 = 0;
  static uint8_t  calib_was_active = 0;

  /* KTH71 校准期间: 暂停速度计算 (VOFA 由 TIM4 vofa_engine 自动处理) */
  if (g_Kth71CalibActive) {
      calib_was_active = 1;
      return;
  }

  /* 校准结束首次: 重置基准, 消除恢复跳变 */
  if (calib_was_active) {
      enc1_prev = g_Enc1_Angle;
      enc2_prev = g_Enc2_Angle;
      g_Enc1_SpeedRPM  = 0;  g_Enc2_SpeedRPM  = 0;
      g_Enc1_SpeedFilt = 0;  g_Enc2_SpeedFilt = 0;
      filt_acc1 = 0;  filt_acc2 = 0;
      calib_was_active = 0;
  }

  /* ---- 速度计算: Δangle × 60000 >> 16 → RPM ---- */
  int16_t d1 = (int16_t)(g_Enc1_Angle - enc1_prev) * M1_ENC_DIR;
  int16_t d2 = (int16_t)(g_Enc2_Angle - enc2_prev);
  enc1_prev = g_Enc1_Angle;
  enc2_prev = g_Enc2_Angle;

  /* 编码器增量防护: 限制单周期最大变化量, 防毛刺导致速度脉冲
   * 3000 counts/ms ≈ 2745 RPM (65536 counts/rev), 远超物理极限
   * 乘法溢出安全: |d|≤3000, |d×60000|=1.8e8 < INT32_MAX ✓ */
  #define ENC_DELTA_MAX  3000
  if (d1 >  ENC_DELTA_MAX) d1 =  ENC_DELTA_MAX;
  if (d1 < -ENC_DELTA_MAX) d1 = -ENC_DELTA_MAX;
  if (d2 >  ENC_DELTA_MAX) d2 =  ENC_DELTA_MAX;
  if (d2 < -ENC_DELTA_MAX) d2 = -ENC_DELTA_MAX;

  g_Enc1_SpeedRPM = ((int32_t)d1 * 60000L) >> 16;
  g_Enc2_SpeedRPM = ((int32_t)d2 * 60000L) >> 16;

  /* 一阶IIR低通: α=1/4, fc≈40Hz @1kHz */
  filt_acc1 += g_Enc1_SpeedRPM - (filt_acc1 >> 2);
  filt_acc2 += g_Enc2_SpeedRPM - (filt_acc2 >> 2);
  g_Enc1_SpeedFilt = filt_acc1 >> 2;
  g_Enc2_SpeedFilt = filt_acc2 >> 2;

  /* ---- 位置累计器: 永远运行, 65536 counts = 1 圈 ---- */
  g_M1_PosFbk += (int32_t)d1;
  g_M2_PosFbk += (int32_t)d2;

  /* ---- M1 / M2 控制环 (Id自适应 + 位置PI + 步距角PD + 速度PI) ---- */
  M1_ControlLoop(d1);
  M2_ControlLoop(d2);
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  LL_TIM_ClearFlag_UPDATE(TIM4);
  Vofa_OnTIM4_1ms();
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles HRTIM master timer global interrupt.
  */
void HRTIM1_Master_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_Master_IRQn 0 */

  /* USER CODE END HRTIM1_Master_IRQn 0 */
  /* USER CODE BEGIN HRTIM1_Master_IRQn 1 */
  LL_HRTIM_ClearFlag_UPDATE(HRTIM1, LL_HRTIM_TIMER_MASTER);

  /* 仅用于ADC_CalibrateOffset()的同步信号, 校准完成后无实际作用 */ ///////////////////////////////////////////////校准完成是不是可以关了中断？
  g_AdcConvDone = 1;


  /* USER CODE END HRTIM1_Master_IRQn 1 */
}

/**
  * @brief This function handles HRTIM fault global interrupt.
  */
void HRTIM1_FLT_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_FLT_IRQn 0 */

  /* USER CODE END HRTIM1_FLT_IRQn 0 */
  /* USER CODE BEGIN HRTIM1_FLT_IRQn 1 */
  /* 清除 HRTIM1 故障中断标志 (FLT1~FLT6) */
  LL_HRTIM_ClearFlag_FLT1(HRTIM1);

  LL_HRTIM_ClearFlag_FLT4(HRTIM1);

  /* USER CODE END HRTIM1_FLT_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* ================================================================
 * M1 控制环 (TIM3 1kHz 调用, 与 M2 镜像结构)
 * 输入: g_Enc1_SpeedFilt, g_M1_PosFbk, d1 (编码器增量)
 * 输出: g_M1_Idq_Ref
 * ================================================================ */
static void M1_ControlLoop(int16_t d1)
{
    /* ---- Id 三级自适应 ---- */
    {
        static int16_t id_still_cnt = 0;
        static uint8_t id_in_boost  = 0;
        int16_t id_eff;
        int16_t abs_spd = g_Enc1_SpeedFilt;
        if (abs_spd < 0) abs_spd = -abs_spd;
        int16_t abs_d = d1;
        if (abs_d < 0) abs_d = -abs_d;

        if (abs_spd < 5 && abs_d <= g_M1_Id_StillDeltaMax) {
            if (id_still_cnt < g_M1_Id_StandbyDelay_ms)
                id_still_cnt++;
            id_eff = (id_still_cnt >= g_M1_Id_StandbyDelay_ms)
                     ? g_M1_Id_Standby_mA : g_M1_Id_Hold_mA;
        } else {
            id_still_cnt = 0;
            id_eff = g_M1_Id_Hold_mA;
        }

        if (g_M1_CtrlMode == MODE_STEP_ANGLE
         || g_M1_CtrlMode == MODE_POSITION) {
            int32_t abs_err = g_M1_StepAngle_Err;
            if (g_M1_CtrlMode == MODE_POSITION)
                abs_err = g_M1_PosCmd - g_M1_PosFbk;
            if (abs_err < 0) abs_err = -abs_err;

            if (!id_in_boost && abs_err > (int32_t)g_M1_Id_BoostEnter)
                id_in_boost = 1;
            else if (id_in_boost && abs_err < (int32_t)g_M1_Id_BoostExit)
                id_in_boost = 0;

            if (id_in_boost) {
                id_eff = g_M1_Id_Boost_mA;
                id_still_cnt = 0;
            }
        } else {
            id_in_boost = 0;
        }
        g_M1_Id_Eff_mA = id_eff;
    }

    /* ---- 位置环 外环: PI → 速度参考 (PI-P 结构) ---- */
    static uint8_t m1_pos_was_active = 0;
    float m1_pos_spd_ref = 0.0f;

    if (g_M1_CtrlMode == MODE_POSITION) {
        if (!m1_pos_was_active) {
            g_M1_PosCmd = g_M1_PosFbk;
            g_M1_PosPID.integral = 0.0f;
            m1_pos_was_active = 1;
        }
        int32_t pos_err = g_M1_PosCmd - g_M1_PosFbk;
        if (pos_err >  30000) pos_err =  30000;
        if (pos_err < -30000) pos_err = -30000;

        float p_out = g_M1_PosPID.kp * (float)pos_err;
        float i_out = g_M1_PosPID.integral;
        if (g_M1_PosPID.ki != 0.0f) {
            if ((i_out > 0.0f && pos_err < 0) ||
                (i_out < 0.0f && pos_err > 0))
                i_out = 0.0f;
            i_out += g_M1_PosPID.ki * (float)pos_err;
            if (i_out >  g_M1_PosPID.integ_limit) i_out =  g_M1_PosPID.integ_limit;
            if (i_out < -g_M1_PosPID.integ_limit) i_out = -g_M1_PosPID.integ_limit;
        }
        m1_pos_spd_ref = p_out + i_out;
        if (m1_pos_spd_ref > g_M1_PosPID.out_limit) {
            i_out -= (m1_pos_spd_ref - g_M1_PosPID.out_limit);
            m1_pos_spd_ref = g_M1_PosPID.out_limit;
        } else if (m1_pos_spd_ref < -g_M1_PosPID.out_limit) {
            i_out -= (m1_pos_spd_ref + g_M1_PosPID.out_limit);
            m1_pos_spd_ref = -g_M1_PosPID.out_limit;
        }
        g_M1_PosPID.integral = i_out;
        g_M1_PosPID.output = m1_pos_spd_ref;
        g_Dbg_M1_Pos_iqmA  = m1_pos_spd_ref;
        g_Dbg_M1_Pos_pTerm = (float)pos_err;
    } else {
        if (m1_pos_was_active) g_M1_PosPID.integral = 0.0f;
        m1_pos_was_active = 0;
    }

    /* ---- 步距角闭环: 直接 PD→Iq ---- */
    static uint8_t m1_step_was_active = 0;

    if (g_M1_CtrlMode == MODE_STEP_ANGLE) {
        if (!m1_step_was_active) {
            g_M1_StepAngle_Ref = g_M1_PosFbk;
            g_M1_StepAnglePD.integral = 0.0f;
            g_M1_PI_d.integral = 0;
            g_M1_PI_q.integral = 0;
            g_M1_AngleDelta = 0;
            m1_step_was_active = 1;
        }
        int32_t pos_err = g_M1_StepAngle_Ref - g_M1_PosFbk;
        if (pos_err >  30000) pos_err =  30000;
        if (pos_err < -30000) pos_err = -30000;
        g_M1_StepAngle_Err = pos_err;

        float p_term = g_M1_StepAnglePD.kp * (float)pos_err;
        float d_term = g_M1_StepAnglePD.kd
                     * ((float)g_Enc1_SpeedFilt - (float)g_M1_RPM_Cmd);

        float i_term = g_M1_StepAnglePD.integral;
        if (g_M1_StepAnglePD.ki != 0.0f) {
            if ((i_term > 0.0f && pos_err < 0) ||
                (i_term < 0.0f && pos_err > 0))
                i_term = 0.0f;
            i_term += g_M1_StepAnglePD.ki * (float)pos_err;
            if (i_term >  g_M1_StepAnglePD.integ_limit) i_term =  g_M1_StepAnglePD.integ_limit;
            if (i_term < -g_M1_StepAnglePD.integ_limit) i_term = -g_M1_StepAnglePD.integ_limit;
        } else {
            i_term = 0.0f;
        }

        float iq_mA = p_term - d_term + i_term;
        if (iq_mA > g_M1_StepAnglePD.out_limit) {
            i_term -= (iq_mA - g_M1_StepAnglePD.out_limit);
            iq_mA = g_M1_StepAnglePD.out_limit;
        } else if (iq_mA < -g_M1_StepAnglePD.out_limit) {
            i_term -= (iq_mA + g_M1_StepAnglePD.out_limit);
            iq_mA = -g_M1_StepAnglePD.out_limit;
        }
        g_M1_StepAnglePD.integral = i_term;
        g_M1_StepAnglePD.output   = iq_mA;

        g_Dbg_M1_Spd_RpmCmd = (float)(g_M1_StepAngle_Ref / 65536);
        g_Dbg_M1_Spd_Err    = (float)pos_err;
        g_Dbg_M1_Spd_IqOut  = iq_mA;

        __disable_irq();
        g_M1_Idq_Ref.d = MA_TO_Q15(g_M1_Id_Eff_mA);
        g_M1_Idq_Ref.q = (int16_t)(iq_mA * MA_TO_Q15_F);
        __enable_irq();
    } else {
        m1_step_was_active = 0;
    }

    /* ---- 速度闭环 PI (速度模式PI / 位置模式P-only内环) ---- */
    static uint8_t m1_spd_was_active = 0;
    static int16_t m1_rpm_cmd_ramped = 0;

    if (g_M1_CtrlMode == MODE_SPEED
     || g_M1_CtrlMode == MODE_POSITION) {

        if (!m1_spd_was_active) {
            g_M1_SpeedPI.integral = 0.0f;
            g_M1_PI_d.integral = 0;
            g_M1_PI_q.integral = 0;
            m1_rpm_cmd_ramped = (int16_t)g_Enc1_SpeedFilt;
            m1_spd_was_active = 1;
        }

        float rpm_target;
        if (g_M1_CtrlMode == MODE_POSITION) {
            rpm_target = m1_pos_spd_ref;
        } else {
            int16_t target = g_M1_RPM_Cmd;
            int16_t ramp   = g_M1_RpmRampRate;
            if (ramp < 1) ramp = 1;
            if (m1_rpm_cmd_ramped < target) {
                m1_rpm_cmd_ramped += ramp;
                if (m1_rpm_cmd_ramped > target) m1_rpm_cmd_ramped = target;
            } else if (m1_rpm_cmd_ramped > target) {
                m1_rpm_cmd_ramped -= ramp;
                if (m1_rpm_cmd_ramped < target) m1_rpm_cmd_ramped = target;
            }
            rpm_target = (float)m1_rpm_cmd_ramped;
            g_Dbg_M1_Spd_RpmCmd = rpm_target;
        }

        float iq_mA;
        if (g_M1_CtrlMode == MODE_SPEED) {
            iq_mA = SpeedPI_Run(&g_M1_SpeedPI, rpm_target,
                                (float)g_Enc1_SpeedFilt);
        } else {
            float spd_err = rpm_target - (float)g_Enc1_SpeedFilt;
            iq_mA = g_M1_SpeedPI.kp * spd_err;
            if (iq_mA >  g_M1_SpeedPI.out_limit) iq_mA =  g_M1_SpeedPI.out_limit;
            if (iq_mA < -g_M1_SpeedPI.out_limit) iq_mA = -g_M1_SpeedPI.out_limit;
            g_M1_SpeedPI.integral = 0.0f;
            g_M1_SpeedPI.output   = iq_mA;
        }

        if (g_M1_CtrlMode == MODE_SPEED) {
            g_Dbg_M1_Spd_Err = rpm_target - (float)g_Enc1_SpeedFilt;
        } else {
            g_Dbg_M1_Pos_dTerm = iq_mA;
        }
        g_Dbg_M1_Spd_IqOut = iq_mA;

        __disable_irq();
        g_M1_Idq_Ref.d = MA_TO_Q15(g_M1_Id_Eff_mA);
        g_M1_Idq_Ref.q = (int16_t)(iq_mA * MA_TO_Q15_F);
        __enable_irq();
    } else if (g_M1_CtrlMode != MODE_STEP_ANGLE) {
        m1_spd_was_active = 0;
        m1_rpm_cmd_ramped = 0;
    }
}

/* ================================================================
 * M2 控制环 (TIM3 1kHz 调用, 与 M1 镜像结构)
 * 输入: g_Enc2_SpeedFilt, g_M2_PosFbk, d2 (编码器增量)
 * 输出: g_M2_Idq_Ref
 * ================================================================ */
static void M2_ControlLoop(int16_t d2)
{
    /* ---- Id 三级自适应 (Standby / Hold / Boost + 迟滞) ---- */
    {
        static int16_t id_still_cnt = 0;
        static uint8_t id_in_boost  = 0;
        int16_t id_eff;
        int16_t abs_spd = g_Enc2_SpeedFilt;
        if (abs_spd < 0) abs_spd = -abs_spd;
        int16_t abs_d = d2;
        if (abs_d < 0) abs_d = -abs_d;

        if (abs_spd < 5 && abs_d <= g_M2_Id_StillDeltaMax) {
            if (id_still_cnt < g_M2_Id_StandbyDelay_ms)
                id_still_cnt++;
            id_eff = (id_still_cnt >= g_M2_Id_StandbyDelay_ms)
                     ? g_M2_Id_Standby_mA : g_M2_Id_Hold_mA;
        } else {
            id_still_cnt = 0;
            id_eff = g_M2_Id_Hold_mA;
        }

        if (g_M2_CtrlMode == MODE_STEP_ANGLE
         || g_M2_CtrlMode == MODE_POSITION) {
            int32_t abs_err = g_M2_StepAngle_Err;
            if (g_M2_CtrlMode == MODE_POSITION)
                abs_err = g_M2_PosCmd - g_M2_PosFbk;
            if (abs_err < 0) abs_err = -abs_err;

            if (!id_in_boost && abs_err > (int32_t)g_M2_Id_BoostEnter)
                id_in_boost = 1;
            else if (id_in_boost && abs_err < (int32_t)g_M2_Id_BoostExit)
                id_in_boost = 0;

            if (id_in_boost) {
                id_eff = g_M2_Id_Boost_mA;
                id_still_cnt = 0;
            }
        } else {
            id_in_boost = 0;
        }
        g_M2_Id_Eff_mA = id_eff;
    }

    /* ---- 位置环 外环: PI → 速度参考 (PI-P 结构) ---- */
    static uint8_t pos_was_active = 0;
    float pos_spd_ref = 0.0f;

    if (g_M2_CtrlMode == MODE_POSITION) {
        if (!pos_was_active) {
            g_M2_PosCmd = g_M2_PosFbk;
            g_M2_PosPID.integral = 0.0f;
            pos_was_active = 1;
        }
        int32_t pos_err = g_M2_PosCmd - g_M2_PosFbk;
        if (pos_err >  30000) pos_err =  30000;
        if (pos_err < -30000) pos_err = -30000;

        float p_out = g_M2_PosPID.kp * (float)pos_err;
        float i_out = g_M2_PosPID.integral;
        if (g_M2_PosPID.ki != 0.0f) {
            if ((i_out > 0.0f && pos_err < 0) ||
                (i_out < 0.0f && pos_err > 0))
                i_out = 0.0f;
            i_out += g_M2_PosPID.ki * (float)pos_err;
            if (i_out >  g_M2_PosPID.integ_limit) i_out =  g_M2_PosPID.integ_limit;
            if (i_out < -g_M2_PosPID.integ_limit) i_out = -g_M2_PosPID.integ_limit;
        }
        pos_spd_ref = p_out + i_out;
        if (pos_spd_ref > g_M2_PosPID.out_limit) {
            i_out -= (pos_spd_ref - g_M2_PosPID.out_limit);
            pos_spd_ref = g_M2_PosPID.out_limit;
        } else if (pos_spd_ref < -g_M2_PosPID.out_limit) {
            i_out -= (pos_spd_ref + g_M2_PosPID.out_limit);
            pos_spd_ref = -g_M2_PosPID.out_limit;
        }
        g_M2_PosPID.integral = i_out;
        g_M2_PosPID.output = pos_spd_ref;
        g_Dbg_Pos_iqmA  = pos_spd_ref;
        g_Dbg_Pos_pTerm = (float)pos_err;
    } else {
        if (pos_was_active) g_M2_PosPID.integral = 0.0f;
        pos_was_active = 0;
    }

    /* ---- 步距角闭环: 直接 PD→Iq ---- */
    static uint8_t step_was_active = 0;

    if (g_M2_CtrlMode == MODE_STEP_ANGLE) {
        if (!step_was_active) {
            g_M2_StepAngle_Ref = g_M2_PosFbk;
            g_M2_StepAnglePD.integral = 0.0f;
            g_M2_PI_d.integral = 0;
            g_M2_PI_q.integral = 0;
            g_M2_AngleDelta = 0;
            step_was_active = 1;
        }
        int32_t pos_err = g_M2_StepAngle_Ref - g_M2_PosFbk;
        if (pos_err >  30000) pos_err =  30000;
        if (pos_err < -30000) pos_err = -30000;
        g_M2_StepAngle_Err = pos_err;

        float p_term = g_M2_StepAnglePD.kp * (float)pos_err;
        float d_term = g_M2_StepAnglePD.kd
                     * ((float)g_Enc2_SpeedFilt - (float)g_M2_RPM_Cmd);

        float i_term = g_M2_StepAnglePD.integral;
        if (g_M2_StepAnglePD.ki != 0.0f) {
            i_term += g_M2_StepAnglePD.ki * (float)pos_err;
            if (i_term >  g_M2_StepAnglePD.integ_limit) i_term =  g_M2_StepAnglePD.integ_limit;
            if (i_term < -g_M2_StepAnglePD.integ_limit) i_term = -g_M2_StepAnglePD.integ_limit;
        } else {
            i_term = 0.0f;
        }

        float iq_mA = p_term - d_term + i_term;
        if (iq_mA > g_M2_StepAnglePD.out_limit) {
            i_term -= (iq_mA - g_M2_StepAnglePD.out_limit);
            iq_mA = g_M2_StepAnglePD.out_limit;
        } else if (iq_mA < -g_M2_StepAnglePD.out_limit) {
            i_term -= (iq_mA + g_M2_StepAnglePD.out_limit);
            iq_mA = -g_M2_StepAnglePD.out_limit;
        }
        g_M2_StepAnglePD.integral = i_term;
        g_M2_StepAnglePD.output   = iq_mA;

        g_Dbg_Spd_RpmCmd = (float)(g_M2_StepAngle_Ref / 65536);
        g_Dbg_Spd_Err    = (float)pos_err;
        g_Dbg_Spd_IqOut  = iq_mA;

        __disable_irq();
        g_M2_Idq_Ref.d = MA_TO_Q15(g_M2_Id_Eff_mA);
        g_M2_Idq_Ref.q = (int16_t)(iq_mA * MA_TO_Q15_F);
        __enable_irq();
    } else {
        step_was_active = 0;
    }

    /* ---- 速度闭环 PI (速度模式PI / 位置模式P-only内环) ---- */
    static uint8_t spd_was_active = 0;
    static int16_t rpm_cmd_ramped = 0;

    if (g_M2_CtrlMode == MODE_SPEED
     || g_M2_CtrlMode == MODE_POSITION) {

        if (!spd_was_active) {
            g_M2_SpeedPI.integral = 0.0f;
            g_M2_PI_d.integral = 0;
            g_M2_PI_q.integral = 0;
            rpm_cmd_ramped = (int16_t)g_Enc2_SpeedFilt;
            spd_was_active = 1;
        }

        float rpm_target;
        if (g_M2_CtrlMode == MODE_POSITION) {
            rpm_target = pos_spd_ref;
        } else {
            int16_t target = g_M2_RPM_Cmd;
            int16_t ramp   = g_M2_RpmRampRate;
            if (ramp < 1) ramp = 1;
            if (rpm_cmd_ramped < target) {
                rpm_cmd_ramped += ramp;
                if (rpm_cmd_ramped > target) rpm_cmd_ramped = target;
            } else if (rpm_cmd_ramped > target) {
                rpm_cmd_ramped -= ramp;
                if (rpm_cmd_ramped < target) rpm_cmd_ramped = target;
            }
            rpm_target = (float)rpm_cmd_ramped;
            g_Dbg_Spd_RpmCmd = rpm_target;
        }

        float iq_mA;
        if (g_M2_CtrlMode == MODE_SPEED) {
            iq_mA = SpeedPI_Run(&g_M2_SpeedPI, rpm_target,
                                (float)g_Enc2_SpeedFilt);
        } else {
            float spd_err = rpm_target - (float)g_Enc2_SpeedFilt;
            iq_mA = g_M2_SpeedPI.kp * spd_err;
            if (iq_mA >  g_M2_SpeedPI.out_limit) iq_mA =  g_M2_SpeedPI.out_limit;
            if (iq_mA < -g_M2_SpeedPI.out_limit) iq_mA = -g_M2_SpeedPI.out_limit;
            g_M2_SpeedPI.integral = 0.0f;
            g_M2_SpeedPI.output   = iq_mA;
        }

        if (g_M2_CtrlMode == MODE_SPEED) {
            g_Dbg_Spd_Err = rpm_target - (float)g_Enc2_SpeedFilt;
        } else {
            g_Dbg_Pos_dTerm = iq_mA;
        }
        g_Dbg_Spd_IqOut = iq_mA;

        __disable_irq();
        g_M2_Idq_Ref.d = MA_TO_Q15(g_M2_Id_Eff_mA);
        g_M2_Idq_Ref.q = (int16_t)(iq_mA * MA_TO_Q15_F);
        __enable_irq();
    } else if (g_M2_CtrlMode != MODE_STEP_ANGLE) {
        spd_was_active = 0;
        rpm_cmd_ramped = 0;
    }
}

/* ================================================================
 * ADC1_2_IRQHandler 子函数 — 保持 ISR 主体精简可读
 * ================================================================ */

/** @brief  M1 电角度更新 (校准/开环/步距角/闭环 四路分发)
 *
 * 校准中: 锁定 0° (PI 对齐 d 轴)
 * 开环:   斜坡分频递增 delta → 累加角度 (高电感防丢步)
 * 步距角: 编码器角度 × 极对数 (Park 用), 同时递增角度指令 (控制用)
 * 闭环:   纯编码器角度 × 极对数 */
static inline void M1_UpdateElecAngle(void)
{
    if (g_CurrLoopCalibInProgress_M1) {
        return;  /* 保持 0° 电角度 */
    }
    if (g_M1_CtrlMode == MODE_OPEN_LOOP) {
        /* 斜坡分频: delta 每 M1_RAMP_DIV 个 ISR 才 ±1, 角度每 ISR 递增 */
        static uint8_t ramp_cnt = 0;
        if (++ramp_cnt >= M1_RAMP_DIV) {
            ramp_cnt = 0;
            int16_t cur = g_M1_AngleDelta;
            if (cur < g_M1_AngleDelta_Target)       cur++;
            else if (cur > g_M1_AngleDelta_Target)  cur--;
            g_M1_AngleDelta = cur;
        }
        g_M1_ElecAngle_Q15 = (int16_t)((uint16_t)g_M1_ElecAngle_Q15
                            + (uint16_t)g_M1_AngleDelta);
        return;
    }
    /* 步距角/速度/位置: 编码器角度 × 极对数 × 方向 + 偏移 */
    g_M1_ElecAngle_Q15 = (int16_t)((uint16_t)g_Enc1_Angle * M1_POLE_PAIRS)
                        * M1_ENC_DIR + g_M1_ElecAngleOffset;
    if (g_M1_CtrlMode == MODE_STEP_ANGLE)
        M1_OpenLoop_IncAngle_Ref();
}

/** @brief  M2 电角度更新 (逻辑同 M1, 无斜坡分频) */
static inline void M2_UpdateElecAngle(void)
{
    if (g_CurrLoopCalibInProgress_M2) {
        return;
    }
    if (g_M2_CtrlMode == MODE_OPEN_LOOP) {
        OpenLoop_IncAngle(&g_M2_ElecAngle_Q15, &g_M2_AngleDelta, g_M2_AngleDelta_Target);
        return;
    }
    g_M2_ElecAngle_Q15 = (int16_t)((uint16_t)g_Enc2_Angle * M2_POLE_PAIRS)
                        + g_M2_ElecAngleOffset;
    if (g_M2_CtrlMode == MODE_STEP_ANGLE)
        OpenLoop_IncAngle_Ref();
}

/** @brief  电角速度估算: 当前 − 上次 (int16 自然回绕) */
static inline int16_t CalcSpeedDpp(int16_t *prev, int16_t current)
{
    int16_t dpp = current - *prev;
    *prev = current;
    return dpp;
}

/** @brief  电流环阶跃响应录制 (17kHz, M1/M2 由 g_StepCapMotor 选择)
 *
 * Phase0: d轴阶跃 (Id=target, Iq=0)
 * Phase1: q轴阶跃 (Id=0, Iq=target)
 * 录满 STEP_CAP_DEPTH 点后 state→2, 由 while(1) StepCap_Playback 回放 */
static inline void StepCap_Record(void)
{
    if (g_StepCapState != 1) return;

    uint16_t idx = g_StepCapIdx;
    int16_t target_q15 = MA_TO_Q15(g_StepCapTarget_mA);

    /* 指针别名: 按 g_StepCapMotor 指向目标电机变量 */
    volatile DQ_Q15_t *ref   = (g_StepCapMotor == 0) ? &g_M2_Idq_Ref   : &g_M1_Idq_Ref;
    volatile DQ_Q15_t *fbk   = (g_StepCapMotor == 0) ? &g_M2_Idq       : &g_M1_Idq;
    volatile int16_t  *iq_ma = (g_StepCapMotor == 0) ? &g_M2_Iq_Ref_mA : &g_M1_Iq_Ref_mA;

    /* 阶跃施加点: Phase0→d轴, Phase1→q轴 */
    if (idx == STEP_CAP_PRE) {
        ref->d = (g_StepCapPhase == 0) ? target_q15 : 0;
        ref->q = (g_StepCapPhase == 0) ? 0 : target_q15;
    }

    if (idx < STEP_CAP_DEPTH) {
        g_StepCapBuf[idx].id_ref = ref->d;
        g_StepCapBuf[idx].iq_ref = ref->q;
        g_StepCapBuf[idx].id_fbk = fbk->d;
        g_StepCapBuf[idx].iq_fbk = fbk->q;
        g_StepCapIdx = idx + 1;
    } else {
        ref->d = 0;
        ref->q = 0;
        *iq_ma = 0;
        g_StepCapState = 2;
    }
}

/* USER CODE END 1 */
