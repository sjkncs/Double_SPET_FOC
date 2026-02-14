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

  /* 进入中断到FOC_GetPhaseCurrent执行完毕耗时0.8us  - */

  /* ---- 编码器 SPI (校准期间暂停, 避免 SPI 总线竞争) ---- */
  if (!g_Kth71CalibActive)
  {
      KTH7111_ReadDual(&g_Enc1_Hw, &g_Enc2_Hw);
  }

  /* 读编码器耗时8.5us  - */

  /* ---- M1 电角度更新 ---- */
  if (g_CurrLoopCalibInProgress_M1) {
      /* 电流环自校准中: 保持 0° 电角度，不更新 */
  } else if (g_M1_CtrlMode == M2_MODE_OPEN_LOOP) {
      OpenLoop_IncAngle(&g_M1_ElecAngle_Q15, &g_M1_AngleDelta, g_M1_AngleDelta_Target);
  } else if (g_M1_CtrlMode == M2_MODE_STEP_ANGLE) {
      g_M1_ElecAngle_Q15 = (int16_t)((uint16_t)g_Enc1_Angle * M1_POLE_PAIRS)
                         + g_M1_ElecAngleOffset;
      M1_OpenLoop_IncAngle_Ref();
  } else {
      /* 速度/位置闭环: 编码器角度 */
      g_M1_ElecAngle_Q15 = (int16_t)((uint16_t)g_Enc1_Angle * M1_POLE_PAIRS)
                         + g_M1_ElecAngleOffset;
  }

  /* ---- M2 电角度更新 ---- */
  if (g_CurrLoopCalibInProgress) {
      /* 电流环自校准中: 保持 0° 电角度，不更新 */
  } else if (g_M2_CtrlMode == M2_MODE_OPEN_LOOP) {
      /* 开环: 外部步进角度 */
      OpenLoop_IncAngle(&g_M2_ElecAngle_Q15, &g_M2_AngleDelta, g_M2_AngleDelta_Target);
  } else if (g_M2_CtrlMode == M2_MODE_STEP_ANGLE) {
      /* 步距角闭环: Park变换用编码器角度(确保dq正确), 同时递增角度指令 */
      g_M2_ElecAngle_Q15 = (int16_t)((uint16_t)g_Enc2_Angle * M2_POLE_PAIRS)
                         + g_M2_ElecAngleOffset;
      /* 角度指令匀速递增 (17kHz 步进, 速度由 AngleDelta 控制) */
      OpenLoop_IncAngle_Ref();
  } else {
      /* 速度/位置闭环: 编码器机械角 × 极对数 + 偏移补偿 → 电角度 Q15
       * g_M2_ElecAngleOffset 校准: 对齐编码器零点与电机电气零点 */
      g_M2_ElecAngle_Q15 = (int16_t)((uint16_t)g_Enc2_Angle * M2_POLE_PAIRS)
                         + g_M2_ElecAngleOffset;
  }

  /* 电角速度估算: 用于逆Park延迟补偿 + dq解耦
   * dpp = Q15 变化量 per ISR周期, int16_t 自然处理 65536 回绕 */
  static int16_t m1_elec_prev = 0;
  int16_t m1_speed_dpp = g_M1_ElecAngle_Q15 - m1_elec_prev;
  m1_elec_prev = g_M1_ElecAngle_Q15;

  static int16_t m2_elec_prev = 0;
  int16_t m2_speed_dpp = g_M2_ElecAngle_Q15 - m2_elec_prev;
  m2_elec_prev = g_M2_ElecAngle_Q15;

  /* 全开环耗时0.8us  - */

  /* CORDIC: Park 变换用当前电角度 */
  SinCos_Q15_t sc_m1, sc_m2;
  FOC_CalcSinCos(g_M1_ElecAngle_Q15, &sc_m1);
  FOC_CalcSinCos(g_M2_ElecAngle_Q15, &sc_m2);
//  g_DbgSinCos_M2 = sc_m2;  /* DEBUG: Live Watch查看cos/sin是否正确 */

  /* CORDIC耗时0.75us  - */

  /* Park变换: αβ→DQ (2相步进电机无需Clarke, Iα=Ia, Iβ=Ib) */
  FOC_ParkTransform(&sc_m1, &g_M1Current, &g_M1_Idq);
  FOC_ParkTransform(&sc_m2, &g_M2Current, &g_M2_Idq);

  /* Park耗时0.8us  - */

  /* ---- 电流环阶跃响应录制 (17kHz采集) ----
   * Phase0: 阶跃施加到 d轴 (Id_ref = target, Iq_ref = 0)
   * Phase1: 阶跃施加到 q轴 (Id_ref = 0, Iq_ref = target)
   * 两轴由 while 自动衔接, while(1)开环Ref赋值已屏蔽 */
  if (g_StepCapState == 1) {
      uint16_t idx = g_StepCapIdx;

      /* 第 STEP_CAP_PRE 个样本时施加阶跃 */
      if (idx == STEP_CAP_PRE) {
          int16_t target_q15 = MA_TO_Q15(g_StepCapTarget_mA);
          if (g_StepCapPhase == 0) {
              g_M2_Idq_Ref.d = target_q15;  /* d轴阶跃 */
              g_M2_Idq_Ref.q = 0;
          } else {
              g_M2_Idq_Ref.d = 0;
              g_M2_Idq_Ref.q = target_q15;  /* q轴阶跃 */
          }
      }

      if (idx < STEP_CAP_DEPTH) {
          g_StepCapBuf[idx].id_ref = g_M2_Idq_Ref.d;
          g_StepCapBuf[idx].iq_ref = g_M2_Idq_Ref.q;
          g_StepCapBuf[idx].id_fbk = g_M2_Idq.d;
          g_StepCapBuf[idx].iq_fbk = g_M2_Idq.q;
          g_StepCapIdx = idx + 1;
      } else {
          g_M2_Idq_Ref.d = 0;            /* 录完归零 */
          g_M2_Idq_Ref.q = 0;
          g_M2_Iq_Ref_mA = 0;
          g_StepCapState = 2;             /* 录满, 等回放 */
      }
  }

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
  if (g_CalibVdDirectActive) {
      g_M2_Vdq.d = g_CalibVdDirect;
      g_M2_Vdq.q = 0;
  } else {
      g_M2_Vdq.d = FOC_PI_Run(&g_M2_PI_d, g_M2_Idq_Ref.d, g_M2_Idq.d);
      g_M2_Vdq.q = FOC_PI_Run(&g_M2_PI_q, g_M2_Idq_Ref.q, g_M2_Idq.q);
  }

  /* 电流闭环耗时2.3us  - */

  /* M1 dq 前馈解耦 */
  if (g_M1_CtrlMode != M2_MODE_OPEN_LOOP && g_M1_wLs_factor != 0) {
      int32_t vd_ff = -(int32_t)m1_speed_dpp * (int32_t)g_M1_Idq.q;
      int32_t vq_ff = +(int32_t)m1_speed_dpp * (int32_t)g_M1_Idq.d;
      g_M1_Vdq.d = (int16_t)((int32_t)g_M1_Vdq.d + (int16_t)((vd_ff * (int32_t)g_M1_wLs_factor) >> 20));
      g_M1_Vdq.q = (int16_t)((int32_t)g_M1_Vdq.q + (int16_t)((vq_ff * (int32_t)g_M1_wLs_factor) >> 20));
  }

  /* M2 dq 前馈解耦 */
  if (g_M2_CtrlMode != M2_MODE_OPEN_LOOP && g_M2_wLs_factor != 0) {
      int32_t vd_ff = -(int32_t)m2_speed_dpp * (int32_t)g_M2_Idq.q;
      int32_t vq_ff = +(int32_t)m2_speed_dpp * (int32_t)g_M2_Idq.d;
      g_M2_Vdq.d = (int16_t)((int32_t)g_M2_Vdq.d + (int16_t)((vd_ff * (int32_t)g_M2_wLs_factor) >> 20));
      g_M2_Vdq.q = (int16_t)((int32_t)g_M2_Vdq.q + (int16_t)((vq_ff * (int32_t)g_M2_wLs_factor) >> 20));
  }

  /* 圆限幅: Vq²+Vd² ≤ MaxModule² (防逆变器过调制, 防逆Park int16溢出) */
  FOC_CircleLimitation(&g_M1_Vdq);
  FOC_CircleLimitation(&g_M2_Vdq);

  /* Vq饱和标志: 圆限幅后的实际电压, 仅 Live Watch 诊断 */
  {
      int16_t vq1 = g_M1_Vdq.q;
      g_M1_VqSaturated = (vq1 > (int16_t)VQ_SAT_THRESHOLD
                       || vq1 < -(int16_t)VQ_SAT_THRESHOLD);
      int16_t vq2 = g_M2_Vdq.q;
      g_M2_VqSaturated = (vq2 > (int16_t)VQ_SAT_THRESHOLD
                       || vq2 < -(int16_t)VQ_SAT_THRESHOLD);
  }

  /* 电流环自校准: 累加 Vd/Id（TIM3 每 1ms 读平均并清零） */
  if (g_CurrLoopCalibInProgress) {
      g_CalibVdAcc += (int32_t)g_M2_Vdq.d;
      g_CalibIdAcc += (int32_t)g_M2_Idq.d;
      g_CalibAccCnt++;
  }
  if (g_CurrLoopCalibInProgress_M1) {
      g_CalibVdAcc_M1 += (int32_t)g_M1_Vdq.d;
      g_CalibIdAcc_M1 += (int32_t)g_M1_Idq.d;
      g_CalibAccCnt_M1++;
  }

  /* 未触发耗时0.5us  - */

  /* M1 逆Park: 补偿 ADC采样→PWM更新 的延迟 (~1个PWM周期), 与M2对称 */
  {
      int16_t m1_inv_angle = g_M1_ElecAngle_Q15 + m1_speed_dpp;
      SinCos_Q15_t sc_m1_inv;
      FOC_CalcSinCos(m1_inv_angle, &sc_m1_inv);
      FOC_InvParkTransform(&sc_m1_inv, &g_M1_Vdq, &g_M1_Vab);
  }

  /* M2 逆Park: 补偿 ADC采样→PWM更新 的延迟 (~1个PWM周期)
   * 50极对@500RPM: 补偿 ~1607 Q15 ≈ 8.8° 电角度, 提升高速力矩 */
  {
      int16_t m2_inv_angle = g_M2_ElecAngle_Q15 + m2_speed_dpp;
      SinCos_Q15_t sc_m2_inv;
      FOC_CalcSinCos(m2_inv_angle, &sc_m2_inv);
      FOC_InvParkTransform(&sc_m2_inv, &g_M2_Vdq, &g_M2_Vab);
  }

  /* 逆Park变换 耗时0.8us  - */

  /* αβ电压 → PWM占空比 */
  FOC_SetPhasePWM(M1_TIMER_PHASE_A, M1_TIMER_PHASE_B, &g_M1_Vab);
  FOC_SetPhasePWM(M2_TIMER_PHASE_A, M2_TIMER_PHASE_B, &g_M2_Vab);

  /* αβ电压 → PWM占空比 耗时1us  - */

  /* 清除3个ADC的JEOS标志 (必须在读取JDR之后) */
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_ClearFlag_JEOS(ADC2);
  LL_ADC_ClearFlag_JEOS(ADC3);
  LED_OFF;
  /* USER CODE END ADC1_2_IRQn 1 */
}

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

        if (g_M1_CtrlMode == M2_MODE_STEP_ANGLE
         || g_M1_CtrlMode == M2_MODE_POSITION) {
            int32_t abs_err = g_M1_StepAngle_Err;
            if (g_M1_CtrlMode == M2_MODE_POSITION)
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

    if (g_M1_CtrlMode == M2_MODE_POSITION) {
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

    if (g_M1_CtrlMode == M2_MODE_STEP_ANGLE) {
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

    if (g_M1_CtrlMode == M2_MODE_SPEED
     || g_M1_CtrlMode == M2_MODE_POSITION) {

        if (!m1_spd_was_active) {
            g_M1_SpeedPI.integral = 0.0f;
            g_M1_PI_d.integral = 0;
            g_M1_PI_q.integral = 0;
            m1_rpm_cmd_ramped = (int16_t)g_Enc1_SpeedFilt;
            m1_spd_was_active = 1;
        }

        float rpm_target;
        if (g_M1_CtrlMode == M2_MODE_POSITION) {
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
        if (g_M1_CtrlMode == M2_MODE_SPEED) {
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

        if (g_M1_CtrlMode == M2_MODE_SPEED) {
            g_Dbg_M1_Spd_Err = rpm_target - (float)g_Enc1_SpeedFilt;
        } else {
            g_Dbg_M1_Pos_dTerm = iq_mA;
        }
        g_Dbg_M1_Spd_IqOut = iq_mA;

        __disable_irq();
        g_M1_Idq_Ref.d = MA_TO_Q15(g_M1_Id_Eff_mA);
        g_M1_Idq_Ref.q = (int16_t)(iq_mA * MA_TO_Q15_F);
        __enable_irq();
    } else if (g_M1_CtrlMode != M2_MODE_STEP_ANGLE) {
        m1_spd_was_active = 0;
        m1_rpm_cmd_ramped = 0;
    }
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
  static uint8_t  spd_was_active   = 0;

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
  int16_t d1 = (int16_t)(g_Enc1_Angle - enc1_prev);
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

  /* ---- M1 控制环 ---- */
  M1_ControlLoop(d1);

  /* ---- Id 三级自适应 (Standby / Hold / Boost + 迟滞) ----
   * Standby(25mA): |speed|<5 且 |Δenc|≤阈值 持续 N ms
   * Hold(50mA):    正常运行
   * Boost(100mA):  |pos_err| > BoostEnter, 退出需 < BoostExit (迟滞防抖振)
   * 优先级: Boost > Hold > Standby */
  {
      static int16_t id_still_cnt = 0;            /* 静止计时 ms */
      static uint8_t id_in_boost  = 0;            /* Boost 迟滞状态 */
      int16_t id_eff;                             /* 本拍有效Id (mA) */
      int16_t abs_spd = g_Enc2_SpeedFilt;
      if (abs_spd < 0) abs_spd = -abs_spd;
      int16_t abs_d = d2;
      if (abs_d < 0) abs_d = -abs_d;

      /* 第1层: Standby / Hold 基础判定 (timer-based) */
      if (abs_spd < 5 && abs_d <= g_M2_Id_StillDeltaMax) {
          if (id_still_cnt < g_M2_Id_StandbyDelay_ms)
              id_still_cnt++;
          id_eff = (id_still_cnt >= g_M2_Id_StandbyDelay_ms)
                   ? g_M2_Id_Standby_mA : g_M2_Id_Hold_mA;
      } else {
          id_still_cnt = 0;
          id_eff = g_M2_Id_Hold_mA;
      }

      /* 第2层: Boost 覆盖 — 位置误差大时增强磁刚度 (仅步距角/位置模式)
       * 迟滞: 进入 > BoostEnter, 退出 < BoostExit, 避免阈值边界抖振 */
      if (g_M2_CtrlMode == M2_MODE_STEP_ANGLE
       || g_M2_CtrlMode == M2_MODE_POSITION) {
          int32_t abs_err = g_M2_StepAngle_Err;
          if (g_M2_CtrlMode == M2_MODE_POSITION)
              abs_err = g_M2_PosCmd - g_M2_PosFbk;
          if (abs_err < 0) abs_err = -abs_err;

          if (!id_in_boost && abs_err > (int32_t)g_M2_Id_BoostEnter) {
              id_in_boost = 1;                  /* 进入 Boost */
          } else if (id_in_boost && abs_err < (int32_t)g_M2_Id_BoostExit) {
              id_in_boost = 0;                  /* 退出 Boost */
          }

          if (id_in_boost) {
              id_eff = g_M2_Id_Boost_mA;
              id_still_cnt = 0;                 /* Boost 中重置静止计时 */
          }
      } else {
          id_in_boost = 0;                      /* 非位置模式清除 Boost */
      }

      g_M2_Id_Eff_mA = id_eff;
  }

  /* ---- M2 位置环 外环: 位置PI → 速度参考 (PI-P 结构) ----
   * PI-P 级联: 位置PI(外环) → 速度P-only(内环) → Iq(mA)
   *   P: 比例 → 快速响应大偏差
   *   I: 积分 → 缓慢爬升克服摩擦/齿槽力矩死区
   * out_limit = velocity_limit: 限制最大移动速度 */
  static uint8_t pos_was_active = 0;
  float pos_spd_ref = 0.0f;              /* 位置外环输出: 速度参考 RPM */

  if (g_M2_CtrlMode == M2_MODE_POSITION) {
      /* 首次进入: 锁定当前位置, 清积分 */
      if (!pos_was_active) {
          g_M2_PosCmd = g_M2_PosFbk;
          g_M2_PosPID.integral = 0.0f;
          pos_was_active = 1;
      }

      int32_t pos_err = g_M2_PosCmd - g_M2_PosFbk;
      #define M2_POS_ERR_MAX  30000
      if (pos_err >  M2_POS_ERR_MAX) pos_err =  M2_POS_ERR_MAX;
      if (pos_err < -M2_POS_ERR_MAX) pos_err = -M2_POS_ERR_MAX;

      /* P 项: 快速响应 */
      float p_out = g_M2_PosPID.kp * (float)pos_err;

      /* I 项: 克服摩擦死区, 过零清积分防过冲 */
      float i_out = g_M2_PosPID.integral;
      if (g_M2_PosPID.ki != 0.0f) {
          /* 过零检测: 积分方向与误差方向相反 → 已过冲, 清积分 */
          if ((i_out > 0.0f && pos_err < 0) ||
              (i_out < 0.0f && pos_err > 0)) {
              i_out = 0.0f;
          }
          i_out += g_M2_PosPID.ki * (float)pos_err;
          if (i_out >  g_M2_PosPID.integ_limit) i_out =  g_M2_PosPID.integ_limit;
          if (i_out < -g_M2_PosPID.integ_limit) i_out = -g_M2_PosPID.integ_limit;
      }

      pos_spd_ref = p_out + i_out;

      /* velocity_limit + anti-windup */
      if (pos_spd_ref > g_M2_PosPID.out_limit) {
          i_out -= (pos_spd_ref - g_M2_PosPID.out_limit);
          pos_spd_ref = g_M2_PosPID.out_limit;
      } else if (pos_spd_ref < -g_M2_PosPID.out_limit) {
          i_out -= (pos_spd_ref + g_M2_PosPID.out_limit);
          pos_spd_ref = -g_M2_PosPID.out_limit;
      }
      g_M2_PosPID.integral = i_out;
      g_M2_PosPID.output = pos_spd_ref;

      /* VOFA 调试 */
      g_Dbg_Pos_iqmA  = pos_spd_ref;     /* 外环输出: 速度参考 RPM */
      g_Dbg_Pos_pTerm = (float)pos_err;   /* 位置误差 counts */
  } else {
      if (pos_was_active) g_M2_PosPID.integral = 0.0f;
      pos_was_active = 0;
  }

  /* ---- M2 步距角闭环: 直接 PD→Iq (高带宽, 丝滑归位) ----
   * 不走速度PI: 位置误差×kp→弹簧力, 速度误差×kd→阻尼, 直出电流
   * 带宽高于级联架构, 归位无过冲, 代价是高速运转噪声略大 */
  static uint8_t step_was_active = 0;

  if (g_M2_CtrlMode == M2_MODE_STEP_ANGLE) {
      if (!step_was_active) {
          g_M2_StepAngle_Ref = g_M2_PosFbk;
          g_M2_StepAnglePD.integral = 0.0f;
          g_M2_PI_d.integral = 0;
          g_M2_PI_q.integral = 0;
          g_M2_AngleDelta = 0;
          step_was_active = 1;
      }

      int32_t pos_err = g_M2_StepAngle_Ref - g_M2_PosFbk;
      #define STEP_POS_ERR_MAX  30000
      if (pos_err >  STEP_POS_ERR_MAX) pos_err =  STEP_POS_ERR_MAX;
      if (pos_err < -STEP_POS_ERR_MAX) pos_err = -STEP_POS_ERR_MAX;
      g_M2_StepAngle_Err = pos_err;

      /* P: 位置误差 → 弹簧力 (mA/count) */
      float p_term = g_M2_StepAnglePD.kp * (float)pos_err;

      /* D: 速度误差 → 阻尼 (mA/RPM), 用 (speed - RPM_Cmd) 防匀速拖拽 */
      float d_term = g_M2_StepAnglePD.kd
                   * ((float)g_Enc2_SpeedFilt - (float)g_M2_RPM_Cmd);

      /* I: ki≠0 时累积, 消除稳态残差, 过零清积分防过冲 */
      float i_term = g_M2_StepAnglePD.integral;
      if (g_M2_StepAnglePD.ki != 0.0f) {
          /* 过零检测: 积分方向与误差方向相反 → 已过冲, 清积分 */
          if ((i_term > 0.0f && pos_err < 0) ||
              (i_term < 0.0f && pos_err > 0)) {
              i_term = 0.0f;
          }
          i_term += g_M2_StepAnglePD.ki * (float)pos_err;
          if (i_term >  g_M2_StepAnglePD.integ_limit) i_term =  g_M2_StepAnglePD.integ_limit;
          if (i_term < -g_M2_StepAnglePD.integ_limit) i_term = -g_M2_StepAnglePD.integ_limit;
      } else {
          i_term = 0.0f;
      }

      float iq_mA = p_term - d_term + i_term;

      /* anti-windup + 输出限幅 */
      if (iq_mA > g_M2_StepAnglePD.out_limit) {
          i_term -= (iq_mA - g_M2_StepAnglePD.out_limit);
          iq_mA = g_M2_StepAnglePD.out_limit;
      } else if (iq_mA < -g_M2_StepAnglePD.out_limit) {
          i_term -= (iq_mA + g_M2_StepAnglePD.out_limit);
          iq_mA = -g_M2_StepAnglePD.out_limit;
      }
      g_M2_StepAnglePD.integral = i_term;
      g_M2_StepAnglePD.output   = iq_mA;

      /* VOFA */
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

  /* ---- M2 速度闭环 PI (速度模式PI / 位置模式P-only内环) ---- */
  static int16_t rpm_cmd_ramped = 0;

  if (g_M2_CtrlMode == M2_MODE_SPEED
   || g_M2_CtrlMode == M2_MODE_POSITION) {

      if (!spd_was_active) {
          g_M2_SpeedPI.integral = 0.0f;
          g_M2_PI_d.integral = 0;
          g_M2_PI_q.integral = 0;
          rpm_cmd_ramped = (int16_t)g_Enc2_SpeedFilt;
          spd_was_active = 1;
      }

      float rpm_target;
      if (g_M2_CtrlMode == M2_MODE_POSITION) {
          rpm_target = pos_spd_ref;           /* 位置外环 → 速度参考 */
      } else {
          /* 速度模式: 斜坡限速 */
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

      /* 内环: 速度模式PI, 位置模式P-only (PI-P结构, 无积分过冲) */
      float iq_mA;
      if (g_M2_CtrlMode == M2_MODE_SPEED) {
          iq_mA = SpeedPI_Run(&g_M2_SpeedPI,
                              rpm_target,
                              (float)g_Enc2_SpeedFilt);
      } else {
          float spd_err = rpm_target - (float)g_Enc2_SpeedFilt;
          iq_mA = g_M2_SpeedPI.kp * spd_err;
          if (iq_mA >  g_M2_SpeedPI.out_limit) iq_mA =  g_M2_SpeedPI.out_limit;
          if (iq_mA < -g_M2_SpeedPI.out_limit) iq_mA = -g_M2_SpeedPI.out_limit;
          g_M2_SpeedPI.integral = 0.0f;
          g_M2_SpeedPI.output   = iq_mA;
      }

      if (g_M2_CtrlMode == M2_MODE_SPEED) {
          g_Dbg_Spd_Err = rpm_target - (float)g_Enc2_SpeedFilt;
      } else {
          g_Dbg_Pos_dTerm = iq_mA;
      }
      g_Dbg_Spd_IqOut = iq_mA;

      /* 原子写入 Idq_Ref */
      __disable_irq();
      g_M2_Idq_Ref.d = MA_TO_Q15(g_M2_Id_Eff_mA);
      g_M2_Idq_Ref.q = (int16_t)(iq_mA * MA_TO_Q15_F);
      __enable_irq();
  } else {
      spd_was_active = 0;
      rpm_cmd_ramped = 0;
  }

  /* VOFA 发送已移至 TIM4 vofa_engine, TIM3 不再管理 */
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

/* USER CODE END 1 */
