/**
 * @file    foc_adapt.c
 * @brief   FOC 自适应辅助 — PI Vbus 缩放 + dq 解耦 + ADC 校准 + 调试
 *
 * 从 main.c (USER CODE BEGIN 0/4) 拆出, 使主循环只做编排, 具体逻辑在此.
 */

#include "foc_adapt.h"
#include "main.h"

/* ---- 模块内部状态 ---- */
static float s_M1_Ls_henry = 0.f;   /**< M1 相电感 (校准/Flash 加载后更新) */
static float s_M2_Ls_henry = 0.f;   /**< M2 相电感 (校准/Flash 加载后更新) */

/* ====================================================================
 * RPM_to_AngleDelta
 * ==================================================================== */
int16_t RPM_to_AngleDelta(int16_t rpm, uint16_t poles, uint32_t isr_hz)
{
    return (int16_t)((int32_t)rpm * poles * 16384 / ((int32_t)isr_hz * 15));
}

/* ====================================================================
 * ADC_CalibrateOffset — 上电零电流偏移标定
 *
 * 累加 4096 次 ADC 原始值 → 右移 12 位取平均, 避免除法.
 * 4096 次 @17kHz ≈ 241ms.
 * 溢出校验: 4096 × 0xFFF0 = 268,369,920 < UINT32_MAX ✓
 * ==================================================================== */
void ADC_CalibrateOffset(void)
{
    const uint32_t SHIFT = 12U;
    const uint32_t COUNT = 1U << SHIFT;  /* 4096 */

    uint32_t acc_m1ia = 0, acc_m1ib = 0;
    uint32_t acc_m2ia = 0, acc_m2ib = 0;

    for (uint32_t i = 0; i < COUNT; i++)
    {
        g_AdcConvDone = 0;              /* 先清标志 */
        while (g_AdcConvDone == 0)      /* 等待下一次HRTIM中断 */
        {
        }
        /* JDR在下次转换前保持不变, 此处读取与ISR读到的是同一次结果 */
        acc_m1ia += (uint16_t)(ADC1->JDR1);  /* PA2, M1_Ia (与ISR一致) */
        acc_m1ib += (uint16_t)(ADC2->JDR1);  /* PA0, M1_Ib (与ISR一致) */
        acc_m2ia += (uint16_t)(ADC2->JDR2);  /* PA1, M2_Ia */
        acc_m2ib += (uint16_t)(ADC1->JDR2);  /* PA3, M2_Ib */
    }

    g_Offset_M1Ia = (uint16_t)(acc_m1ia >> SHIFT);
    g_Offset_M1Ib = (uint16_t)(acc_m1ib >> SHIFT);
    g_Offset_M2Ia = (uint16_t)(acc_m2ia >> SHIFT);
    g_Offset_M2Ib = (uint16_t)(acc_m2ib >> SHIFT);
}

/* FOC_GetPhaseCurrent 已移至 main.h (ALWAYS_INLINE, 含修正后的ADC映射) */

/* ====================================================================
 * Debug_ReadCurrent_mA — Q15 → mA 转换
 *
 * I_mA = Q15 × I_FULLSCALE_MA / 32768
 * 在 main 循环中按需调用, Live Watch 观测 g_DebugCurrent.
 * ==================================================================== */
void Debug_ReadCurrent_mA(void)
{
    g_DebugCurrent.M1_Ia_mA = Q15_TO_MA(g_M1Current.Ia);
    g_DebugCurrent.M1_Ib_mA = Q15_TO_MA(g_M1Current.Ib);
    g_DebugCurrent.M2_Ia_mA = Q15_TO_MA(g_M2Current.Ia);
    g_DebugCurrent.M2_Ib_mA = Q15_TO_MA(g_M2Current.Ib);
}

/* ====================================================================
 * FOC_UpdateDecoupleFactors — dq 前馈解耦因子计算
 *
 * wLs_factor = round(2π × fs × Ls × I_fullscale / (Vbus/2) × 2^20 / 2^15)
 * ISR 中: Vd_ff = -(speed_dpp × Iq_q15 × wLs_factor) >> 20
 *
 * 数值示例: fs=17000, Ls=0.008, I_fs=0.825A, Vbus=12V
 *   factor_f = 2π×17000×0.008×0.825 / (6) × 2^20 / 32768 = 3771
 *   @500RPM 50pp: speed_dpp≈803, Iq=200mA(q15=7946)
 *   Vd_ff_q15 = 803×7946×3771>>20 ≈ 22943 → 4.2V ✓
 * ==================================================================== */
/** @brief 根据 Ls 和 Vbus 计算解耦因子 (通用, M1/M2 共用) */
static int16_t compute_wLs_factor(float ls_henry)
{
    if (ls_henry <= 0.f) return 0;
    uint32_t vbus = g_Vbus_mV;
    if (vbus < 3000U) vbus = 3000U;
    float vbus_half = (float)vbus / 2000.f;
    float i_fs = (float)I_FULLSCALE_MA / 1000.f;
    float factor_f = 6.2831853f * (float)ENC_ISR_FREQ_HZ * ls_henry * i_fs
                     / vbus_half * 1048576.f / 32768.f;
    if (factor_f > 32767.f) factor_f = 32767.f;
    if (factor_f < 0.f) factor_f = 0.f;
    return (int16_t)(factor_f + 0.5f);
}

static void FOC_UpdateDecoupleFactors(void)
{
    g_M1_wLs_factor = compute_wLs_factor(s_M1_Ls_henry);
    g_M2_wLs_factor = compute_wLs_factor(s_M2_Ls_henry);
}

/* ====================================================================
 * FOC_UpdatePI_ByVbus — PI 增益按 Vbus 等比缩放
 *
 * 基准来源: g_Mx_PI_Base (校准/Flash 加载后更新, 否则为编译时 NOM)
 * 公式: kp_actual = kp_base × vbus_base / vbus_actual
 * 校准期间跳过 (PI 由校准模块直接控制)
 * ==================================================================== */
void FOC_UpdatePI_ByVbus(void)
{
    uint32_t vbus = g_Vbus_mV;
    if (vbus < 3000U) vbus = 3000U;  /* 防除零 / 异常低压保护 */

    /* M1: 基于 g_M1_PI_Base 缩放 (校准期间跳过, PI 由校准模块直接控制) */
    if (!g_CurrLoopCalibInProgress_M1) {
        int32_t m1_kp = (int32_t)((uint64_t)g_M1_PI_Base.kp * g_M1_PI_Base.vbus_mv / vbus);
        int32_t m1_ki = (int32_t)((uint64_t)g_M1_PI_Base.ki * g_M1_PI_Base.vbus_mv / vbus);
        g_M1_PI_d.kp = m1_kp;  g_M1_PI_d.ki = m1_ki;
        g_M1_PI_q.kp = m1_kp;  g_M1_PI_q.ki = m1_ki;
    }

    /* M2: 基于 g_M2_PI_Base 缩放 (校准期间跳过) */
    if (!g_CurrLoopCalibInProgress) {
        int32_t m2_kp = (int32_t)((uint64_t)g_M2_PI_Base.kp * g_M2_PI_Base.vbus_mv / vbus);
        int32_t m2_ki = (int32_t)((uint64_t)g_M2_PI_Base.ki * g_M2_PI_Base.vbus_mv / vbus);
        g_M2_PI_d.kp = m2_kp;  g_M2_PI_d.ki = m2_ki;
        g_M2_PI_q.kp = m2_kp;  g_M2_PI_q.ki = m2_ki;
    }

    /* dq 解耦因子随 Vbus 变化而更新 */
    FOC_UpdateDecoupleFactors();
}

/* ====================================================================
 * FOC_SetM2Ls / FOC_GetM2Ls — Ls 访问接口
 * ==================================================================== */
void FOC_SetM1Ls(float ls_henry) { s_M1_Ls_henry = ls_henry; }
float FOC_GetM1Ls(void) { return s_M1_Ls_henry; }
void FOC_SetM2Ls(float ls_henry) { s_M2_Ls_henry = ls_henry; }
float FOC_GetM2Ls(void) { return s_M2_Ls_henry; }
