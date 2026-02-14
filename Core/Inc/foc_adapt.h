/**
 * @file    foc_adapt.h
 * @brief   FOC 自适应辅助 — PI Vbus 缩放 + dq 解耦 + ADC 校准 + 调试
 *
 * 职责:
 *   1. FOC_UpdatePI_ByVbus()     : 根据实测 Vbus 等比缩放 PI 增益
 *   2. FOC_UpdateDecoupleFactors(): 计算 dq 前馈解耦因子 (wLs_factor)
 *   3. ADC_CalibrateOffset()     : 上电零电流偏移标定
 *   4. Debug_ReadCurrent_mA()    : Q15 → mA 调试转换
 *   5. RPM_to_AngleDelta()       : RPM → ISR 电角度步进量
 *   6. FOC_SetM1Ls/GetM1Ls, FOC_SetM2Ls/GetM2Ls : Ls 访问接口 (校准/Flash)
 *
 * 依赖:
 *   - main.h : 全局变量 + 宏 (PI_BaseGains_t, g_Vbus_mV 等)
 */
#ifndef FOC_ADAPT_H
#define FOC_ADAPT_H

#include <stdint.h>

/**
 * @brief  RPM → 电角度步进量 (每 ISR 周期, Q15 增量)
 *
 * 公式: delta = rpm × poles × 65536 / (isr_hz × 60)
 * 分子分母同除 4 → rpm × poles × 16384 / (isr_hz × 15)
 * int32_t 安全范围: |rpm| ≤ 2000, poles ≤ 100 时无溢出
 *
 * @param  rpm     目标转速, 支持负值反转, |rpm| ≤ 2000
 * @param  poles   极对数
 * @param  isr_hz  ISR 频率 (Hz)
 * @return int16_t Q15 角度步进量
 */
int16_t RPM_to_AngleDelta(int16_t rpm, uint16_t poles, uint32_t isr_hz);

/**
 * @brief  ADC 零电流偏移校准 (阻塞, 约 241ms)
 *
 * 上电调用一次, 电机必须不通电 (PWM=50% 时净电压为 0).
 * 累加 4096 次 ADC 原始值, 右移 12 位取平均.
 */
void ADC_CalibrateOffset(void);

/**
 * @brief  将 Q15 电流转换为物理电流 (mA), 调试观测用
 *
 * 写入 g_DebugCurrent 结构体, Live Watch 直接查看.
 */
void Debug_ReadCurrent_mA(void);

/**
 * @brief  PI 增益按 Vbus 等比缩放 (保持物理增益恒定)
 *
 * 基准来源: g_Mx_PI_Base (校准/Flash 加载后更新, 否则为编译时 NOM)
 * 公式: kp_actual = kp_base × vbus_base / vbus_actual
 * 校准期间跳过 (PI 由校准模块直接控制)
 * 同时更新 dq 解耦因子.
 */
void FOC_UpdatePI_ByVbus(void);

void FOC_SetM1Ls(float ls_henry);
float FOC_GetM1Ls(void);
void FOC_SetM2Ls(float ls_henry);
float FOC_GetM2Ls(void);

#endif /* FOC_ADAPT_H */
