/**
 * @file    calib_platform_m2.h
 * @brief   M2 电流环自校准平台层 — 回调 + 启动/完成 + TIM3 驱动
 *
 * 职责:
 *   1. 提供 curr_loop_autocalib 模块所需的平台回调 (设 Vd, 读 Idq/Vdq, 读 Vbus)
 *   2. 封装校准启动 (切模式 + Init + Start) 和完成 (写 PI + Flash + VOFA)
 *   3. 提供 TIM3 1ms ISR 入口 CalibM2_OnTIM3_1ms()
 *
 * 依赖:
 *   - curr_loop_autocalib.h : 校准引擎
 *   - flash_params.h        : Flash 持久化
 *   - main.h                : 全局变量 + 宏
 */
#ifndef CALIB_PLATFORM_M2_H
#define CALIB_PLATFORM_M2_H

#include <stdint.h>

/**
 * @brief  启动 M2 电流环自校准
 *
 * 切开环模式, 角度/电流清零, 初始化校准引擎, 由 TIM3 1ms 驱动后续状态.
 * 调用后 g_CurrLoopCalibInProgress = 1, 主循环勿覆盖 M2 Idq_Ref.
 */
void CalibM2_Start(void);

/**
 * @brief  校准完成处理: 写 PI + Flash 持久化 + 发 VOFA 最终结果 + 恢复 PI 控制
 *
 * 仅在 g_CurrLoopCalibResultReady == 1 时由主循环调用.
 * 内部会关 PWM → Flash erase+program (~40ms) → 恢复 PWM.
 */
void CalibM2_OnDone(void);

/**
 * @brief  TIM3 1ms 中断入口: 驱动校准状态机
 *
 * 若校准未启动则直接返回, 不影响其他 TIM3 任务.
 * DONE/ERROR 时置 g_CurrLoopCalibResultReady = 1, 由主循环处理.
 */
void CalibM2_OnTIM3_1ms(void);

/**
 * @brief  校准 VOFA 帧填充 (由 TIM4 vofa_engine 调用)
 *
 * 将当前校准状态、计数、Rs/Ls 等写入 g_VofaFrame.
 * 仅填帧, 不触发 DMA (TIM4 统一管理发送).
 */
void CalibM2_FillVofa(void);

#endif /* CALIB_PLATFORM_M2_H */
