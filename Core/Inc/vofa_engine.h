/**
 * @file    vofa_engine.h
 * @brief   VOFA+ 调试波形引擎 — TIM4 1kHz 低优先级自动发送
 *
 * 架构:
 *   各功能模块只需设 g_VofaSrc 切换数据源, TIM4 自动填帧 + DMA 发送.
 *   彻底消除各模块手动管理 DMA 发送的负担.
 *
 * 数据源:
 *   VOFA_SRC_NORMAL         — 速度 / 位置 / 积分 / PID 输出
 *   VOFA_SRC_CURR_CALIB     — 电流环自校准 (DC+AC 辨识 Rs/Ls)
 *   VOFA_SRC_KTH71_CALIB    — KTH71 编码器非线性校准 (ANLC)
 *   VOFA_SRC_STEP_PLAYBACK  — 电流环 dq 阶跃响应回放
 *
 * 使用方法:
 *   1. main.c USER CODE BEGIN 2: 调 Vofa_StartTIM4() 启动
 *   2. 进入某模式前: g_VofaSrc = VOFA_SRC_xxx;
 *   3. 退出时: g_VofaSrc = VOFA_SRC_NORMAL;
 *   4. 阶跃回放: 当 TIM4 播完一轴后置 g_StepCapState = 4,
 *      while(1) 负责 d→q 轴切换 (涉及 PI 复位, 不宜在低优先级 ISR 做)
 */

#ifndef VOFA_ENGINE_H
#define VOFA_ENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ====================================================================
 * VOFA 数据源枚举
 * ==================================================================== */
typedef enum {
    VOFA_SRC_IDLE = 0,          /**< 空闲: 不填帧、不发送 (上电默认) */
    VOFA_SRC_NORMAL,            /**< 正常: 速度/位置/PID 调试 */
    VOFA_SRC_CURR_CALIB,        /**< M2 电流环自校准 (Rs/Ls 辨识) */
    VOFA_SRC_CURR_CALIB_M1,     /**< M1 电流环自校准 (Rs/Ls 辨识) */
    VOFA_SRC_KTH71_CALIB,       /**< KTH71 ANLC 非线性校准 */
    VOFA_SRC_STEP_PLAYBACK,     /**< 电流环 dq 阶跃响应回放 */
} VofaSrc_t;

extern volatile VofaSrc_t g_VofaSrc;
extern volatile uint8_t g_VofaMotorSel;   /* 0=M2(默认), 1=M1 — VOFA 显示哪个电机 */

/* ====================================================================
 * API
 * ==================================================================== */

/**
 * @brief  启动 TIM4 定时器 (1kHz) + 使能更新中断
 *
 * 在 main.c USER CODE BEGIN 2, DMA 初始化之后调用一次.
 */
void Vofa_StartTIM4(void);

/**
 * @brief  TIM4 1kHz 中断服务函数体 — 填帧 + DMA 发送
 *
 * 由 TIM4_IRQHandler 调用.
 * 根据 g_VofaSrc 选择数据源, 填 g_VofaFrame, 触发 DMA.
 */
void Vofa_OnTIM4_1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_ENGINE_H */
