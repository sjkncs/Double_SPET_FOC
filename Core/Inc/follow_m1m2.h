/**
 * @file    follow_m1m2.h
 * @brief   M1→M2 主从随动模块 — M1 当手轮, M2 跟随 M1 编码器位置
 *
 * 用法:
 *   1. while(1) 中调用 Follow_M1M2_Poll()
 *   2. Live Watch 置 g_FollowM1M2 = 1 启动随动
 *   3. Live Watch 置 g_FollowM1M2 = 0 停止, 两轴回开环零电流
 *
 * 随动期间:
 *   - M1: 开环模式, Id=Iq=0, 电机自由转动, 编码器持续读取
 *   - M2: 步距角模式, PD 控制器驱动 M2 跟随 M1 位置 (1:1)
 */

#ifndef FOLLOW_M1M2_H
#define FOLLOW_M1M2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Live Watch 启停开关: 0=独立控制(默认), 1=M1→M2 随动 */
extern volatile uint8_t g_FollowM1M2;

/**
 * @brief  主循环轮询 — 检测启停边沿 + 持续更新位置
 *
 * 在 while(1) 中每轮调用一次, 开销极小 (1 次比较 + 1 次赋值).
 * 内部自动处理进入/退出模式切换.
 */
void Follow_M1M2_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* FOLLOW_M1M2_H */
