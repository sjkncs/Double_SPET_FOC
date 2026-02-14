/**
 * @file    zero_calib.h
 * @brief   电角度零点标定模块 — Id 吸合对齐 + 编码器 ZERO 寄存器写入
 *
 * 原理:
 *   步距角 1.8° = 90° 电角度 (50 极对).
 *   摆动 ±1 步距角 (总 2 步距角) 消除齿槽/摩擦后锁定在电角度 0°,
 *   读编码器原始角度写入 ZERO 寄存器, 使 elec = enc × poles 成立.
 *
 * 依赖:
 *   - kth71xx.h : 编码器 SPI 读写
 *   - main.h    : DQ_Q15_t, MA_TO_Q15, g_Kth71CalibActive
 *
 * 用法:
 *   ZeroCalib_Motor_t m2 = { &g_M2_ElecAngle_Q15, ... };
 *   ZeroCalib_RunBlocking(&g_Enc2_Hw, &m2);   // 阻塞约 4s
 */
#ifndef ZERO_CALIB_H
#define ZERO_CALIB_H

#include <stdint.h>
#include "main.h"
#include "kth71xx.h"

/* ---- 可调参数 (Live Watch / 批量调参时修改此处) ---- */
#define ZERO_CALIB_ID_MA       300      /**< d 轴吸合电流 mA */
#define ZERO_CALIB_SWING_Q15   16384    /**< ±90° 电角度 = ±1步距角, 总摆幅 2 步距角 */
#define ZERO_CALIB_SWING_MS    500U     /**< 摆动保持时间 ms */
#define ZERO_CALIB_LOCK_MS     2000U    /**< 最终 0° 定位保持时间 ms */
#define ZERO_CALIB_SETTLE_MS   200U     /**< 停机机械停稳等待 ms */

/**
 * @brief  零点标定所需的电机控制指针 (M1/M2 通用)
 *
 * 使用指针而非硬编码全局变量, 便于复用到不同电机轴.
 */
typedef struct {
    volatile int16_t  *p_elec_angle;        /**< → g_Mx_ElecAngle_Q15 */
    volatile int16_t  *p_delta;             /**< → g_Mx_AngleDelta (ISR 实际步进) */
    int16_t           *p_delta_target;      /**< → g_Mx_AngleDelta_Target */
    volatile DQ_Q15_t *p_idq_ref;          /**< → g_Mx_Idq_Ref */
    volatile uint16_t *p_zero_calib_angle;  /**< → g_ZeroCalibAngle_Mx (各轴独立) */
} ZeroCalib_Motor_t;

/**
 * @brief  电角度零点标定 (阻塞, 约 4s)
 *
 * 流程: 停机 → Id 吸合 → 摆动 ±90° × 2 → 0° 锁定 → 读角度写 ZERO → 恢复
 *
 * @param  hw   编码器 SPI+CS 实例 (如 &g_Enc2_Hw)
 * @param  mc   电机控制指针 (如 &m2_zero)
 * @return 1=成功, 0=SPI 读角度失败
 */
uint8_t ZeroCalib_RunBlocking(const KTH7111_Hw_t *hw,
                              const ZeroCalib_Motor_t *mc);

#endif /* ZERO_CALIB_H */
