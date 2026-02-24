/**
 * @file    kth71xx.h
 * @brief   KTH7111 磁编码器驱动 — SPI 读写 · 寄存器访问 · 非线性自校准
 *
 * 数据手册 KTH7111_datasheet_0.9.pdf
 *   6.2–6.6  SPI 协议: 读角度 / 读写寄存器 / 解锁锁定 / MTP
 *   12       非线性自校准 (ANLC): 恒速 100–1000 rpm, 推荐 600 rpm
 *
 * 用法
 *   底层接口传入 KTH7111_Hw_t (SPI + CS GPIO), 支持多编码器实例。
 *   校准接口 KTH71_CalibRunBlocking 全自动完成:
 *     电流闭环 + 角度开环加速 → 启动 ANLC → 轮询状态 → 减速停机, 全程阻塞。
 *     校准期间自动暂停 ISR 编码器 SPI 读取 / 速度计算 / VOFA, 完成后自动恢复。
 */

#ifndef __KTH71XX_H
#define __KTH71XX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ====================================================================
 * SPI 命令字 (datasheet 6.2–6.6)
 * ==================================================================== */
#define KTH71_CMD_READ_ANGLE    ((uint8_t)0x00)
#define KTH71_CMD_READ_REG      ((uint8_t)0x11)
#define KTH71_CMD_WRITE_REG     ((uint8_t)0x33)

/* ---- 解锁 / 锁定 32-bit 密码 (6.3) ---- */
#define KTH71_UNLOCK_B0         ((uint8_t)0x20)
#define KTH71_UNLOCK_B1         ((uint8_t)0x24)
#define KTH71_UNLOCK_B2         ((uint8_t)0x01)
#define KTH71_UNLOCK_B3         ((uint8_t)0x01)

#define KTH71_LOCK_B0           ((uint8_t)0x20)
#define KTH71_LOCK_B1           ((uint8_t)0x24)
#define KTH71_LOCK_B2           ((uint8_t)0x12)
#define KTH71_LOCK_B3           ((uint8_t)0x31)

/* ---- MTP 烧写 24-bit (6.6) ---- */
#define KTH71_MTP_B0            ((uint8_t)0x22)
#define KTH71_MTP_B1            ((uint8_t)0x55)
#define KTH71_MTP_B2            ((uint8_t)0xAA)
#define KTH71_MTP_DELAY_MS      400U

/* ====================================================================
 * 寄存器地址 (表 6 / 表 17)
 * ==================================================================== */
#define KTH71_REG_ZERO_LO       ((uint8_t)0x00)
#define KTH71_REG_ZERO_HI       ((uint8_t)0x01)
#define KTH71_REG_SYS_CTRL      ((uint8_t)0x02)   /* bit7: RD, bit6: AUTO_ZERO_SET */
#define KTH71_RD_BIT             7U                /* RD=1: out=raw+ZERO, RD=0: out=raw-ZERO */
#define KTH71_REG_ANLC_CTRL     ((uint8_t)0x16)   /* bit4: REG_CAL, bit3: ANLC_EN */
#define KTH71_REG_ANLC_STATUS   ((uint8_t)0x72)   /* [5:4]: ANLC_STATUS */

/* ---- ANLC 控制位 (0x16) ----
 * 寄存器布局: RESERVE[7:5] | REG_CAL[4] | ANLC_EN[3] | RESERVE[2:0]
 * 默认值 0x08 → ANLC_EN=1(bit3), REG_CAL=0(bit4) */
#define KTH71_ANLC_REG_CAL_BIT  ((uint8_t)0x10)   /* bit4=1 启动, 完成后清 0 */

/* ---- ANLC 状态 (0x72, 表 18) ----
 * 寄存器布局: RESERVE[7:6] | ANLC_STATUS[5:4] | RESERVE[3:0]
 * 实测: IDLE→0x00, BUSY→0x10, DONE→0x32 (低 4 位为芯片内部标志) */
#define KTH71_ANLC_STATUS_SHIFT 4U
#define KTH71_ANLC_STATUS_MASK  ((uint8_t)0x03)  /* 移位后再掩码 */
#define KTH71_ANLC_STATUS_IDLE  ((uint8_t)0x00)
#define KTH71_ANLC_STATUS_BUSY  ((uint8_t)0x01)
#define KTH71_ANLC_STATUS_FAIL  ((uint8_t)0x02)
#define KTH71_ANLC_STATUS_DONE  ((uint8_t)0x03)
#define KTH71_ANLC_STATUS_SPI_ERR ((uint8_t)0xFF) /* SPI 读失败 (非芯片值) */

/* ---- CRC8/ITU 参数 (6.2) ---- */
#define KTH71_CRC_XOR_OUT       ((uint8_t)0x55)

/* ====================================================================
 * 校准时序参数 (按需微调)
 * ==================================================================== */
#define KTH71_CALIB_RPM          800U    /* 推荐校准转速 rpm (datasheet 12.4) */
#define KTH71_CALIB_POLL_MS      50U     /* 状态轮询间隔 ms */
#define KTH71_CALIB_TIMEOUT_MS   20000U   /* 校准超时 ms */
#define KTH71_CALIB_RAMPUP_MS    800U    /* 加速 + 机械稳定等待 ms */
#define KTH71_CALIB_RAMPDN_MS    500U    /* 减速等待 ms */

/* ====================================================================
 * 辅助宏: RPM → 开环 AngleDelta (Q16 电角度步进 / ISR 周期)
 *
 *   delta = rpm × pole_pairs × 65536 / (isr_hz × 60)
 *
 *   示例: KTH71_RPM_TO_DELTA(600, 50, 17000) ≈ 1927
 *   使用 int64_t 中间量, 避免 rpm×poles×65536 溢出 INT32_MAX
 * ==================================================================== */
#define KTH71_RPM_TO_DELTA(rpm, poles, isr_hz) \
    ((int16_t)((int64_t)(rpm) * (poles) * 65536 / ((int64_t)(isr_hz) * 60)))

/* ====================================================================
 * 校准电机控制接口
 *
 * 将开环角度步进变量的指针传入校准函数, 实现 "自动加速 → 校准 → 停机",
 * 电压由电流环 PI 自动生成, 无需手动设 Vq。
 * ==================================================================== */
typedef struct {
    int16_t *p_delta_target;   /* → g_Mx_AngleDelta_Target (ISR 缓变目标) */
    int16_t  calib_delta;      /* 校准转速对应的 AngleDelta, 用 KTH71_RPM_TO_DELTA 算 */
} KTH71_MotorCtrl_t;

/** 校准进行标志: 1=校准中 (ISR 跳过 SPI 读取 + 速度计算 + VOFA), 0=正常 */
extern volatile uint8_t g_Kth71CalibActive;

/** 校准调试步骤 (Live Watch 观察 g_CalibDbgStep, 校准结束后保留最终状态) */
#define KTH71_CALIB_DBG_IDLE       0U   /* 未启动 */
#define KTH71_CALIB_DBG_RAMP       1U   /* 加速中 */
#define KTH71_CALIB_DBG_UNLOCK     2U   /* 解锁 + 启动 ANLC */
#define KTH71_CALIB_DBG_POLLING    3U   /* 轮询 ANLC_STATUS */
#define KTH71_CALIB_DBG_DONE       4U   /* ANLC 成功 ✓ */
#define KTH71_CALIB_DBG_ANLC_FAIL  5U   /* ANLC 报告失败 */
#define KTH71_CALIB_DBG_TIMEOUT    6U   /* 轮询超时 */
#define KTH71_CALIB_DBG_SPI_FAIL   0xF0U /* calib_start SPI 通信失败 */
#define KTH71_CALIB_DBG_SPI_POLL   0xF1U /* 轮询 SPI 读 0x72 失败 */

extern volatile uint8_t g_CalibDbgStep;

/* ====================================================================
 * SPI 帧间延时 (datasheet §6.4/6.5: 连续 SPI 操作间需 >150ns)
 * ==================================================================== */
static inline void KTH71_SpiGap(void)
{
    for (volatile uint32_t i = 0; i < 5; i++) {}  /* @170MHz: ~176ns */
}

/* ====================================================================
 * API — 基础 SPI 接口
 * ==================================================================== */

/** ISR 双编码器交错读取 (17kHz 热路径, 直接寄存器操作)
 *  结果写入 g_Enc1/2_Angle, g_Enc1/2_CrcOk, g_Enc1/2_RxBuf */
void KTH7111_ReadDual(const KTH7111_Hw_t *encA, const KTH7111_Hw_t *encB);

/** 读角度 (16-bit, [0,65535] → 0–360°)
 *  @return 1=CRC 通过, 0=失败; crc_ok 可传 NULL */
uint8_t KTH71_ReadAngle(const KTH7111_Hw_t *hw, uint16_t *angle, uint8_t *crc_ok);

/** 读单字节寄存器 (带 CRC)  @return 1=CRC 通过 */
uint8_t KTH71_ReadReg(const KTH7111_Hw_t *hw, uint8_t addr, uint8_t *value);

/** 写单字节寄存器 (需先 UnlockReg)  @return 芯片回传字节 */
uint8_t KTH71_WriteReg(const KTH7111_Hw_t *hw, uint8_t addr, uint8_t value);

/** 解锁寄存器写保护 */
void KTH71_UnlockReg(const KTH7111_Hw_t *hw);

/** 锁定寄存器 */
void KTH71_LockReg(const KTH7111_Hw_t *hw);

/** 将寄存器写入 MTP (阻塞 KTH71_MTP_DELAY_MS) */
void KTH71_WriteRegToMTP(const KTH7111_Hw_t *hw);

/** 写入零点寄存器 (volatile, 掉电丢失; 固化需另调 WriteRegToMTP)
 *  内部自行 Unlock/Lock, 调用前需确保 ISR 未占用 SPI */
void KTH71_WriteZero(const KTH7111_Hw_t *hw, uint16_t angle);

/** 从芯片 MTP 回读 ZERO 寄存器 (16-bit, ZERO_HI:ZERO_LO)
 *  @return 当前零点角度 [0, 65535] */
uint16_t KTH71_ReadZero(const KTH7111_Hw_t *hw);

/* ====================================================================
 * API — 非线性自校准 (开环 600 rpm, 全程阻塞)
 *
 * 前提: 调用前电流环已运行 (Iq_Ref 已设定), 校准使用电流闭环 + 角度开环。
 * 内部流程:
 *   1. 置 g_Kth71CalibActive=1 (ISR 暂停编码器 SPI / 速度 / VOFA)
 *   2. 写 AngleDelta → ISR 缓变加速到 ~600 rpm (电压由电流环 PI 自动生成)
 *   3. 等待 RAMPUP_MS (电气加速 + 机械稳定)
 *   4. 解锁 → REG_CAL=1 启动 ANLC
 *   5. 每 POLL_MS 轮询 ANLC_STATUS, 直至 DONE / FAIL / 超时
 *   6. REG_CAL=0 → 锁定
 *   7. AngleDelta→0 (缓变减速) → 等待 RAMPDN_MS
 *   8. 置 g_Kth71CalibActive=0 (ISR 自动恢复编码器读取 + 速度 + VOFA)
 *
 * @param hw         编码器 SPI+CS 实例
 * @param motor      开环电机控制参数 (指针 + 转速 + 电压)
 * @param timeout_ms 校准阶段超时, 建议 ≥ 3000
 * @return 1=校准成功, 0=失败或超时
 * ==================================================================== */
uint8_t KTH71_CalibRunBlocking(const KTH7111_Hw_t *hw,
                                const KTH71_MotorCtrl_t *motor,
                                uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __KTH71XX_H */
