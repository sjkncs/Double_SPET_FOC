/**
 * @file    zero_calib.c
 * @brief   电角度零点标定 — Id 吸合对齐 + 编码器 ZERO 寄存器写入
 *
 * 从 main.c 拆出, 使零点标定逻辑独立可测试.
 * 本模块仅包含阻塞式标定流程, 不涉及 FOC ISR / VOFA / Flash.
 */

#include "zero_calib.h"

/* ====================================================================
 * ZeroCalib_RunBlocking
 *
 * 完整流程 (阻塞约 4s):
 *   ① 停机: 角度步进归零, 等 ISR 缓变到 0
 *   ② 切 Id 吸合, Iq 清零
 *   ③ 摆动吸合 4 次: -90° → 0° → +90° → 0° (最终锁定)
 *   ④ 暂停 ISR 编码器读取, 读角度 + 写零点寄存器
 *   ⑤ 恢复: Id/Iq 清零
 *
 * 注意:
 *   - 调用前应先切到开环模式 (MODE_OPEN_LOOP) 并等待 1s 以消除惯性
 *   - g_Kth71CalibActive 置 1 期间 ISR 暂停编码器 SPI, 避免总线竞争
 *   - mc->p_zero_calib_angle 用 C 变量记录 old_zero, 不读寄存器 (规避 ReadReg bug)
 * ==================================================================== */
uint8_t ZeroCalib_RunBlocking(const KTH7111_Hw_t *hw,
                              const ZeroCalib_Motor_t *mc)
{
    /* ① 停机: 角度步进归零, 等 ISR 缓变到 0 */
    *mc->p_delta_target = 0;
    while (*mc->p_delta != 0) {}
    HAL_Delay(ZERO_CALIB_SETTLE_MS);

    /* ② 切 Id 吸合, Iq 清零 */
    mc->p_idq_ref->d = MA_TO_Q15(ZERO_CALIB_ID_MA);
    mc->p_idq_ref->q = 0;

    /* ③ 摆动吸合 4 次: -90° → 0° → +90° → 0° (最终锁定) */
    *mc->p_elec_angle = (int16_t)(-ZERO_CALIB_SWING_Q15);
    HAL_Delay(ZERO_CALIB_SWING_MS);

    *mc->p_elec_angle = 0;
    HAL_Delay(ZERO_CALIB_SWING_MS);

    *mc->p_elec_angle = (int16_t)(+ZERO_CALIB_SWING_Q15);
    HAL_Delay(ZERO_CALIB_SWING_MS);

    *mc->p_elec_angle = 0;
    HAL_Delay(ZERO_CALIB_LOCK_MS);

    /* ④ 暂停 ISR 编码器读取, 读角度 + 写零点寄存器
     *    公式取自 datasheet 13.2.1 AUTO_ZERO_SET 算法:
     *      RD=1: new_zero = old_zero - enc_angle  (out = raw + ZERO)
     *      RD=0: new_zero = old_zero + enc_angle  (out = raw - ZERO)
     *    uint16 自然溢出即为 mod 65536, 无需特殊处理
     *
     *    注意: old_zero 取 C 变量而非 ReadReg(ZERO_LO/HI).
     *    KTH7111 SPI ReadReg 对 volatile 寄存器可能返回 MTP 默认值 (0),
     *    导致公式结果错误 ("swap" bug). p_zero_calib_angle 初值 = 0 = MTP 默认,
     *    每次 WriteZero 同步更新, 与芯片实际 ZERO 一致. */
    g_Kth71CalibActive = 1;

    uint8_t reg02;
    KTH71_ReadReg(hw, KTH71_REG_SYS_CTRL, &reg02);
    uint8_t rd = (reg02 >> KTH71_RD_BIT) & 1U;

    uint16_t old_zero = *mc->p_zero_calib_angle;   /* 用 C 变量, 不读寄存器 */

    KTH71_SpiGap();
    uint16_t enc_angle;
    uint8_t ok = KTH71_ReadAngle(hw, &enc_angle, NULL);

    if (ok) {
        uint16_t new_zero = rd ? (uint16_t)(old_zero - enc_angle)
                                : (uint16_t)(old_zero + enc_angle);
        *mc->p_zero_calib_angle = new_zero;
        KTH71_WriteZero(hw, new_zero);

        KTH71_SpiGap();
        KTH71_UnlockReg(hw);
        KTH71_SpiGap();
        KTH71_WriteReg(hw, KTH71_REG_ANLC_CTRL, 0x08);

        KTH71_SpiGap();
        KTH71_WriteRegToMTP(hw);
    }

    g_Kth71CalibActive = 0;

    /* ⑤ 恢复: Id/Iq 清零, while 循环下一轮自动从 mA 变量恢复;
     *         calib_was_active 逻辑自动重置速度基准 */
    mc->p_idq_ref->d = 0;
    mc->p_idq_ref->q = 0;

    return ok;
}
