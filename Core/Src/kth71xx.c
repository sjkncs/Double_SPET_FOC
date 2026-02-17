/**
 * @file    kth71xx.c
 * @brief   KTH7111 磁编码器驱动 — LL SPI · 寄存器访问 · 非线性自校准 (开环 600 rpm)
 *
 * 数据手册 KTH7111_datasheet_0.9.pdf, 接口说明见 kth71xx.h
 */

#include "kth71xx.h"

/** 校准进行标志: ISR 检查此标志跳过编码器 SPI 读取 / 速度计算 / VOFA */
volatile uint8_t g_Kth71CalibActive = 0;

/** 校准调试: Live Watch 观察校准进度 (见 KTH71_CALIB_DBG_* 定义) */
volatile uint8_t g_CalibDbgStep = 0;

/* ====================================================================
 * 1. CRC8/ITU  (poly 0x07, init 0x00, xorout 0x55 — datasheet 6.2)
 * ==================================================================== */
static uint8_t s_crc8_lut[256] = {
    0x00,0x07,0x0e,0x09,0x1c,0x1b,0x12,0x15,0x38,0x3f,0x36,0x31,0x24,0x23,0x2a,0x2d,
    0x70,0x77,0x7e,0x79,0x6c,0x6b,0x62,0x65,0x48,0x4f,0x46,0x41,0x54,0x53,0x5a,0x5d,
    0xe0,0xe7,0xee,0xe9,0xfc,0xfb,0xf2,0xf5,0xd8,0xdf,0xd6,0xd1,0xc4,0xc3,0xca,0xcd,
    0x90,0x97,0x9e,0x99,0x8c,0x8b,0x82,0x85,0xa8,0xaf,0xa6,0xa1,0xb4,0xb3,0xba,0xbd,
    0xc7,0xc0,0xc9,0xce,0xdb,0xdc,0xd5,0xd2,0xff,0xf8,0xf1,0xf6,0xe3,0xe4,0xed,0xea,
    0xb7,0xb0,0xb9,0xbe,0xab,0xac,0xa5,0xa2,0x8f,0x88,0x81,0x86,0x93,0x94,0x9d,0x9a,
    0x27,0x20,0x29,0x2e,0x3b,0x3c,0x35,0x32,0x1f,0x18,0x11,0x16,0x03,0x04,0x0d,0x0a,
    0x57,0x50,0x59,0x5e,0x4b,0x4c,0x45,0x42,0x6f,0x68,0x61,0x66,0x73,0x74,0x7d,0x7a,
    0x89,0x8e,0x87,0x80,0x95,0x92,0x9b,0x9c,0xb1,0xb6,0xbf,0xb8,0xad,0xaa,0xa3,0xa4,
    0xf9,0xfe,0xf7,0xf0,0xe5,0xe2,0xeb,0xec,0xc1,0xc6,0xcf,0xc8,0xdd,0xda,0xd3,0xd4,
    0x69,0x6e,0x67,0x60,0x75,0x72,0x7b,0x7c,0x51,0x56,0x5f,0x58,0x4d,0x4a,0x43,0x44,
    0x19,0x1e,0x17,0x10,0x05,0x02,0x0b,0x0c,0x21,0x26,0x2f,0x28,0x3d,0x3a,0x33,0x34,
    0x4e,0x49,0x40,0x47,0x52,0x55,0x5c,0x5b,0x76,0x71,0x78,0x7f,0x6a,0x6d,0x64,0x63,
    0x3e,0x39,0x30,0x37,0x22,0x25,0x2c,0x2b,0x06,0x01,0x08,0x0f,0x1a,0x1d,0x14,0x13,
    0xae,0xa9,0xa0,0xa7,0xb2,0xb5,0xbc,0xbb,0x96,0x91,0x98,0x9f,0x8a,0x8d,0x84,0x83,
    0xde,0xd9,0xd0,0xd7,0xc2,0xc5,0xcc,0xcb,0xe6,0xe1,0xe8,0xef,0xfa,0xfd,0xf4,0xf3
};

/** 计算 CRC8: 对 data[0..len-1] 做查表, 最后异或 0x55 */
static uint8_t crc8_calc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
        crc = s_crc8_lut[crc ^ data[i]];
    return crc ^ KTH71_CRC_XOR_OUT;
}

/** 校验接收帧: 前 (len-1) 字节做 CRC, 末字节为期望值 */
static uint8_t crc8_verify(const uint8_t *frame, uint8_t len)
{
    if (len == 0) return 0;
    return (crc8_calc(frame, len - 1) == frame[len - 1]) ? 1 : 0;
}

/* ====================================================================
 * 2. LL SPI 半双工底层 (1-line BIDIOE, CS 软件控制)
 *
 * 注意: 若 ISR 中也有 SPI 操作 (如 KTH7111_ReadDual) 且共享同一 SPI 外设,
 * 需调用方确保互斥。当前校准期间的 SPI 访问极稀疏 (每 50ms 一次, ~6μs),
 * 与 17kHz ISR 的 SPI 读角度存在约 0.01% 碰撞概率。
 * 如需彻底消除, 可在 spi_tx/spi_tx_then_rx 前后加 __disable_irq/__enable_irq。
 * ==================================================================== */

/** SPI 帧间延时: 保证 > 150ns (KTH7111 datasheet §6.4/6.5)
 *  volatile 循环 5 迭代 @170MHz: 每迭代 ~6 cycles → 30 cycles ≈ 176ns */
static inline void spi_gap_150ns(void)
{
    for (volatile uint32_t i = 0; i < 5; i++) {}
}

/** 纯发送: SPE使能 → CS↓ → TX → CS↑
 *  关键: KTH7111 在 CS↓+SCK_idle 时会提前准备数据, 必须 SPE 先使能再拉 CS */
static void spi_tx(const KTH7111_Hw_t *hw, const uint8_t *buf, uint8_t len)
{
    SPI_TypeDef *spi = hw->spi;

    /* 排空残留 RXNE */
    while (spi->SR & SPI_SR_RXNE)
        (void)*(volatile uint8_t *)&spi->DR;

    spi->CR1 = (spi->CR1 | SPI_CR1_BIDIOE) & ~SPI_CR1_SPE;
    spi->CR1 |= SPI_CR1_SPE;
    hw->cs_port->BRR = hw->cs_pin;                         /* CS ↓ (SPE 之后!) */

    for (uint8_t i = 0; i < len; i++) {
        *(volatile uint8_t *)&spi->DR = buf[i];
        while (!(spi->SR & SPI_SR_TXE)) {}
    }
    while (spi->SR & SPI_SR_BSY) {}

    spi->CR1 &= ~SPI_CR1_SPE;
    hw->cs_port->BSRR = hw->cs_pin;                        /* CS ↑ */
}

/** 先发后收: SPE使能 → CS↓ → TX → RX → CS↑
 *  关键: CS 必须在 SPE=1 之后拉低, 否则 KTH7111 提前准备数据导致 1-bit 偏移
 *  (同 main.c KTH7111_ReadDual 的经验) */
static void spi_tx_then_rx(const KTH7111_Hw_t *hw,
                           const uint8_t *tx, uint8_t tx_len,
                           uint8_t *rx, uint8_t rx_len)
{
    SPI_TypeDef *spi = hw->spi;

    while (spi->SR & SPI_SR_RXNE)
        (void)*(volatile uint8_t *)&spi->DR;

    /* TX 阶段: 先 SPE 再 CS */
    spi->CR1 = (spi->CR1 | SPI_CR1_BIDIOE) & ~SPI_CR1_SPE;
    spi->CR1 |= SPI_CR1_SPE;
    hw->cs_port->BRR = hw->cs_pin;                         /* CS ↓ (SPE 之后!) */
    for (uint8_t i = 0; i < tx_len; i++) {
        *(volatile uint8_t *)&spi->DR = tx[i];
        while (!(spi->SR & SPI_SR_TXE)) {}
    }
    while (spi->SR & SPI_SR_BSY) {}
    spi->CR1 &= ~SPI_CR1_SPE;

    /* RX 阶段: BIDIOE=0 时 SCK 持续运行, 必须先 BIDIOE=1 停钟再关 SPE
     * (RM0440 §40.5.10) */
    spi->CR1 &= ~SPI_CR1_BIDIOE;
    spi->CR1 |= SPI_CR1_SPE;
    for (uint8_t i = 0; i < rx_len; i++) {
        while (!(spi->SR & SPI_SR_RXNE)) {}
        rx[i] = *(volatile uint8_t *)&spi->DR;
    }
    spi->CR1 |= SPI_CR1_BIDIOE;    /* 切 TX 方向 → 立即停止 RX 时钟 */
    spi->CR1 &= ~SPI_CR1_SPE;      /* 然后关闭 SPI */

    hw->cs_port->BSRR = hw->cs_pin;                        /* CS ↑ */
}

/* ====================================================================
 * 3. 寄存器级 API
 * ==================================================================== */

uint8_t KTH71_ReadAngle(const KTH7111_Hw_t *hw, uint16_t *angle, uint8_t *crc_ok)
{
    uint8_t cmd = KTH71_CMD_READ_ANGLE;
    uint8_t rx[3];
    spi_tx_then_rx(hw, &cmd, 1, rx, 3);

    *angle = ((uint16_t)rx[0] << 8) | rx[1];
    uint8_t ok = crc8_verify(rx, 3);
    if (crc_ok) *crc_ok = ok;
    return ok;
}

uint8_t KTH71_ReadReg(const KTH7111_Hw_t *hw, uint8_t addr, uint8_t *value)
{
    uint8_t tx[2] = { KTH71_CMD_READ_REG, addr };
    uint8_t rx[2];
    spi_tx_then_rx(hw, tx, 2, rx, 2);

    *value = rx[0];
    return crc8_verify(rx, 2);
}

uint8_t KTH71_WriteReg(const KTH7111_Hw_t *hw, uint8_t addr, uint8_t data)
{
    uint8_t tx[3] = { KTH71_CMD_WRITE_REG, addr, data };
    uint8_t rx;
    spi_tx_then_rx(hw, tx, 3, &rx, 1);
    return rx;
}

void KTH71_UnlockReg(const KTH7111_Hw_t *hw)
{
    const uint8_t cmd[] = {
        KTH71_UNLOCK_B0, KTH71_UNLOCK_B1, KTH71_UNLOCK_B2, KTH71_UNLOCK_B3
    };
    spi_tx(hw, cmd, sizeof(cmd));
}

void KTH71_LockReg(const KTH7111_Hw_t *hw)
{
    const uint8_t cmd[] = {
        KTH71_LOCK_B0, KTH71_LOCK_B1, KTH71_LOCK_B2, KTH71_LOCK_B3
    };
    spi_tx(hw, cmd, sizeof(cmd));
}

void KTH71_WriteRegToMTP(const KTH7111_Hw_t *hw)
{
    KTH71_UnlockReg(hw);                   /* datasheet §6.6: 烧写前须解锁 */
    spi_gap_150ns();
    const uint8_t cmd[] = { KTH71_MTP_B0, KTH71_MTP_B1, KTH71_MTP_B2 };
    spi_tx(hw, cmd, sizeof(cmd));
    HAL_Delay(KTH71_MTP_DELAY_MS);
}

void KTH71_WriteZero(const KTH7111_Hw_t *hw, uint16_t angle)
{
    KTH71_UnlockReg(hw);
    spi_gap_150ns();                                        /* 帧间隔 >150ns */
    KTH71_WriteReg(hw, KTH71_REG_ZERO_LO, (uint8_t)(angle & 0xFF));
    spi_gap_150ns();                                        /* 帧间隔 >150ns */
    KTH71_WriteReg(hw, KTH71_REG_ZERO_HI, (uint8_t)(angle >> 8));
    /* 不 Lock: Lock 会将 volatile 寄存器回退到 MTP 值, 导致写入丢失.
     * ISR 仅读角度, 不写寄存器, 解锁状态安全.
     * 需固化时另调 WriteRegToMTP. */
}

uint16_t KTH71_ReadZero(const KTH7111_Hw_t *hw)
{
    uint8_t lo, hi;
    KTH71_ReadReg(hw, KTH71_REG_ZERO_LO, &lo);
    KTH71_ReadReg(hw, KTH71_REG_ZERO_HI, &hi);
    return ((uint16_t)hi << 8) | lo;
}

/* ====================================================================
 * 4. ISR 快速双编码器读取 (17kHz ADC ISR 调用)
 *
 * 交错 SPI 操作: A 发命令→B 发命令→并行等→并行收, 最大化时间利用
 * CS 必须在 SPE=1 之后拉低 (KTH7111 在 CS↓+SCK_idle 时会提前准备数据)
 * ==================================================================== */
void KTH7111_ReadDual(const KTH7111_Hw_t *encA, const KTH7111_Hw_t *encB)
{
    SPI_TypeDef *sA = encA->spi, *sB = encB->spi;
    uint8_t bA[3], bB[3];

    /* ---- TX A: disable+设TX方向, 再enable ---- */
    sA->CR1 = (sA->CR1 | SPI_CR1_BIDIOE) & ~SPI_CR1_SPE;
    sA->CR1 |= SPI_CR1_SPE;
    encA->cs_port->BRR = encA->cs_pin;
    *(volatile uint8_t *)&sA->DR = 0x00;

    /* ---- TX B: A的TX进行中, 利用这段时间设置B ---- */
    sB->CR1 = (sB->CR1 | SPI_CR1_BIDIOE) & ~SPI_CR1_SPE;
    sB->CR1 |= SPI_CR1_SPE;
    encB->cs_port->BRR = encB->cs_pin;
    *(volatile uint8_t *)&sB->DR = 0x00;

    /* 等两路TX完成 */
    while (!(sA->SR & SPI_SR_TXE)) {}
    while ( (sA->SR & SPI_SR_BSY)) {}
    while (!(sB->SR & SPI_SR_TXE)) {}
    while ( (sB->SR & SPI_SR_BSY)) {}

    /* ---- 切RX: disable+清BIDIOE ---- */
    sA->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_BIDIOE);
    sB->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_BIDIOE);

    /* 两路同时enable, RX时钟开始 */
    sA->CR1 |= SPI_CR1_SPE;
    sB->CR1 |= SPI_CR1_SPE;

    /* ---- RXNE轮询交错读取 ---- */
    while (!(sA->SR & SPI_SR_RXNE)) {}
    bA[0] = *(volatile uint8_t *)&sA->DR;
    while (!(sB->SR & SPI_SR_RXNE)) {}
    bB[0] = *(volatile uint8_t *)&sB->DR;

    while (!(sA->SR & SPI_SR_RXNE)) {}
    bA[1] = *(volatile uint8_t *)&sA->DR;
    while (!(sB->SR & SPI_SR_RXNE)) {}
    bB[1] = *(volatile uint8_t *)&sB->DR;

    while (!(sA->SR & SPI_SR_RXNE)) {}
    bA[2] = *(volatile uint8_t *)&sA->DR;
    while (!(sB->SR & SPI_SR_RXNE)) {}
    bB[2] = *(volatile uint8_t *)&sB->DR;

    /* ---- 停止时钟 + CS高 ---- */
    sA->CR1 &= ~SPI_CR1_SPE;
    sB->CR1 &= ~SPI_CR1_SPE;
    encA->cs_port->BSRR = encA->cs_pin;
    encB->cs_port->BSRR = encB->cs_pin;

    /* ---- 解析 ---- */
    g_Enc1_Angle  = ((uint16_t)bA[0] << 8) | bA[1];
    g_Enc1_RxBuf[0] = bA[0]; g_Enc1_RxBuf[1] = bA[1]; g_Enc1_RxBuf[2] = bA[2];
    g_Enc1_CrcOk = (crc8_calc(bA, 2) == bA[2]) ? 1U : 0U;

    g_Enc2_Angle  = ((uint16_t)bB[0] << 8) | bB[1];
    g_Enc2_RxBuf[0] = bB[0]; g_Enc2_RxBuf[1] = bB[1]; g_Enc2_RxBuf[2] = bB[2];
    g_Enc2_CrcOk = (crc8_calc(bB, 2) == bB[2]) ? 1U : 0U;
}

/* ====================================================================
 * 5. 非线性自校准 (ANLC) — 内部辅助
 * ==================================================================== */

/** 置 REG_CAL=1 启动自校准  @return 1=OK, 0=寄存器读失败 */
static uint8_t calib_start(const KTH7111_Hw_t *hw)
{
    uint8_t val;
    if (!KTH71_ReadReg(hw, KTH71_REG_ANLC_CTRL, &val))
        return 0;
    spi_gap_150ns();                                        /* 帧间隔 >150ns */
    KTH71_WriteReg(hw, KTH71_REG_ANLC_CTRL, val | KTH71_ANLC_REG_CAL_BIT);
    return 1;
}

/** 查询 ANLC_STATUS  @return 状态枚举, SPI 失败返回 SPI_ERR */
static uint8_t calib_get_status(const KTH7111_Hw_t *hw)
{
    uint8_t val;
    if (!KTH71_ReadReg(hw, KTH71_REG_ANLC_STATUS, &val))
        return KTH71_ANLC_STATUS_SPI_ERR;
    return (val >> KTH71_ANLC_STATUS_SHIFT) & KTH71_ANLC_STATUS_MASK;
}

/** 清 REG_CAL=0 结束自校准 */
static void calib_end(const KTH7111_Hw_t *hw)
{
    uint8_t val;
    if (KTH71_ReadReg(hw, KTH71_REG_ANLC_CTRL, &val))
        KTH71_WriteReg(hw, KTH71_REG_ANLC_CTRL, val & (uint8_t)~KTH71_ANLC_REG_CAL_BIT);
}

/* ====================================================================
 * 5. 对外校准接口 — 全自动: 开环加速 → ANLC → 减速停机
 * ==================================================================== */

uint8_t KTH71_CalibRunBlocking(const KTH7111_Hw_t *hw,
                                const KTH71_MotorCtrl_t *motor,
                                uint32_t timeout_ms)
{
    uint8_t result = 0;

    /* ---- 阶段 1: 暂停 ISR 编码器 SPI / 速度 / VOFA, 消除总线竞争 ---- */
    g_Kth71CalibActive = 1;
    g_CalibDbgStep = KTH71_CALIB_DBG_RAMP;

    /* ---- 阶段 2: 电流闭环 + 角度开环加速到校准转速 ----
     * 仅设 AngleDelta, 电压由电流环 PI 自动生成;
     * ISR 中 OpenLoop_IncAngle 缓变加速, FOC 电流环持续运行 */
    *motor->p_delta_target = motor->calib_delta;
    HAL_Delay(KTH71_CALIB_RAMPUP_MS);

    /* ---- 阶段 3: 执行 ANLC 校准 ---- */
    g_CalibDbgStep = KTH71_CALIB_DBG_UNLOCK;
    KTH71_UnlockReg(hw);

    /* 显式清除上次校准残留状态:
     * REG_CAL=0 → 等 200ms → 芯片复位 ANLC 状态机 → REG_CAL=1
     * 不做此步, 第二次校准时 ANLC_STATUS 残留 DONE(3), 芯片不接受重启 */
    calib_end(hw);
    HAL_Delay(200);

    if (calib_start(hw)) {
        /* ---- 等待芯片进入 BUSY(1), 确认真正开始校准 ----
         * 写 REG_CAL=1 后, ANLC_STATUS 可能残留旧值.
         * 必须等到 BUSY 才能开始轮询结果, 否则会误读旧 DONE. */
        g_CalibDbgStep = KTH71_CALIB_DBG_POLLING;
        uint8_t entered = 0;
        for (uint32_t w = 0; w < 2000U; w += KTH71_CALIB_POLL_MS) {
            HAL_Delay(KTH71_CALIB_POLL_MS);
            uint8_t st = calib_get_status(hw);
            if (st == KTH71_ANLC_STATUS_BUSY) {
                entered = 1;
                break;
            }
            if (st == KTH71_ANLC_STATUS_SPI_ERR) {
                g_CalibDbgStep = KTH71_CALIB_DBG_SPI_POLL;
                break;
            }
        }

        if (entered) {
            /* ---- 芯片已进入校准, 轮询等待 DONE / FAIL ---- */
            for (uint32_t elapsed = 0; elapsed < timeout_ms; elapsed += KTH71_CALIB_POLL_MS) {
                HAL_Delay(KTH71_CALIB_POLL_MS);
                uint8_t st = calib_get_status(hw);

                if (st == KTH71_ANLC_STATUS_SPI_ERR) {
                    g_CalibDbgStep = KTH71_CALIB_DBG_SPI_POLL;
                    break;
                }
                if (st == KTH71_ANLC_STATUS_DONE) {
                    g_CalibDbgStep = KTH71_CALIB_DBG_DONE;
                    result = 1;
                    break;
                }
                if (st == KTH71_ANLC_STATUS_FAIL) {
                    g_CalibDbgStep = KTH71_CALIB_DBG_ANLC_FAIL;
                    break;
                }
            }

            if (g_CalibDbgStep == KTH71_CALIB_DBG_POLLING)
                g_CalibDbgStep = KTH71_CALIB_DBG_TIMEOUT;
        } else if (g_CalibDbgStep != KTH71_CALIB_DBG_SPI_POLL) {
            /* 2s 内未进入 BUSY → 芯片未响应 REG_CAL */
            g_CalibDbgStep = KTH71_CALIB_DBG_TIMEOUT;
        }

        calib_end(hw);
    } else {
        g_CalibDbgStep = KTH71_CALIB_DBG_SPI_FAIL;
    }

    KTH71_LockReg(hw);

    /* ---- 阶段 4: 减速停机 ----
     * 目标步进归零 → ISR 缓变减速 → 等待机械停稳 */
    *motor->p_delta_target = 0;
    HAL_Delay(KTH71_CALIB_RAMPDN_MS);

    /* ---- 阶段 5: 恢复 ISR 编码器 / 速度 / VOFA ---- */
    g_Kth71CalibActive = 0;
    /* g_CalibDbgStep 保留最终状态, 供 Live Watch 事后查看 */

    return result;
}
