/**
 * @file    comm_protocol.c
 * @brief   RK3576 ↔ STM32G474 二进制通信协议实现
 *
 * RX 路径:
 *   DMA1_CH2 Circular → s_rxBuf[128]
 *   USART1 IDLE ISR  → 更新 s_rxWrIdx (不解析)
 *   while(1)         → Protocol_Poll() 字节流解析 + CRC + 命令派发
 *
 * TX 路径:
 *   Comm_OnTIM4_1ms() (1kHz TIM4 ISR):
 *     PROTOCOL 模式 → 按配置频率填状态帧 + DMA1_CH1 发送
 *     VOFA 模式     → 调 Vofa_OnTIM4_1ms()
 *
 * 看门狗:
 *   TIM3 ISR 每 1ms 递增 g_CommWatchdogMs
 *   Protocol_Poll() 收到有效帧时清零
 *   超时 → 切开环 + Iq/Id 归零 (零电流自由滑行)
 */

#include "comm_protocol.h"
#include "main.h"
#include "vofa_engine.h"
#include "follow_m1m2.h"
#include <string.h>   /* memcpy for float extraction */

/* ====================================================================
 * CRC-CCITT 查表 (poly=0x1021, init=0xFFFF)
 * 256 × 2B = 512B Flash, 40 字节帧 CRC < 1µs
 * ==================================================================== */
static const uint16_t s_crcTable[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x54A5,
    0xA54A,0xB56B,0x8508,0x9529,0xE5CE,0xF5EF,0xC58C,0xD5AD,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x4864,0x5845,0x6826,0x7807,0x08E0,0x18C1,0x28A2,0x38A3,
    0xC94C,0xD96D,0xE90E,0xF92F,0x89C8,0x99E9,0xA98A,0xB9AB,
    0x5A55,0x4A74,0x7A17,0x6A36,0x1AD1,0x0AF0,0x3A93,0x2AB2,
    0xDB5D,0xCB7C,0xFB1F,0xEB3E,0x9BD9,0x8BF8,0xBB9B,0xAB9A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x85A9,0xF56E,0xE54F,0xD52C,0xC50D,
    0x34E2,0x24C3,0x14A0,0x04A1,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BD9,0x9BF8,0xAB9B,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AD1,0x1AF0,0x2A93,0x3AB2,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CE1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0,
};

static uint16_t crc_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc = (crc << 8) ^ s_crcTable[((crc >> 8) ^ *data++) & 0xFF];
    }
    return crc;
}

/* ====================================================================
 * 全局变量 (extern 在 comm_protocol.h)
 * ==================================================================== */
volatile CommMode_t g_CommMode        = COMM_MODE_PROTOCOL;
volatile uint8_t    g_UpSeqNo         = 0;
volatile uint16_t   g_CrcErrCount     = 0;
volatile uint16_t   g_CommWatchdogMs  = 0;
volatile uint16_t   g_CommTimeoutMs   = 200;  /* 默认 200ms, 运动模式时 Protocol_Poll 改为 20ms */

/* ====================================================================
 * RX 环形缓冲区 (DMA1_CH2 Circular 写入, while(1) 读取)
 * ==================================================================== */
static uint8_t  s_rxBuf[COMM_RX_BUF_SIZE] __attribute__((aligned(4)));
static volatile uint16_t s_rxWrIdx = 0;   /* IDLE ISR 更新 */
static uint16_t s_rxRdIdx = 0;            /* Protocol_Poll 更新 */

/* ====================================================================
 * TX 缓冲区 (状态帧 + CAPS 响应共用)
 * ==================================================================== */
#define TX_BUF_SIZE  64
static uint8_t s_txBuf[TX_BUF_SIZE];

/* ====================================================================
 * 内部状态
 * ==================================================================== */
/* while(1) 写 → TIM4 ISR 读 的变量需要 volatile */
static volatile uint8_t  s_lastRxSeq      = 0;     /* 最后收到的运动指令 SeqNo */
static uint8_t  s_txSeqNo        = 0;              /* 状态帧自增序列号 (TIM4 only) */
static volatile uint8_t  s_lastCmdResult  = CMD_OK;
static uint8_t  s_flashUnlocked  = 0;              /* while(1) only */
static uint32_t s_flashUnlockTick = 0;             /* HAL_GetTick() 时刻 */
#define FLASH_UNLOCK_TIMEOUT_MS  5000U
#define FLASH_ERASE_MAGIC        0xDEADU

static volatile uint8_t  s_pendingCapsRsp = 0;     /* while(1) 写, TIM4 读+清 */
static volatile uint8_t  s_forceStatusOnce = 0;    /* while(1) 写, TIM4 读+清 */

/* SET_IQ_REF 超时保护 (用 HAL_GetTick() 确保精确 ms 计时) */
static uint32_t s_iqRefLastTick = 0;
static uint8_t  s_iqRefActive   = 0;   /* 收到过 SET_IQ_REF 且仍在 OpenLoop */
#define IQ_REF_TIMEOUT_MS  30000U  /* 30 秒无新 SET_IQ_REF → 自动归零 */

/* ====================================================================
 * RX 环形缓冲区辅助函数
 * ==================================================================== */
static inline uint16_t rx_available(void)
{
    return (s_rxWrIdx - s_rxRdIdx) & COMM_RX_BUF_MASK;
}

static inline uint8_t rx_peek(uint16_t offset)
{
    return s_rxBuf[(s_rxRdIdx + offset) & COMM_RX_BUF_MASK];
}

static inline void rx_advance(uint16_t n)
{
    s_rxRdIdx = (s_rxRdIdx + n) & COMM_RX_BUF_MASK;
}

static void rx_read_block(uint16_t offset, uint8_t *dst, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        dst[i] = s_rxBuf[(s_rxRdIdx + offset + i) & COMM_RX_BUF_MASK];
    }
}

/* ====================================================================
 * 状态帧构造
 * ==================================================================== */
static void fill_motor_block(CommMotorBlock_t *blk,
                              volatile MotorCtrlMode_t *mode,
                              volatile uint8_t *vq_sat,
                              volatile uint8_t *calib_active,
                              volatile int32_t *speed_filt,
                              volatile int32_t *pos_fbk,
                              volatile DQ_Q15_t *idq,
                              volatile int16_t *id_eff)
{
    blk->ctrl_mode  = (uint8_t)*mode;
    blk->flags      = 0;
    if (*vq_sat)       blk->flags |= MFLAG_VQ_SAT;
    if (*calib_active) blk->flags |= MFLAG_CALIB_ACTIVE;
    blk->speed_mrpm = (int32_t)*speed_filt * 1000;  /* RPM → milliRPM */
    blk->pos_fbk    = *pos_fbk;
    blk->id_ma      = (int16_t)((int32_t)idq->d * I_FULLSCALE_MA / 32768);
    blk->iq_ma      = (int16_t)((int32_t)idq->q * I_FULLSCALE_MA / 32768);
    blk->id_eff_ma  = *id_eff;
}

static uint16_t build_status_frame(void)
{
    CommStatusPayload_t *p = (CommStatusPayload_t *)&s_txBuf[3];

    p->cmd_id          = RSP_STATUS;
    p->tx_seq_no       = s_txSeqNo++;
    p->last_rx_seq     = s_lastRxSeq;
    p->up_seq_no       = g_UpSeqNo;
    p->proto_ver       = COMM_PROTO_VER;

    /* SysFlags */
    p->sys_flags = 0;
    if (g_FlashParamsLoaded)                          p->sys_flags |= SYSFLAG_FLASH_LOADED;
    if (g_CurrLoopCalibInProgress_M1 ||
        g_CurrLoopCalibInProgress_M2)                 p->sys_flags |= SYSFLAG_CALIB_ACTIVE;
    if (s_flashUnlocked)                              p->sys_flags |= SYSFLAG_FLASH_UNLOCKED;
    if (g_CommWatchdogMs > g_CommTimeoutMs)            p->sys_flags |= SYSFLAG_COMM_WDG;
    if (g_CommMode == COMM_MODE_VOFA)                  p->sys_flags |= SYSFLAG_VOFA_MODE;

    p->last_cmd_result = s_lastCmdResult;
    p->vbus_mv         = (uint16_t)g_Vbus_mV;
    p->crc_err_count   = g_CrcErrCount;

    fill_motor_block(&p->m1,
                     &g_M1_CtrlMode, &g_M1_VqSaturated,
                     &g_CurrLoopCalibInProgress_M1,
                     &g_Enc1_SpeedFilt, &g_M1_PosFbk,
                     &g_M1_Idq, &g_M1_Id_Eff_mA);

    fill_motor_block(&p->m2,
                     &g_M2_CtrlMode, &g_M2_VqSaturated,
                     &g_CurrLoopCalibInProgress_M2,
                     &g_Enc2_SpeedFilt, &g_M2_PosFbk,
                     &g_M2_Idq, &g_M2_Id_Eff_mA);

    /* 帧头 + Len */
    uint8_t payload_len = sizeof(CommStatusPayload_t);
    s_txBuf[0] = COMM_HEADER_0;
    s_txBuf[1] = COMM_HEADER_1;
    s_txBuf[2] = payload_len;

    /* CRC 覆盖 Header + Len + Payload */
    uint16_t total_before_crc = 3 + payload_len;
    uint16_t crc = crc_ccitt(s_txBuf, total_before_crc);
    s_txBuf[total_before_crc]     = (uint8_t)(crc & 0xFF);
    s_txBuf[total_before_crc + 1] = (uint8_t)(crc >> 8);

    return total_before_crc + 2;  /* 总帧长 */
}

static uint16_t build_caps_frame(void)
{
    CommCapsPayload_t *p = (CommCapsPayload_t *)&s_txBuf[3];

    p->cmd_id     = RSP_CAPS;
    p->fw_version = COMM_FW_VERSION;
    p->proto_ver  = COMM_PROTO_VER;
    p->axis_count = 2;
    p->features   = FEAT_VOFA | FEAT_FOLLOW | FEAT_FLASH_PARAMS |
                    FEAT_CURR_CALIB | FEAT_KTH71_CALIB |
                    FEAT_ZERO_CALIB | FEAT_STEP_CAPTURE;

    uint8_t payload_len = sizeof(CommCapsPayload_t);
    s_txBuf[0] = COMM_HEADER_0;
    s_txBuf[1] = COMM_HEADER_1;
    s_txBuf[2] = payload_len;

    uint16_t total_before_crc = 3 + payload_len;
    uint16_t crc = crc_ccitt(s_txBuf, total_before_crc);
    s_txBuf[total_before_crc]     = (uint8_t)(crc & 0xFF);
    s_txBuf[total_before_crc + 1] = (uint8_t)(crc >> 8);

    return total_before_crc + 2;
}

/* ====================================================================
 * DMA TX 发送 (复用 DMA1_CH1, 与 VOFA 共享)
 * ==================================================================== */
static void dma_tx_send(const uint8_t *buf, uint16_t len)
{
    if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1))
        return;  /* DMA 忙, 丢弃本帧 */

    LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)buf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, len);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

/* ====================================================================
 * 命令处理
 * ==================================================================== */

static void handle_set_motion(const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(CommSetMotion_t)) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    const CommSetMotion_t *cmd = (const CommSetMotion_t *)payload;
    s_lastRxSeq = cmd->seq_no;

    g_M1_PosCmd += (int32_t)cmd->m1_delta;

    int16_t m2t = cmd->m2_target;
    if (m2t >  16384) m2t =  16384;
    if (m2t < -16384) m2t = -16384;
    g_M2_PosCmd = (int32_t)m2t;

    /* 故意不写 s_lastCmdResult = CMD_OK:
     * SET_MOTION 是高频运动指令 (≤200Hz), 不应覆盖偶发管理命令的诊断结果,
     * 否则上位机无法通过 LastCmdResult 观察到配置/校准命令的错误 */
}

static void handle_set_position(const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(CommSetPosition_t)) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    const CommSetPosition_t *cmd = (const CommSetPosition_t *)payload;

    if (cmd->motor == 1)
        g_M1_PosCmd = cmd->target;
    else
        g_M2_PosCmd = cmd->target;

    s_lastCmdResult = CMD_OK;
}

static void handle_set_iq_ref(const uint8_t *payload, uint8_t len)
{
    if (len < COMM_SET_IQ_REF_MIN_LEN) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    if (g_M1_CtrlMode != MODE_OPEN_LOOP && g_M2_CtrlMode != MODE_OPEN_LOOP) {
        s_lastCmdResult = CMD_ERR_WRONG_MODE;
        return;
    }
    const CommSetIqRef_t *cmd = (const CommSetIqRef_t *)payload;
    uint8_t has_id = (len >= sizeof(CommSetIqRef_t));

    if (g_M1_CtrlMode == MODE_OPEN_LOOP) {
        g_M1_Iq_Ref_mA = cmd->m1_iq_ma;
        if (has_id) g_M1_Id_Hold_mA = cmd->m1_id_ma;
    }
    if (g_M2_CtrlMode == MODE_OPEN_LOOP) {
        g_M2_Iq_Ref_mA = cmd->m2_iq_ma;
        if (has_id) g_M2_Id_Hold_mA = cmd->m2_id_ma;
    }

    /* 可选: AngleDelta 字段 (12B payload) — 仅 M1 连续旋转用
     * M2 开环由位置控制器 (M2_UpdateCtrlRef) 管 AngleDelta_Target，此处不覆盖 */
    if (len >= 12) {
        int16_t m1_ad;
        memcpy(&m1_ad, &payload[8], 2);
        if (g_M1_CtrlMode == MODE_OPEN_LOOP) g_M1_AngleDelta_Target = m1_ad;
    }

    s_iqRefLastTick = HAL_GetTick();
    s_iqRefActive = 1;
    s_lastCmdResult = CMD_OK;
}

static void handle_set_ctrl_mode(const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(CommSetCtrlMode_t)) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    if (g_CurrLoopCalibInProgress_M1 || g_CurrLoopCalibInProgress_M2) {
        s_lastCmdResult = CMD_ERR_CALIB_ACTIVE;
        return;
    }
    const CommSetCtrlMode_t *cmd = (const CommSetCtrlMode_t *)payload;
    if (cmd->mode > 3) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    MotorCtrlMode_t m = (MotorCtrlMode_t)cmd->mode;
    if (cmd->motor == 1 || cmd->motor == 2) g_M1_CtrlMode = m;
    if (cmd->motor == 0 || cmd->motor == 2) g_M2_CtrlMode = m;
    s_lastCmdResult = CMD_OK;
}

static void handle_set_follow(const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(CommSetFollow_t)) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    const CommSetFollow_t *cmd = (const CommSetFollow_t *)payload;
    g_FollowM1M2 = cmd->enable ? 1 : 0;
    s_lastCmdResult = CMD_OK;
}

static void handle_set_speed_pi(const uint8_t *payload, uint8_t len)
{
    /* payload: Motor(u8) + kp(f32) + ki(f32) = 9B */
    if (len < 9) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    uint8_t motor = payload[0];
    float kp, ki;
    memcpy(&kp, &payload[1], 4);
    memcpy(&ki, &payload[5], 4);

    if (motor == 1 || motor == 2) {
        g_M1_SpeedPI.kp = kp;
        g_M1_SpeedPI.ki = ki;
    }
    if (motor == 0 || motor == 2) {
        g_M2_SpeedPI.kp = kp;
        g_M2_SpeedPI.ki = ki;
    }
    s_lastCmdResult = CMD_OK;
}

static void handle_flash_unlock(void)
{
    /* 校准进行中禁止 Flash 操作 */
    if (g_CurrLoopCalibInProgress_M1 || g_CurrLoopCalibInProgress_M2) {
        s_lastCmdResult = CMD_ERR_CALIB_ACTIVE;
        return;
    }
    /* FlashParams_EraseAndReset 会关 HRTIM PWM, 闭环运行中执行会导致失力 */
    if (g_M1_CtrlMode != MODE_OPEN_LOOP || g_M2_CtrlMode != MODE_OPEN_LOOP) {
        s_lastCmdResult = CMD_ERR_MOTOR_ACTIVE;
        return;
    }
    s_flashUnlocked   = 1;
    s_flashUnlockTick = HAL_GetTick();
    s_lastCmdResult   = CMD_OK;
}

static void handle_flash_erase(const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(CommFlashErase_t)) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    if (!s_flashUnlocked) {
        s_lastCmdResult = CMD_ERR_FLASH_LOCKED;
        return;
    }
    const CommFlashErase_t *cmd = (const CommFlashErase_t *)payload;
    if (cmd->magic != FLASH_ERASE_MAGIC) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        s_flashUnlocked = 0;
        return;
    }
    g_FlashParamsErase = 1;  /* PollCalibTriggers() 会执行实际擦除 */
    s_flashUnlocked = 0;
    s_lastCmdResult = CMD_OK;
}

static void handle_set_vofa_mode(const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(CommSetVofaMode_t)) {
        s_lastCmdResult = CMD_ERR_BAD_PARAM;
        return;
    }
    const CommSetVofaMode_t *cmd = (const CommSetVofaMode_t *)payload;
    g_CommMode = COMM_MODE_VOFA;
    g_VofaSrc  = (VofaSrc_t)cmd->vofa_src;
    g_VofaMotorSel = cmd->motor_sel;
    s_lastCmdResult = CMD_OK;
}

static void handle_set_status_rate(const uint8_t *payload, uint8_t len)
{
    /* 一问一答模式: 帧率由上位机发送频率决定, 此命令仅返回 OK */
    (void)payload; (void)len;
    s_lastCmdResult = CMD_OK;
}

static void handle_trig_calib(uint8_t cmd_id, const uint8_t *payload, uint8_t len)
{
    uint8_t motor = 0;
    if (len >= 1) motor = payload[0];

    switch (cmd_id) {
    case CMD_TRIG_CURR_CALIB:
        if (motor == 1) g_DoCurrLoopCalib_M1 = 1;
        else            g_DoCurrLoopCalib_M2 = 1;
        break;
    case CMD_TRIG_KTH71_CALIB:
        if (motor == 1) g_DoKth71Calib_M1 = 1;
        else            g_DoKth71Calib_M2 = 1;
        break;
    case CMD_TRIG_ZERO_CALIB:
        if (motor == 1) g_DoZeroCalib_M1 = 1;
        else            g_DoZeroCalib_M2 = 1;
        break;
    default:
        s_lastCmdResult = CMD_ERR_UNKNOWN;
        return;
    }
    g_CommTimeoutMs = 20000;  /* 校准期间放宽看门狗 (阻塞式校准可达 15s) */
    s_lastCmdResult = CMD_OK;
}

/* ====================================================================
 * 命令派发
 * ==================================================================== */
static void dispatch_cmd(uint8_t cmd_id, const uint8_t *data, uint8_t data_len)
{
    switch (cmd_id) {
    /* 系统类 */
    case CMD_QUERY_CAPS:
        s_pendingCapsRsp = 1;
        s_lastCmdResult = CMD_OK;
        break;
    case CMD_QUERY_STATUS:
        s_forceStatusOnce = 1;
        break;

    /* 运动类 */
    case CMD_SET_MOTION:
        handle_set_motion(data, data_len);
        break;
    case CMD_SET_POSITION:
        handle_set_position(data, data_len);
        break;
    case CMD_SET_IQ_REF:
        handle_set_iq_ref(data, data_len);
        break;

    /* 模式类 */
    case CMD_SET_CTRL_MODE:
        handle_set_ctrl_mode(data, data_len);
        break;
    case CMD_SET_FOLLOW:
        handle_set_follow(data, data_len);
        break;

    /* 参数类 */
    case CMD_SET_SPEED_PI:
        handle_set_speed_pi(data, data_len);
        break;
    case CMD_SET_POS_PID:
    case CMD_SET_ID_ADAPT:
    case CMD_SET_RAMP_RATE:
        s_lastCmdResult = CMD_OK;  /* 暂存根, 后续实现 */
        break;

    /* 校准类 */
    case CMD_TRIG_CURR_CALIB:
    case CMD_TRIG_KTH71_CALIB:
    case CMD_TRIG_ZERO_CALIB:
        handle_trig_calib(cmd_id, data, data_len);
        break;
    case CMD_FLASH_UNLOCK:
        handle_flash_unlock();
        break;
    case CMD_FLASH_ERASE:
        handle_flash_erase(data, data_len);
        break;

    /* VOFA / 系统 */
    case CMD_SET_VOFA_MODE:
        handle_set_vofa_mode(data, data_len);
        break;
    case CMD_EXIT_VOFA:
        g_CommMode = COMM_MODE_PROTOCOL;
        s_lastCmdResult = CMD_OK;
        break;
    case CMD_SET_STATUS_RATE:
        handle_set_status_rate(data, data_len);
        break;

    default:
        s_lastCmdResult = CMD_ERR_UNKNOWN;
        break;
    }
}

/* ====================================================================
 * 看门狗安全动作 (超时 → 零电流自由滑行)
 * ==================================================================== */
static void watchdog_safety_action(void)
{
    g_M1_CtrlMode = MODE_OPEN_LOOP;
    g_M2_CtrlMode = MODE_OPEN_LOOP;
    g_M1_Iq_Ref_mA = 0;
    g_M2_Iq_Ref_mA = 0;
    g_M1_AngleDelta_Target = 0;
    g_M2_AngleDelta_Target = 0;
    g_M1_Idq_Ref.d = 0;  g_M1_Idq_Ref.q = 0;
    g_M2_Idq_Ref.d = 0;  g_M2_Idq_Ref.q = 0;
}

/* ====================================================================
 * API 实现
 * ==================================================================== */

void Comm_Init(void)
{
    /* ---- RX DMA: 绑定地址 + 使能 ---- */
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&USART1->RDR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)s_rxBuf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, COMM_RX_BUF_SIZE);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_USART_EnableDMAReq_RX(USART1);

    /* DMA RX 完成中断不需要 (Circular 模式, 靠 IDLE 通知) — 禁用 */
    NVIC_DisableIRQ(DMA1_Channel2_IRQn);

    /* USART1 IDLE 中断使能 (NVIC 优先级和使能由 CubeMX MX_USART1_UART_Init 配置) */
    LL_USART_ClearFlag_IDLE(USART1);
    LL_USART_EnableIT_IDLE(USART1);

    /* ---- 初始化读写指针 ---- */
    s_rxWrIdx = 0;
    s_rxRdIdx = 0;
}

void Comm_OnIdleIRQ(void)
{
    /* 读 DMA NDTR 计算当前写位置
     * NDTR 是"剩余待传输字节数", 从 COMM_RX_BUF_SIZE 递减 */
    uint16_t ndtr = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);
    s_rxWrIdx = (COMM_RX_BUF_SIZE - ndtr) & COMM_RX_BUF_MASK;
}

void Protocol_Poll(void)
{
    uint16_t avail;
    uint16_t safety_limit = 256;  /* 单次最多处理帧数, 防死循环 */

    while ((avail = rx_available()) >= COMM_MIN_FRAME_LEN && safety_limit--) {
        /* ---- 寻找帧头 0xAA 0x55 ---- */
        if (rx_peek(0) != COMM_HEADER_0) {
            rx_advance(1);
            continue;
        }
        if (rx_peek(1) != COMM_HEADER_1) {
            rx_advance(1);
            continue;
        }

        /* ---- 读 Len ---- */
        uint8_t payload_len = rx_peek(2);
        if (payload_len == 0 || payload_len > COMM_MAX_PAYLOAD_LEN) {
            rx_advance(2);  /* 无效 Len, 跳过帧头继续搜索 */
            continue;
        }

        /* ---- 检查数据是否攒够: Header(2) + Len(1) + Payload(N) + CRC(2) ---- */
        uint16_t frame_len = 3U + payload_len + 2U;
        if (avail < frame_len) {
            break;  /* 数据不足, 等下一轮 */
        }

        /* ---- 提取帧并校验 CRC ---- */
        uint8_t frame[COMM_MAX_PAYLOAD_LEN + 8];
        rx_read_block(0, frame, frame_len);

        uint16_t crc_calc = crc_ccitt(frame, 3 + payload_len);
        uint16_t crc_recv = (uint16_t)frame[3 + payload_len] |
                            ((uint16_t)frame[3 + payload_len + 1] << 8);

        if (crc_calc != crc_recv) {
            g_CrcErrCount++;
            rx_advance(1);
            continue;
        }

        /* ---- CRC 通过: 派发命令 ---- */
        uint8_t cmd_id = frame[3];
        dispatch_cmd(cmd_id, &frame[4], payload_len - 1);
        g_CommWatchdogMs = 0;  /* 有效帧 → 喂狗 */
        rx_advance(frame_len);

        /* 一问一答: 直接发送响应, 不经 TIM4 中转 */
        if (g_CommMode == COMM_MODE_PROTOCOL) {
            if (s_pendingCapsRsp) {
                s_pendingCapsRsp = 0;
                uint16_t rlen = build_caps_frame();
                dma_tx_send(s_txBuf, rlen);
            } else {
                uint16_t rlen = build_status_frame();
                dma_tx_send(s_txBuf, rlen);
            }
        } else {
            s_forceStatusOnce = 1; /* VOFA 模式: 仍用 TIM4 插帧 */
        }
    }

    /* ---- 看门狗检查 ---- */
    if (g_CommWatchdogMs > g_CommTimeoutMs) {
        watchdog_safety_action();
    }

    /* ---- SET_IQ_REF 超时保护 (HAL_GetTick() 精确 ms 计时) ---- */
    if (s_iqRefActive) {
        if (g_M1_CtrlMode != MODE_OPEN_LOOP && g_M2_CtrlMode != MODE_OPEN_LOOP) {
            s_iqRefActive = 0;  /* 离开开环模式, 取消超时跟踪 */
        } else if (HAL_GetTick() - s_iqRefLastTick > IQ_REF_TIMEOUT_MS) {
            if (g_M1_CtrlMode == MODE_OPEN_LOOP) {
                g_M1_Iq_Ref_mA = 0;
                g_M1_AngleDelta_Target = 0;
            }
            if (g_M2_CtrlMode == MODE_OPEN_LOOP) {
                g_M2_Iq_Ref_mA = 0;
                g_M2_AngleDelta_Target = 0;
            }
            s_iqRefActive = 0;
        }
    }

    /* ---- Flash 解锁超时 (HAL_GetTick 精确 ms 计时) ---- */
    if (s_flashUnlocked) {
        if (HAL_GetTick() - s_flashUnlockTick > FLASH_UNLOCK_TIMEOUT_MS) {
            s_flashUnlocked = 0;
        }
    }
}

void Comm_OnTIM4_1ms(void)
{
    /* ---- DMA 忙: 跳过本 tick ---- */
    if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1))
        return;

    /* ---- VOFA 模式 ---- */
    if (g_CommMode == COMM_MODE_VOFA) {
        /* 临时插入一帧状态帧 (QUERY_STATUS 触发) */
        if (s_forceStatusOnce) {
            s_forceStatusOnce = 0;
            uint16_t len = build_status_frame();
            dma_tx_send(s_txBuf, len);
            return;
        }
        /* 优先响应 CAPS 查询 */
        if (s_pendingCapsRsp) {
            s_pendingCapsRsp = 0;
            uint16_t len = build_caps_frame();
            dma_tx_send(s_txBuf, len);
            return;
        }
        /* 正常 VOFA 路径: 确保 DMA 源地址指向 VofaFrame (可能被协议帧改过) */
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&g_VofaFrame);
        Vofa_OnTIM4_1ms();
        return;
    }

    /* ---- PROTOCOL 模式: 响应已在 Protocol_Poll 直接发送, TIM4 无需处理 ---- */
}
