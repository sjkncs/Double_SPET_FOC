/**
 * @file    comm_protocol.h
 * @brief   RK3576 ↔ STM32G474 二进制通信协议
 *
 * 架构:
 *   RX: DMA1_CH2 Circular → 128B 环形缓冲区, IDLE ISR 仅更新写指针
 *   解析: while(1) Protocol_Poll() — CRC 校验 + 命令派发
 *   TX: TIM4 ISR Comm_OnTIM4_1ms() 按 g_CommMode 选择填协议状态帧或调 VOFA 引擎
 *
 * 帧格式:
 *   [0xAA][0x55][Len(1B)][Payload(N B)][CRC16(2B)]
 *   CRC-CCITT (poly=0x1021, init=0xFFFF), 覆盖 Header + Len + Payload
 *   最小帧: 2+1+1+2 = 6B (仅 CmdID, 无 Data)
 *
 * 与 VOFA 引擎共存:
 *   vofa_engine.c/h 完整保留不修改.
 *   Comm_OnTIM4_1ms() 按 g_CommMode 分流:
 *     PROTOCOL → 填协议状态帧 + DMA
 *     VOFA     → 调 Vofa_OnTIM4_1ms()
 */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ====================================================================
 * 协议常量
 * ==================================================================== */
#define COMM_PROTO_VER        1U      /**< 协议版本号 */
#define COMM_FW_VERSION       0x0100U /**< 固件版本 v1.0 (主版本×0x100+次版本) */

#define COMM_HEADER_0         0xAAU
#define COMM_HEADER_1         0x55U
#define COMM_MIN_FRAME_LEN    6U      /**< 最小帧: Header(2)+Len(1)+CmdID(1)+CRC(2) */
#define COMM_MAX_PAYLOAD_LEN  64U     /**< 单帧 Payload 最大字节数 */

#define COMM_RX_BUF_SIZE      128U    /**< DMA 环形接收缓冲区大小 (2 的幂次) */
#define COMM_RX_BUF_MASK      (COMM_RX_BUF_SIZE - 1U)

/* 状态帧发送速率 (TIM4 tick 分频) */
#define COMM_STATUS_RATE_HZ   100U    /**< 默认 100Hz */
#define COMM_STATUS_DIV       (1000U / COMM_STATUS_RATE_HZ)

/* ====================================================================
 * 通信模式
 * ==================================================================== */
typedef enum {
    COMM_MODE_PROTOCOL = 0, /**< 默认: TX 发协议状态帧 */
    COMM_MODE_VOFA,         /**< 调试: TX 发 VOFA JustFloat 帧 */
} CommMode_t;

/* ====================================================================
 * 命令 ID (RK3576 → STM32)
 * ==================================================================== */
typedef enum {
    /* 系统类 */
    CMD_QUERY_CAPS       = 0x01,  /**< 握手: 查询固件能力 */

    /* 运动类 (高频路径) */
    CMD_SET_MOTION       = 0x10,  /**< M1增量+M2绝对位置 (≤200Hz) */
    CMD_SET_POSITION     = 0x11,  /**< 单轴绝对位置 (偶发) */
    CMD_SET_IQ_REF       = 0x12,  /**< 开环力矩 (仅 OpenLoop 模式) */

    /* 模式类 */
    CMD_SET_CTRL_MODE    = 0x20,  /**< 切控制模式 */
    CMD_SET_FOLLOW       = 0x21,  /**< 主从随动开关 */

    /* 参数类 */
    CMD_SET_SPEED_PI     = 0x30,  /**< 速度环 PI */
    CMD_SET_POS_PID      = 0x31,  /**< 位置/步距角 PD */
    CMD_SET_ID_ADAPT     = 0x32,  /**< Id 自适应参数 */
    CMD_SET_RAMP_RATE    = 0x33,  /**< 斜坡速率 */

    /* 校准类 */
    CMD_TRIG_CURR_CALIB  = 0x40,  /**< 触发电流环校准 */
    CMD_TRIG_KTH71_CALIB = 0x41,  /**< 触发 ANLC 校准 */
    CMD_TRIG_ZERO_CALIB  = 0x42,  /**< 触发零点标定 */
    CMD_FLASH_UNLOCK     = 0x43,  /**< Flash 解锁 (第一步) */
    CMD_FLASH_ERASE      = 0x44,  /**< Flash 擦除 (第二步, 需 Magic) */

    /* 系统类 */
    CMD_QUERY_STATUS     = 0x50,  /**< 请求状态帧 */
    CMD_SET_VOFA_MODE    = 0x51,  /**< 进入 VOFA 模式 */
    CMD_EXIT_VOFA        = 0x52,  /**< 退出 VOFA → 协议模式 */
    CMD_SET_STATUS_RATE  = 0x53,  /**< 设状态上报频率 */
} CommCmdId_t;

/* ====================================================================
 * 响应 ID (STM32 → RK3576)
 * ==================================================================== */
typedef enum {
    RSP_STATUS           = 0x80,  /**< 周期状态帧 */
    RSP_CAPS             = 0x81,  /**< QUERY_CAPS 响应 */
} CommRspId_t;

/* ====================================================================
 * 命令执行结果 (状态帧 LastCmdResult)
 * ==================================================================== */
typedef enum {
    CMD_OK               = 0x00,
    CMD_ERR_UNKNOWN      = 0x01,  /**< 未知命令 */
    CMD_ERR_WRONG_MODE   = 0x02,  /**< 模式不匹配 (如非开环下发 SET_IQ_REF) */
    CMD_ERR_CALIB_ACTIVE = 0x03,  /**< 校准进行中 */
    CMD_ERR_MOTOR_ACTIVE = 0x04,  /**< 电机运行中 (Flash 操作需停机) */
    CMD_ERR_FLASH_LOCKED = 0x05,  /**< Flash 未解锁 */
    CMD_ERR_BAD_PARAM    = 0x06,  /**< 参数越界 */
} CommCmdResult_t;

/* ====================================================================
 * SysFlags 位定义 (状态帧 [5])
 * ==================================================================== */
#define SYSFLAG_FLASH_LOADED    (1U << 0)  /**< Flash 校准参数已加载 */
#define SYSFLAG_CALIB_ACTIVE    (1U << 1)  /**< 任一电机校准中 */
#define SYSFLAG_FLASH_UNLOCKED  (1U << 2)  /**< Flash 擦除已解锁 */
#define SYSFLAG_COMM_WDG        (1U << 3)  /**< 通信看门狗已触发 */
#define SYSFLAG_VOFA_MODE       (1U << 4)  /**< 当前 VOFA 模式 */

/* ====================================================================
 * MotorFlags 位定义 (状态帧 M1_Flags / M2_Flags)
 * ==================================================================== */
#define MFLAG_VQ_SAT            (1U << 0)  /**< Vq 饱和 */
#define MFLAG_CALIB_ACTIVE      (1U << 1)  /**< 该电机校准中 */

/* ====================================================================
 * 命令 Payload 结构体 (packed, 不含 CmdID 字节)
 * ==================================================================== */

/** SET_MOTION (0x10): 主运动指令 — M1 位置增量 + M2 绝对位置 */
typedef struct __attribute__((packed)) {
    uint8_t  seq_no;        /**< 0~255 循环序列号 */
    int16_t  m1_delta;      /**< M1 增量 (encoder counts) */
    int16_t  m2_target;     /**< M2 绝对位置 (encoder counts, ±16384=±90°) */
} CommSetMotion_t;
_Static_assert(sizeof(CommSetMotion_t) == 5, "SET_MOTION payload");

/** SET_POSITION (0x11): 单轴绝对位置 */
typedef struct __attribute__((packed)) {
    uint8_t  motor;         /**< 0=M2, 1=M1 */
    int32_t  target;        /**< 绝对位置 (encoder counts) */
} CommSetPosition_t;
_Static_assert(sizeof(CommSetPosition_t) == 5, "SET_POSITION payload");

/** SET_IQ_REF (0x12): 开环力矩 + 磁场
 *  向后兼容: 4B(仅Iq) 或 8B(Iq+Id) 均可接受 */
typedef struct __attribute__((packed)) {
    int16_t  m1_iq_ma;      /**< M1 Iq (mA) */
    int16_t  m2_iq_ma;      /**< M2 Iq (mA) */
    int16_t  m1_id_ma;      /**< M1 Id (mA), 可选 — 4B payload 时不含此字段 */
    int16_t  m2_id_ma;      /**< M2 Id (mA), 可选 */
} CommSetIqRef_t;
_Static_assert(sizeof(CommSetIqRef_t) == 8, "SET_IQ_REF payload");
#define COMM_SET_IQ_REF_MIN_LEN  4U  /**< 最小 payload: 仅 Iq (向后兼容) */

/** SET_CTRL_MODE (0x20): 切控制模式 */
typedef struct __attribute__((packed)) {
    uint8_t  motor;         /**< 0=M2, 1=M1, 2=Both */
    uint8_t  mode;          /**< MotorCtrlMode_t 枚举值 */
} CommSetCtrlMode_t;
_Static_assert(sizeof(CommSetCtrlMode_t) == 2, "SET_CTRL_MODE payload");

/** SET_VOFA_MODE (0x51) */
typedef struct __attribute__((packed)) {
    uint8_t  vofa_src;      /**< VofaSrc_t 枚举值 */
    uint8_t  motor_sel;     /**< 0=M2, 1=M1 */
} CommSetVofaMode_t;
_Static_assert(sizeof(CommSetVofaMode_t) == 2, "SET_VOFA_MODE payload");

/** SET_STATUS_RATE (0x53) */
typedef struct __attribute__((packed)) {
    uint16_t rate_hz;       /**< 状态帧频率 (Hz), 1~1000 */
} CommSetStatusRate_t;
_Static_assert(sizeof(CommSetStatusRate_t) == 2, "SET_STATUS_RATE payload");

/** FLASH_ERASE (0x44): 第二步, 携带 Magic */
typedef struct __attribute__((packed)) {
    uint16_t magic;         /**< 必须 = 0xDEAD */
} CommFlashErase_t;
_Static_assert(sizeof(CommFlashErase_t) == 2, "FLASH_ERASE payload");

/** SET_FOLLOW (0x21) */
typedef struct __attribute__((packed)) {
    uint8_t  enable;        /**< 0=关闭, 1=启用 */
} CommSetFollow_t;
_Static_assert(sizeof(CommSetFollow_t) == 1, "SET_FOLLOW payload");

/* ====================================================================
 * 状态帧结构体 (STM32 → RK3576, packed)
 *
 * 速度单位: milliRPM (0.001 RPM), int32_t
 *   范围 ±2,147,483 RPM, 开环低速可精确表示 0.001 RPM
 * ==================================================================== */

/** 单电机状态块 (16 字节) */
typedef struct __attribute__((packed)) {
    uint8_t  ctrl_mode;     /**< MotorCtrlMode_t */
    uint8_t  flags;         /**< MFLAG_xxx 位掩码 */
    int32_t  speed_mrpm;    /**< 滤波转速 (milliRPM, 0.001 RPM) */
    int32_t  pos_fbk;       /**< 编码器位置 (counts) */
    int16_t  id_ma;         /**< Id 反馈 (mA) */
    int16_t  iq_ma;         /**< Iq 反馈 (mA) */
    int16_t  id_eff_ma;     /**< Id 有效值 (三级自适应输出, mA) */
} CommMotorBlock_t;
_Static_assert(sizeof(CommMotorBlock_t) == 16, "motor block size");

/** 完整状态帧 Payload (43 字节) */
typedef struct __attribute__((packed)) {
    uint8_t  cmd_id;            /**< = RSP_STATUS (0x80) */
    uint8_t  tx_seq_no;         /**< 状态帧自身序列号 (TIM4 递增) */
    uint8_t  last_rx_seq;       /**< 回报最后收到的命令 SeqNo */
    uint8_t  up_seq_no;         /**< while(1) 递增 — 主循环存活证明 */
    uint8_t  proto_ver;         /**< = COMM_PROTO_VER */
    uint8_t  sys_flags;         /**< SYSFLAG_xxx */
    uint8_t  last_cmd_result;   /**< CommCmdResult_t */
    uint16_t vbus_mv;           /**< 母线电压 (mV) */
    uint16_t crc_err_count;     /**< 累计 CRC 校验失败次数 */
    CommMotorBlock_t m1;        /**< M1 状态 */
    CommMotorBlock_t m2;        /**< M2 状态 */
} CommStatusPayload_t;
_Static_assert(sizeof(CommStatusPayload_t) == 43, "status payload size");

/** QUERY_CAPS 响应 Payload (9 字节) */
typedef struct __attribute__((packed)) {
    uint8_t  cmd_id;            /**< = RSP_CAPS (0x81) */
    uint16_t fw_version;        /**< 固件版本 */
    uint8_t  proto_ver;         /**< 协议版本 */
    uint8_t  axis_count;        /**< 轴数 = 2 */
    uint32_t features;          /**< 特性 bitmask */
} CommCapsPayload_t;
_Static_assert(sizeof(CommCapsPayload_t) == 9, "CAPS payload size");

/* 特性标志 (QUERY_CAPS 响应 features 字段) */
#define FEAT_VOFA           (1U << 0)
#define FEAT_FOLLOW         (1U << 1)
#define FEAT_FLASH_PARAMS   (1U << 2)
#define FEAT_CURR_CALIB     (1U << 3)
#define FEAT_KTH71_CALIB    (1U << 4)
#define FEAT_ZERO_CALIB     (1U << 5)
#define FEAT_STEP_CAPTURE   (1U << 6)

/* ====================================================================
 * 外部全局变量
 * ==================================================================== */
extern volatile CommMode_t g_CommMode;
extern volatile uint8_t    g_UpSeqNo;      /**< while(1) 递增, 主循环存活证明 */
extern volatile uint16_t   g_CrcErrCount;  /**< CRC 校验失败累计 */
extern volatile uint16_t   g_CommWatchdogMs;/**< 通信看门狗计数 (TIM3 递增, 有效帧清零) */
extern volatile uint16_t   g_CommTimeoutMs; /**< 看门狗超时阈值 (ms) */

/* ====================================================================
 * API
 * ==================================================================== */

/**
 * @brief  通信模块初始化
 *
 * 绑定 DMA1_CH2 → USART1_RDR (RX Circular),
 * 使能 USART1 IDLE 中断标志 (NVIC 优先级由 CubeMX 配置),
 * 禁用不需要的 DMA1_CH2 完成中断.
 *
 * 调用时机: main() 中, MX_USART1_UART_Init() 之后, while(1) 之前.
 */
void Comm_Init(void);

/**
 * @brief  协议轮询 — while(1) 每轮调用
 *
 * 从 DMA 环形缓冲区解析帧:
 *   查找 0xAA55 → 读 Len → 攒够字节 → CRC 校验 → dispatch
 * CRC 失败: g_CrcErrCount++, 跳过继续找帧头.
 */
void Protocol_Poll(void);

/**
 * @brief  TIM4 1kHz 中断服务调度层
 *
 * 替代原 TIM4_IRQHandler 中直接调用 Vofa_OnTIM4_1ms():
 *   PROTOCOL 模式 → 按配置频率填状态帧 + DMA 发送
 *   VOFA 模式     → 确保 DMA 源地址正确后调 Vofa_OnTIM4_1ms()
 *
 * DMA 忙时跳过, 下一 tick 再发.
 */
void Comm_OnTIM4_1ms(void);

/**
 * @brief  USART1 IDLE 中断处理
 *
 * 由 USART1_IRQHandler 调用.
 * 仅读取 DMA NDTR 更新环形缓冲区写指针, 不做帧解析.
 */
void Comm_OnIdleIRQ(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_PROTOCOL_H */
