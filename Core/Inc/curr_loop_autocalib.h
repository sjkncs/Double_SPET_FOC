/**
 * @file  curr_loop_autocalib.h
 * @brief 电流环自校准：开环电压注入测 Rs/Ls + ST 公式算 PI
 *
 * 参考 ST MCSDK profiler_dcac.c + profiler_impedest.c 的开环方案：
 *   - DC 阶段：斜坡升压（开环 duty），测稳态 Rs = Vd/Id
 *   - AC 阶段：叠加正弦电压，同步解调测 Z → Rs_ac, Ls
 *   - 全部状态由 1ms 定时器驱动，无阻塞延时
 */
#ifndef CURR_LOOP_AUTOCALIB_H
#define CURR_LOOP_AUTOCALIB_H

#include <stdint.h>
#include <stddef.h>

/* ---- 状态枚举 ---- */
typedef enum {
    CURR_CALIB_STATE_IDLE = 0,
    CURR_CALIB_STATE_DC_RAMP,       /* 1: 斜坡升压 */
    CURR_CALIB_STATE_DC_SETTLE,     /* 2: 等稳态 */
    CURR_CALIB_STATE_DC_MEASURE,    /* 3: DC 测量 Rs */
    CURR_CALIB_STATE_AC_SETTLE,     /* 4: AC 注入前等稳态 */
    CURR_CALIB_STATE_AC_MEASURE,    /* 5: AC 注入 + 解调 Ls */
    CURR_CALIB_STATE_RAMP_DOWN,     /* 6: 降压归零 */
    CURR_CALIB_STATE_DONE   = 7,
    CURR_CALIB_STATE_ERROR  = 0xFF
} CurrLoopCalib_State_t;

/* ---- 平台回调（开环电压注入） ---- */
typedef struct {
    void     (*set_vd_q15)(int16_t vd_q15);              /* 直接设置 D 轴电压 Q15 */
    void     (*get_idq_q15)(int16_t *id, int16_t *iq);   /* 读实测电流 (ISR 均值) */
    void     (*get_vdq_q15)(int16_t *vd, int16_t *vq);   /* 读实际施加电压 (ISR 均值)，读后清累加器 */
    uint32_t (*get_vbus_mv)(void);
} CurrLoopCalib_Platform_t;

/* ---- 参数 ---- */
typedef struct {
    float    control_freq_hz;           /* PI 执行频率 (Hz), 用于算 Kp/Ki */
    float    current_scale_si;          /* 电流满量程 (A) */
    float    rs_min_ohm, rs_max_ohm;    /* Rs 有效范围 */
    float    ls_min_henry, ls_max_henry;/* Ls 有效范围 */
    /* DC 斜坡 */
    float    dc_current_limit_amp;      /* 电流安全限值 (A)：到达后停止升压 */
    int16_t  dc_duty_max_q15;           /* 最大占空比 Q15 */
    uint16_t dc_ramp_rate_q15_per_ms;   /* 每 ms 升压步长 Q15 */
    uint16_t dc_settle_ms;              /* 升压完等稳态 ms */
    uint16_t dc_measure_samples;        /* DC 测量 1ms 采样数 */
    /* AC 注入 */
    float    ac_inject_freq_hz;         /* 注入频率 (Hz) */
    float    ac_duty_fraction;          /* AC 幅值 = dc_duty × 此系数 (典型 0.5) */
    uint16_t ac_settle_ms;              /* AC 前等稳态 ms */
    uint16_t ac_measure_samples;        /* AC 解调 1ms 采样数 (应为整数周期) */
    uint16_t rampdown_rate_q15_per_ms;  /* 降压速率 Q15/ms */
} CurrLoopCalib_Params_t;

/* ---- 结果 ---- */
typedef struct {
    uint8_t  success;
    uint8_t  error_code;        /* 0=OK, 1=Rs异常, 2=Ls异常, 3=DC电流太小, 4=AC信号太弱 */
    float    rs_ohm;            /* AC 阻抗实部 */
    float    ls_henry;          /* AC 阻抗虚部 → 电感 */
    float    debug_dc_vd_avg_si;/* DC 阶段 Vd 均值 (V) */
    float    debug_dc_id_avg_si;/* DC 阶段 Id 均值 (A) */
} CurrLoopCalib_Result_t;

/* ---- 不透明句柄 ---- */
#define CURR_LOOP_CALIB_HANDLE_SIZE  320
typedef uint8_t CurrLoopCalib_Handle_t[CURR_LOOP_CALIB_HANDLE_SIZE]
    __attribute__((aligned(4)));

/* ---- API ---- */
void CurrLoopCalib_Params_SetDefaults(CurrLoopCalib_Params_t *params);

void CurrLoopCalib_Init(CurrLoopCalib_Handle_t *h,
                         const CurrLoopCalib_Platform_t *platform,
                         const CurrLoopCalib_Params_t *params);

void CurrLoopCalib_Start(CurrLoopCalib_Handle_t *h);

/** 每 1ms 调用一次（TIM3），驱动所有状态（全部非阻塞） */
CurrLoopCalib_State_t CurrLoopCalib_Run(CurrLoopCalib_Handle_t *h);

void CurrLoopCalib_GetStateAndCounter(CurrLoopCalib_Handle_t *h,
                                      CurrLoopCalib_State_t *state_out,
                                      uint32_t *counter_out);

void CurrLoopCalib_GetResult(CurrLoopCalib_Handle_t *h,
                              CurrLoopCalib_Result_t *out);

/** PI 系数换算（Rs/Ls → Q15 定点 Kp/Ki） */
void CurrLoopCalib_ComputePI_Q15(const CurrLoopCalib_Result_t *result,
                                  uint32_t vbus_mv, int32_t i_fullscale_ma,
                                  float control_freq_hz,
                                  int32_t *kp_q15, int32_t *ki_q15);

#endif /* CURR_LOOP_AUTOCALIB_H */
