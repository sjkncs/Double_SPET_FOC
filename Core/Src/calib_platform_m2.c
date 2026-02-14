/**
 * @file    calib_platform_m2.c
 * @brief   M2 电流环自校准平台层 — 回调 + 启动/完成 + TIM3 驱动
 *
 * 从 main.c 拆出, 使校准平台逻辑与主循环编排分离.
 * 本模块包含:
 *   - ISR 累加器均值读取回调 (setVd / getIdq / getVdq / getVbus)
 *   - 校准启动 (切模式 + Init + Start)
 *   - 校准完成 (写 PI + Flash + VOFA + 恢复)
 *   - TIM3 1ms 驱动入口
 *   - VOFA 实时监控
 */

#include "calib_platform_m2.h"
#include "main.h"
#include "curr_loop_autocalib.h"
#include "flash_params.h"
#include "foc_adapt.h"
#include "vofa_engine.h"
#include <string.h>

/* ---- 所有 PWM 输出位掩码 (关/恢复 PWM 用, 避免重复展开) ---- */
#define HRTIM_ALL_OUTPUTS  (LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | \
                            LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TB2 | \
                            LL_HRTIM_OUTPUT_TC1 | LL_HRTIM_OUTPUT_TC2 | \
                            LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2)

/* ---- 文件内静态变量 ---- */
static CurrLoopCalib_Handle_t s_calibHandle;   /**< 校准引擎句柄 (不暴露给外部) */

/* ====================================================================
 * 平台回调: ISR 累加器均值读取
 *
 * 17kHz FOC ISR 每拍累加 Vd/Id, TIM3 每 1ms 调用校准引擎,
 * 引擎内部通过这些回调读取上一个 1ms 窗口的平均值.
 * ==================================================================== */

/** @brief 设置 D 轴开环电压 (ISR 直接写 PWM, 跳过 PI) */
static void plat_setVd(int16_t vd_q15)
{
    g_CalibVdDirect = vd_q15;
}

/** @brief 读取 1ms 平均 Id (ISR 累加值 / 累加次数) */
static void plat_getIdq(int16_t *id_q15, int16_t *iq_q15)
{
    uint16_t cnt = g_CalibAccCnt;
    if (cnt > 0)
        *id_q15 = (int16_t)(g_CalibIdAcc / (int32_t)cnt);
    else
        *id_q15 = g_M2_Idq.d;
    *iq_q15 = g_M2_Idq.q;
}

/**
 * @brief 读取 1ms 平均 Vd, 并清零累加器为下一窗口准备
 *
 * 注意: 读完立即清零, 每个 1ms 窗口只读一次.
 */
static void plat_getVdq(int16_t *vd_q15, int16_t *vq_q15)
{
    uint16_t cnt = g_CalibAccCnt;
    if (cnt > 0)
        *vd_q15 = (int16_t)(g_CalibVdAcc / (int32_t)cnt);
    else
        *vd_q15 = g_M2_Vdq.d;
    *vq_q15 = g_M2_Vdq.q;
    /* 读完清零累加器，为下一个 1ms 窗口准备 */
    g_CalibVdAcc  = 0;
    g_CalibIdAcc  = 0;
    g_CalibAccCnt = 0;
}

/** @brief 读取当前母线电压 (mV) */
static uint32_t plat_getVbusMv(void) { return g_Vbus_mV; }

/* ====================================================================
 * CalibM2_Start — 启动 M2 电流环自校准
 *
 * 切开环, 清零角度/电流/PI积分, 初始化校准引擎, 由 TIM3 1ms 驱动.
 * ==================================================================== */
void CalibM2_Start(void)
{
    g_CurrLoopCalibInProgress = 1;
    g_CalibVdDirectActive     = 1;      /* ISR 跳过 PI, 直接写 Vd */
    g_CalibVdDirect           = 0;
    g_M2_CtrlMode = M2_MODE_OPEN_LOOP;
    g_M2_AngleDelta_Target = 0;
    g_M2_ElecAngle_Q15 = 0;
    g_M2_Idq_Ref.d = 0;    /* PI 不参与，清零以防万一 */
    g_M2_Idq_Ref.q = 0;
    /* 清 PI 积分项，校准结束恢复时从零开始 */
    g_M2_PI_d.integral = 0;
    g_M2_PI_q.integral = 0;

    CurrLoopCalib_Platform_t plat = {
        .set_vd_q15    = plat_setVd,
        .get_idq_q15   = plat_getIdq,
        .get_vdq_q15   = plat_getVdq,
        .get_vbus_mv   = plat_getVbusMv
    };
    CurrLoopCalib_Params_t params;
    CurrLoopCalib_Params_SetDefaults(&params);
    params.control_freq_hz  = (float)ENC_ISR_FREQ_HZ;
    params.current_scale_si = (float)I_FULLSCALE_MA / 1000.f;

    CurrLoopCalib_Init(&s_calibHandle, &plat, &params);
    CurrLoopCalib_Start(&s_calibHandle);

    g_VofaSrc = VOFA_SRC_CURR_CALIB;   /* TIM4 自动发送校准监控帧 */
}

/* ====================================================================
 * CalibM2_OnDone — 校准完成: 写 PI + Flash + VOFA + 恢复
 *
 * 流程:
 *   1. 读校准结果 (Rs, Ls, success)
 *   2. 计算 PI 增益 (×0.8 安全裕量)
 *   3. 更新 PI 控制器 + 基准参数
 *   4. 关 PWM → Flash erase+program (~40ms) → 恢复 PWM
 *   5. VOFA 发送最终结果
 *   6. 恢复 PI 控制 (关闭电压直驱)
 * ==================================================================== */
void CalibM2_OnDone(void)
{
    CurrLoopCalib_Result_t res;
    CurrLoopCalib_GetResult(&s_calibHandle, &res);
    if (res.success) {
        int32_t kp, ki;
        CurrLoopCalib_ComputePI_Q15(&res, g_Vbus_mV, I_FULLSCALE_MA,
                                    (float)ENC_ISR_FREQ_HZ, &kp, &ki);
        /* ×0.8 安全裕量：吸收 R/L 测量误差 + 温漂（等效 margin 5→6.25） */
        kp = (int32_t)((int64_t)kp * 4 / 5);
        ki = (int32_t)((int64_t)ki * 4 / 5);
        g_M2_PI_d.kp = kp;  g_M2_PI_d.ki = ki;  g_M2_PI_d.integral = 0;
        g_M2_PI_q.kp = kp;  g_M2_PI_q.ki = ki;  g_M2_PI_q.integral = 0;

        /* 更新 Vbus 自适应基准 + dq 解耦 Ls */
        g_M2_PI_Base.kp      = kp;
        g_M2_PI_Base.ki      = ki;
        g_M2_PI_Base.vbus_mv = g_Vbus_mV;
        FOC_SetM2Ls(res.ls_henry);

        /* ---- Flash 持久化 (关 PWM → erase+program ~40ms → 恢复 PWM) ---- */
        /* RAMP_DOWN 已将电压降至 0, 关 PWM 对电机无影响 */
        LL_HRTIM_DisableOutput(HRTIM1, HRTIM_ALL_OUTPUTS);

        /* 读取现有 Flash 数据 (保留 M1 校准), 更新 M2 字段 */
        FlashCalibData_t flash_data;
        if (!FlashParams_Read(&flash_data)) {
            /* 首次: 清零整个结构 */
            memset(&flash_data, 0, sizeof(flash_data));
        }
        flash_data.m2_rs_ohm         = res.rs_ohm;
        flash_data.m2_ls_henry       = res.ls_henry;
        flash_data.m2_kp             = kp;
        flash_data.m2_ki             = ki;
        flash_data.m2_vbus_calib_mv  = g_Vbus_mV;
        flash_data.m2_valid          = 1;
        flash_data.calib_count++;
        FlashParams_Write(&flash_data);   /* CPU stall ~40ms */

        /* 恢复 PWM 输出 */
        LL_HRTIM_EnableOutput(HRTIM1, HRTIM_ALL_OUTPUTS);
    }
    /* VOFA 最终结果帧 (TIM4 下一 1ms 自动发送, 之后切回 NORMAL)
     * 通道含义同 CalibM2_FillVofa:
     * I0: 状态 (7=DONE 成功, 8=ERROR 失败)
     * I1: 0 (结束)
     * I2: 错误码 (0=无错误)
     * I3: DC 测量电流 (A), 目标 ~0.3
     * I4: Rs 相电阻 (Ω)
     * I5: Ls 相电感 (mH) */
    g_VofaFrame.ch[0] = (float)(uint8_t)(res.success ? CURR_CALIB_STATE_DONE : CURR_CALIB_STATE_ERROR);
    g_VofaFrame.ch[1] = 0.f;
    g_VofaFrame.ch[2] = (float)res.error_code;
    g_VofaFrame.ch[3] = res.debug_dc_id_avg_si;
    g_VofaFrame.ch[4] = res.rs_ohm;
    g_VofaFrame.ch[5] = res.ls_henry * 1000.0f;
    g_VofaFrame.ch[6] = 0.0f;
    g_VofaSrc = VOFA_SRC_NORMAL;   /* 恢复正常 VOFA 数据源 */
    /* 恢复 PI 控制（RAMP_DOWN 已将电压降至 0，切回 PI 不会有跳变） */
    g_CalibVdDirect       = 0;
    g_CalibVdDirectActive = 0;
    g_CurrLoopCalibInProgress = 0;
}

/* ====================================================================
 * CalibM2_OnTIM3_1ms — TIM3 1ms 中断入口
 *
 * 若校准未启动则直接返回 (约 3 条指令开销).
 * 校准引擎返回 DONE/ERROR 时置 g_CurrLoopCalibResultReady,
 * 由主循环下一轮调用 CalibM2_OnDone() 处理.
 * ==================================================================== */
void CalibM2_OnTIM3_1ms(void)
{
    if (!g_CurrLoopCalibInProgress) return;
    CurrLoopCalib_State_t st = CurrLoopCalib_Run(&s_calibHandle);
    if (st == CURR_CALIB_STATE_DONE || st == CURR_CALIB_STATE_ERROR)
        g_CurrLoopCalibResultReady = 1;
}

/* ====================================================================
 * CalibM2_FillVofa — 校准进行中: 填充 VOFA 帧 (由 TIM4 vofa_engine 调用)
 *
 * 仅填 g_VofaFrame 通道, 不触发 DMA (TIM4 统一发送).
 *
 * VOFA 通道分配:
 *   I0: 校准状态 (0=IDLE, 1=DC_RAMP, 2=DC_SETTLE, 3=DC_MEASURE,
 *                 4=AC_SETTLE, 5=AC_MEASURE, 6=RAMP_DOWN, 7=DONE, 8=ERROR)
 *   I1: 状态内计数器 (ms)
 *   I2: 错误码 (0=无错误)
 *   I3: DC 测量电流 (A), 目标 ~0.3
 *   I4: Rs 相电阻 (Ω)
 *   I5: Ls 相电感 (mH)
 * ==================================================================== */
void CalibM2_FillVofa(void)
{
    CurrLoopCalib_Result_t res;
    CurrLoopCalib_State_t st;
    uint32_t cnt;
    CurrLoopCalib_GetResult(&s_calibHandle, &res);
    CurrLoopCalib_GetStateAndCounter(&s_calibHandle, &st, &cnt);
    g_VofaFrame.ch[0] = (float)(uint8_t)st;          /* I0: 状态 (7=DONE) */
    g_VofaFrame.ch[1] = (float)cnt;                   /* I1: 计数 ms */
    g_VofaFrame.ch[2] = (float)res.error_code;        /* I2: 错误码 */
    g_VofaFrame.ch[3] = res.debug_dc_id_avg_si;       /* I3: DC 电流 A */
    g_VofaFrame.ch[4] = res.rs_ohm;                   /* I4: Rs Ω */
    g_VofaFrame.ch[5] = res.ls_henry * 1000.0f;       /* I5: Ls mH */
    g_VofaFrame.ch[6] = 0.0f;
}
