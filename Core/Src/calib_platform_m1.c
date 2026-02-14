/**
 * @file    calib_platform_m1.c
 * @brief   M1 电流环自校准平台层 — 镜像 calib_platform_m2, 适配 M1 硬件
 *
 * 结构与 M2 完全一致:
 *   - ISR 累加器均值读取回调 (setVd / getIdq / getVdq / getVbus)
 *   - 校准启动 (切模式 + Init + Start)
 *   - 校准完成 (写 PI + Flash + VOFA + 恢复)
 *   - TIM3 1ms 驱动入口
 *   - VOFA 实时监控
 */

#include "calib_platform_m1.h"
#include "main.h"
#include "curr_loop_autocalib.h"
#include "flash_params.h"
#include "foc_adapt.h"
#include "vofa_engine.h"
#include <string.h>

#define HRTIM_ALL_OUTPUTS  (LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | \
                            LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TB2 | \
                            LL_HRTIM_OUTPUT_TC1 | LL_HRTIM_OUTPUT_TC2 | \
                            LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2)

static CurrLoopCalib_Handle_t s_calibHandle_M1;

/* ---- 平台回调: M1 ISR 累加器 ---- */

static void plat_setVd_M1(int16_t vd_q15)
{
    g_CalibVdDirect_M1 = vd_q15;
}

static void plat_getIdq_M1(int16_t *id_q15, int16_t *iq_q15)
{
    uint16_t cnt = g_CalibAccCnt_M1;
    if (cnt > 0)
        *id_q15 = (int16_t)(g_CalibIdAcc_M1 / (int32_t)cnt);
    else
        *id_q15 = g_M1_Idq.d;
    *iq_q15 = g_M1_Idq.q;
}

static void plat_getVdq_M1(int16_t *vd_q15, int16_t *vq_q15)
{
    uint16_t cnt = g_CalibAccCnt_M1;
    if (cnt > 0)
        *vd_q15 = (int16_t)(g_CalibVdAcc_M1 / (int32_t)cnt);
    else
        *vd_q15 = g_M1_Vdq.d;
    *vq_q15 = g_M1_Vdq.q;
    g_CalibVdAcc_M1  = 0;
    g_CalibIdAcc_M1  = 0;
    g_CalibAccCnt_M1 = 0;
}

static uint32_t plat_getVbusMv_M1(void) { return g_Vbus_mV; }

/* ---- CalibM1_Start ---- */
void CalibM1_Start(void)
{
    g_CurrLoopCalibInProgress_M1 = 1;
    g_CalibVdDirectActive_M1    = 1;
    g_CalibVdDirect_M1          = 0;
    g_M1_CtrlMode = M2_MODE_OPEN_LOOP;
    g_M1_AngleDelta_Target = 0;
    g_M1_ElecAngle_Q15 = 0;
    g_M1_Idq_Ref.d = 0;
    g_M1_Idq_Ref.q = 0;
    g_M1_PI_d.integral = 0;
    g_M1_PI_q.integral = 0;

    CurrLoopCalib_Platform_t plat = {
        .set_vd_q15    = plat_setVd_M1,
        .get_idq_q15   = plat_getIdq_M1,
        .get_vdq_q15   = plat_getVdq_M1,
        .get_vbus_mv   = plat_getVbusMv_M1
    };
    CurrLoopCalib_Params_t params;
    CurrLoopCalib_Params_SetDefaults(&params);
    params.control_freq_hz  = (float)ENC_ISR_FREQ_HZ;
    params.current_scale_si = (float)I_FULLSCALE_MA / 1000.f;

    CurrLoopCalib_Init(&s_calibHandle_M1, &plat, &params);
    CurrLoopCalib_Start(&s_calibHandle_M1);

    g_VofaSrc = VOFA_SRC_CURR_CALIB_M1;
}

/* ---- CalibM1_OnDone ---- */
void CalibM1_OnDone(void)
{
    CurrLoopCalib_Result_t res;
    CurrLoopCalib_GetResult(&s_calibHandle_M1, &res);
    if (res.success) {
        int32_t kp, ki;
        CurrLoopCalib_ComputePI_Q15(&res, g_Vbus_mV, I_FULLSCALE_MA,
                                    (float)ENC_ISR_FREQ_HZ, &kp, &ki);
        kp = (int32_t)((int64_t)kp * 4 / 5);
        ki = (int32_t)((int64_t)ki * 4 / 5);
        g_M1_PI_d.kp = kp;  g_M1_PI_d.ki = ki;  g_M1_PI_d.integral = 0;
        g_M1_PI_q.kp = kp;  g_M1_PI_q.ki = ki;  g_M1_PI_q.integral = 0;

        g_M1_PI_Base.kp      = kp;
        g_M1_PI_Base.ki      = ki;
        g_M1_PI_Base.vbus_mv = g_Vbus_mV;
        FOC_SetM1Ls(res.ls_henry);

        /* Flash 持久化 */
        LL_HRTIM_DisableOutput(HRTIM1, HRTIM_ALL_OUTPUTS);
        FlashCalibData_t flash_data;
        if (!FlashParams_Read(&flash_data))
            memset(&flash_data, 0, sizeof(flash_data));
        flash_data.m1_rs_ohm         = res.rs_ohm;
        flash_data.m1_ls_henry       = res.ls_henry;
        flash_data.m1_kp             = kp;
        flash_data.m1_ki             = ki;
        flash_data.m1_vbus_calib_mv  = g_Vbus_mV;
        flash_data.m1_valid          = 1;
        flash_data.calib_count++;
        FlashParams_Write(&flash_data);
        LL_HRTIM_EnableOutput(HRTIM1, HRTIM_ALL_OUTPUTS);
    }

    g_VofaFrame.ch[0] = (float)(uint8_t)(res.success ? CURR_CALIB_STATE_DONE : CURR_CALIB_STATE_ERROR);
    g_VofaFrame.ch[1] = 0.f;
    g_VofaFrame.ch[2] = (float)res.error_code;
    g_VofaFrame.ch[3] = res.debug_dc_id_avg_si;
    g_VofaFrame.ch[4] = res.rs_ohm;
    g_VofaFrame.ch[5] = res.ls_henry * 1000.0f;
    g_VofaFrame.ch[6] = 0.0f;
    g_VofaSrc = VOFA_SRC_NORMAL;
    g_CalibVdDirect_M1       = 0;
    g_CalibVdDirectActive_M1 = 0;
    g_CurrLoopCalibInProgress_M1 = 0;
}

/* ---- CalibM1_OnTIM3_1ms ---- */
void CalibM1_OnTIM3_1ms(void)
{
    if (!g_CurrLoopCalibInProgress_M1) return;
    CurrLoopCalib_State_t st = CurrLoopCalib_Run(&s_calibHandle_M1);
    if (st == CURR_CALIB_STATE_DONE || st == CURR_CALIB_STATE_ERROR)
        g_CurrLoopCalibResultReady_M1 = 1;
}

/* ---- CalibM1_FillVofa ---- */
void CalibM1_FillVofa(void)
{
    CurrLoopCalib_Result_t res;
    CurrLoopCalib_State_t st;
    uint32_t cnt;
    CurrLoopCalib_GetResult(&s_calibHandle_M1, &res);
    CurrLoopCalib_GetStateAndCounter(&s_calibHandle_M1, &st, &cnt);
    g_VofaFrame.ch[0] = (float)(uint8_t)st;
    g_VofaFrame.ch[1] = (float)cnt;
    g_VofaFrame.ch[2] = (float)res.error_code;
    g_VofaFrame.ch[3] = res.debug_dc_id_avg_si;
    g_VofaFrame.ch[4] = res.rs_ohm;
    g_VofaFrame.ch[5] = res.ls_henry * 1000.0f;
    g_VofaFrame.ch[6] = 0.0f;
}
