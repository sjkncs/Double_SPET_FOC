/**
 * @file curr_loop_autocalib.c
 * @brief 电流环自校准：开环电压注入 DC 测 Rs + AC 同步解调测 Ls + PI 计算
 *
 * 参考 ST MCSDK profiler_dcac.c + profiler_impedest.c 的开环电压注入方案。
 * 所有状态由 1ms 定时器（TIM3）驱动，无阻塞延时：
 *   1. DC_RAMP:    每 1ms 增加 D 轴占空比，监控电流安全限值
 *   2. DC_SETTLE:  保持占空比，等 LP 收敛（计数器倒计时）
 *   3. DC_MEASURE: 采集 N 个 1ms 平均 Vd/Id → Rs_dc
 *   4. AC_SETTLE:  保持 DC 电压，等稳态
 *   5. AC_MEASURE: Vd = Vdc + Vac·sin(ωt)，读 1ms 均值 Vd/Id，同步解调
 *                  N 点(整数周期)后 Z = V/I → Rs_ac, Ls
 *   6. RAMP_DOWN:  每 1ms 降低 Vd 至 0
 *   7. DONE / ERROR
 */
#include "curr_loop_autocalib.h"
#include <string.h>
#include <math.h>

#define PI_MARGIN   5.0f
#define TWO_PI      6.283185307179586f

/* ---- 内部状态 ---- */
typedef struct {
    CurrLoopCalib_Platform_t platform;
    CurrLoopCalib_Params_t   params;
    CurrLoopCalib_State_t    state;
    uint32_t counter;           /* 总 Run() 调用次数 */
    uint32_t step;              /* 当前状态内步数 */
    CurrLoopCalib_Result_t result;
    /* DC */
    int16_t  dc_duty_q15;      /* 当前 DC 占空比 */
    float    rs_dc;
    float    dc_vd_sum;
    float    dc_id_sum;
    uint32_t dc_sample_count;
    /* AC */
    int16_t  ac_duty_q15;      /* AC 幅值 Q15 */
    uint32_t ac_sample_idx;
    float    sum_vd_sin, sum_vd_cos;
    float    sum_id_sin, sum_id_cos;
} CurrLoopCalib_Internal_t;

_Static_assert(sizeof(CurrLoopCalib_Internal_t) <= CURR_LOOP_CALIB_HANDLE_SIZE,
               "Handle size too small");

#define H2I(h)  ((CurrLoopCalib_Internal_t *)(void *)(h))

/* ---- 辅助换算 ---- */
static float q15_to_current_si(int16_t q, float scale_si)
{
    return (float)q * scale_si / 32768.0f;
}

static float q15_to_voltage_si(int16_t q, uint32_t vbus_mv)
{
    return (float)q * (float)vbus_mv / (1000.0f * 32768.0f);
}

/* ---- 公开 API ---- */

void CurrLoopCalib_Params_SetDefaults(CurrLoopCalib_Params_t *params)
{
    if (!params) return;
    memset(params, 0, sizeof(*params));
    params->control_freq_hz         = 17000.f;
    params->current_scale_si        = 1.f;
    params->rs_min_ohm              = 1.f;
    params->rs_max_ohm              = 100.f;
    params->ls_min_henry            = 0.0005f;
    params->ls_max_henry            = 0.1f;
    /* DC */
    params->dc_current_limit_amp    = 0.3f;
    params->dc_duty_max_q15         = 19661;  /* 60% full scale (safety cap) */
    params->dc_ramp_rate_q15_per_ms = 98;     /* ~200ms ramp to 19661 */
    params->dc_settle_ms            = 300;
    params->dc_measure_samples      = 200;
    /* AC */
    params->ac_inject_freq_hz       = 200.f;
    params->ac_duty_fraction        = 0.5f;
    params->ac_settle_ms            = 200;
    params->ac_measure_samples      = 2000;   /* 200Hz × 1ms = 5 点/周期, 2000 = 400 周期 */
    params->rampdown_rate_q15_per_ms= 197;    /* ~100ms ramp down from 19661 */
}

void CurrLoopCalib_Init(CurrLoopCalib_Handle_t *h,
                         const CurrLoopCalib_Platform_t *platform,
                         const CurrLoopCalib_Params_t *params)
{
    if (!h) return;
    CurrLoopCalib_Internal_t *s = H2I(h);
    memset(s, 0, sizeof(*s));
    if (platform) s->platform = *platform;
    if (params)   s->params   = *params;
    else          CurrLoopCalib_Params_SetDefaults(&s->params);
    s->state = CURR_CALIB_STATE_IDLE;
}

void CurrLoopCalib_Start(CurrLoopCalib_Handle_t *h)
{
    if (!h) return;
    CurrLoopCalib_Internal_t *s = H2I(h);
    s->state           = CURR_CALIB_STATE_DC_RAMP;
    s->counter         = 0;
    s->step            = 0;
    s->dc_duty_q15     = 0;
    s->ac_duty_q15     = 0;
    s->rs_dc           = 0.0f;
    s->dc_vd_sum       = 0.0f;
    s->dc_id_sum       = 0.0f;
    s->dc_sample_count = 0;
    s->ac_sample_idx   = 0;
    s->sum_vd_sin = s->sum_vd_cos = 0.0f;
    s->sum_id_sin = s->sum_id_cos = 0.0f;
    memset(&s->result, 0, sizeof(s->result));
    /* 初始电压 = 0 */
    if (s->platform.set_vd_q15) s->platform.set_vd_q15(0);
}

/* ---- 状态机主体（每 1ms 调用，全部非阻塞） ---- */

CurrLoopCalib_State_t CurrLoopCalib_Run(CurrLoopCalib_Handle_t *h)
{
    if (!h) return CURR_CALIB_STATE_ERROR;
    CurrLoopCalib_Internal_t *s = H2I(h);
    const CurrLoopCalib_Params_t *p = &s->params;
    CurrLoopCalib_Platform_t     *plat = &s->platform;

    s->counter++;

    switch (s->state) {

    /* ---- DC_RAMP: 每 1ms 升压，监控电流安全限值 ---- */
    case CURR_CALIB_STATE_DC_RAMP: {
        int32_t duty = (int32_t)s->dc_duty_q15 + (int32_t)p->dc_ramp_rate_q15_per_ms;
        if (duty > (int32_t)p->dc_duty_max_q15)
            duty = (int32_t)p->dc_duty_max_q15;
        s->dc_duty_q15 = (int16_t)duty;
        if (plat->set_vd_q15) plat->set_vd_q15(s->dc_duty_q15);

        /* 读 Id + 清累加器（保持 1ms 窗口新鲜） */
        int16_t id_q = 0, iq_q = 0, vd_q = 0, vq_q = 0;
        if (plat->get_idq_q15) plat->get_idq_q15(&id_q, &iq_q);
        if (plat->get_vdq_q15) plat->get_vdq_q15(&vd_q, &vq_q);

        float id_si = q15_to_current_si(id_q, p->current_scale_si);
        if (id_si < 0.0f) id_si = -id_si;

        if (id_si >= p->dc_current_limit_amp
         || duty >= (int32_t)p->dc_duty_max_q15) {
            s->state = CURR_CALIB_STATE_DC_SETTLE;
            s->step  = 0;
        }
        return s->state;
    }

    /* ---- DC_SETTLE: 保持占空比，等电流稳态 ---- */
    case CURR_CALIB_STATE_DC_SETTLE: {
        if (plat->set_vd_q15) plat->set_vd_q15(s->dc_duty_q15);
        /* 每 1ms 读一次保持累加器新鲜 */
        int16_t dummy_i, dummy_v;
        if (plat->get_idq_q15) plat->get_idq_q15(&dummy_i, &dummy_i);
        if (plat->get_vdq_q15) plat->get_vdq_q15(&dummy_v, &dummy_v);
        s->step++;
        if (s->step >= p->dc_settle_ms) {
            s->state = CURR_CALIB_STATE_DC_MEASURE;
            s->step  = 0;
        }
        return s->state;
    }

    /* ---- DC_MEASURE: 采 N 个 1ms 均值 → Rs_dc ---- */
    case CURR_CALIB_STATE_DC_MEASURE: {
        if (plat->set_vd_q15) plat->set_vd_q15(s->dc_duty_q15);

        if (s->dc_sample_count < p->dc_measure_samples) {
            int16_t id_q = 0, iq_q = 0, vd_q = 0, vq_q = 0;
            if (plat->get_idq_q15) plat->get_idq_q15(&id_q, &iq_q);
            if (plat->get_vdq_q15) plat->get_vdq_q15(&vd_q, &vq_q);
            uint32_t vbus = plat->get_vbus_mv ? plat->get_vbus_mv() : 12000u;
            s->dc_vd_sum += q15_to_voltage_si(vd_q, vbus);
            s->dc_id_sum += q15_to_current_si(id_q, p->current_scale_si);
            s->dc_sample_count++;
            s->result.debug_dc_vd_avg_si = s->dc_vd_sum / (float)s->dc_sample_count;
            s->result.debug_dc_id_avg_si = s->dc_id_sum / (float)s->dc_sample_count;
            return CURR_CALIB_STATE_DC_MEASURE;
        }
        /* 算 Rs_dc */
        if (s->dc_id_sum < 1e-6f) {
            s->result.error_code = 3;   /* DC 电流太小 */
            s->state = CURR_CALIB_STATE_RAMP_DOWN;
            s->step  = 0;
            return CURR_CALIB_STATE_RAMP_DOWN;
        }
        s->rs_dc = s->dc_vd_sum / s->dc_id_sum;
        /* AC 幅值 = DC 占空比 × fraction（参考 profiler_dcac.c: ac_duty = dutyDC × 0.5） */
        s->ac_duty_q15 = (int16_t)((float)s->dc_duty_q15 * p->ac_duty_fraction);
        if (s->ac_duty_q15 < 1) s->ac_duty_q15 = 1;
        s->state = CURR_CALIB_STATE_AC_SETTLE;
        s->step  = 0;
        return CURR_CALIB_STATE_AC_SETTLE;
    }

    /* ---- AC_SETTLE: 保持 DC 电压，等稳态 ---- */
    case CURR_CALIB_STATE_AC_SETTLE: {
        if (plat->set_vd_q15) plat->set_vd_q15(s->dc_duty_q15);
        int16_t dummy_i, dummy_v;
        if (plat->get_idq_q15) plat->get_idq_q15(&dummy_i, &dummy_i);
        if (plat->get_vdq_q15) plat->get_vdq_q15(&dummy_v, &dummy_v);
        s->step++;
        if (s->step >= p->ac_settle_ms) {
            s->state         = CURR_CALIB_STATE_AC_MEASURE;
            s->ac_sample_idx = 0;
            s->sum_vd_sin = s->sum_vd_cos = 0.0f;
            s->sum_id_sin = s->sum_id_cos = 0.0f;
        }
        return s->state;
    }

    /* ---- AC_MEASURE: Vd = Vdc + Vac·sin(ωt)，同步解调 ---- */
    case CURR_CALIB_STATE_AC_MEASURE: {
        uint32_t idx = s->ac_sample_idx;
        float t     = (float)idx * 0.001f;
        float phase = TWO_PI * p->ac_inject_freq_hz * t;
        float sp    = sinf(phase);
        float cp    = cosf(phase);

        /* 设注入电压：DC + AC·sin */
        int32_t vd_cmd = (int32_t)s->dc_duty_q15
                       + (int32_t)((float)s->ac_duty_q15 * sp);
        if (vd_cmd >  32767) vd_cmd =  32767;
        if (vd_cmd < -32768) vd_cmd = -32768;
        if (plat->set_vd_q15) plat->set_vd_q15((int16_t)vd_cmd);

        /* 读实际 Vd / Id（17kHz ISR 累加的 1ms 均值） */
        int16_t id_q = 0, iq_q = 0, vd_q = 0, vq_q = 0;
        if (plat->get_idq_q15) plat->get_idq_q15(&id_q, &iq_q);
        if (plat->get_vdq_q15) plat->get_vdq_q15(&vd_q, &vq_q);
        uint32_t vbus = plat->get_vbus_mv ? plat->get_vbus_mv() : 12000u;

        float vd_si = q15_to_voltage_si(vd_q, vbus);
        float id_si = q15_to_current_si(id_q, p->current_scale_si);

        /* 同步解调累加（DC 分量在整数周期求和后自动抵消） */
        s->sum_vd_sin += vd_si * sp;
        s->sum_vd_cos += vd_si * cp;
        s->sum_id_sin += id_si * sp;
        s->sum_id_cos += id_si * cp;

        s->ac_sample_idx++;
        if (s->ac_sample_idx >= p->ac_measure_samples) {
            /* Z = V_phasor / I_phasor（复数除法，参考 profiler_impedest.c） */
            float denom = s->sum_id_sin * s->sum_id_sin
                        + s->sum_id_cos * s->sum_id_cos;
            if (denom < 1e-12f) {
                s->result.error_code = 4;   /* AC 电流信号太弱 */
                s->state = CURR_CALIB_STATE_RAMP_DOWN;
                s->step  = 0;
                return CURR_CALIB_STATE_RAMP_DOWN;
            }
            float re_z = (s->sum_vd_sin * s->sum_id_sin
                        + s->sum_vd_cos * s->sum_id_cos) / denom;
            float im_z = (s->sum_vd_sin * s->sum_id_cos
                        - s->sum_vd_cos * s->sum_id_sin) / denom;
            float omega = TWO_PI * p->ac_inject_freq_hz;

            s->result.rs_ohm   = re_z;
            s->result.ls_henry = -im_z / omega;

            /* 校验（参考 profiler_dcac.c: 0.8~1.4×Rs_dc，Ls>0） */
            float rs = s->result.rs_ohm;
            float ls = s->result.ls_henry;
            if (rs < p->rs_min_ohm || rs > p->rs_max_ohm
                || rs < s->rs_dc * 0.5f || rs > s->rs_dc * 2.0f) {
                s->result.error_code = 1;
            } else if (ls < p->ls_min_henry || ls > p->ls_max_henry) {
                s->result.error_code = 2;
            } else {
                s->result.success    = 1;
                s->result.error_code = 0;
            }
            /* 无论成败都先降压再报告 */
            s->state = CURR_CALIB_STATE_RAMP_DOWN;
            s->step  = 0;
            return CURR_CALIB_STATE_RAMP_DOWN;
        }
        return CURR_CALIB_STATE_AC_MEASURE;
    }

    /* ---- RAMP_DOWN: 降压至 0 后报告结果 ---- */
    case CURR_CALIB_STATE_RAMP_DOWN: {
        int32_t duty = (int32_t)s->dc_duty_q15
                     - (int32_t)p->rampdown_rate_q15_per_ms;
        if (duty < 0) duty = 0;
        s->dc_duty_q15 = (int16_t)duty;
        if (plat->set_vd_q15) plat->set_vd_q15(s->dc_duty_q15);
        /* 保持累加器新鲜 */
        int16_t dummy_i, dummy_v;
        if (plat->get_idq_q15) plat->get_idq_q15(&dummy_i, &dummy_i);
        if (plat->get_vdq_q15) plat->get_vdq_q15(&dummy_v, &dummy_v);

        if (duty <= 0) {
            s->state = s->result.success
                     ? CURR_CALIB_STATE_DONE
                     : CURR_CALIB_STATE_ERROR;
        }
        return s->state;
    }

    default:
        return s->state;
    }
}

/* ---- 查询 ---- */

void CurrLoopCalib_GetStateAndCounter(CurrLoopCalib_Handle_t *h,
                                      CurrLoopCalib_State_t *state_out,
                                      uint32_t *counter_out)
{
    if (!h) return;
    CurrLoopCalib_Internal_t *s = H2I(h);
    if (state_out)   *state_out   = s->state;
    if (counter_out) *counter_out = s->counter;
}

void CurrLoopCalib_GetResult(CurrLoopCalib_Handle_t *h, CurrLoopCalib_Result_t *out)
{
    if (!h || !out) return;
    *out = H2I(h)->result;
}

/* ---- PI 系数换算（ST 公式 → 本工程 Q15 定点） ---- */

void CurrLoopCalib_ComputePI_Q15(const CurrLoopCalib_Result_t *result,
                                  uint32_t vbus_mv, int32_t i_fullscale_ma,
                                  float control_freq_hz,
                                  int32_t *kp_q15, int32_t *ki_q15)
{
    if (!result || !kp_q15 || !ki_q15 || vbus_mv < 1000u) return;
    float R  = result->rs_ohm;
    float L  = result->ls_henry;
    if (L <= 0.0f || control_freq_hz <= 0.0f) return;

    /* 参考 pidregdqx_current.c:
     * Kp_si = (2/margin) × L × fs   [V/A]
     * Wi    = R / L                  [rad/s]
     * kp_q15 = Kp_si × I_fs × 32768 / Vbus
     * ki_q15 = kp_q15 × Wi / fs = kp_q15 × R / (L × fs)  */
    float Kp_si = (2.0f / PI_MARGIN) * L * control_freq_hz;
    float vbus  = (float)vbus_mv / 1000.0f;
    float i_fs  = (float)i_fullscale_ma / 1000.0f;
    if (vbus < 1.0f) vbus = 1.0f;
    if (i_fs < 0.001f) i_fs = 0.001f;

    float kp_f = Kp_si * i_fs * 32768.0f / vbus;
    float ki_f = kp_f * R / (L * control_freq_hz);

    if (kp_f > 2147483647.0f) kp_f = 2147483647.0f;
    if (ki_f > 2147483647.0f) ki_f = 2147483647.0f;
    *kp_q15 = (int32_t)kp_f;
    *ki_q15 = (int32_t)ki_f;
}
