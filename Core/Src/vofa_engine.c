/**
 * @file    vofa_engine.c
 * @brief   VOFA+ 调试波形引擎 — TIM4 1kHz 状态机, 自动填帧 + DMA 发送
 *
 * 设计思路:
 *   1. TIM4 (1kHz, 最低优先级) 每 1ms 执行一次
 *   2. 根据 g_VofaSrc 选择数据源, 填充 g_VofaFrame 6 个浮点通道
 *   3. DMA 空闲时自动触发 USART1 DMA 发送
 *   4. 各模块只需写 g_VofaSrc 切换, 无需关心 DMA
 *
 * 中断优先级: TIM4(10) < TIM3(3) < HRTIM(2) < ADC(1)
 *   - 不会抢占任何控制环路
 *   - 即使 while(1) 被 HAL_Delay 阻塞, 仍然正常发送
 */

#include "vofa_engine.h"
#include "main.h"
#include "kth71xx.h"               /* g_CalibDbgStep, g_Kth71CalibActive */
#include "calib_platform_m2.h"     /* CalibM2_FillVofa() */
#include "calib_platform_m1.h"     /* CalibM1_FillVofa() */

/* ---- 全局: 当前 VOFA 数据源 (上电默认 IDLE, 不发送) ---- */
volatile VofaSrc_t g_VofaSrc = VOFA_SRC_IDLE;
volatile uint8_t g_VofaMotorSel = 0;    /* 0=M2(默认), 1=M1 */

/* ====================================================================
 * 各数据源填帧 (static, 仅本文件调用)
 * ==================================================================== */

/**
 * @brief  正常模式: 按控制模式自动切换 VOFA 通道
 *
 * === 速度模式 (M2_MODE_SPEED) ===
 *   I0: 原始转速 RPM      — 观测编码器量化噪声
 *   I1: 滤波转速 RPM      — PI 实际反馈信号
 *   I2: 斜坡后 RPM 指令   — PI 实际跟踪目标
 *   I3: 速度误差 (RPM)    — cmd-fbk, 看稳态精度
 *   I4: PI 积分 (mA)      — 积分饱和诊断
 *   I5: PI 输出 Iq (mA)   — 力矩电流指令
 *
 * === 步距角模式 (M2_MODE_STEP_ANGLE, 直接PD→Iq) ===
 *   I0: 原始转速 RPM      — 观测平滑度
 *   I1: 滤波转速 RPM      — 阻尼D项反馈信号
 *   I2: 位置误差 (counts)  — 核心: 跟踪精度 + 失步诊断
 *   I3: 积分 (mA)         — 积分饱和诊断 (ki=0 时为 0)
 *   I4: PD 输出 Iq (mA)   — 力矩电流指令
 *   I5: 圈数              — 位置追踪 (StepAngle_Ref / 65536)
 *
 * === 位置模式 (M2_MODE_POSITION, 级联P→速度P-only) ===
 *   I0: 原始转速 RPM
 *   I1: 滤波转速 RPM
 *   I2: 位置误差 (counts)
 *   I3: 外环速度参考 (RPM)
 *   I4: 内环 Iq (mA)
 *   I5: 当前圈数
 *
 * === 其他模式 (开环等) ===
 *   I0: 原始转速 RPM
 *   I1: 滤波转速 RPM
 *   I2~I5: 0 (无控制器输出)
 */
static void fill_normal(void)
{
    /* 根据 g_VofaMotorSel 选择数据源: 0=M2, 1=M1 */
    if (g_VofaMotorSel == 0) {
        /* ---- M2 ---- */
        g_VofaFrame.ch[0] = (float)g_Enc2_SpeedRPM;
        g_VofaFrame.ch[1] = (float)g_Enc2_SpeedFilt;

        if (g_M2_CtrlMode == M2_MODE_SPEED) {
            g_VofaFrame.ch[2] = g_Dbg_Spd_RpmCmd;
            g_VofaFrame.ch[3] = g_Dbg_Spd_Err;
            g_VofaFrame.ch[4] = g_M2_SpeedPI.integral;
            g_VofaFrame.ch[5] = g_Dbg_Spd_IqOut;
        } else if (g_M2_CtrlMode == M2_MODE_STEP_ANGLE) {
            g_VofaFrame.ch[2] = (float)g_M2_StepAngle_Err;
            g_VofaFrame.ch[3] = g_M2_StepAnglePD.integral;
            g_VofaFrame.ch[4] = g_M2_StepAnglePD.output;
            g_VofaFrame.ch[5] = (float)(g_M2_StepAngle_Ref / 65536);
        } else if (g_M2_CtrlMode == M2_MODE_POSITION) {
            g_VofaFrame.ch[2] = g_Dbg_Pos_pTerm;
            g_VofaFrame.ch[3] = g_Dbg_Pos_iqmA;
            g_VofaFrame.ch[4] = g_Dbg_Pos_dTerm;
            g_VofaFrame.ch[5] = (float)(g_M2_PosFbk / 65536);
        } else {
            g_VofaFrame.ch[2] = 0.0f;
            g_VofaFrame.ch[3] = 0.0f;
            g_VofaFrame.ch[4] = 0.0f;
            g_VofaFrame.ch[5] = 0.0f;
        }
        g_VofaFrame.ch[6] = (float)g_M2_Id_Eff_mA;
    } else {
        /* ---- M1 ---- */
        g_VofaFrame.ch[0] = (float)g_Enc1_SpeedRPM;
        g_VofaFrame.ch[1] = (float)g_Enc1_SpeedFilt;

        if (g_M1_CtrlMode == M2_MODE_SPEED) {
            g_VofaFrame.ch[2] = g_Dbg_M1_Spd_RpmCmd;
            g_VofaFrame.ch[3] = g_Dbg_M1_Spd_Err;
            g_VofaFrame.ch[4] = g_M1_SpeedPI.integral;
            g_VofaFrame.ch[5] = g_Dbg_M1_Spd_IqOut;
        } else if (g_M1_CtrlMode == M2_MODE_STEP_ANGLE) {
            g_VofaFrame.ch[2] = (float)g_M1_StepAngle_Err;
            g_VofaFrame.ch[3] = g_M1_StepAnglePD.integral;
            g_VofaFrame.ch[4] = g_M1_StepAnglePD.output;
            g_VofaFrame.ch[5] = (float)(g_M1_StepAngle_Ref / 65536);
        } else if (g_M1_CtrlMode == M2_MODE_POSITION) {
            g_VofaFrame.ch[2] = g_Dbg_M1_Pos_pTerm;
            g_VofaFrame.ch[3] = g_Dbg_M1_Pos_iqmA;
            g_VofaFrame.ch[4] = g_Dbg_M1_Pos_dTerm;
            g_VofaFrame.ch[5] = (float)(g_M1_PosFbk / 65536);
        } else {
            g_VofaFrame.ch[2] = 0.0f;
            g_VofaFrame.ch[3] = 0.0f;
            g_VofaFrame.ch[4] = 0.0f;
            g_VofaFrame.ch[5] = 0.0f;
        }
        g_VofaFrame.ch[6] = (float)g_M1_Id_Eff_mA;
    }
}

/**
 * @brief  KTH71 非线性校准: 电机运行状态诊断
 *
 * 校准全程阻塞 while(1), 但 TIM4 仍在运行, 持续发送.
 *
 * VOFA 通道:
 *   I0: g_CalibDbgStep (1=RAMP, 2=UNLOCK, 3=POLLING, 4=DONE, 5=FAIL, 6=TIMEOUT)
 *   I1: Id 反馈 (mA) — 验证电流环工作
 *   I2: Iq 反馈 (mA) — 验证力矩电流足够
 *   I3: Vd 输出 (Q15) — PI 输出电压
 *   I4: Vq 输出 (Q15) — PI 输出电压
 *   I5: 电角度 (度)   — 验证电机在旋转 (应见锯齿波)
 */
static void fill_kth71_calib(void)
{
    g_VofaFrame.ch[0] = (float)g_CalibDbgStep;
    g_VofaFrame.ch[1] = (float)Q15_TO_MA(g_M2_Idq.d);
    g_VofaFrame.ch[2] = (float)Q15_TO_MA(g_M2_Idq.q);
    g_VofaFrame.ch[3] = (float)g_M2_Vdq.d;
    g_VofaFrame.ch[4] = (float)g_M2_Vdq.q;
    g_VofaFrame.ch[5] = (float)g_M2_ElecAngle_Q15 * (360.0f / 65536.0f);
    g_VofaFrame.ch[6] = 0.0f;
}

/**
 * @brief  阶跃响应回放: 逐帧从录制缓冲区读取并发送
 *
 * TIM4 自动以 1kHz 步进 s_stepPlayIdx, 天然 1ms/帧节拍.
 * 播完一轴 (idx >= STEP_CAP_DEPTH) 后置 g_StepCapState = 4,
 * 由 while(1) 负责 d→q 轴切换 (涉及 PI 复位, 不宜在此 ISR 做).
 *
 * VOFA 通道:
 *   I0: Id_ref (mA)
 *   I1: Iq_ref (mA)
 *   I2: Id_fbk (mA)
 *   I3: Iq_fbk (mA)
 *   I4: 样本序号 (阶跃在第 STEP_CAP_PRE 点)
 *   I5: 轴号 + 100×翻转方波 (便于数周期)
 */
static void fill_step_playback(void)
{
    static uint16_t s_stepPlayIdx  = 0;
    static uint8_t  s_vofaToggle   = 0;

    if (g_StepCapState != 2) {
        s_stepPlayIdx = 0;      /* 非回放态: 复位索引 */
        return;                 /* 保持上一帧数据不变 */
    }

    if (s_stepPlayIdx < STEP_CAP_DEPTH) {
        const StepCapSample_t *s = &g_StepCapBuf[s_stepPlayIdx];
        const float q15_to_mA = (float)I_FULLSCALE_MA / 32768.0f;

        s_vofaToggle ^= 1U;
        g_VofaFrame.ch[0] = (float)s->id_ref * q15_to_mA;
        g_VofaFrame.ch[1] = (float)s->iq_ref * q15_to_mA;
        g_VofaFrame.ch[2] = (float)s->id_fbk * q15_to_mA;
        g_VofaFrame.ch[3] = (float)s->iq_fbk * q15_to_mA;
        g_VofaFrame.ch[4] = (float)s_stepPlayIdx;
        g_VofaFrame.ch[5] = (float)g_StepCapPhase + 100.0f * (float)s_vofaToggle;
        g_VofaFrame.ch[6] = 0.0f;
        s_stepPlayIdx++;
    }

    if (s_stepPlayIdx >= STEP_CAP_DEPTH) {
        /* 一轴播完 → 通知 while(1) 处理轴切换 */
        s_stepPlayIdx = 0;
        g_StepCapState = 4;     /* 4 = 回放完成, 等 while(1) 处理 */
    }
}

/* ====================================================================
 * API 实现
 * ==================================================================== */

void Vofa_StartTIM4(void)
{
    LL_TIM_ClearFlag_UPDATE(TIM4);
    LL_TIM_EnableIT_UPDATE(TIM4);
    LL_TIM_EnableCounter(TIM4);
}

void Vofa_OnTIM4_1ms(void)
{
    /* ---- IDLE: 不填帧、不发送, 直接返回 ---- */
    if (g_VofaSrc == VOFA_SRC_IDLE) return;

    /* ---- 填帧: 根据当前数据源选择填充函数 ---- */
    switch (g_VofaSrc) {
    case VOFA_SRC_NORMAL:
        fill_normal();
        break;
    case VOFA_SRC_CURR_CALIB:
        CalibM2_FillVofa();      /* 由 calib_platform_m2 填充 */
        break;
    case VOFA_SRC_CURR_CALIB_M1:
        CalibM1_FillVofa();      /* 由 calib_platform_m1 填充 */
        break;
    case VOFA_SRC_KTH71_CALIB:
        fill_kth71_calib();
        break;
    case VOFA_SRC_STEP_PLAYBACK:
        fill_step_playback();
        break;
    default:
        return;                  /* 未知源: 不发送 */
    }

    /* ---- DMA 发送: 空闲时触发, 忙则跳过 (下一 1ms 再发) ---- */
    if (!LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1)) {
        LL_DMA_ClearFlag_GI1(DMA1);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, sizeof(VofaFrame_t));
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    }
}
