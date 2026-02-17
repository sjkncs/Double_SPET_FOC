/**
 * @file    follow_m1m2.c
 * @brief   M1→M2 主从随动 — M1 手轮 + M2 步距角跟随
 *
 * 设计:
 *   - 进入 (0→1): 配置 M1 开环零电流 + M2 步距角零自进
 *   - 活跃:       g_M2_StepAngle_Ref = g_M1_PosFbk (1:1, 65536 counts = 1 圈)
 *   - 退出 (1→0): 两轴回开环零电流 (安全停机)
 *
 *   进入时设好 g_M1_Iq_Ref_mA=0 和 g_M2_RPM_Cmd=0,
 *   while(1) 已有代码自然处理 M1 开环写零 / M2 步距角 AngleDelta=0,
 *   模块只需额外写 StepAngle_Ref 一行, 不与现有逻辑冲突.
 */

#include "follow_m1m2.h"
#include "main.h"
#include "vofa_engine.h"

/* ---- 全局: Live Watch 启停开关 ---- */
volatile uint8_t g_FollowM1M2 = 0;

void Follow_M1M2_Poll(void)
{
    static uint8_t was_active = 0;

    if (g_FollowM1M2 && !was_active) {
        /* ---- 进入随动 ---- */

        /* M1: 开环, 零电流, 电机自由转动 */
        g_M1_CtrlMode          = MODE_OPEN_LOOP;
        g_M1_Iq_Ref_mA         = 0;
        g_M1_RPM_Cmd           = 0;
        g_M1_AngleDelta_Target = 0;

        /* M2: 步距角模式, 不自行推进 (RPM=0), 位置由本模块写入 */
        g_M2_CtrlMode          = MODE_STEP_ANGLE;
        g_M2_RPM_Cmd           = 0;
        g_M2_AngleDelta_Target = 0;

        /* VOFA: 切换到随动专用通道 (双轴位置 + 误差°) */
        g_VofaSrc = VOFA_SRC_FOLLOW;

        was_active = 1;
    }

    if (g_FollowM1M2 && was_active) {
        /* ---- 持续随动: M1 编码器位置 → M2 步距角目标 ---- */
        g_M2_StepAngle_Ref = g_M1_PosFbk;
    }

    if (!g_FollowM1M2 && was_active) {
        /* ---- 退出随动: 安全停机 (两轴开环零电流) ---- */
        g_M1_CtrlMode          = MODE_OPEN_LOOP;
        g_M1_AngleDelta_Target = 0;
        g_M1_Idq_Ref.d         = 0;
        g_M1_Idq_Ref.q         = 0;

        g_M2_CtrlMode          = MODE_OPEN_LOOP;
        g_M2_AngleDelta_Target = 0;
        g_M2_Idq_Ref.d         = 0;
        g_M2_Idq_Ref.q         = 0;

        /* VOFA: 恢复正常模式 */
        g_VofaSrc = VOFA_SRC_NORMAL;

        was_active = 0;
    }
}
