# 电流环自校准方案（ST 做法 + 本工程实现）

> 从参考工程 `STOPLL_FOC_2205`（ST MCSDK v6.4.1）提取公式与流程；本工程在 **Core/Src/curr_loop_autocalib.c** 中实现可移植的 DC 测 R + AC 注入测 L + ST 公式写 PI，**换电机无需手册**，上电或调试时跑一次即可。

---

## 1. ST MCSDK 方案概览

ST 在 Profiler 里实现的是 **DC/AC 辨识 R、L → 用 R/L 自动设定电流环 PI**，无继电器振荡、无阶跃响应曲线拟合，适合上电或调试时自动跑一遍。

### 1.1 流程（profiler_dcac.c + profiler.c）

| 阶段 | 做法 | 得到 |
|------|------|------|
| **DC** | 斜坡加 duty/电流，滤波 Udq、Idq | Rs_dc = Umag/Imag（直流电阻） |
| **AC** | 在 DC 基础上注入交流（ZeST 阻抗估计） | Rs_inject, Ls_inject（交流 R、L） |
| **校验** | Rs_inject 需在 0.8~1.4×Rs_dc；Ls_inject > 0 | 通过则继续 |
| **设 PI** | 调用 `PIDREGDQX_CURRENT_setKpWiRLmargin_si(pid, Rs_inject, Ls_inject, 5.0f)` | 电流环 Kp、Ki 一次设好 |

### 1.2 电流环 PI 公式（pidregdqx_current.c）

```c
/* 入参: Rsi (Ω), Lsi (H), margin (建议 5) */
void PIDREGDQX_CURRENT_setKpWiRLmargin_si(..., const float Rsi, const float Lsi, const float margin)
{
  float kp_idq = (2.0f / margin) * Lsi * pHandle->pid_freq_hz;   /* Kp = (2/margin)×L×fs, 单位 V/A */
  float wi_idq = (Rsi / Lsi);                                     /* Wi = R/L rad/s, 极零对消 */
  PIDREGDQX_CURRENT_setKp_si(pHandle, kp_idq);
  PIDREGDQX_CURRENT_setWi_si(pHandle, wi_idq);
}
```

- **Kp**：\( K_p = \dfrac{2}{\text{margin}} \times L_s \times f_s \)。margin=5 时约 0.4×L×fs，留增益裕度。
- **Wi**：\( \omega_i = R_s/L_s \)（rad/s），积分交叉频率，对应极零对消。
- **Ki**：由 Wi 和内部换算得到（该库用 Wi rad/s 表示积分强度）。

即：**先辨识 R、L，再用“L×fs/margin + R/L”这一套公式得到电流环 PI，无需手调。**

---

## 2. 关键代码位置（STOPLL_FOC_2205）

| 功能 | 文件 | 说明 |
|------|------|------|
| DC 测 Rs | `MCSDK_v6.4.1-Full/.../profiler_dcac.c` | DC_Measuring: Umag/Imag → Rs_dc |
| AC 测 R、L | 同上 | AC_Measuring: ZEST_getR/L → Rs_inject, Ls_inject |
| 设电流环 PI | 同上 311 行 | `PIDREGDQX_CURRENT_setKpWiRLmargin_si(..., 5.0f)` |
| Kp/Wi 公式 | `MCSDK_v6.4.1-Full/.../pidregdqx_current.c` | `setKpWiRLmargin_si()` 内 Kp=(2/margin)*L*fs, Wi=R/L |
| ZeST 阻抗估计 | 库内 ZeST 模块 | 交流注入 + 估计 R、L |

---

## 2.5 与 Odrive 对比（参考）

Odrive 电机校准（`Firmware/MotorControl/motor.cpp`）也做 **R + L**，做法可作参考：

| 项目 | Odrive | 本工程 (curr_loop_autocalib) |
|------|--------|-----------------------------|
| **R** | PI 控电流到目标，稳态 R = V/I | DC 斜坡 + 稳态平均 Vd/Id |
| **L** | **方波电压** ±V，测电流纹波 ΔI，L = V/(ΔI/Δt) | **正弦注入** + 同步解调，Ls = -Im(Z)/ω |
| **校验** | R 后查 I_beta 平衡（三相不平衡则报错） | Rs/Ls 范围检查 |
| **PI 公式** | Kp = bandwidth×L，Ki = (R/L)×Kp | Kp=(2/margin)×L×fs，Wi=R/L，与 ST 一致 |

正弦注入 + 解调得到 L 与方波测纹波等价，本工程已按 **Ls = -z_im/ω** 修正符号（电感 V=L·dI/dt 与 I∝sin 正交时 Im(Z)=-ωL）。

---

## 3. 对本项目（g474-48）的适用方式

### 3.1 不移植 Profiler 时（只用公式）

你已有 **Rs、Ls 标称值**（见 `docs/MOTOR_8HA0205-10_SPEC.md`：18.5Ω、5.3mH），可直接用 ST 的公式算“理论最优”PI，再换算到本工程的 Q15 定点：

- **Kp_si** = (2 / margin) × Ls × fs  
  - margin=5，Ls=5.3e-3 H，fs=17000 Hz → Kp_si ≈ 36 V/A。
- **Wi** = Rs/Ls = 18.5/0.0053 ≈ 3490 rad/s（与现有极零对消一致）。
- 本工程为 Q15 定点，需把 Kp_si、Wi 按 `current_scale`、`voltage_scale`、`Ts` 换算成 `PI_M2_KP_NOM`、`PI_M2_KI_NOM`（换算关系见 `main.c` 与 `main.h` 中 PI 注释）。

这样得到的是**与 ST 等价的“按 R/L 算出的最优电流环”**，不跑辨识也能统一成同一套设计准则。

### 3.2 要做“自校准”时（推荐最小方案）

1. **只做 DC 测 R**  
   - 开环或电流环下加恒定 Id（或固定 duty），等稳态后测 Udq、Idq → Rs = U/I。  
   - 不做 AC 注入时，**L 用规格书或典型值**（如 5.3 mH），再用上面 Kp/Wi 公式算 PI。

2. **DC + AC 辨识 R、L（类 ST）**  
   - DC 同上看 Rs_dc。  
   - 在 DC 基础上叠加小信号交流注入（需自己实现或参考 ZeST 思路），估计 Rs、Ls。  
   - 校验 Rs 与 Rs_dc 是否合理（例如 0.8~1.4 倍），再代入 `Kp=(2/margin)*L*fs`、`Wi=R/L` 设 PI。

3. **阶跃响应自动算 Kp（半自动）**  
   - 保留你现有阶跃录制 + VOFA 回放；  
   - 在 PC 或 MCU 后处理里根据“上升时间 / 超调”反推带宽或等效 τ，用带宽目标反算 Kp，Ki 仍用 Wi=R/L。  
   - 不要求在线辨识 R、L，但需要一次阶跃数据。

---

## 4. 小结：何为“最优电流环自校准”

- **ST 方案**：Profiler DC/AC 辨识 **R、L** → 用 **Kp=(2/margin)×L×fs**、**Wi=R/L** 设电流环，margin=5。  
- **最优**体现在：  
  - 增益由 L 和采样频率决定，带宽一致；  
  - 积分由 R/L 极零对消，响应形状一致；  
  - margin 固定裕度，避免过调。  
- 在本项目中：  
  - **不跑辨识**：用规格书 R、L 直接代上述公式，再换算到本工程 PI 系数。  
  - **要自校准**：优先做 DC 测 R + 固定 L 用公式；若需 L 也辨识，再考虑 DC+AC 或 ZeST 类注入。

---

## 5. 本工程用法（g474-48）

- **单文件模块**：`Core/Inc/curr_loop_autocalib.h` + `Core/Src/curr_loop_autocalib.c`，无其他依赖，可整体拷贝到新工程。
- **触发**：Live Watch 将 **`g_DoCurrLoopCalib`** 置 1，主循环会：切开环 → 固定 0° 电角度 → DC 斜坡测 Rs → AC 注入测 Ls → 用 ST 公式算 Kp/Wi → 换算为 Q15 的 kp/ki 写入 `g_M2_PI_d`/`g_M2_PI_q` 并清零积分。
- **前提**：电流环 FOC 已在运行（ISR 正常），电机静止或空载；换电机后无需改代码，只需再置 1 跑一次。
- **移植**：在新工程中实现 `CurrLoopCalib_Platform_t` 的 5 个回调（设 Idq 参考、读 Idq/Vdq、延时、读 Vbus），调用 `CurrLoopCalib_Init/Start/Run` 直至 DONE/ERROR，用 `CurrLoopCalib_ComputePI_Q15` 得到 kp/ki 后写入自己的 PI 结构体即可。
- **VOFA 打印调试**：校准时每步发一帧。**I0=状态(0~7)，I1=计数，I2=error_code，I3=DC 平均电流(A)**。进行中时 I4=vd_avg(V)、I5=Id 原始 Q15；**I0=6(DONE) 或 I0=7(ERROR) 时 I4=Rs(Ω)、I5=Ls(H)**，便于判断辨识结果是否合理。

**参数没变 / 怎么判断自校准参数合理**：  
- 只有 **I0=6 (DONE)** 且 **I2=0** 时才会把算出的 kp/ki 写入 `g_M2_PI_d/q`。若 **I0=7、I2=2**（Ls 超范围），校准未成功，PI 保持初值，所以“参数没变”。  
- 结束时刻看 **I4=Rs(Ω)、I5=Ls(H)**：与电机规格对比（如 8HA0205-10 约 18.5Ω、5.3mH）。若 I5 为负或远大于 0.1，可适当放宽 `ls_min_henry`/`ls_max_henry` 或检查 AC 测量。  
- 合理性：Kp_si=(2/5)×L×fs，Wi=R/L；本工程 kp 约 5e4~15e4、ki/kp≈0.2 量级。校准时算出的 Rs/Ls 与规格接近、且 kp/ki 在此范围即合理。

**AC 校准没听见声音**：AC 段注入 200 Hz、约 0.05 A 正弦，电机可能只有轻微振动或听不见。**I2=2** 表示 AC 段已跑完并算出 Ls，但 Ls 被判超范围导致 ERROR，并非 AC 未执行。

**怎么配合调试**：  
1. 校准时打开 VOFA，结束时刻看 **I0/I2**：I0=6 且 I2=0 才写入 PI；I0=7 时看 I2（1=Rs 超范围，2=Ls 超范围，3=DC 电流太小，4=AC 异常）。  
2. **I2=2** 时看 **I4=Rs、I5=Ls**：若 I5 负（多为 AC 解调符号反，已按 Ls=-z_im/ω 修正）或异常大，可放宽 `ls_min_henry`/`ls_max_henry` 或查 AC 注入/解调。  
3. 把 **I0~I5（尤其 I4、I5）** 截图或数值发给我即可给出改参建议。
