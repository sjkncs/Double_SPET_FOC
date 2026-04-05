# BiStep-FOC AI 知识库汇总

> **导出日期**: 2026-04-05
> **来源**: `.cursor/rules/*.mdc` (共 8 个规则文件)
> **用途**: 供团队成员阅读，了解 AI 助手积累的项目知识全貌

---

# 第一章 项目架构与编码约定

> 来源: `project-conventions.mdc` (全局生效)

## 1.1 项目概述

STM32G474 + 双电机 FOC，将两相混合式步进电机变成闭环伺服。
4 路独立半桥 + 双电阻采样 + KTH7111 磁编码器，纯软件 FOC，无需专用电机驱动芯片。

## 1.2 架构分层

### ISR 层（实时控制, `stm32g4xx_it.c`）

**ADC1_2_IRQHandler (17kHz, 优先级 1)** — FOC 电流环主循环:
- `FOC_GetPhaseCurrent()` → 采样
- `KTH7111_ReadDual()` → 编码器 SPI
- `M1_UpdateElecAngle()` / `M2_UpdateElecAngle()` → 电角度
- `CalcSpeedDpp()` → 电角速度
- `FOC_CalcSinCos()` + `FOC_ParkTransform()` → Park 变换
- `StepCap_Record()` → 阶跃录制
- PI 闭环 / 校准开环电压注入
- `FOC_DecoupleFF()` → dq 前馈解耦
- `FOC_IsVqSaturated()` → 饱和诊断
- `FOC_CircleLimitation()` → 电压圆限幅
- `FOC_InvParkWithComp()` → 逆 Park + 延迟补偿
- `FOC_SetPhasePWM()` → PWM 输出

**TIM3_IRQHandler (1kHz, 优先级 3)** — 速度/位置/校准:
- `CalibM1/M2_OnTIM3_1ms()` → 电流环校准状态机
- 速度计算 + IIR 滤波 + 位置累计器
- `M1_ControlLoop(d1)` / `M2_ControlLoop(d2)` → 外环控制

**TIM4_IRQHandler (1kHz, 优先级 10)** — `Comm_OnTIM4_1ms()` 通信调度层 (PROTOCOL 模式发状态帧 / VOFA 模式调 `Vofa_OnTIM4_1ms()`)

### 模块层

- `foc_globals.c`: **全局变量集中定义** — 按信号流分 7 区 (硬件采样/FOC链/电流PI/M2外环/M1外环/校准/调试)
- `vofa_engine.c/h`: **VOFA+ 调试引擎** — `Vofa_InitDMA()` + TIM4 自动填帧 + DMA 发送
- `main.c`: 主循环编排 — `M1/M2_UpdateCtrlRef()` + `M1/M2_ClampParams()` + `PollCalibTriggers()` + 阶跃测试
- `zero_calib.c/h`: 电角度零点标定（Id 吸合摆动 + 编码器 ZERO 写入, 阻塞式）
- `calib_platform_m1.c/h` / `calib_platform_m2.c/h`: 电流环校准平台层（回调 + 启动/完成 + TIM3 驱动）
- `curr_loop_autocalib.c/h`: 电流环自校准引擎（纯算法, 不访问硬件）
- `foc_adapt.c/h`: FOC 自适应辅助（PI Vbus 缩放 + dq 解耦 + ADC 校准 + 调试）
- `flash_params.c/h`: Flash 校准参数持久化（Page 127, Magic+CRC32）
- `kth71xx.c/h`: KTH7111 编码器驱动 + ANLC 非线性校准 + `KTH71_ReadZero()`
- `follow_m1m2.c/h`: M1→M2 主从随动（M1 手轮 + M2 步距角跟随）
- `comm_protocol.c/h`: **RK3576 通信协议** — UART DMA 收发 + 帧解析 + 命令派发 + 状态帧 + 看门狗 (详见 `docs/RK3576通信协议规格书.md`)

### main.h ALWAYS_INLINE 函数

- `FOC_CalcSinCos` / `FOC_ParkTransform` / `FOC_InvParkTransform` — 坐标变换
- `FOC_CircleLimitation` — 电压圆限幅
- `FOC_IsVqSaturated` — 饱和诊断
- `FOC_DecoupleFF` — dq 前馈解耦
- `FOC_InvParkWithComp` — 逆 Park + 延迟补偿
- `SpeedPI_Run` / `PosPID_Run` — 速度/位置 PI
- `OpenLoop_IncAngle` / `OpenLoop_IncAngle_Ref` / `M1_OpenLoop_IncAngle_Ref` — 开环角度递增

## 1.3 VOFA+ 调试引擎

### 架构
- **TIM4 (1kHz, 优先级 10 最低)** 每 1ms 自动填帧 + DMA 发送
- 各模块只需设 `g_VofaSrc` 切换数据源，无需关心 DMA
- 即使 while(1) 被 HAL_Delay 阻塞（如 KTH71 校准），TIM4 仍正常发送

### 通道分配 (10 通道 I0~I9)
- **I0~I5**: 数据源相关
- **I6**: Id_Eff (mA) — 三级自适应有效 Id (Standby/Hold/Boost)
- **I7**: Id_fbk (mA) — 实时 d 轴电流反馈 (Q15→mA)
- **I8**: Iq_fbk (mA) — 实时 q 轴电流反馈 (Q15→mA)
- **I9**: VqSat 报警 — 0=正常, 1000=Vq 超出电压圆

### 数据源枚举 (`VofaSrc_t`)

| 数据源 | 说明 | I0~I5 含义 |
|---|---|---|
| `VOFA_SRC_IDLE` | 不发送 | — |
| `VOFA_SRC_NORMAL` | 按控制模式自动切换 | 速度/步距角/位置各不同 |
| `VOFA_SRC_CURR_CALIB_M1/M2` | 电流环校准 | State/Counter/Err/Id(A)/Rs(Ω)/Ls(mH) |
| `VOFA_SRC_KTH71_CALIB` | KTH71 校准 | DbgStep/Id/Iq/Vd/Vq/Angle |
| `VOFA_SRC_STEP_PLAYBACK` | 阶跃回放 | Id_ref/Iq_ref/Id_fbk/Iq_fbk/Idx/Phase |
| `VOFA_SRC_FOLLOW` | M1→M2 随动 | M1_Pos/M2_Pos/Err/Iq/M1_RPM/M2_RPM |

## 1.4 控制模式 (`MotorCtrlMode_t`)

| 值 | 模式 | Park 角度 | D 轴 | Q 轴 |
|---|---|---|---|---|
| 0 | MODE_OPEN_LOOP | θ_cmd(步进) | Id_Ref(mA) | Iq_Ref(mA) |
| 1 | MODE_SPEED | θ_encoder | Id_Eff | 速度PI→Iq |
| 2 | MODE_POSITION | θ_encoder | Id_Eff | 位置PI→速度P→Iq |
| 3 | MODE_STEP_ANGLE | θ_encoder | Id_Eff | 直接PD→Iq |

### Id 三级自适应 (Standby / Hold / Boost + 迟滞)
- **Standby(25mA)**: |speed|<5 且 |Δenc|≤18 持续 500ms → 省电
- **Hold(50mA)**: 正常运行态
- **Boost(100mA)**: |pos_err| > 500 counts(2.75°) → 增强刚度抗失步
- 迟滞防抖: 进入Boost阈值500, 退出阈值250

## 1.5 全局变量分区 (`foc_globals.c` 定义, `main.h` extern)

1. **硬件采样区**: `g_VbusRaw`, `g_Vbus_mV`
2. **FOC 信号链区**: `g_M1/M2_Idq`, `g_M1/M2_Idq_Ref`, `g_M1/M2_Vdq`, `g_M1/M2_Vab`, `g_M1/M2_ElecAngle_Q15`, `g_M1/M2Current`
3. **电流环 PI 区**: `g_M1/M2_PI_d`, `g_M1/M2_PI_q`, `g_M1/M2_PI_Base`, `g_M1/M2_wLs_factor`
4. **M2 外环区**: `g_M2_SpeedPI`, `g_M2_PosPID`, `g_M2_StepAnglePD`, `g_M2_CtrlMode`, `g_M2_PosCmd`, ...
5. **M1 外环区**: `g_M1_SpeedPI`, `g_M1_PosPID`, `g_M1_StepAnglePD`, `g_M1_CtrlMode`, `g_M1_PosCmd`, ...
6. **校准区**: `g_DoCurrLoopCalib_M1/M2`, `g_CalibVdDirectActive_M1/M2`, `g_DoKth71Calib_M1/M2`, `g_DoZeroCalib_M1/M2`, ...
7. **调试/VOFA 区**: `g_VofaSrc`, `g_VofaMotorSel`, `g_StepCapMotor`, `g_FollowM1M2`, `g_FlashParamsErase`, ...

## 1.6 中断优先级（数值越小优先级越高, 以 CubeMX .ioc 为准）

| 优先级 | 中断 | 说明 |
|---|---|---|
| 0 | HRTIM FLT | 硬件故障保护 |
| 1 | ADC1_2 (17kHz) | FOC 电流环 |
| 2 | HRTIM Master | PWM 周期更新 |
| 3 | TIM3 (1kHz) | 速度环 / 位置环 / 校准驱动 / 通信看门狗递增 |
| 7 | DMA1_CH2 (RX) | 已禁用 (Circular 模式不需要) |
| 8 | DMA1_CH1 (TX) | USART1 TX DMA 完成 |
| 9 | USART1 (IDLE) | 通信 RX 写指针更新 |
| 10 | TIM4 (1kHz) | 通信状态帧 / VOFA 引擎 |
| 15 | SysTick | HAL_IncTick |

## 1.7 main.c 主循环结构

`while(1)` 已重构为函数调用链:
- `Protocol_Poll()` → RK3576 通信帧解析 + 命令派发 + 看门狗超时检测
- `g_UpSeqNo++` → 主循环存活计数 (状态帧回传给上位机)
- `Follow_M1M2_Poll()` → 主从随动
- `M1_UpdateCtrlRef()` / `M2_UpdateCtrlRef()` → 开环/步距角刷新 Idq_Ref + AngleDelta
- `M1_ClampParams()` / `M2_ClampParams()` → Live Watch 参数安全钳位
- `FOC_UpdatePI_ByVbus()` → Vbus 采样 + PI 自适应缩放
- `Debug_ReadCurrent_mA()` → 调试电流显示
- `PollCalibTriggers()` → KTH71/零点/电流环校准 + Flash 擦除
- `StepCap_ArmAndAlign()` / `StepCap_Playback()` → 电流环阶跃响应测试

初始化:
- `KTH71_ReadZero()` → 编码器 ZERO 寄存器回读
- `HRTIM_PostInit()` → HRTIM 外设后置配置
- `Vofa_InitDMA()` → USART1 TX DMA 绑定
- `Comm_Init()` → 通信协议 RX DMA + IDLE 中断配置
- `TIM3_StartSpeedLoop()` → 速度环定时器启动

## 1.8 编码规范

- Q15 定点格式：int16_t 范围 [-32768, 32767] 映射到 [-1.0, 1.0)
- 平台回调模式：算法模块通过函数指针访问硬件，不直接 include HAL
- 所有中文注释，代码标识符用英文
- 全局变量集中定义在 `foc_globals.c`, `extern` 声明在 `main.h`
- ISR 辅助函数用 `static inline` 置于 `stm32g4xx_it.c` 的 `USER CODE BEGIN 1` 区域
- CubeMX 兼容: 自定义代码放 `USER CODE BEGIN/END` 区域

---

# 第二章 RK3576 通信协议模块

> 来源: `comm-protocol-knowledge.mdc`
> 完整规格书: `docs/RK3576通信协议规格书.md`

## 2.1 架构

```
RK3576 (Linux) ──── UART 1152000 8N1 ──── STM32G474 (FOC 控制器)
               PB6(TX) / PB7(RX)
字节序: 小端 (Little-Endian)
```

**RX 路径**: USART1_RDR → DMA1_CH2 Circular → s_rxBuf[128] → IDLE ISR 更新写指针 → while(1) Protocol_Poll() 帧解析

**TX 路径**: TIM4 ISR (1kHz) → Comm_OnTIM4_1ms() → PROTOCOL 模式发状态帧 / VOFA 模式调 Vofa_OnTIM4_1ms()

## 2.2 帧格式

```
[0xAA][0x55][Len(1B)][Payload(N B)][CRC16(2B, 小端)]
CRC-CCITT: poly=0x1021, init=0xFFFF, 无反射, 无最终异或
测试向量: CRC("123456789") = 0x29B1
```

## 2.3 命令总表

| CmdID | 名称 | Payload大小(含CmdID) | 状态 |
|---|---|---|---|
| 0x01 | QUERY_CAPS | 1B | ✅ |
| 0x10 | SET_MOTION | 6B | ✅ |
| 0x11 | SET_POSITION | 6B | ✅ |
| 0x12 | SET_IQ_REF | 5B/9B/13B | ✅ (+Id+AngleDelta) |
| 0x20 | SET_CTRL_MODE | 3B | ✅ |
| 0x21 | SET_FOLLOW | 2B | ✅ |
| 0x30 | SET_SPEED_PI | 10B | ✅ |
| 0x31~0x33 | SET_POS_PID / SET_ID_ADAPT / SET_RAMP_RATE | — | TODO |
| 0x40~0x42 | 校准触发 | 2B | ✅ |
| 0x43 | FLASH_UNLOCK | 1B | ✅ |
| 0x44 | FLASH_ERASE | 3B | ✅ |
| 0x50 | QUERY_STATUS | 1B | ✅ |
| 0x51~0x53 | VOFA/状态率 | — | ✅ |

## 2.4 看门狗

- `g_CommWatchdogMs`: TIM3 ISR 每 1ms 无条件递增, Protocol_Poll 中有效帧清零
- 超时阈值: `g_CommTimeoutMs` 默认 200ms
- 超时动作: 切开环 + Iq/Id/AngleDelta 归零 (零电流自由滑行)
- 校准期间放宽: handle_trig_calib() 设 20000ms, PollCalibTriggers() 完成后恢复 200ms

## 2.5 CubeMX 注意事项

- NVIC 优先级只由 CubeMX 管理, 代码中不手动调用 `NVIC_SetPriority`
- USART1_IRQHandler IDLE 处理放在 `USER CODE BEGIN USART1_IRQn 0`
- Comm_Init() 仅做 DMA 地址绑定 + LL_USART_EnableIT_IDLE

---

# 第三章 校准与调试知识库

> 来源: `motor-calib-knowledge.mdc`

## 3.1 硬件平台

- MCU: STM32G474CC (256KB Flash, 128KB RAM)
- M2 电机: 8HA0205-10 两相步进, 1.8° 步距角, **50 极对**
- M1 电机: 14HYTF5203Z-15-500, 0.9° 步距角, **100 极对**
- 供电: 标称 12V, 实测 Vbus ≈ 11.9V
- 电流采样满量程: 825mA

### HRTIM → 电机 → ADC 引脚映射

| HRTIM | Motor | Phase | 电流采样引脚 | ADC寄存器 |
|-------|-------|-------|------------|-----------|
| HRTIMA | M1 | Phase A | PA2 | ADC1→JDR1 |
| HRTIMB | M1 | Phase B | PA0 | ADC2→JDR1 |
| HRTIMC | M2 | Phase A | PA1 | ADC2→JDR2 |
| HRTIMD | M2 | Phase B | PA3 | ADC1→JDR2 |

## 3.2 电机规格 vs 实测

### M2 (8HA0205-10, NEMA08, 50pp)
- Rs: 规格 20.3Ω, 实测 23.3Ω (+15%) — 含引线+接插件电阻
- Ls: 规格 5.3mH, 实测 7.83mH (+48%) @293mA — 低电流铁芯未饱和

### M1 (14HYTF5203Z-15-500, 100pp)
- Rs: 实测 32.68Ω
- Ls: 实测 37.78mH — 高电感, M1 dc_duty_max 提至 85%
- τ_e = Ls/Rs ≈ 1.16ms（M2 的 4 倍, PI 带宽更低）

## 3.3 关键设计决策

1. **开环电压注入校准** — 不依赖预设 PI, 不同电机无需查手册
2. **校准电流接近工作电流** — Ls 随电流变化, 低电流 Ls 偏高
3. **PI 安全裕量 ×0.8** — 吸收 R/L 测量误差 + 温漂
4. **Flash 持久化** — Page 127, Magic+CRC32, 写入前关 HRTIM PWM
5. **PI Vbus 自适应** — kp_actual = kp_base × vbus_base / vbus_actual
6. **dq 前馈解耦** — 仅闭环模式, 开环严格禁止 (丢步时正反馈失控)
7. **KTH7111 ANLC** — 二次校准需先 REG_CAL=0 + 延时复位; MTP 前必须写 0x16=0x08

## 3.4 常见陷阱

1. Clang linter 误报是假阳性
2. Rs 偏高包含引线电阻, 对 PI 反而更正确
3. KTH71 ANLC 二次校准: 必须先 REG_CAL=0 + 延时
4. KTH71 SPI 帧间时序 >150ns: 外部调用须插入 `KTH71_SpiGap()`
5. 步距角闭环必须在机械位置空间 (不是电角度)
6. 100pp 步进电机超出 KTH7111 编码器精度能力 (INL ×100pp = ±20° 电角度)
7. 静止检测: 编码器量化噪声 → 用 `abs_d <= 1` 而非 `== 0`

---

# 第四章 编码器校准操作文档

> 来源: `编码器校准.mdc`

## 4.1 校准顺序

**必须按顺序**: ① KTH71 ANLC 非线性校准 → ② 电角度零点标定

> ANLC 修改编码器非线性补偿表, 会改变角度输出。先零点再 ANLC 会让零点偏移。

## 4.2 KTH71 ANLC 非线性校准

- **触发**: `g_DoKth71Calib_M1/M2 = 1` (代码全自动接管)
- **过程**: ~15-20s, 芯片 PWM 引脚 3.8Hz 闪烁 = BUSY
- **判定**: VOFA I0 = 4 (DONE) 成功, 6 (TIMEOUT) 失败
- **自动 MTP 固化**: 成功后自动写 0x16=0x08 + 烧 MTP

## 4.3 电角度零点标定

- **触发**: `g_DoZeroCalib_M1/M2 = 1`
- **过程**: ~4s 阻塞, Id 300mA 吸合 → 摆动 4 次 → 锁定 2s → 读角度 → 写 ZERO + MTP
- **公式**: `new_zero = old_zero ∓ enc_angle` (取决于 RD 位)

## 4.4 完整校准流程

```
Step 1: 电流环校准 → g_DoCurrLoopCalib_M1/M2 = 1 (~3s)
Step 2: KTH71 ANLC → g_DoKth71Calib_M1/M2 = 1 (~15s)
Step 3: 零点标定 → g_DoZeroCalib_M1/M2 = 1 (~4s)
Step 4: 验证 → 切速度模式, RPM_Cmd=100, 观察跟踪
```

---

# 第五章 电流环自整定与阶跃响应测试

> 来源: `电流环参数自整定与阶跃响应测试.mdc`

## 5.1 电流环 Rs/Ls 自校准

- **触发**: `g_DoCurrLoopCalib_M1/M2 = 1`
- **过程**: ~3s, DC_RAMP → DC_SETTLE → DC_MEASURE → AC_SETTLE → AC_MEASURE → RAMP_DOWN
- **VOFA**: I0=状态(7=DONE), I3=DC电流(A), I4=Rs(Ω), I5=Ls(mH)
- **自动写入**: PI 参数 + PI 基准 + Flash 保存

## 5.2 dq 阶跃响应测试

- **触发**: `g_StepCapArm_M1/M2 = 1`
- **过程**: 吸合对齐 d 轴 → Phase 0 (d 轴) 128 点 @17kHz → Phase 1 (q 轴) 128 点
- **评价**: 上升时间 Tr, 过冲 <15%, 5~10 点内稳定 = 良好

---

# 第六章 速度环 PI 调参指南

> 来源: `编码器校准完成后的速度环 PI 调参.mdc`

## 6.1 架构

```
RPM_Cmd → [斜坡限速] → SpeedPI_Run(1kHz) → Iq(mA) → 电流环 PI(17kHz) → PWM
```

## 6.2 参数含义 (`SpeedPI_Float_t`)

| 参数 | 默认值 | 单位 | 说明 |
|---|---|---|---|
| kp | 0.5 | mA/RPM | 每 1 RPM 误差产生的力矩电流 |
| ki | 0.01 | mA/(RPM·ms) | 消除摩擦/齿槽导致的稳态偏差 |
| integ_limit | 100 | mA | 防堵转积分飙升 |
| out_limit | 300 | mA | 最大力矩电流 |

## 6.3 调参步骤

1. **先调 kp (关积分 ki=0)**: 太慢加大, 振荡减半
2. **加 ki (消偏差)**: 太慢加倍, 过冲减半
3. **调 integ_limit**: ≈ 1/3 × out_limit
4. **验证**: 变速扫描 (100→400→-400→0) + 手捏电机轴抗扰测试

**kp 参考范围**: NEMA08 0.3~1.0, NEMA17 0.5~2.0, NEMA23 0.5~3.0

---

# 第七章 换电机后的极对数配置与开环测试

> 来源: `换电机后的极对数配置与开环测试规程.mdc`

## 7.1 极对数计算

极对数 = 360° / 步距角 / 2 (1.8°→50pp, 0.9°→100pp)

## 7.2 修改代码 (3 处宏, `main.h`)

1. `M1_POLE_PAIRS` — 极对数
2. `M1_RAMP_DIV` — 开环斜坡分频 (高电感用 2~4, 低电感用 1)
3. `M1_ENC_DIR` — 编码器方向 (+1 或 -1)

## 7.3 开环验证

- 上电后 M1 自动开环旋转 (Iq=50mA, RPM=400)
- 检查 `g_Enc1_SpeedFilt ≈ 400` 且同号
- 转速扫描: 100→400→-400, 确认极对数正确
- 极对数错误表现: 抖动/啸叫/不转/转速是指令的整数倍

---

# 第八章 两相混合步进伺服 vs 三相永磁同步伺服

> 来源: `步进伺服 vs 永磁同步.mdc`

## 8.1 核心差异

| | 步进 (2相) | 三相 PMSM |
|---|---|---|
| 极对数 | 50~100pp | 2~8pp |
| 电感 | 5~40mH | 0.1~5mH |
| 效率 | 40~60% | 80~95% |
| 过载能力 | 无 | 3倍额定 |
| 额定转速 | 300~800 RPM | 2000~3000 RPM |
| 编码器要求 | 极高 (INL×pp 放大) | 低 |

## 8.2 步进伺服优势

- 低速大扭矩 (同体积保持力矩 1.5~2 倍)
- 成本低 (电机 ¥15~30 vs 伺服 ¥200+)
- 天然齿槽自锁

## 8.3 本项目选型逻辑

- 工作转速 <600 RPM → 步进舒适区
- 小负载 50~100mA → 步进绰绰有余
- 成本敏感 → 步进大优势
- M2 需保持位置 → 齿槽自锁

## 8.4 编码器精度放大效应 (实测教训)

```
电角度误差 = 机械角度 INL × 极对数

KTH7111 INL ≈ ±0.2° 机械:
  × 50pp (M2) = ±10° 电角度 → 力矩损失 ~1.5%, 可接受
  × 100pp (M1) = ±20° 电角度 → 力矩反向, 电机卡死!
```

**结论**: 100pp 超出普通磁编码器能力。三相伺服 4~8pp 同样 INL 仅 ±0.8°~1.6°。
