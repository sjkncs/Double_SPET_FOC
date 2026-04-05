# STM32 固件待完成项

> **日期**: 2026-04-05
> **最后更新**: 2026-04-05 (固件侧已完成全部 P0 项 + P2 部分项)
> **现状**: `comm_protocol.h/c` 核心功能 100% 就绪，可烧录对接调试工具。
> **剩余**: P1 校准状态回报 + P2 参数命令存根 (不影响调试工具初期对接)。

---

## 一、已完成清单 (无需修改)

| 模块 | 状态 | 文件 |
|---|---|---|
| 帧格式 (AA 55 Len Payload CRC16) | ✅ 已完成 | comm_protocol.c |
| CRC-CCITT 查表 | ✅ 已完成 | comm_protocol.c |
| RX DMA Circular 初始化 | ✅ 已完成 | Comm_Init() |
| IDLE 中断处理 | ✅ 已完成 | Comm_OnIdleIRQ() |
| while(1) 帧解析 (Protocol_Poll) | ✅ 已完成 | comm_protocol.c |
| 命令派发 (dispatch_cmd) | ✅ 已完成 | comm_protocol.c |
| SET_MOTION (0x10) | ✅ 已完成 | M1 增量 + M2 绝对 |
| SET_POSITION (0x11) | ✅ 已完成 | 单轴绝对位置 |
| SET_IQ_REF (0x12) | ✅ 已完成 | 含 OpenLoop 模式校验 + 30s 超时 + **Id 扩展** |
| SET_CTRL_MODE (0x20) | ✅ 已完成 | 含校准中拒绝 |
| SET_FOLLOW (0x21) | ✅ 已完成 | |
| TRIG_CURR_CALIB (0x40) | ✅ 已完成 | 设 g_DoCurrLoopCalib 标志 + **看门狗放宽** |
| TRIG_KTH71_CALIB (0x41) | ✅ 已完成 | 设 g_DoKth71Calib 标志 + **看门狗放宽** |
| TRIG_ZERO_CALIB (0x42) | ✅ 已完成 | 设 g_DoZeroCalib 标志 + **看门狗放宽** |
| FLASH_UNLOCK (0x43) | ✅ 已完成 | 含条件校验 |
| FLASH_ERASE (0x44) | ✅ 已完成 | 含 Magic 0xDEAD 校验 |
| QUERY_CAPS (0x01) | ✅ 已完成 | 响应帧含 FwVer/ProtoVer/Features |
| QUERY_STATUS (0x50) | ✅ 已完成 | 触发一帧状态帧 |
| SET_VOFA_MODE (0x51) | ✅ 已完成 | |
| EXIT_VOFA (0x52) | ✅ 已完成 | |
| SET_STATUS_RATE (0x53) | ✅ 已完成 | |
| SET_SPEED_PI (0x30) | ✅ **本次新增** | Motor(u8)+kp(f32)+ki(f32)=9B |
| 状态帧 (0x80, 48B) | ✅ 已完成 | 含 UpSeqNo/CrcErrCount/milliRPM |
| CAPS 响应帧 (0x81) | ✅ 已完成 | |
| TX DMA 发送 + DMA 忙守卫 | ✅ 已完成 | |
| 看门狗 (超时→零电流) | ✅ 已完成 | **含校准期间自动放宽** |
| SET_IQ_REF 超时保护 | ✅ 已完成 | 30s 无新帧→自动归零 |
| Flash 解锁超时 | ✅ 已完成 | 5s (HAL_GetTick 精确计时) |

---

## 二、处理结果汇总

### ✅ P0 — 已全部完成

#### 2.1 USART1_IRQHandler ← Comm_OnIdleIRQ()

**状态**: ✅ **此前已完成，无需修改**

**证据**: `stm32g4xx_it.c` 的 CubeMX 生成的 `USART1_IRQHandler` 中，`USER CODE BEGIN USART1_IRQn 0` 区域已包含 IDLE 检测 + `Comm_OnIdleIRQ()` 调用。CubeMX USART1 global interrupt 已使能，NVIC 优先级 = 9。

---

#### 2.2 TIM4_IRQHandler ← Comm_OnTIM4_1ms()

**状态**: ✅ **此前已完成，无需修改**

**证据**: `stm32g4xx_it.c` 的 `TIM4_IRQHandler` 中已调用 `Comm_OnTIM4_1ms()` (替代原来直接调 `Vofa_OnTIM4_1ms()`)。`Comm_OnTIM4_1ms()` 内部按 `g_CommMode` 分流 PROTOCOL/VOFA 双模。

---

#### 2.3 TIM3 ISR 看门狗递增

**状态**: ✅ **此前已完成，无需修改**

**证据**: `stm32g4xx_it.c` 的 `TIM3_IRQHandler` 中已有 `g_CommWatchdogMs++;`，位于速度环/位置环处理之前，确保无条件 1ms 递增。

---

#### 2.4 SET_IQ_REF 扩展 Id 字段

**状态**: ✅ **本次完成** — 采用方案 A (扩展 payload)，并增加向后兼容

**修改文件**:
- `comm_protocol.h`: `CommSetIqRef_t` 从 4B 扩展到 8B，增加 `m1_id_ma`/`m2_id_ma` 字段，定义 `COMM_SET_IQ_REF_MIN_LEN=4`
- `comm_protocol.c`: `handle_set_iq_ref()` 接受 4B 或 8B 两种 payload:
  - 4B (旧版): 仅设 Iq，Id 不变
  - 8B (新版): 同时设 Iq 和 Id (`g_Mx_Id_Hold_mA`)

**向后兼容设计**: 固件检查 `len >= sizeof(CommSetIqRef_t)` 判断是否包含 Id 字段。旧版上位机发 4B payload 仍然正常工作，新版上位机发 8B payload 可同时控制 Id。

---

### ✅ P1 部分完成

#### 2.5 校准状态回报增强

**状态**: ⏭️ **暂不实现** (P1)

**跳过理由**:
1. **影响面大**: 需修改校准平台代码 (`calib_platform_m1.c`/`calib_platform_m2.c`、`zero_calib.c`、`kth71xx.c`)，这些模块的回调链与 `s_lastCmdResult` (static in `comm_protocol.c`) 之间没有现成的桥接机制
2. **现有替代方案可用**: `LastCmdResult` 字段 + `SysFlags.CalibInProgress` 已能提供基本的校准成功/失败信息，调试工具初期够用
3. **建议后续方案**: 增加 `Comm_SetLastCmdResult(uint8_t result)` 公开函数，在校准完成回调中调用，比改状态帧 Flags 位更简洁

---

#### 2.6 校准看门狗放宽

**状态**: ✅ **本次完成**

**修改文件**:
- `comm_protocol.c`: `handle_trig_calib()` 末尾设 `g_CommTimeoutMs = 20000` (20s，覆盖 KTH71 最长 ~15s 阻塞)
- `main.c`: `PollCalibTriggers()` 中每个校准完成后恢复 `g_CommTimeoutMs = 200`:
  - KTH71 M1/M2: 阻塞调用返回后立即恢复
  - Zero M1/M2: 阻塞调用返回后立即恢复
  - CurrLoop M1/M2: `CalibMx_OnDone()` 回调后恢复 (非阻塞，while(1) 不卡)

---

### ✅ P2 部分完成

#### 2.7 SET_SPEED_PI (0x30)

**状态**: ✅ **本次完成**

**修改文件**:
- `comm_protocol.c`: 新增 `handle_set_speed_pi()` 函数，解析 Motor(u8)+kp(f32)+ki(f32)=9B
- dispatch_cmd 中 `CMD_SET_SPEED_PI` 从存根改为调用 `handle_set_speed_pi()`

---

#### 2.8 SET_POS_PID (0x31), SET_ID_ADAPT (0x32), SET_RAMP_RATE (0x33)

**状态**: ⏭️ **暂不实现** (P2)

**跳过理由**: 上位机文档自己明确标注"不紧急，调试工具初期不会用到这些"。结构与 2.7 相同 (`memcpy` + 赋值)，需要时半小时内可完成。

---

## 三、集成检查清单

| # | 检查项 | 状态 | 验证位置 |
|---|---|---|---|
| 1 | CubeMX USART1 global interrupt 使能, 优先级=9 | ✅ | MX_USART1_UART_Init() |
| 2 | CubeMX DMA1_CH2 Circular 模式 | ✅ | .ioc 文件 |
| 3 | USART1_IRQHandler 调用 Comm_OnIdleIRQ() | ✅ | stm32g4xx_it.c USER CODE USART1_IRQn 0 |
| 4 | TIM4 中断调用 Comm_OnTIM4_1ms() | ✅ | stm32g4xx_it.c TIM4_IRQHandler |
| 5 | TIM3 ISR 有 g_CommWatchdogMs++ | ✅ | stm32g4xx_it.c TIM3_IRQHandler |
| 6 | while(1) 有 Protocol_Poll() + g_UpSeqNo++ | ✅ | main.c while(1) |
| 7 | main() 初始化有 Comm_Init() | ✅ | main.c, MX_USART1_UART_Init() 之后 |
| 8 | #include "comm_protocol.h" | ✅ | main.c + stm32g4xx_it.c |

**结论**: 全部通过，可直接烧录。

---

## 四、⚠️ 审视与潜在风险

### 4.1 motor 字段编码约定 — 需双方确认

**问题**: 协议中多个命令 (SET_CTRL_MODE, SET_SPEED_PI, TRIG_CALIB 等) 使用 `motor` 字段选择电机。当前固件实现为:

| motor 值 | 含义 |
|---|---|
| 0 | M2 |
| 1 | M1 |
| 2 | 双轴 (M1+M2) |

**风险**: 如果上位机团队按直觉理解为 `0=M1, 1=M2`，则 M1/M2 会写反。**建议**: 上位机团队对接前确认此约定，或在协议文档中加粗标注。

### 4.2 校准阻塞期间命令堆积

校准看门狗已放宽到 20s，但阻塞期间上位机发送的命令会堆积在 128B RX DMA 缓冲区。如果上位机持续高频发送 (如 200Hz SET_MOTION)，128B 缓冲区可能溢出 (DMA Circular 会覆盖旧数据)。

**建议**: 上位机在触发校准后停止发送运动命令，仅保留低频 QUERY_STATUS 轮询 (1~5Hz)。

### 4.3 SET_IQ_REF Id 写入的是 Hold 级别

`SET_IQ_REF` 的 Id 字段写入的是 `g_Mx_Id_Hold_mA` (三级自适应的 Hold 级别)，不是直接写 `g_Mx_Idq_Ref.d`。在开环模式下，Id 自适应逻辑仍然生效，实际 Id 可能被 Standby/Boost 逻辑修改。如果调试工具需要精确的 Id 直通，需要确认开环模式下自适应逻辑是否会干扰。

### 4.4 SET_SPEED_PI 不重置积分

`handle_set_speed_pi()` 只修改 kp/ki，不清零 `integral` 和 `output`。运行中改参数如果 ki 变化剧烈，残留积分可能导致瞬间大电流。上位机调参时建议先停车 → 改参 → 再启动。

---

## 五、新电机首次校准流程

通过调试工具操作的完整流程：

```
1. 连接串口 → 自动发 QUERY_CAPS → 确认固件版本和能力

2. M1 校准 (三步):
   a. [电流环校准] 按钮 → 发 TRIG_CURR_CALIB(Motor=1)
      等待 SysFlags.CalibInProgress 变 0 (~3秒)
      检查 LastCmdResult == OK
      
   b. [KTH71校准] 按钮 → 发 TRIG_KTH71_CALIB(Motor=1)  
      等待 ~15秒 (看门狗已放宽)
      检查 LastCmdResult == OK
      
   c. [零点标定] 按钮 → 发 TRIG_ZERO_CALIB(Motor=1)
      等待 ~4秒
      检查 LastCmdResult == OK

3. M2 校准 (同上三步, Motor=0)

4. 保存到 Flash:
   a. 点 [Flash保存] → 发 FLASH_UNLOCK(0x43)
      检查 SysFlags.FlashUnlocked == 1
   b. 5秒内点 [确认擦除] → 发 FLASH_ERASE(0x44, Magic=0xDEAD)
      检查 LastCmdResult == OK
      检查 SysFlags.FlashLoaded == 1

5. 校准完成，可切换到闭环模式开始调试
```

---

## 六、调试工具侧 (网页 JS) 需要实现的部分

这些不需要改固件，全在网页端 JS 中实现:

| 功能 | 说明 |
|---|---|
| CRC-CCITT JS 实现 | 查表法，与固件相同的 256 项表 |
| buildFrame(cmdId, payload) | 组装 AA 55 Len Payload CRC |
| Web Serial 读取循环 | ReadableStream → 字节流 → 帧解析状态机 |
| 状态帧解析 (48B) | DataView 按偏移提取各字段 → 更新 UI + 喂 uPlot |
| 握手流程 | 连接后发 QUERY_CAPS → 验证响应 |
| 校准面板 UI | 6 个按钮 (M1/M2 × 电流环/KTH71/零点) + Flash 保存 |
| 校准进度显示 | 轮询状态帧 SysFlags + LastCmdResult |
