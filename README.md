# BiStep-FOC

双轴 FOC 控制器 — 将两相混合式步进电机变成闭环伺服。

## 硬件

- **MCU**: STM32G474CC (170MHz, Cortex-M4F)
- **功率级**: 4 路独立半桥, 双电阻采样
- **编码器**: 双 KTH7111 磁编码器 (SPI, 16-bit)
- **PWM**: HRTIM 高分辨率定时器 (17kHz)

## 特性

- 纯软件 FOC, 无需专用电机驱动芯片
- 双电机独立控制 (M1 + M2), 四种运行模式可 Live Watch 实时切换:
  - **开环 (OPEN_LOOP)**: 调试 / 初始启动
  - **速度环 (SPEED)**: PI 闭环 + 斜坡限速
  - **位置环 (POSITION)**: PI-P 级联伺服定位
  - **步距角 (STEP_ANGLE)**: 直接 PD 弹簧-阻尼器, 高带宽匀速步进
- Id 三级自适应 (Standby / Hold / Boost), 类似 Trinamic IRun/IHold
- M1→M2 主从随动模式 (手轮跟随)
- dq 前馈解耦 + 逆 Park 延迟补偿
- 电流环自校准 (Rs / Ls 在线辨识)
- KTH7111 ANLC 非线性校准
- 电角度零点自动标定
- Flash 参数持久化 (Magic + CRC32)
- VOFA+ 实时波形调试引擎 (TIM4 1kHz DMA, 10 通道)

## 软件架构

```
17kHz  ADC1_2_IRQHandler   电流环 FOC (Park/Clarke/PI/SVPWM/解耦)
 1kHz  TIM3_IRQHandler     速度环 / 位置环 / 校准状态机
 1kHz  TIM4_IRQHandler     VOFA+ 调试引擎
 main  while(1)            模式切换 / 参数钳位 / 校准触发 / 阶跃测试
```

## 工具链

- STM32CubeIDE (CubeMX 代码生成 + GCC 编译)
- VOFA+ (串口波形调试)
- Live Watch (运行时参数调参)

## 构建

用 STM32CubeIDE 直接打开项目目录, Build Project 即可。
