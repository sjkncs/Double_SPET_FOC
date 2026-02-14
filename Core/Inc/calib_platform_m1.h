/**
 * @file    calib_platform_m1.h
 * @brief   M1 电流环自校准平台层 — 镜像 M2, 适配 M1 硬件
 */
#ifndef CALIB_PLATFORM_M1_H
#define CALIB_PLATFORM_M1_H

#include <stdint.h>

void CalibM1_Start(void);
void CalibM1_OnDone(void);
void CalibM1_OnTIM3_1ms(void);
void CalibM1_FillVofa(void);

#endif /* CALIB_PLATFORM_M1_H */
