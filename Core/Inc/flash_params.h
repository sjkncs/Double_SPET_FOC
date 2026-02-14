/**
 * @file  flash_params.h
 * @brief 校准参数 Flash 持久化（最后一页 Page 127 @ 0x0803F800）
 *
 * 数据结构:  FlashCalibData_t (~76 bytes, 远小于 2KB 页)
 * 校验方式:  Magic(0xCA1B0001) + CRC32
 * 写入时机:  校准成功后自动写入（关 PWM → erase+program → 恢复 PWM）
 * 读取时机:  main() 初始化阶段
 * 手动清除:  Live Watch 置 g_FlashParamsErase=1
 */
#ifndef FLASH_PARAMS_H
#define FLASH_PARAMS_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Flash 布局 ---- */
#define FLASH_PARAMS_MAGIC   0xCA1B0001U   /* 版本号在低位, 结构变化时递增 */
#define FLASH_PARAMS_PAGE    127U           /* STM32G474CC 最后一页 (单 bank) */
#define FLASH_PARAMS_ADDR    0x0803F800U    /* Page 127 起始地址 */

/* ---- 校准数据结构 (对齐到 8 字节, Flash double-word 编程要求) ---- */
typedef struct {
    uint32_t magic;                /* FLASH_PARAMS_MAGIC */

    /* M1 校准数据 */
    float    m1_rs_ohm;
    float    m1_ls_henry;
    int32_t  m1_kp;
    int32_t  m1_ki;
    uint32_t m1_vbus_calib_mv;
    uint8_t  m1_valid;             /* 1=M1 数据有效 */
    uint8_t  _pad1[3];

    /* M2 校准数据 */
    float    m2_rs_ohm;
    float    m2_ls_henry;
    int32_t  m2_kp;
    int32_t  m2_ki;
    uint32_t m2_vbus_calib_mv;
    uint8_t  m2_valid;             /* 1=M2 数据有效 */
    uint8_t  _pad2[3];

    /* 元信息 */
    uint32_t calib_count;          /* 累计校准次数 */
    uint32_t _reserved[3];         /* 预留扩展 */

    uint32_t crc32;                /* 以上所有字段的 CRC32 */
} FlashCalibData_t;

/* ---- API ---- */

/**
 * @brief  从 Flash 读取校准数据并校验 magic + CRC32
 * @param  out  输出结构体
 * @retval true=数据有效, false=无效或未校准
 */
bool FlashParams_Read(FlashCalibData_t *out);

/**
 * @brief  写入校准数据到 Flash (page erase + program)
 * @param  data  待写入的数据 (magic/crc 由内部填充)
 * @retval true=写入成功, false=失败
 * @note   调用前必须已关闭 PWM 输出 (CPU stall ~40ms)
 */
bool FlashParams_Write(const FlashCalibData_t *data);

/**
 * @brief  擦除 Flash 参数页 (恢复出厂)
 * @retval true=成功
 * @note   调用前必须已关闭 PWM 输出
 */
bool FlashParams_Erase(void);

#endif /* FLASH_PARAMS_H */
