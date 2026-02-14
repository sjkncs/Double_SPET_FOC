/**
 * @file  flash_params.c
 * @brief 校准参数 Flash 持久化：CRC32 + HAL Flash 读/写/擦除
 *
 * STM32G474CC: 256KB Flash, 页大小 2KB (单 bank)
 * 使用最后一页 Page 127 @ 0x0803F800
 * Flash 擦写寿命 10000 次, 校准频率极低, 无需磨损均衡
 */
#include "flash_params.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* ---- 软件 CRC32 (ISO 3309 / ITU-T V.42, 与 STM32 CRC 外设兼容) ---- */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    crc = ~crc;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1U));
        }
    }
    return ~crc;
}

static uint32_t calc_crc(const FlashCalibData_t *d)
{
    /* CRC 覆盖 magic 到 crc32 字段之前的所有字节 */
    uint32_t payload_size = offsetof(FlashCalibData_t, crc32);
    return crc32_update(0, (const uint8_t *)d, payload_size);
}

/* ---- API 实现 ---- */

bool FlashParams_Read(FlashCalibData_t *out)
{
    if (!out) return false;

    /* Flash 可直接 memcpy 读取 (memory-mapped) */
    memcpy(out, (const void *)FLASH_PARAMS_ADDR, sizeof(FlashCalibData_t));

    /* 校验 magic */
    if (out->magic != FLASH_PARAMS_MAGIC) return false;

    /* 校验 CRC32 */
    uint32_t expected = calc_crc(out);
    return (out->crc32 == expected);
}

bool FlashParams_Write(const FlashCalibData_t *data)
{
    if (!data) return false;

    /* 准备写入缓冲: 填充 magic + CRC */
    FlashCalibData_t buf;
    memcpy(&buf, data, sizeof(buf));
    buf.magic = FLASH_PARAMS_MAGIC;
    buf.crc32 = calc_crc(&buf);

    /* 解锁 Flash */
    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) return false;

    /* 擦除 Page 127 */
    FLASH_EraseInitTypeDef erase_cfg = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = FLASH_BANK_1,
        .Page      = FLASH_PARAMS_PAGE,
        .NbPages   = 1
    };
    uint32_t page_error = 0;
    st = HAL_FLASHEx_Erase(&erase_cfg, &page_error);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    /* 逐 double-word (8 字节) 编程 */
    const uint64_t *src = (const uint64_t *)(const void *)&buf;
    uint32_t addr = FLASH_PARAMS_ADDR;
    uint32_t dwords = (sizeof(FlashCalibData_t) + 7U) / 8U;

    for (uint32_t i = 0; i < dwords; i++) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]);
        if (st != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 8U;
    }

    HAL_FLASH_Lock();

    /* 回读验证 */
    FlashCalibData_t verify;
    return FlashParams_Read(&verify);
}

bool FlashParams_Erase(void)
{
    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) return false;

    FLASH_EraseInitTypeDef erase_cfg = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = FLASH_BANK_1,
        .Page      = FLASH_PARAMS_PAGE,
        .NbPages   = 1
    };
    uint32_t page_error = 0;
    st = HAL_FLASHEx_Erase(&erase_cfg, &page_error);

    HAL_FLASH_Lock();
    return (st == HAL_OK);
}
