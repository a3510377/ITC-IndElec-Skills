#ifndef __TMP1075_H__
#define __TMP1075_H__

#include "stm32l0xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==== TMP1075 Register Map (8-bit addresses) ==== */
#define TMP1075_REG_TEMP       0x00  /* Temperature register (12-bit two's complement, MSB first) */
#define TMP1075_REG_CONFIG     0x01  /* Configuration register (optional, keep default if unsure) */
#define TMP1075_REG_TLOW       0x02  /* Low temperature threshold (12-bit format) */
#define TMP1075_REG_THIGH      0x03  /* High temperature threshold (12-bit format) */

/* 建議：先用預設設定即可量測溫度；如需進階警報/省電，再寫 CONFIG */
#define TMP1075_DEFAULT_ADDR   0x48  /* 7-bit I2C address; change per your ADDR pins */

/* Driver handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr_7bit;  /* 7-bit address (e.g., 0x48~0x4B) */
    uint32_t timeout;   /* I2C timeout (ms) */
} TMP1075_Handle_t;

/* API */
HAL_StatusTypeDef TMP1075_Init(TMP1075_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
HAL_StatusTypeDef TMP1075_ReadTemperatureC(TMP1075_Handle_t *dev, float *temp_c);
HAL_StatusTypeDef TMP1075_ReadRaw12(TMP1075_Handle_t *dev, int16_t *raw12);
HAL_StatusTypeDef TMP1075_SetThresholdsC(TMP1075_Handle_t *dev, float t_low_c, float t_high_c);

/* Optional config R/W (進階用途；不確定設定時可不使用) */
HAL_StatusTypeDef TMP1075_ReadConfig(TMP1075_Handle_t *dev, uint16_t *cfg);
HAL_StatusTypeDef TMP1075_WriteConfig(TMP1075_Handle_t *dev, uint16_t cfg);

/* Helpers */
static inline float TMP1075_Raw12_to_C(int16_t raw12) {
    /* raw12 is signed 12-bit, LSB = 0.0625°C */
    return (float)raw12 * 0.0625f;
}

#ifdef __cplusplus
}
#endif

#endif /* __TMP1075_H__ */
