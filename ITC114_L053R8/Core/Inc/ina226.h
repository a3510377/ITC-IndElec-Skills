#ifndef __INA226_H__
#define __INA226_H__

#include "stm32l0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== I2C 7-bit 位址（A1/A0 決定，常見預設 0x40；共 0x40~0x4F） ===== */
#define INA226_ADDR_MIN   0x40u
#define INA226_ADDR_MAX   0x4Fu

/* ===== 暫存器位址（16-bit，大端序 MSB first） ===== */
#define INA226_REG_CONFIG       0x00u
#define INA226_REG_SHUNT_V      0x01u  /* LSB = 2.5 µV (signed) */
#define INA226_REG_BUS_V        0x02u  /* LSB = 1.25 mV (unsigned, MSB=0) */
#define INA226_REG_POWER        0x03u  /* LSB = 25 * Current_LSB */
#define INA226_REG_CURRENT      0x04u  /* LSB = Current_LSB */
#define INA226_REG_CALIB        0x05u
#define INA226_REG_MASK_ENABLE  0x06u
#define INA226_REG_ALERT_LIMIT  0x07u
#define INA226_REG_MFG_ID       0xFEu  /* 0x5449 (TI) */
#define INA226_REG_DIE_ID       0xFFu  /* 0x2660 (含修訂號於[3:0]) */

/* ===== CONFIG 預設值（連續量測 Shunt+Bus、tCT=1.1ms、AVG=1） ===== */
#define INA226_CONFIG_DEFAULT   0x4127u

/* ===== 轉換時間/平均（資料表定義）方便巨集 =====
 * tCT 級距（Shunt/Bus）：140µs, 204µs, 332µs, 588µs, 1.1ms, 2.116ms, 4.156ms, 8.244ms
 */
typedef enum {
    INA226_CT_140US = 0,
    INA226_CT_204US,
    INA226_CT_332US,
    INA226_CT_588US,
    INA226_CT_1100US,
    INA226_CT_2116US,
    INA226_CT_4156US,
    INA226_CT_8244US
} INA226_ConvTime_t;

typedef enum {
    INA226_AVG_1 = 0,
    INA226_AVG_4,
    INA226_AVG_16,
    INA226_AVG_64,
    INA226_AVG_128,
    INA226_AVG_256,
    INA226_AVG_512,
    INA226_AVG_1024
} INA226_Avg_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t  addr_7bit;         /* 0x40..0x4F */
    uint32_t timeout_ms;        /* HAL I2C timeout */
    float    r_shunt_ohm;       /* 你的分流電阻值 (Ω) */
    float    current_lsb_a;     /* A/bit，決定解析度（由 max current 推導或自行指定） */
    uint16_t calib_reg;         /* 寫入的校正暫存器值（記錄用） */
} INA226_Handle_t;

/* ===== API ===== */
HAL_StatusTypeDef INA226_Init(INA226_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);

HAL_StatusTypeDef INA226_WriteConfig(INA226_Handle_t *dev, INA226_Avg_t avg,
                                     INA226_ConvTime_t ct_shunt, INA226_ConvTime_t ct_bus,
                                     bool continuous_shunt_bus);

HAL_StatusTypeDef INA226_SetCalibration(INA226_Handle_t *dev, float r_shunt_ohm, float current_lsb_a);
/* 便利：依「最大預期電流」計算 Current_LSB = Imax / 32768 */
HAL_StatusTypeDef INA226_SetCalibrationByImax(INA226_Handle_t *dev, float r_shunt_ohm, float i_max_a);

HAL_StatusTypeDef INA226_ReadBusVoltage_mV  (INA226_Handle_t *dev, uint16_t *mV);
HAL_StatusTypeDef INA226_ReadShuntVoltage_uV(INA226_Handle_t *dev, int32_t *uV);
HAL_StatusTypeDef INA226_ReadCurrent_A     (INA226_Handle_t *dev, float *A);
HAL_StatusTypeDef INA226_ReadPower_mW       (INA226_Handle_t *dev, float *mW);
HAL_StatusTypeDef INA226_ReadBusVoltage_V  (INA226_Handle_t *dev, float *V);

/* 身分確認（可選） */
HAL_StatusTypeDef INA226_ReadIDs(INA226_Handle_t *dev, uint16_t *mfg_id, uint16_t *die_id);

/* 低階存取 */
HAL_StatusTypeDef INA226_ReadReg (INA226_Handle_t *dev, uint8_t reg, uint16_t *val_be);
HAL_StatusTypeDef INA226_WriteReg(INA226_Handle_t *dev, uint8_t reg, uint16_t val_be);

#ifdef __cplusplus
}
#endif
#endif /* __INA226_H__ */
