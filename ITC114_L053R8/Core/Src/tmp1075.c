#include "tmp1075.h"


static int16_t decode_temp12(uint16_t reg_be)
{
    int16_t v = (int16_t)(reg_be >> 4);
    if (v & 0x0800) v |= 0xF000;
    return v;
}

static uint16_t encode_temp12(float celsius)
{
    float steps = celsius / 0.0625f;
    int32_t raw = (int32_t)(steps >= 0 ? (steps + 0.5f) : (steps - 0.5f));

    if (raw >  2047) raw =  2047;
    if (raw < -2048) raw = -2048;

    uint16_t u = (uint16_t)(raw & 0x0FFF);
    return (uint16_t)(u << 4);
}

HAL_StatusTypeDef TMP1075_Init(TMP1075_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    if (!dev || !hi2c) return HAL_ERROR;
    dev->hi2c = hi2c;
    dev->addr_7bit = addr_7bit;
    dev->timeout = 100;

    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(dev->hi2c,
                                            (uint16_t)(dev->addr_7bit << 1),
                                            TMP1075_REG_CONFIG,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf, 2, dev->timeout);
    return st;
}

HAL_StatusTypeDef TMP1075_ReadRaw12(TMP1075_Handle_t *dev, int16_t *raw12)
{
    if (!dev || !raw12) return HAL_ERROR;

    uint8_t rx[2] = {0};
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(dev->hi2c,
                                            (uint16_t)(dev->addr_7bit << 1),
                                            TMP1075_REG_TEMP,
                                            I2C_MEMADD_SIZE_8BIT,
                                            rx, 2, dev->timeout);
    if (st != HAL_OK) return st;

    uint16_t reg = ((uint16_t)rx[0] << 8) | rx[1];
    *raw12 = decode_temp12(reg);
    return HAL_OK;
}

HAL_StatusTypeDef TMP1075_ReadTemperatureC(TMP1075_Handle_t *dev, float *temp_c)
{
    if (!dev || !temp_c) return HAL_ERROR;
    int16_t r12 = 0;
    HAL_StatusTypeDef st = TMP1075_ReadRaw12(dev, &r12);
    if (st != HAL_OK) return st;
    *temp_c = TMP1075_Raw12_to_C(r12);
    return HAL_OK;
}

HAL_StatusTypeDef TMP1075_SetThresholdsC(TMP1075_Handle_t *dev, float t_low_c, float t_high_c)
{
    if (!dev) return HAL_ERROR;

    uint16_t tl = encode_temp12(t_low_c);
    uint16_t th = encode_temp12(t_high_c);

    uint8_t bufL[2] = { (uint8_t)(tl >> 8), (uint8_t)(tl & 0xFF) };
    uint8_t bufH[2] = { (uint8_t)(th >> 8), (uint8_t)(th & 0xFF) };

    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->addr_7bit << 1),
                           TMP1075_REG_TLOW, I2C_MEMADD_SIZE_8BIT,
                           bufL, 2, dev->timeout);
    if (st != HAL_OK) return st;

    st = HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->addr_7bit << 1),
                           TMP1075_REG_THIGH, I2C_MEMADD_SIZE_8BIT,
                           bufH, 2, dev->timeout);
    return st;
}

HAL_StatusTypeDef TMP1075_ReadConfig(TMP1075_Handle_t *dev, uint16_t *cfg)
{
    if (!dev || !cfg) return HAL_ERROR;
    uint8_t rx[2] = {0};
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(dev->hi2c,
                                            (uint16_t)(dev->addr_7bit << 1),
                                            TMP1075_REG_CONFIG,
                                            I2C_MEMADD_SIZE_8BIT,
                                            rx, 2, dev->timeout);
    if (st != HAL_OK) return st;
    *cfg = ((uint16_t)rx[0] << 8) | rx[1];
    return HAL_OK;
}

HAL_StatusTypeDef TMP1075_WriteConfig(TMP1075_Handle_t *dev, uint16_t cfg)
{
    if (!dev) return HAL_ERROR;
    uint8_t tx[2] = { (uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF) };
    return HAL_I2C_Mem_Write(dev->hi2c,
                             (uint16_t)(dev->addr_7bit << 1),
                             TMP1075_REG_CONFIG,
                             I2C_MEMADD_SIZE_8BIT,
                             tx, 2, dev->timeout);
}
