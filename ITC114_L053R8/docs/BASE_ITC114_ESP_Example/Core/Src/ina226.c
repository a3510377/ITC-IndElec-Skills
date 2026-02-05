#include "ina226.h"
#include <math.h>

static inline uint16_t clamp16(uint16_t x, uint16_t lo, uint16_t hi){ return (x<lo)?lo:((x>hi)?hi:x); }
static inline uint8_t  a7_to_hal(uint8_t a7){ return (uint8_t)(a7 << 1); }

HAL_StatusTypeDef INA226_ReadReg(INA226_Handle_t *dev, uint8_t reg, uint16_t *val_be)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(dev->hi2c, a7_to_hal(dev->addr_7bit),
                                            reg, I2C_MEMADD_SIZE_8BIT,
                                            buf, 2, dev->timeout_ms);
    if(st != HAL_OK) return st;
    *val_be = ((uint16_t)buf[0] << 8) | buf[1];
    return HAL_OK;
}

HAL_StatusTypeDef INA226_WriteReg(INA226_Handle_t *dev, uint8_t reg, uint16_t val_be)
{
    uint8_t buf[2] = { (uint8_t)(val_be >> 8), (uint8_t)(val_be & 0xFF) };
    return HAL_I2C_Mem_Write(dev->hi2c, a7_to_hal(dev->addr_7bit),
                             reg, I2C_MEMADD_SIZE_8BIT,
                             buf, 2, dev->timeout_ms);
}

HAL_StatusTypeDef INA226_Init(INA226_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    if(!dev || !hi2c) return HAL_ERROR;
    dev->hi2c = hi2c;
    dev->addr_7bit = addr7;
    dev->timeout_ms = 100;
    dev->r_shunt_ohm = 0.0f;
    dev->current_lsb_a = 0.0f;
    dev->calib_reg = 0;

    uint16_t cfg=0;
    return INA226_ReadReg(dev, INA226_REG_CONFIG, &cfg);
}

HAL_StatusTypeDef INA226_WriteConfig(INA226_Handle_t *dev, INA226_Avg_t avg,
                                     INA226_ConvTime_t ct_shunt, INA226_ConvTime_t ct_bus,
                                     bool continuous_shunt_bus)
{
    uint16_t cfg = 0;
    cfg |= ((uint16_t)(avg      & 0x7) << 9);
    cfg |= ((uint16_t)(ct_bus   & 0x7) << 6);
    cfg |= ((uint16_t)(ct_shunt & 0x7) << 3);
    cfg |= (continuous_shunt_bus ? 0x7 : 0x4);

    return INA226_WriteReg(dev, INA226_REG_CONFIG, cfg);
}

HAL_StatusTypeDef INA226_SetCalibration(INA226_Handle_t *dev, float r_shunt_ohm, float current_lsb_a)
{
    if (r_shunt_ohm <= 0.0f || current_lsb_a <= 0.0f) return HAL_ERROR;

    float cal_f = 0.00512f / (current_lsb_a * r_shunt_ohm);
    int32_t cal_i = (int32_t)floorf(cal_f + 0.5f);
    cal_i = (int32_t)clamp16((uint16_t)cal_i, 0, 0x7FFF);

    dev->r_shunt_ohm   = r_shunt_ohm;
    dev->current_lsb_a = current_lsb_a;
    dev->calib_reg     = (uint16_t)cal_i;

    return INA226_WriteReg(dev, INA226_REG_CALIB, dev->calib_reg);
}

HAL_StatusTypeDef INA226_SetCalibrationByImax(INA226_Handle_t *dev, float r_shunt_ohm, float i_max_a)
{
    if (i_max_a <= 0.0f) return HAL_ERROR;
    float current_lsb = i_max_a / 32768.0f;

    return INA226_SetCalibration(dev, r_shunt_ohm, current_lsb);
}


HAL_StatusTypeDef INA226_ReadBusVoltage_mV(INA226_Handle_t *dev, uint16_t *mV)
{
    if(!mV) return HAL_ERROR;
    uint16_t raw = 0;
    HAL_StatusTypeDef st = INA226_ReadReg(dev, INA226_REG_BUS_V, &raw);
    if(st != HAL_OK) return st;

    *mV = (uint16_t)((float)raw * 1.25f);
    return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadShuntVoltage_uV(INA226_Handle_t *dev, int32_t *uV)
{
    if(!uV) return HAL_ERROR;
    uint16_t raw = 0;
    HAL_StatusTypeDef st = INA226_ReadReg(dev, INA226_REG_SHUNT_V, &raw);
    if(st != HAL_OK) return st;
    int16_t s = (int16_t)raw;
    *uV = (int32_t)llroundf((float)s * 2.5f);
    return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadCurrent_A(INA226_Handle_t *dev, float *A)
{
    if(!A) return HAL_ERROR;
    uint16_t raw = 0;
    HAL_StatusTypeDef st = INA226_ReadReg(dev, INA226_REG_CURRENT, &raw);
    if(st != HAL_OK) return st;
    int16_t i = (int16_t)raw;
    *A = (((float)i * dev->current_lsb_a) >= 0) ? ((float)i * dev->current_lsb_a) : 0;
    return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadPower_mW(INA226_Handle_t *dev, float *mW)
{
    if(!mW) return HAL_ERROR;
    uint16_t raw = 0;
    HAL_StatusTypeDef st = INA226_ReadReg(dev, INA226_REG_POWER, &raw);
    if(st != HAL_OK) return st;
    float power_lsb_w = 25.0f * dev->current_lsb_a;
    *mW = (float)raw * power_lsb_w * 1000.0f;

    return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadBusVoltage_V  (INA226_Handle_t *dev, float *V)
{
	uint16_t raw = 0;
	INA226_ReadBusVoltage_mV(dev, &raw);
	*V = ((raw / 1000.0f) >= 0) ? (raw / 1000.0f) : 0;

	return HAL_OK;
}

HAL_StatusTypeDef INA226_ReadIDs(INA226_Handle_t *dev, uint16_t *mfg_id, uint16_t *die_id)
{
    HAL_StatusTypeDef st;
    if(mfg_id){
        st = INA226_ReadReg(dev, INA226_REG_MFG_ID, mfg_id);
        if(st != HAL_OK) return st;
    }
    if(die_id){
        st = INA226_ReadReg(dev, INA226_REG_DIE_ID, die_id);
        if(st != HAL_OK) return st;
    }
    return HAL_OK;
}
