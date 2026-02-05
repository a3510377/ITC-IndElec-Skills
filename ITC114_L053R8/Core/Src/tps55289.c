#include "tps55289.h"
#include <math.h>

static inline uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi){ return (x<lo)?lo:((x>hi)?hi:x); }
static inline uint8_t  dev7_to_hal(uint8_t a7){ return (uint8_t)(a7 << 1); }

HAL_StatusTypeDef TPS55289_Read8(TPS55289_Handle_t *h, uint8_t reg, uint8_t *val){
    return HAL_I2C_Mem_Read(h->hi2c, dev7_to_hal(h->addr_7bit), reg, I2C_MEMADD_SIZE_8BIT, val, 1, h->timeout_ms);
}
HAL_StatusTypeDef TPS55289_Write8(TPS55289_Handle_t *h, uint8_t reg, uint8_t val){
    return HAL_I2C_Mem_Write(h->hi2c, dev7_to_hal(h->addr_7bit), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, h->timeout_ms);
}

HAL_StatusTypeDef TPS55289_Init(TPS55289_Handle_t *device, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    if(!device || !hi2c) return HAL_ERROR;
    device->hi2c = hi2c;
    device->addr_7bit = addr_7bit;
    device->timeout_ms = 100;

    uint8_t v;
    TPS55289_Read8(device, TPS55289_REG_VOUT_SR, &v);
    v = (uint8_t)((v & ~0x30) | (0x00u << 4));
    v = (uint8_t)((v & ~0x03) | 0x03u);
    TPS55289_Write8(device, TPS55289_REG_VOUT_SR, v);

    uint8_t dummy = 0;

    return TPS55289_Read8(device, TPS55289_REG_STATUS, &dummy);
}


HAL_StatusTypeDef TPS55289_SetIntFB(TPS55289_Handle_t *h, uint8_t intfb_sel, bool use_internal_fb)
{
    uint8_t v = 0;
    HAL_StatusTypeDef st = TPS55289_Read8(h, TPS55289_REG_VOUT_FS, &v);
    if(st != HAL_OK) return st;

    if(use_internal_fb) v &= ~(1u << TPS55289_VFS_FB_Pos);
    else                v |=  (1u << TPS55289_VFS_FB_Pos);

    v = (uint8_t)((v & ~0x03u) | (intfb_sel & 0x03u));
    return TPS55289_Write8(h, TPS55289_REG_VOUT_FS, v);
}


static uint16_t tps55289_vref_code_from_volt(float vref_v)
{
    if (vref_v < TPS55289_VREF_MIN_V) vref_v = TPS55289_VREF_MIN_V;
    if (vref_v > TPS55289_VREF_MAX_V) vref_v = TPS55289_VREF_MAX_V;
    float code_f = (vref_v - TPS55289_VREF_MIN_V) / TPS55289_VREF_LSB_V;
    int32_t code  = (int32_t)lrintf(code_f);
    if(code < 0) code = 0;
    if(code > ((1<<TPS55289_VREF_BITS)-1)) code = (1<<TPS55289_VREF_BITS)-1;
    return (uint16_t)code;
}


HAL_StatusTypeDef TPS55289_SetVout(TPS55289_Handle_t *device, float output_voltage)
{

    HAL_StatusTypeDef st = TPS55289_SetIntFB(device, TPS55289_INTFB_3, true);
    if(st != HAL_OK) return st;


    float vref = output_voltage * TPS55289_INTFB_RATIO_3;
    uint16_t code = tps55289_vref_code_from_volt(vref);


    uint8_t lsb = (uint8_t)(code & 0xFFu);
    uint8_t msb = (uint8_t)((code >> 8) & 0x07u);

    st = TPS55289_Write8(device, TPS55289_REG_REF_LSB, lsb);
    if(st != HAL_OK) return st;
    st = TPS55289_Write8(device, TPS55289_REG_REF_MSB, msb);

    return st;
}


HAL_StatusTypeDef TPS55289_SetIlimit(TPS55289_Handle_t *device, float current_limit)
{
    float vsense_mV = current_limit * 10.0;

    if (vsense_mV > 0.0f) vsense_mV += 0.5f;
    if (vsense_mV < 0.0f) vsense_mV = 0.0f;
    if (vsense_mV > 63.5f) vsense_mV = 63.5f;

    uint8_t code = (uint8_t)lrintf(vsense_mV / 0.5f);
    uint8_t reg  = (uint8_t)(1u << TPS55289_ILIM_EN_Pos) | (code & 0x7Fu);
    return TPS55289_Write8(device, TPS55289_REG_IOUT_LIMIT, reg);
}

HAL_StatusTypeDef TPS55289_EnableOut(TPS55289_Handle_t *device, bool Output_Enable)
{
    uint8_t mode = 0;
    HAL_StatusTypeDef st = TPS55289_Read8(device, TPS55289_REG_MODE, &mode);
    if(st != HAL_OK) return st;

    if(Output_Enable)
    	mode |=  (1u << TPS55289_MODE_OE_Pos);
    else
    	mode &= ~(1u << TPS55289_MODE_OE_Pos);

    mode &= ~(1u << TPS55289_MODE_FPWM_Pos);

    mode |=  (1u << TPS55289_MODE_HICCUP_Pos);

    mode &= ~(1u << TPS55289_MODE_DISCHG_Pos);

    return TPS55289_Write8(device, TPS55289_REG_MODE, mode);
}

HAL_StatusTypeDef TPS55289_ReadStatus(TPS55289_Handle_t *device, TPS55289_Status_t *state)
{
    if(!state) return HAL_ERROR;
    uint8_t v=0;
    HAL_StatusTypeDef st = TPS55289_Read8(device, TPS55289_REG_STATUS, &v);
    if(st != HAL_OK) return st;

    state->scp = (v >> TPS55289_STATUS_SCP_Pos) & 0x1;
    state->ocp = (v >> TPS55289_STATUS_OCP_Pos) & 0x1;
    state->ovp = (v >> TPS55289_STATUS_OVP_Pos) & 0x1;
    state->op_mode = (v >> TPS55289_STATUS_MODE_Pos) & 0x3;
    return HAL_OK;
}
