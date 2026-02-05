#ifndef __TPS55289_H__
#define __TPS55289_H__

#include "stm32l0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TPS55289_ADDR_74   0x74u
#define TPS55289_ADDR_75   0x75u

/* ===== Register map ===== */
#define TPS55289_REG_REF_LSB    0x00u
#define TPS55289_REG_REF_MSB    0x01u
#define TPS55289_REG_IOUT_LIMIT 0x02u
#define TPS55289_REG_VOUT_SR    0x03u
#define TPS55289_REG_VOUT_FS    0x04u
#define TPS55289_REG_CDC        0x05u
#define TPS55289_REG_MODE       0x06u
#define TPS55289_REG_STATUS     0x07u

/* ===== MODE register bits ===== */
#define TPS55289_MODE_OE_Pos     7
#define TPS55289_MODE_FSWDBL_Pos 6
#define TPS55289_MODE_HICCUP_Pos 5
#define TPS55289_MODE_DISCHG_Pos 4
#define TPS55289_MODE_FPWM_Pos   1

/* ===== VOUT_FS register bits ===== */
#define TPS55289_VFS_FB_Pos      7
#define TPS55289_VFS_INTFB_Pos   0
#define TPS55289_INTFB_0         0x0u
#define TPS55289_INTFB_1         0x1u
#define TPS55289_INTFB_2         0x2u
#define TPS55289_INTFB_3         0x3u

/* ===== IOUT_LIMIT register bits ===== */
#define TPS55289_ILIM_EN_Pos     7      /* 1=enable current limit */
#define TPS55289_ILIM_VAL_Pos    0      /* [6:0], LSB=0.5mV across ISP-ISN */

/* ===== STATUS register bits ===== */
#define TPS55289_STATUS_SCP_Pos  7
#define TPS55289_STATUS_OCP_Pos  6
#define TPS55289_STATUS_OVP_Pos  5
#define TPS55289_STATUS_MODE_Pos 0      /* [1:0] 00=Boost,01=Buck,10=Buck-Boost */


#define TPS55289_VREF_MIN_V      0.045f     /* 45 mV */
#define TPS55289_VREF_MAX_V      1.200f     /* 1200 mV */
#define TPS55289_VREF_LSB_V      0.0005645f /* 0.5645 mV */
#define TPS55289_VREF_BITS       11


#define TPS55289_INTFB_RATIO_0   0.2256f
#define TPS55289_INTFB_RATIO_1   0.1128f
#define TPS55289_INTFB_RATIO_2   0.0752f
#define TPS55289_INTFB_RATIO_3   0.0564f

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr_7bit;
    uint32_t timeout_ms;
} TPS55289_Handle_t;

typedef struct {
    bool scp, ocp, ovp;
    uint8_t op_mode; /* 0=Boost,1=Buck,2=Buck-Boost */
} TPS55289_Status_t;


typedef enum {
    OFF,
	ON
} TPS55289_Output_Status;

/* ===== API ===== */
HAL_StatusTypeDef TPS55289_Init(TPS55289_Handle_t *device, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
HAL_StatusTypeDef TPS55289_EnableOut(TPS55289_Handle_t *device, bool Output_Enable);
HAL_StatusTypeDef TPS55289_SetIntFB  (TPS55289_Handle_t *h, uint8_t intfb_sel, bool use_internal_fb);
HAL_StatusTypeDef TPS55289_SetVout(TPS55289_Handle_t *device, float output_voltage);
HAL_StatusTypeDef TPS55289_SetIlimit(TPS55289_Handle_t *device, float current_limit);
HAL_StatusTypeDef TPS55289_ReadStatus(TPS55289_Handle_t *device, TPS55289_Status_t *state);

HAL_StatusTypeDef TPS55289_Read8 (TPS55289_Handle_t *h, uint8_t reg, uint8_t *val);
HAL_StatusTypeDef TPS55289_Write8(TPS55289_Handle_t *h, uint8_t reg, uint8_t  val);

#ifdef __cplusplus
}
#endif
#endif /* __TPS55289_H__ */
