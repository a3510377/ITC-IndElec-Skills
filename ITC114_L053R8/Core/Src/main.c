/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "tmp1075.h"
#include "tps55289.h"
#include "ina226.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SW1_Read HAL_GPIO_ReadPin(SW1_IN_GPIO_Port, SW1_IN_Pin)
#define SW2_Read HAL_GPIO_ReadPin(SW2_IN_GPIO_Port, SW2_IN_Pin)
#define EN1_SW_Read HAL_GPIO_ReadPin(EN1_SW_GPIO_Port, EN1_SW_Pin)

#define EN1_CLA HAL_GPIO_ReadPin(EN1_CLA_GPIO_Port, EN1_CLA_Pin)
#define EN1_CLB HAL_GPIO_ReadPin(EN1_CLB_GPIO_Port, EN1_CLB_Pin)

#define LED1(x) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, x ? 1 : 0)
#define LED2(x) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, x ? 1 : 0)
#define LED3(x) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, x ? 1 : 0)

#define EEPROM_ADDR_BASE 0x08080000
#define EEPROM_ADDR_BZ_state (EEPROM_ADDR_BASE)           // size = 4 byte
#define EEPROM_ADDR_BZ_freq (EEPROM_ADDR_BASE + 4)        // size = 4 byte
#define EEPROM_ADDR_OLED_contrast (EEPROM_ADDR_BASE + 8)  // size = 4 byte
#define EEPROM_ADDR_Vset (EEPROM_ADDR_BASE + 16)          // size = 4 byte
#define EEPROM_ADDR_Ilim (EEPROM_ADDR_BASE + 32)          // size = 4 byte

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
int             mode;
char           *SW_STATE[] = {"ON", "OFF"};
int             SW1, SW2, SW3;
int             cnt, lastCLA;
int             timerSW1, task;
int             freq;
int             menu, lastEN1 = 1;
int             buzz = 1, light = 100;
int             lastSW1 = 1, lastSW2 = 1;
int             timerbuzz;
int             timer;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

TMP1075_Handle_t TMP1075_U7;

TPS55289_Handle_t TPS55289_U5;
TPS55289_Status_t TPS55289_output_state;
uint8_t           output_state = 0;

uint32_t output_start_time = 0;

INA226_Handle_t INA226_U3;
INA226_Handle_t INA226_U8;

float Input_voltage = 0.0;
float Input_current = 0.0;

float Output_voltage = 0.0;
float Output_current = 0.0;

float set_output_voltage       = 22.0;
float set_output_limit_current = 3.0;

float temperature = 0.0;

char OLED_buf[22];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void     EEPROM_WriteUInt32(uint32_t addr, uint32_t data);
uint32_t EEPROM_ReadUInt32(uint32_t addr);
void     EEPROM_WriteFloat(uint32_t addr, float value);
float    EEPROM_ReadFloat(uint32_t addr);
void     EEPROM_Check_Upload(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_TIM21_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* -------------------------------- Initial the OLED --------------------------------*/
  ssd1306_Init(&hi2c2);

  /* -------------------------------- Initial the TMP1075 --------------------------------*/
  TMP1075_Init(&TMP1075_U7, &hi2c1, TMP1075_DEFAULT_ADDR);

  /* -------------------------------- Initial the Read Input INA226 U3 --------------------------------*/
  INA226_Init(&INA226_U3, &hi2c1, 0x40);
  INA226_WriteConfig(&INA226_U3, INA226_AVG_16, INA226_CT_1100US, INA226_CT_1100US, true);
  INA226_SetCalibration(&INA226_U3, 0.010f, 0.00025f);

  /* -------------------------------- Initial the Read Output INA226 U8 --------------------------------*/
  INA226_Init(&INA226_U8, &hi2c1, 0x45);
  INA226_WriteConfig(&INA226_U8, INA226_AVG_16, INA226_CT_1100US, INA226_CT_1100US, true);
  INA226_SetCalibration(&INA226_U8, 0.010f, 0.00025f);

  /* -------------------------------- Initial the TPS55289 --------------------------------*/
  TPS55289_Init(&TPS55289_U5, &hi2c1, TPS55289_ADDR_75);

  /* -------------------------------- Initial the Base Timer, Period = 1ms --------------------------------*/
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
  /* -------------------------------- Set the TPS55289 Output  --------------------------------*/
  TPS55289_SetVout(&TPS55289_U5, 0.0f);    // Set TPS55289 Output Voltage = 0.0 V
  TPS55289_SetIlimit(&TPS55289_U5, 0.1f);  // Set TPS55289 Current Limit  = 0.1 A
  TPS55289_EnableOut(&TPS55289_U5, OFF);   // Set TPS55289 Output = OFF
  output_state = 0;

  /* -------------------------------- OLED display example --------------------------------*/
  //	  ssd1306_SetCursor(0, 0);
  //	  ssd1306_WriteString("Welcome to ITC 114", Font_7x10, White);
  //
  //	  ssd1306_SetCursor(0, 10);
  //	  ssd1306_WriteString("This font is 7x10", Font_7x10, White);
  //
  //	  ssd1306_SetCursor(0, 20);
  //	  ssd1306_WriteString("This font", Font_11x18, White);
  //
  //	  ssd1306_SetCursor(0, 38);
  //	  ssd1306_WriteString("is 11x18", Font_11x18, White);
  //
  //	  ssd1306_UpdateScreen(&hi2c2);
  sTime.Hours   = 23;
  sTime.Minutes = 59;
  sTime.Seconds = 50;
  //    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  //    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  cnt = 50;
  if (SW1_Read == 0) {
    mode = 0;
  } else mode = 1;

  buzz = (int)EEPROM_ReadUInt32(EEPROM_ADDR_BZ_state);
  freq = (int)EEPROM_ReadUInt32(EEPROM_ADDR_BZ_freq);

  light = (int)EEPROM_ReadUInt32(EEPROM_ADDR_OLED_contrast);

  set_output_voltage       = EEPROM_ReadFloat(EEPROM_ADDR_Vset);
  set_output_limit_current = EEPROM_ReadFloat(EEPROM_ADDR_Ilim);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* -------------------------------- Read TPS55289 status Register --------------------------------*/
    TPS55289_ReadStatus(&TPS55289_U5, &TPS55289_output_state);

    /* -------------------------------- Read TMP1075 temperature --------------------------------*/
    TMP1075_ReadTemperatureC(&TMP1075_U7, &temperature);  // temperature's unit = 'c

    /* -------------------------------- Read INA226 Input voltage and current --------------------------------*/
    INA226_ReadBusVoltage_V(&INA226_U3, &Input_voltage);  // Read Input Voltage, Unit = V
    INA226_ReadCurrent_A(&INA226_U3, &Input_current);     // Read Input Current, Unit = A

    INA226_ReadBusVoltage_V(&INA226_U8, &Output_voltage);
    INA226_ReadCurrent_A(&INA226_U8, &Output_current);

    HAL_Delay(1);

    if (timer == 0) {
      timer = 1000;
      sTime.Seconds++;
    }
    if (sTime.Seconds >= 60) {
      sTime.Seconds = 0;
      sTime.Minutes++;
    }
    if (sTime.Minutes >= 60) {
      sTime.Minutes = 0;
      sTime.Hours++;
    }
    if (sTime.Hours >= 24) {
      sTime.Hours = 0;
    }
    if (mode == 0) {
      if (SW1_Read == 0) {
        LED1(0);
        SW1 = 0;
      } else {
        LED1(1);
        SW1 = 1;
      }
      if (EN1_SW_Read == 0) {
        LED2(0);
        SW2 = 0;
      } else {
        LED2(1);
        SW2 = 1;
      }
      if (SW2_Read == 0) {
        LED3(0);
        SW3 = 0;
      } else {
        LED3(1);
        SW3 = 1;
      }
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("ITC114 Electronics", Font_7x10, White);
      ssd1306_SetCursor(0, 10);
      ssd1306_WriteString("Competitor:30", Font_7x10, White);
      ssd1306_SetCursor(0, 20);
      sprintf(OLED_buf, "SW1:%3s EN1_SW:%-3s", SW_STATE[SW1], SW_STATE[SW2]);
      ssd1306_WriteString(OLED_buf, Font_7x10, White);
      ssd1306_SetCursor(0, 30);
      sprintf(OLED_buf, "SW2:%s", SW_STATE[SW3]);
      ssd1306_WriteString(OLED_buf, Font_7x10, White);
      ssd1306_SetCursor(0, 40);
      sprintf(OLED_buf, "Rotary:%d", cnt);
      ssd1306_WriteString(OLED_buf, Font_7x10, White);
      ssd1306_SetCursor(0, 50);
      sprintf(OLED_buf, "SystemRTC:%2d:%2d:%2d", sTime.Hours, sTime.Minutes, sTime.Seconds);
      ssd1306_WriteString(OLED_buf, Font_7x10, White);
      ssd1306_UpdateScreen(&hi2c2);

    } else if (mode == 1) {
      LED1(0);
      LED2(!TPS55289_output_state.ocp);
      LED3(!output_state);

      if (task == 1) {
        if (EN1_SW_Read == 1 && lastEN1 == 0) {
          menu = (menu + 1) % 3;

          if (menu == 3) {
          }
        }
        lastEN1 = EN1_SW_Read;

        ssd1306_Fill(Black);
        ssd1306_SetCursor(2, 0);
        ssd1306_WriteString("System Settings:", Font_7x10, White);
        ssd1306_SetCursor(2, 10);
        ssd1306_WriteString("Buzz:", Font_11x18, White);
        ssd1306_SetCursor(57, 10);
        sprintf(OLED_buf, "%s", SW_STATE[!buzz]);
        ssd1306_WriteString(OLED_buf, Font_11x18, menu == 0 ? Black : White);
        ssd1306_SetCursor(2, 28);
        ssd1306_WriteString("Bz_F:", Font_11x18, White);
        ssd1306_SetCursor(57, 28);
        sprintf(OLED_buf, "%d", freq);
        ssd1306_WriteString(OLED_buf, Font_11x18, menu == 1 ? Black : White);
        ssd1306_SetCursor(90, 28);
        ssd1306_WriteString("kHz", Font_11x18, White);
        ssd1306_SetCursor(2, 46);
        ssd1306_WriteString("OLED", Font_11x18, White);
        ssd1306_SetCursor(57, 46);
        sprintf(OLED_buf, "%d", light);
        ssd1306_WriteString(OLED_buf, Font_11x18, menu == 2 ? Black : White);
        ssd1306_SetCursor(90, 46);
        ssd1306_WriteString("%", Font_11x18, White);
        ssd1306_UpdateScreen(&hi2c2);

      } else if (task == 0) {
        if (EN1_SW_Read == 1 && lastEN1 == 0) {
          menu = (menu + 1) % 3;
        }
        lastEN1 = EN1_SW_Read;

        uint8_t sw2 = SW2_Read == 1 && lastSW2 == 0;
        if ((SW1_Read == 1 && lastSW1 == 0) || (EN1_SW_Read == 1 && lastEN1 == 0) || sw2) {
          if (buzz && !timerbuzz) {
            timerbuzz = 50;
          }

          if (sw2) {
            output_state = !output_state;
            if (output_state) {
              TPS55289_SetVout(&TPS55289_U5, set_output_voltage);
              TPS55289_SetIlimit(&TPS55289_U5, set_output_limit_current);
              output_start_time = HAL_GetTick();
            }
            TPS55289_EnableOut(&TPS55289_U5, output_state);
          }
        }
        lastSW1 = SW1_Read;
        lastEN1 = EN1_SW_Read;
        lastSW2 = SW2_Read;

        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        sprintf(OLED_buf, "%05.2fV %04.2fA %04.1f", Input_voltage, Input_current, set_output_voltage);
        ssd1306_WriteString(OLED_buf, Font_7x10, White);

        ssd1306_SetCursor(0, 10);
        ssd1306_WriteString("Vset:", Font_11x18, White);
        sprintf(OLED_buf, "%05.2f", output_state ? Output_voltage : set_output_voltage);
        ssd1306_SetCursor(55, 10);
        ssd1306_WriteString(OLED_buf, Font_11x18, menu == 0 ? Black : White);
        ssd1306_SetCursor(110, 10);
        ssd1306_WriteString("V", Font_11x18, White);

        ssd1306_SetCursor(0, 28);
        ssd1306_WriteString("Ilim:", Font_11x18, White);
        sprintf(OLED_buf, "%05.2f", output_state ? Output_current : set_output_limit_current);
        ssd1306_SetCursor(55, 28);
        ssd1306_WriteString(OLED_buf, Font_11x18, menu == 1 ? Black : White);
        ssd1306_SetCursor(110, 28);
        ssd1306_WriteString("A", Font_11x18, White);

        ssd1306_SetCursor(0, 46);
        ssd1306_WriteString("Output:", Font_7x10, White);
        sprintf(OLED_buf, "%3s", SW_STATE[!output_state]);
        ssd1306_SetCursor(55, 46);
        ssd1306_WriteString(OLED_buf, Font_7x10, menu == 2 ? Black : White);

        if (output_state) {
          uint32_t now  = HAL_GetTick();
          uint32_t diff = (now - output_start_time) / 1000;

          ssd1306_SetCursor(85, 46);
          sprintf(OLED_buf, "%03lu:%02lu", diff / 60, diff % 60);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);
        }

        ssd1306_UpdateScreen(&hi2c2);
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (timer > 0) timer--;
  if (htim == &htim6)  // 1 ms Timer
  {
    if (timerbuzz > 0) {
      timerbuzz--;

      uint32_t arr = freq * 1000;
      TIM21->ARR   = arr;
      TIM21->CCR1  = arr / 2;
    } else {
      TIM21->CCR1 = 0;
    }

    if (EN1_CLA == 1 && lastCLA == 0) {
      if (EN1_CLB == 1) {
        if (mode == 1 || mode == 0) {
          if (mode == 1 && task == 0) {
            if (!output_state) {  // is not output state
              if (menu == 0) {
                if (EN1_SW_Read == 0) {  // down
                  set_output_voltage -= 0.01;
                  lastEN1 = 1;
                } else {
                  set_output_voltage -= 0.1;
                }
              } else if (menu == 1) {
                if (EN1_SW_Read == 0) {  // down
                  set_output_limit_current -= 0.05;
                  lastEN1 = 1;
                } else {
                  set_output_limit_current -= 0.1;
                }
              } else if (menu == 2) {
              }
            }
          } else {
            cnt++;
            if (menu == 0) buzz = 1;
            else if (menu == 1) freq--;
            else if (menu == 2) light -= 5;
          }
        }
      } else {
        if (mode == 1 || mode == 0) {
          if (mode == 1 && task == 0) {
            if (!output_state) {  // is not output state
              if (menu == 0) {
                if (EN1_SW_Read == 0) {  // down
                  set_output_voltage -= 0.01;
                  lastEN1 = 1;
                } else {
                  set_output_voltage -= 0.1;
                }
              } else if (menu == 1) {
                if (EN1_SW_Read == 0) {  // down
                  set_output_limit_current -= 0.05;
                  lastEN1 = 1;
                } else {
                  set_output_limit_current -= 0.1;
                }
              }
            }
          } else {
            cnt--;
            if (menu == 0) buzz = 0;
            else if (menu == 1) freq++;
            else if (menu == 2) light += 5;
          }
        }
      }

      // :> 我懶 >>
      EEPROM_Check_Upload();
    }
    lastCLA = EN1_CLA;

    if (SW1_Read == 0) {
      timerSW1--;
      if (timerSW1 == 0) {
        task      = (task + 1) % 2;
        timerbuzz = 300;
      }
    } else timerSW1 = 1500;
  }
}

void EEPROM_WriteUInt32(uint32_t addr, uint32_t data) {
  HAL_FLASH_Unlock();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data);
  HAL_FLASH_Lock();
}

uint32_t EEPROM_ReadUInt32(uint32_t addr) {
  return *(__IO uint32_t *)addr;
}

void EEPROM_WriteFloat(uint32_t addr, float value) {
  uint32_t data = *(uint32_t *)&value;
  EEPROM_WriteUInt32(addr, data);
}

float EEPROM_ReadFloat(uint32_t addr) {
  uint32_t data  = EEPROM_ReadUInt32(addr);
  float    value = *(float *)&data;
  return value;
}

void EEPROM_Check_Upload(void) {
  if (cnt > 100) cnt = 100;
  else if (cnt < 0) cnt = 0;

  if (freq > 10) freq = 10;
  else if (freq < 1) freq = 1;

  if (light > 100) light = 100;
  else if (light < 0) light = 0;

  if (set_output_limit_current > 3) set_output_limit_current = 3;
  else if (set_output_limit_current < 0) set_output_limit_current = 0;

  if (set_output_voltage > 21) set_output_voltage = 21;
  else if (set_output_voltage < 0) set_output_voltage = 0;

  EEPROM_WriteUInt32(EEPROM_ADDR_BZ_state, buzz);
  EEPROM_WriteUInt32(EEPROM_ADDR_BZ_freq, freq);
  EEPROM_WriteUInt32(EEPROM_ADDR_OLED_contrast, light);

  EEPROM_WriteFloat(EEPROM_ADDR_Vset, set_output_voltage);
  EEPROM_WriteFloat(EEPROM_ADDR_Ilim, set_output_limit_current);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
