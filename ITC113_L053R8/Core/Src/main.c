/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INA3221_ADDRESS 0x40 << 1

#define BASE_SW_PORT SW1_IN_GPIO_Port
#define BTN_DOWN(pin) (HAL_GPIO_ReadPin(pin##_IN_GPIO_Port, pin##_IN_Pin))
#define HAS_BTN_FLAG(pin) (btn_flag & (pin##_IN_Pin))
#define CHECK_BTN(pin) (BTN_DOWN(pin) && !HAS_BTN_FLAG(pin))
#define UPDATE_STATUS (btn_flag = BASE_SW_PORT->IDR)

#define LED_OFF (0x000000)
#define LED_RED (0x000f00)
#define LED_GREEN (0x0f0000)
#define LED_BLUE (0x00000f)
#define LED_WHITE (0x080808)
#define LED_YELLOW (0x080800)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
RTC_DateTypeDef sDate;
RTC_TimeTypeDef sTime;

int total_energy = 0;
uint8_t mode = 1, task = 0, old_task = 0, btn_flag = 0, select_alert = 0;
uint8_t use_alert = 1, use_led = 1;
uint16_t alert_time = 0, led_timer = 0;
uint8_t btn_time_down_flag = 0;
uint32_t btn_down_starttime = 0;

char OLED_buf[30];
int voltages[3] = {0}, currents[3] = {0}, powers[3] = {0};
uint8_t counts[3] = {0};
uint32_t led_colors_buff[3] = {0}, energys_acc[3] = {0}, Q_acc[3] = {0},
         energys[3] = {0}, Q[3] = {0};

uint32_t MODE0_LED_MAP[5] = {LED_OFF, LED_RED, LED_GREEN, LED_BLUE, LED_WHITE};
char *LED_NAME[5] = {"OFF", "RED", "GREEN", "BLUE", "WHITE"};
char *SWITCH_NAME[2] = {"OFF", "ON"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM6_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void ws2812b_display_all_led_colors(uint32_t *buff);
void ws2812b_set_colors(uint32_t grb);

void update_data(void);
void clear_leds(void);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_TIM21_Init();
  MX_TIM6_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  ssd1306_Init(&hi2c2);

  clear_leds();
  HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim6);

  if (BTN_DOWN(SW2)) mode = 0;
  UPDATE_STATUS;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    if (mode == 0) {
      if (CHECK_BTN(SW1)) counts[0] = (counts[0] + 1) % 5;
      if (CHECK_BTN(SW2)) counts[1] = (counts[1] + 1) % 5;
      if (CHECK_BTN(SW3)) counts[2] = (counts[2] + 1) % 5;
      UPDATE_STATUS;

      sprintf(OLED_buf, "ITC113_Electronics");
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString(OLED_buf, Font_7x10, White);

      sprintf(OLED_buf, "Competitor: 00");
      ssd1306_SetCursor(0, 10);
      ssd1306_WriteString(OLED_buf, Font_7x10, White);

      uint8_t btns_stats[] = {BTN_DOWN(SW1), BTN_DOWN(SW2), BTN_DOWN(SW3)};
      for (int i = 0; i < 3; i++) {
        led_colors_buff[i] = MODE0_LED_MAP[counts[i]];
        sprintf(OLED_buf, "SW%d:%3s LED%d:%-5s", i + 1,
                btns_stats[i] ? "ON" : "OFF", i + 1, LED_NAME[counts[i]]);
        ssd1306_SetCursor(0, 10 + 10 * (i + 1));
        ssd1306_WriteString(OLED_buf, Font_7x10, White);
      }

      sprintf(OLED_buf, "SystemRTC:%02d:%02d:%02d", sTime.Hours, sTime.Minutes,
              sTime.Seconds);
      ssd1306_SetCursor(0, 10 * 5);
      ssd1306_WriteString(OLED_buf, Font_7x10, White);
    } else {
      update_data();

      if (CHECK_BTN(SW1)) {
        alert_time = 50;
        if (task > 0) task--;
      } else if (CHECK_BTN(SW3)) {
        alert_time = 50;
        if (task < 4) task++;
      }
      if (old_task != task) {
        clear_leds();
        ssd1306_Fill(Black);
        old_task = task;
        btn_down_starttime = select_alert = 0;
      }
      if (CHECK_BTN(SW2)) {
        btn_time_down_flag = 1;
        btn_down_starttime = HAL_GetTick();
      } else if (btn_time_down_flag) {
        if (HAL_GetTick() - btn_down_starttime > 2000) {
          if (task == 0) {
            alert_time = 300;
            total_energy = 0;
          } else if (task == 1 || task == 2 || task == 3) {
            alert_time = 300;
            Q[task - 1] = Q_acc[task - 1] = 0;
            energys[task - 1] = energys_acc[task - 1] = 0;
          } else if (task == 4) {
            if (select_alert) {
              use_led = !use_led;
              clear_leds();
            } else use_alert = !use_alert;
          }
          btn_time_down_flag = 0;
        } else if (!BTN_DOWN(SW2) && task == 4) {
          select_alert = !select_alert;
          btn_time_down_flag = 0;
        }
      }
      UPDATE_STATUS;

      switch (task) {
        case 0: {
          if (use_led) {
            for (int i = 0; i < 3; i++) {
              int x1 = (i + 1) % 3;
              int x2 = (i + 2) % 3;

              if (powers[i] == 0) led_colors_buff[i] = LED_OFF;
              else if (powers[i] > powers[x1] && powers[i] > powers[x2]) {
                led_colors_buff[i] = LED_RED;  // highest
              } else if (powers[i] < powers[x1] && powers[i] < powers[x2]) {
                led_colors_buff[i] = LED_BLUE;         // 2nd highest
              } else led_colors_buff[i] = LED_YELLOW;  // lowest
            }
          }

          sprintf(OLED_buf, "Total Wattage:");
          ssd1306_SetCursor(0, 0);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);

          int total_power = powers[0];
          for (int i = 1; i < 3; i++) total_power += powers[i];
          total_power /= 10;  // only show 2 decimal
          sprintf(OLED_buf, "%3d.%02dW", total_power / 100,
                  total_power % 100);
          ssd1306_SetCursor(15, 10);
          ssd1306_WriteString(OLED_buf, Font_16x26, White);

          sprintf(OLED_buf, "Energy Used:");
          ssd1306_SetCursor(0, 34);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);

          sprintf(OLED_buf, "%4d.%04dWh", total_energy / 10000,
                  total_energy % 10000);
          ssd1306_SetCursor(6, 44);
          ssd1306_WriteString(OLED_buf, Font_11x18, White);
        } break;
        case 1:
        case 2:
        case 3: {
          if (use_led) led_colors_buff[task - 1] = LED_WHITE;

          sprintf(OLED_buf, "CH%d Information", task);
          ssd1306_SetCursor(0, 0);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);

          int voltage = voltages[task - 1];
          sprintf(OLED_buf, "Voltage:%2d.%03dV", voltage / 1000,
                  voltage % 1000);
          ssd1306_SetCursor(0, 10);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);

          int current = currents[task - 1];
          sprintf(OLED_buf, "Current:%2d.%03dA", current / 1000,
                  current % 1000);
          ssd1306_SetCursor(0, 20);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);

          int power = powers[task - 1];
          sprintf(OLED_buf, "Power:%4d.%03dW", power / 1000, power % 1000);
          ssd1306_SetCursor(0, 30);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);

          int energy = energys[task - 1];
          sprintf(OLED_buf, "Energy:%3d.%04dWh", energy / 10000,
                  energy % 10000);
          ssd1306_SetCursor(0, 40);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);

          int Q_ = Q[task - 1];
          sprintf(OLED_buf, "q:%8d.%03dmAh ", Q_ / 1000, Q_ % 1000);
          ssd1306_SetCursor(0, 50);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);
        } break;
        case 4: {
          sprintf(OLED_buf, "System settings");
          ssd1306_SetCursor(0, 0);
          ssd1306_WriteString(OLED_buf, Font_7x10, White);

          sprintf(OLED_buf, "Buzz:");
          ssd1306_SetCursor(0, 10);
          ssd1306_WriteString(OLED_buf, Font_11x18, White);

          sprintf(OLED_buf, "%-3s", SWITCH_NAME[!!use_alert]);
          ssd1306_SetCursor(60, 10);
          ssd1306_WriteString(OLED_buf, Font_11x18,
                              select_alert ? White : Black);

          sprintf(OLED_buf, "LEDs:");
          ssd1306_SetCursor(0, 28);
          ssd1306_WriteString(OLED_buf, Font_11x18, White);

          sprintf(OLED_buf, "%-3s", SWITCH_NAME[!!use_led]);
          ssd1306_SetCursor(60, 28);
          ssd1306_WriteString(OLED_buf, Font_11x18,
                              select_alert ? Black : White);

          if (use_led) {
            led_colors_buff[counts[0]] = LED_GREEN;
            if (!led_timer) {
              led_timer = 500;
              led_colors_buff[counts[0]] = LED_OFF;
              counts[0] = (counts[0] + 1) % 3;
            }
          }
        } break;
      }
    }
    ssd1306_UpdateScreen(&hi2c2);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100413;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
   */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {
  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00100413;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
   */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {
  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief TIM21 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM21_Init(void) {
  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 32 - 1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 250 - 1;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) !=
      HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
  HAL_TIM_MspPostInit(&htim21);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_DATA_GPIO_Port, RGB_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_IN_Pin SW2_IN_Pin SW3_IN_Pin */
  GPIO_InitStruct.Pin = SW1_IN_Pin | SW2_IN_Pin | SW3_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_DATA_Pin */
  GPIO_InitStruct.Pin = RGB_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RGB_DATA_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// every 1ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim6) {
  ws2812b_display_all_led_colors(led_colors_buff);

  if (use_alert && alert_time > 0) {
    alert_time--;
    // 32 * 1e6 / 32 / 250 => 4kHz
    // 250 / 2 => 125
    TIM21->CCR1 = 125;
  } else TIM21->CCR1 = 0;

  if (led_timer > 0) led_timer--;

  for (int i = 0; i < 3; i++) {
    Q_acc[i] += currents[i]; // Q = It
    energys_acc[i] += powers[i]; // w = Pt

    if (energys_acc[i] >= 360000) {
      energys_acc[i] = 0;
      energys[i]++;
      total_energy++;
    }

    if (Q_acc[i] >= 3600) {
      Q_acc[i] = 0;
      Q[i]++;
    }
  }
}
void ws2812b_set_colors(uint32_t grb) {
  uint_fast32_t ns;                 // variable for delay
  uint_fast32_t mask = 0x00800000;  // start from the highest bit

  for (uint_fast8_t i = 0; i < 24; i++)  // 24 bits
  {
    if (grb & mask)  // *** bit 1 ***
    {
      RGB_DATA_GPIO_Port->BSRR = RGB_DATA_Pin;  // 800 ns   should be
      ns = 11;                                  // 820 ns   measured
      while (ns--) __asm("nop");

      RGB_DATA_GPIO_Port->BRR = RGB_DATA_Pin;  // 450 ns   should be
      ns = 5;                                  // 410 ns   measured
      while (ns--) __asm("nop");
    } else  // *** bit 0 ***
    {
      RGB_DATA_GPIO_Port->BSRR = RGB_DATA_Pin;  // 400 ns   should be
      ns = 5;                                   // 440 ns   measured
      while (ns--) __asm("nop");
      RGB_DATA_GPIO_Port->BRR = RGB_DATA_Pin;  // 850 ns   should be
      ns = 10;                                 // 840 ns   measured
      while (ns--) __asm("nop");
    }
    mask >>= 1;  // next bit
  }
}
void ws2812b_display_all_led_colors(uint32_t *buff) {
  for (uint8_t i = 0; i < 3; i++) ws2812b_set_colors(buff[i]);
}

/* ina3221.pdf 8.6.1 */

uint16_t read_reg(uint8_t reg) {
  uint8_t rx[2], tx[] = {reg};
  HAL_I2C_Master_Transmit(&hi2c1, 0x80, tx, 1, 5);
  HAL_I2C_Master_Receive(&hi2c1, 0x80, rx, 2, 5);
  return (uint16_t)(rx[0] << 8) | (rx[1]);
}

void update_data() {
  for (uint8_t i = 0; i < 3; i++) {
    voltages[i] = read_reg(0x02 + i * 2);
    currents[i] = read_reg(0x01 + i * 2) / 20;
    powers[i] = voltages[i] * currents[i] / 1e3;
  }
}

void clear_leds() {
  led_colors_buff[0] = led_colors_buff[1] = led_colors_buff[2] = LED_OFF;
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
