/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OUT_BARO0_nCS_Pin GPIO_PIN_3
#define OUT_BARO0_nCS_GPIO_Port GPIOE
#define OUT_BARO1_nCS_Pin GPIO_PIN_4
#define OUT_BARO1_nCS_GPIO_Port GPIOE
#define OUT_ACT_PWR_INH_Pin GPIO_PIN_13
#define OUT_ACT_PWR_INH_GPIO_Port GPIOC
#define OUT_NAV_G_LED_Pin GPIO_PIN_0
#define OUT_NAV_G_LED_GPIO_Port GPIOC
#define OUT_NAV_R_LED_Pin GPIO_PIN_1
#define OUT_NAV_R_LED_GPIO_Port GPIOC
#define V_SENSE_3V3_Pin GPIO_PIN_3
#define V_SENSE_3V3_GPIO_Port GPIOC
#define OUT_MCU_G_LED_Pin GPIO_PIN_0
#define OUT_MCU_G_LED_GPIO_Port GPIOA
#define OUT_MCU_R_LED_Pin GPIO_PIN_1
#define OUT_MCU_R_LED_GPIO_Port GPIOA
#define V_SENSE_5V_Pin GPIO_PIN_3
#define V_SENSE_5V_GPIO_Port GPIOA
#define OUT_IMU1_nCS_Pin GPIO_PIN_4
#define OUT_IMU1_nCS_GPIO_Port GPIOA
#define V_SENSE_ACT_PWR_Pin GPIO_PIN_4
#define V_SENSE_ACT_PWR_GPIO_Port GPIOC
#define I_SENSE_ACT_PWR_Pin GPIO_PIN_5
#define I_SENSE_ACT_PWR_GPIO_Port GPIOC
#define I_SENSE_3V3_Pin GPIO_PIN_0
#define I_SENSE_3V3_GPIO_Port GPIOB
#define V_SENSE_VCC_Pin GPIO_PIN_1
#define V_SENSE_VCC_GPIO_Port GPIOB
#define INTR_IMU0_ODR_Pin GPIO_PIN_3
#define INTR_IMU0_ODR_GPIO_Port GPIOD
#define OUT_IMU0_nRST_Pin GPIO_PIN_4
#define OUT_IMU0_nRST_GPIO_Port GPIOD
#define INTR_SD_CD_Pin GPIO_PIN_5
#define INTR_SD_CD_GPIO_Port GPIOD
#define OUT_IMU0_nCS_Pin GPIO_PIN_6
#define OUT_IMU0_nCS_GPIO_Port GPIOD
#define INTR_IMU1_INT1_Pin GPIO_PIN_7
#define INTR_IMU1_INT1_GPIO_Port GPIOD
#define INTR_IMU1_INT2_Pin GPIO_PIN_4
#define INTR_IMU1_INT2_GPIO_Port GPIOB
#define OUT_GPS_nRST_Pin GPIO_PIN_7
#define OUT_GPS_nRST_GPIO_Port GPIOB
#define OUT_GPS_ENABLE_Pin GPIO_PIN_0
#define OUT_GPS_ENABLE_GPIO_Port GPIOE
#define INP_ACT_PWR_PG_Pin GPIO_PIN_1
#define INP_ACT_PWR_PG_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
