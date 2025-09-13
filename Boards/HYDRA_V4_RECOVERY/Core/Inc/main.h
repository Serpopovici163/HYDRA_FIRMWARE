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
#define OUT_BARO_nCS_Pin GPIO_PIN_3
#define OUT_BARO_nCS_GPIO_Port GPIOE
#define OUT_IMU_nCS_Pin GPIO_PIN_4
#define OUT_IMU_nCS_GPIO_Port GPIOE
#define V_SENSE_VCC_Pin GPIO_PIN_0
#define V_SENSE_VCC_GPIO_Port GPIOC
#define V_SENSE_5V_Pin GPIO_PIN_3
#define V_SENSE_5V_GPIO_Port GPIOC
#define V_SENSE_3V3_Pin GPIO_PIN_0
#define V_SENSE_3V3_GPIO_Port GPIOA
#define V_SENSE_PYRO_PRE_SW_Pin GPIO_PIN_1
#define V_SENSE_PYRO_PRE_SW_GPIO_Port GPIOA
#define V_SENSE_DROGUE_Pin GPIO_PIN_2
#define V_SENSE_DROGUE_GPIO_Port GPIOA
#define V_SENSE_MAIN_Pin GPIO_PIN_3
#define V_SENSE_MAIN_GPIO_Port GPIOA
#define OUT_BUZZ_Pin GPIO_PIN_5
#define OUT_BUZZ_GPIO_Port GPIOA
#define I_SENSE_DROGUE_Pin GPIO_PIN_7
#define I_SENSE_DROGUE_GPIO_Port GPIOA
#define I_SENSE_MAIN_Pin GPIO_PIN_4
#define I_SENSE_MAIN_GPIO_Port GPIOC
#define V_SENSE_PYRO_POST_SW_Pin GPIO_PIN_5
#define V_SENSE_PYRO_POST_SW_GPIO_Port GPIOC
#define I_SENSE_3V3_Pin GPIO_PIN_0
#define I_SENSE_3V3_GPIO_Port GPIOB
#define I_SENSE_5V_Pin GPIO_PIN_1
#define I_SENSE_5V_GPIO_Port GPIOB
#define INTR_ADV7280_nINTRQ_Pin GPIO_PIN_10
#define INTR_ADV7280_nINTRQ_GPIO_Port GPIOA
#define OUT_ADV7280_nPWRDN_Pin GPIO_PIN_1
#define OUT_ADV7280_nPWRDN_GPIO_Port GPIOD
#define OUT_ADV7280_nRST_Pin GPIO_PIN_2
#define OUT_ADV7280_nRST_GPIO_Port GPIOD
#define OUT_DROGUE_TRIG_Pin GPIO_PIN_5
#define OUT_DROGUE_TRIG_GPIO_Port GPIOB
#define OUT_MAIN_TRIG_Pin GPIO_PIN_6
#define OUT_MAIN_TRIG_GPIO_Port GPIOB
#define OUT_IMU_nRST_Pin GPIO_PIN_0
#define OUT_IMU_nRST_GPIO_Port GPIOE
#define INTR_IMU_ODR_Pin GPIO_PIN_1
#define INTR_IMU_ODR_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
