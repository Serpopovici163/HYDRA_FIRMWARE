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
#include "stm32g4xx_hal.h"

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
#define V_SENSE_INT_3V3_Pin GPIO_PIN_0
#define V_SENSE_INT_3V3_GPIO_Port GPIOC
#define I_SENSE_M2_Pin GPIO_PIN_1
#define I_SENSE_M2_GPIO_Port GPIOC
#define V_SENSE_M2_Pin GPIO_PIN_2
#define V_SENSE_M2_GPIO_Port GPIOC
#define V_SENSE_M1_Pin GPIO_PIN_3
#define V_SENSE_M1_GPIO_Port GPIOC
#define I_SENSE_M1_Pin GPIO_PIN_0
#define I_SENSE_M1_GPIO_Port GPIOA
#define INP_M1_OC_FLT_Pin GPIO_PIN_1
#define INP_M1_OC_FLT_GPIO_Port GPIOA
#define I_SENSE_M3_Pin GPIO_PIN_2
#define I_SENSE_M3_GPIO_Port GPIOA
#define MCU_R_LED_Pin GPIO_PIN_3
#define MCU_R_LED_GPIO_Port GPIOA
#define MCU_G_LED_Pin GPIO_PIN_4
#define MCU_G_LED_GPIO_Port GPIOA
#define V_SENSE_INT_VCC_Pin GPIO_PIN_5
#define V_SENSE_INT_VCC_GPIO_Port GPIOA
#define V_SENSE_INT_5V_Pin GPIO_PIN_7
#define V_SENSE_INT_5V_GPIO_Port GPIOA
#define V_SENSE_3V3_Pin GPIO_PIN_4
#define V_SENSE_3V3_GPIO_Port GPIOC
#define V_SENSE_5V_Pin GPIO_PIN_5
#define V_SENSE_5V_GPIO_Port GPIOC
#define CELL_ADC_nRST_Pin GPIO_PIN_10
#define CELL_ADC_nRST_GPIO_Port GPIOB
#define I_SENSE_VCC_LS_Pin GPIO_PIN_11
#define I_SENSE_VCC_LS_GPIO_Port GPIOB
#define CELL_ADC_nCS_Pin GPIO_PIN_12
#define CELL_ADC_nCS_GPIO_Port GPIOB
#define TCAN1146_nCS_Pin GPIO_PIN_6
#define TCAN1146_nCS_GPIO_Port GPIOC
#define INP_VCC_LS_PG_Pin GPIO_PIN_7
#define INP_VCC_LS_PG_GPIO_Port GPIOC
#define INP_5V_LS_PG_Pin GPIO_PIN_9
#define INP_5V_LS_PG_GPIO_Port GPIOC
#define I_SENSE_5V_LS_Pin GPIO_PIN_8
#define I_SENSE_5V_LS_GPIO_Port GPIOA
#define I_SENSE_3V3_LS_Pin GPIO_PIN_9
#define I_SENSE_3V3_LS_GPIO_Port GPIOA
#define INP_3V3_LS_PG_Pin GPIO_PIN_10
#define INP_3V3_LS_PG_GPIO_Port GPIOA
#define OUT_EMAG_Pin GPIO_PIN_15
#define OUT_EMAG_GPIO_Port GPIOA
#define OUT_FORCE_ON_Pin GPIO_PIN_10
#define OUT_FORCE_ON_GPIO_Port GPIOC
#define OUT_M1_OC_RST_Pin GPIO_PIN_11
#define OUT_M1_OC_RST_GPIO_Port GPIOC
#define OUT_M3_EN_Pin GPIO_PIN_12
#define OUT_M3_EN_GPIO_Port GPIOC
#define OUT_M2_INH_Pin GPIO_PIN_2
#define OUT_M2_INH_GPIO_Port GPIOD
#define INP_GEN_PG_Pin GPIO_PIN_4
#define INP_GEN_PG_GPIO_Port GPIOB
#define OUT_3V3_REG_EN_Pin GPIO_PIN_7
#define OUT_3V3_REG_EN_GPIO_Port GPIOB
#define OUT_5V_REG_EN_Pin GPIO_PIN_8
#define OUT_5V_REG_EN_GPIO_Port GPIOB
#define OUT_M1_INH_Pin GPIO_PIN_9
#define OUT_M1_INH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
