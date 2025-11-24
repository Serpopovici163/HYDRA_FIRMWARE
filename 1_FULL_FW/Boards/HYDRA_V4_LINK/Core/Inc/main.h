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
#define OUT_LORA_nCS_Pin GPIO_PIN_13
#define OUT_LORA_nCS_GPIO_Port GPIOC
#define I_SENSE_5V_Pin GPIO_PIN_0
#define I_SENSE_5V_GPIO_Port GPIOC
#define V_SENSE_5V_Pin GPIO_PIN_1
#define V_SENSE_5V_GPIO_Port GPIOC
#define I_SENSE_3V3_Pin GPIO_PIN_2
#define I_SENSE_3V3_GPIO_Port GPIOC
#define V_SENSE_3V3_Pin GPIO_PIN_3
#define V_SENSE_3V3_GPIO_Port GPIOC
#define V_SENSE_VBAT_Pin GPIO_PIN_0
#define V_SENSE_VBAT_GPIO_Port GPIOA
#define I_SENSE_VBAT_Pin GPIO_PIN_1
#define I_SENSE_VBAT_GPIO_Port GPIOA
#define I_SENSE_PYRO_Pin GPIO_PIN_2
#define I_SENSE_PYRO_GPIO_Port GPIOA
#define V_SENSE_PYRO_Pin GPIO_PIN_3
#define V_SENSE_PYRO_GPIO_Port GPIOA
#define OUT_ETH_nCS_Pin GPIO_PIN_4
#define OUT_ETH_nCS_GPIO_Port GPIOA
#define OUT_ETH_nRST_Pin GPIO_PIN_4
#define OUT_ETH_nRST_GPIO_Port GPIOC
#define INTR_ETH_nINT_Pin GPIO_PIN_5
#define INTR_ETH_nINT_GPIO_Port GPIOC
#define OUT_ETH_nENABLE_Pin GPIO_PIN_0
#define OUT_ETH_nENABLE_GPIO_Port GPIOB
#define V_SENSE_AUX_RAD_Pin GPIO_PIN_1
#define V_SENSE_AUX_RAD_GPIO_Port GPIOB
#define V_SENSE_VCC_Pin GPIO_PIN_2
#define V_SENSE_VCC_GPIO_Port GPIOB
#define INP_nAUX_RAD_REG_FLT_Pin GPIO_PIN_10
#define INP_nAUX_RAD_REG_FLT_GPIO_Port GPIOB
#define INTR_SD_nCD_Pin GPIO_PIN_11
#define INTR_SD_nCD_GPIO_Port GPIOB
#define OUT_nBATT_EN_Pin GPIO_PIN_6
#define OUT_nBATT_EN_GPIO_Port GPIOC
#define INTR_nUPS_CHG_ACTIVE_Pin GPIO_PIN_7
#define INTR_nUPS_CHG_ACTIVE_GPIO_Port GPIOC
#define OUT_nEN_FST_CHG_Pin GPIO_PIN_8
#define OUT_nEN_FST_CHG_GPIO_Port GPIOC
#define OUT_MCU_G_LED_Pin GPIO_PIN_9
#define OUT_MCU_G_LED_GPIO_Port GPIOC
#define INTR_nAUX_RAD_ON_PYRO_PWR_Pin GPIO_PIN_8
#define INTR_nAUX_RAD_ON_PYRO_PWR_GPIO_Port GPIOA
#define V_SENSE_INT_3V3_Pin GPIO_PIN_9
#define V_SENSE_INT_3V3_GPIO_Port GPIOA
#define INTR_nUPS_BST_FLT_Pin GPIO_PIN_10
#define INTR_nUPS_BST_FLT_GPIO_Port GPIOA
#define OUT_MCU_R_LED_Pin GPIO_PIN_11
#define OUT_MCU_R_LED_GPIO_Port GPIOA
#define INTR_nUPS_ACTIVE_Pin GPIO_PIN_12
#define INTR_nUPS_ACTIVE_GPIO_Port GPIOA
#define OUT_AUX_RAD_PWR_EN_Pin GPIO_PIN_15
#define OUT_AUX_RAD_PWR_EN_GPIO_Port GPIOA
#define OUT_AUX_RAD_REG_INH_Pin GPIO_PIN_10
#define OUT_AUX_RAD_REG_INH_GPIO_Port GPIOC
#define OUT_LORA_nRST_Pin GPIO_PIN_11
#define OUT_LORA_nRST_GPIO_Port GPIOC
#define OUT_TCAN1146_nCS_Pin GPIO_PIN_4
#define OUT_TCAN1146_nCS_GPIO_Port GPIOB
#define OUT_LORA_nENABLE_Pin GPIO_PIN_7
#define OUT_LORA_nENABLE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
