/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Process status structure
typedef struct{
	uint32_t timestamp; // Not used
	uint16_t rangeTh, minScan, maxScan, capturedCCR, currentCCR; //Not used
	uint8_t computer:1,sensor:1,transmit:1, scan:1; // Flags. Scan flag doesn't have specific implementation, but used to determine computer flag
	uint8_t scanMode:2, scanInc; // Not used
}stat_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BUFLEN 512 // UART process buffer length
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile stat_t process;
extern char txBuff[BUFLEN], rxBuff[BUFLEN];
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define DOF_1_Pin GPIO_PIN_0
#define DOF_1_GPIO_Port GPIOA
#define DOF_2_Pin GPIO_PIN_1
#define DOF_2_GPIO_Port GPIOA
#define DOF_3_Pin GPIO_PIN_2
#define DOF_3_GPIO_Port GPIOA
#define DOF_4_Pin GPIO_PIN_3
#define DOF_4_GPIO_Port GPIOA
#define GRP_1_Pin GPIO_PIN_0
#define GRP_1_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define VL_INT_Pin GPIO_PIN_8
#define VL_INT_GPIO_Port GPIOB
#define VL_INT_EXTI_IRQn EXTI9_5_IRQn
#define VL_SHT_Pin GPIO_PIN_9
#define VL_SHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
