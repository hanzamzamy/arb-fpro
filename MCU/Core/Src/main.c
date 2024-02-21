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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "usbd_cdc_if.h"
#include "cmd_handler.h"
//#include "custom_daq.h"
#include "vl53l0x_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INRANGE(x,m,M)(x > m && x < M)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
Servo_t dof1, dof2, dof3, dof4, grp1;

volatile stat_t process = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
char rxBuff[BUFLEN], txBuff[BUFLEN];

VL53L0X_Dev_t tof;
VL53L0X_RangingMeasurementData_t rangeData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Sensor_Init(void);
void Actuator_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
	process.sensor = 1;

	//Custom_DAQ();
}
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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USB_DEVICE_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(1000);
	Sensor_Init();
	Actuator_Init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (process.sensor) {
			process.sensor = 0;
			VL53L0X_RangingMeasurementData_t range;
			VL53L0X_GetRangingMeasurementData(&tof, &range);
			if (range.RangeStatus == 0) {
				rangeData = range;
			}

			uint8_t n = 0;
			while (VL53L0X_ClearInterruptMask(&tof, 0) != VL53L0X_ERROR_NONE) {
				if (n == 16) {
					if (VL53L0X_ResetDevice(&tof) == VL53L0X_ERROR_NONE) {
						Sensor_Init();
					}
				}
				n++;
			}
		}

		if (process.computer) {
			process.computer = 0;
			parseCommand(rxBuff);
			if (process.transmit) {
				process.transmit = 0;
				snprintf(txBuff, BUFLEN, "DIST:%u ROT0:%u RAW0:%lu"
						" ROT1:%u RAW1:%lu ROT2:%u RAW2:%lu"
						" ROT3:%u RAW3:%lu ROT4:%u RAW4:%lu\n",
						rangeData.RangeMilliMeter, grp1.rotDegree,
						htim3.Instance->CCR3, dof1.rotDegree,
						htim2.Instance->CCR1, dof2.rotDegree,
						htim2.Instance->CCR2, dof3.rotDegree,
						htim2.Instance->CCR3, dof4.rotDegree,
						htim2.Instance->CCR4);
				CDC_Transmit_FS((uint8_t*) txBuff, strlen(txBuff));
			}
		}

		if (process.scan && rangeData.TimeStamp != process.timestamp) {
			if (rangeData.RangeMilliMeter <= process.rangeTh) {
				if (process.scanMode == 1) {
					process.scan = 0;
				} else {
					if (process.capturedCCR == 0) {
						process.capturedCCR = process.currentCCR;
					}
					if (!INRANGE(process.currentCCR + process.scanInc,
							process.minScan, process.maxScan)) {
						process.scanInc = -process.scanInc;
					}
					process.currentCCR += process.scanInc;
				}
			} else if (process.capturedCCR != 0) {
				if (process.scanMode == 2) {
					process.scan = 0;
					process.currentCCR -= process.scanInc;
				} else {
					process.scan = 0;
					process.currentCCR = process.capturedCCR
							+ (process.currentCCR - process.capturedCCR) / 2;
				}
			} else {
				if (!INRANGE(process.currentCCR + process.scanInc,
						process.minScan, process.maxScan)) {
					process.scanInc = -process.scanInc;
				}
				process.currentCCR += process.scanInc;
			}
			process.timestamp = rangeData.TimeStamp;
			Servo_setCCR(&dof1, process.currentCCR);
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */
	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 42 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 40000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 42 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 40000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(VL_SHT_GPIO_Port, VL_SHT_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : VL_INT_Pin */
	GPIO_InitStruct.Pin = VL_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VL_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : VL_SHT_Pin */
	GPIO_InitStruct.Pin = VL_SHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(VL_SHT_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Sensor_Init(void) {
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
// Long Range Profile
	FixPoint1616_t signalLimit = (FixPoint1616_t) (0.25 * 65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t) (32 * 65536);
	uint32_t timingBudget = 20000;
	//uint8_t preRangeVcselPeriod = 18;
	//uint8_t finalRangeVcselPeriod = 14;

	tof.I2cHandle = &hi2c1;
	tof.I2cDevAddr = 0x52;
	tof.comms_type = 1;
	tof.comms_speed_khz = 100;

	VL53L0X_DataInit(&tof);
	VL53L0X_StaticInit(&tof);
	VL53L0X_PerformRefSpadManagement(&tof, &refSpadCount, &isApertureSpads);
	VL53L0X_PerformRefCalibration(&tof, &VhvSettings, &PhaseCal);

	VL53L0X_SetLimitCheckEnable(&tof,
	VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(&tof,
	VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(&tof,
	VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
	VL53L0X_SetLimitCheckValue(&tof,
	VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&tof, timingBudget);
//	VL53L0X_SetVcselPulsePeriod(&tof, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
//			preRangeVcselPeriod);
//	VL53L0X_SetVcselPulsePeriod(&tof, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
//			finalRangeVcselPeriod);
	VL53L0X_SetGpioConfig(&tof, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
	VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
	VL53L0X_INTERRUPTPOLARITY_HIGH);
	VL53L0X_SetDeviceMode(&tof, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	VL53L0X_StartMeasurement(&tof);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Actuator_Init(void) {
	Servo_Init(&dof1, &htim2, TIM2, 1, 1000, 4999);
	Servo_Init(&dof2, &htim2, TIM2, 2, 1000, 4999);
	Servo_Init(&dof3, &htim2, TIM2, 3, 1000, 4999);
	Servo_Init(&dof4, &htim2, TIM2, 4, 1000, 4999);
	Servo_Init(&grp1, &htim3, TIM3, 3, 1000, 4999);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	NVIC_SystemReset();

	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
