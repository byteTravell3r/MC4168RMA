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
#include <stdio.h>
#include "USR_CAN.h"
#include "USR_PID.h"
#include "USR_RC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART7_Init(void);
static void MX_CRC_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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
	MX_CAN1_Init();
	MX_UART7_Init();
	MX_CRC_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(POWER1_CTRL_GPIO_Port, POWER1_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(POWER3_CTRL_GPIO_Port, POWER3_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GND_OF_5VO_GPIO_Port, GND_OF_5VO_Pin, GPIO_PIN_SET);

	PID_StructInit(&PID_MotorSpeed[MotorX], 4000.f, 10000.f, 5.0f, 0, 5.1f, 0);
	PID_StructInit(&PID_MotorSpeed[MotorY], 4000.f, 10000.f, 5.0f, 0, 5.0f, 0);
	PID_StructInit(&PID_MotorPosition[MotorX], 100000.f, 10000.f, 0.08f, 0, 0.84f, 0);
	PID_StructInit(&PID_MotorPosition[MotorY], 100000.f, 10000.f, 0.08f, 0, 0.84f, 0);

	static double targetSpeed[2], targetPosition[2];
	targetSpeed[0] = 192;
	targetSpeed[1] = 192;

	iBUS_Start_RxIT();
	CAN1_START_IRQ();

	HAL_Delay(100);
	/* USER CODE END 2 */
	iBUS_Channel[0] = 1500;
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		targetPosition[MotorX] += (iBUS_Channel[0] - 1500) * 1.2;
		targetSpeed[MotorX] = PID_Calculate(&PID_MotorPosition[MotorX], Motor[MotorX].TOTAL_ANGLE, targetPosition[MotorX]);
		PID_Calculate(&PID_MotorSpeed[MotorX], Motor[MotorX].RPM, targetSpeed[MotorX]);

		targetPosition[MotorY] += (iBUS_Channel[3] - 1500) * 1.2;
		targetSpeed[MotorY] = PID_Calculate(&PID_MotorPosition[MotorY], Motor[MotorY].TOTAL_ANGLE, targetPosition[MotorY]);
		PID_Calculate(&PID_MotorSpeed[MotorY], Motor[MotorY].RPM, targetSpeed[MotorY]);

		Motor_SendCmd(PID_MotorSpeed[MotorX].Output, PID_MotorSpeed[MotorY].Output);
		HAL_Delay(2);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 90;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* CAN1_RX0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	/* CAN1_RX1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
	/* CAN1_TX_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 5;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef CAN_FilterConfigStructure;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.FilterBank = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterActivation = CAN_FILTER_ENABLE;
	if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void) {

	/* USER CODE BEGIN UART7_Init 0 */

	/* USER CODE END UART7_Init 0 */

	/* USER CODE BEGIN UART7_Init 1 */

	/* USER CODE END UART7_Init 1 */
	huart7.Instance = UART7;
	huart7.Init.BaudRate = 115200;
	huart7.Init.WordLength = UART_WORDLENGTH_8B;
	huart7.Init.StopBits = UART_STOPBITS_1;
	huart7.Init.Parity = UART_PARITY_NONE;
	huart7.Init.Mode = UART_MODE_TX_RX;
	huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart7.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart7) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART7_Init 2 */

	/* USER CODE END UART7_Init 2 */

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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GND_OF_5VO_GPIO_Port, GND_OF_5VO_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, OLED_B9__Pin | OLED_B0__Pin | SYNC_IN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH,
	POWER1_CTRL_Pin | POWER2_CTRL_Pin | POWER3_CTRL_Pin | POWER4_CTRL_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG,
	LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_E_Pin | LED_F_Pin | LED_G_Pin | LED_H_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : HEAT_IMU_Pin */
	GPIO_InitStruct.Pin = HEAT_IMU_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(HEAT_IMU_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GND_OF_5VO_Pin LED_A_Pin LED_B_Pin LED_C_Pin
	 LED_D_Pin LED_E_Pin LED_F_Pin LED_G_Pin
	 LED_H_Pin */
	GPIO_InitStruct.Pin = GND_OF_5VO_Pin | LED_A_Pin | LED_B_Pin | LED_C_Pin | LED_D_Pin | LED_E_Pin | LED_F_Pin | LED_G_Pin | LED_H_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : OLED_B9__Pin OLED_B0__Pin SYNC_IN_Pin */
	GPIO_InitStruct.Pin = OLED_B9__Pin | OLED_B0__Pin | SYNC_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : DBUS_RX_Pin */
	GPIO_InitStruct.Pin = DBUS_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(DBUS_RX_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SERVO_PWM_Y_Pin SERVO_PWM_X_Pin SERVO_PWM_W_Pin SERVO_PWM_Z_Pin */
	GPIO_InitStruct.Pin = SERVO_PWM_Y_Pin | SERVO_PWM_X_Pin | SERVO_PWM_W_Pin | SERVO_PWM_Z_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pin : SERVO_PWM_A_Pin */
	GPIO_InitStruct.Pin = SERVO_PWM_A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(SERVO_PWM_A_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SERVO_PWM_A9_Pin */
	GPIO_InitStruct.Pin = SERVO_PWM_A9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(SERVO_PWM_A9_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : POWER1_CTRL_Pin POWER2_CTRL_Pin POWER3_CTRL_Pin POWER4_CTRL_Pin */
	GPIO_InitStruct.Pin = POWER1_CTRL_Pin | POWER2_CTRL_Pin | POWER3_CTRL_Pin | POWER4_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : SERVO_PWM_B_Pin SERVO_PWM_C_Pin SERVO_PWM_D_Pin */
	GPIO_InitStruct.Pin = SERVO_PWM_B_Pin | SERVO_PWM_C_Pin | SERVO_PWM_D_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : SERVO_PWM_E_Pin SERVO_PWM_F_Pin SERVO_PWM_G_Pin SERVO_PWM_H_Pin */
	GPIO_InitStruct.Pin = SERVO_PWM_E_Pin | SERVO_PWM_F_Pin | SERVO_PWM_G_Pin | SERVO_PWM_H_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_Pin */
	GPIO_InitStruct.Pin = BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SERVO_PWM_T_Pin SERVO_PWM_S_Pin SERVO_PWM_U_Pin SERVO_PWM_V_Pin */
	GPIO_InitStruct.Pin = SERVO_PWM_T_Pin | SERVO_PWM_S_Pin | SERVO_PWM_U_Pin | SERVO_PWM_V_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SERVO_PWM_E13_Pin */
	GPIO_InitStruct.Pin = SERVO_PWM_E13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(SERVO_PWM_E13_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OLED_MI_Pin */
	GPIO_InitStruct.Pin = OLED_MI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(OLED_MI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_RED_Pin */
	GPIO_InitStruct.Pin = LED_RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_GREEN_Pin */
	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_EXTI__Pin */
	GPIO_InitStruct.Pin = SD_EXTI__Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SD_EXTI__GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SYNC_OUT_Pin */
	GPIO_InitStruct.Pin = SYNC_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SYNC_OUT_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
	while (1) {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(1000);
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
