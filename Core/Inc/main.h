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
#define HEAT_IMU_Pin GPIO_PIN_5
#define HEAT_IMU_GPIO_Port GPIOB
#define GND_OF_5VO_Pin GPIO_PIN_13
#define GND_OF_5VO_GPIO_Port GPIOG
#define OLED_B9__Pin GPIO_PIN_9
#define OLED_B9__GPIO_Port GPIOB
#define DBUS_RX_Pin GPIO_PIN_7
#define DBUS_RX_GPIO_Port GPIOB
#define SERVO_PWM_Y_Pin GPIO_PIN_7
#define SERVO_PWM_Y_GPIO_Port GPIOI
#define SERVO_PWM_X_Pin GPIO_PIN_6
#define SERVO_PWM_X_GPIO_Port GPIOI
#define SERVO_PWM_W_Pin GPIO_PIN_5
#define SERVO_PWM_W_GPIO_Port GPIOI
#define SERVO_PWM_Z_Pin GPIO_PIN_2
#define SERVO_PWM_Z_GPIO_Port GPIOI
#define SERVO_PWM_A_Pin GPIO_PIN_0
#define SERVO_PWM_A_GPIO_Port GPIOI
#define SERVO_PWM_A9_Pin GPIO_PIN_9
#define SERVO_PWM_A9_GPIO_Port GPIOA
#define POWER1_CTRL_Pin GPIO_PIN_2
#define POWER1_CTRL_GPIO_Port GPIOH
#define POWER2_CTRL_Pin GPIO_PIN_3
#define POWER2_CTRL_GPIO_Port GPIOH
#define POWER3_CTRL_Pin GPIO_PIN_4
#define POWER3_CTRL_GPIO_Port GPIOH
#define LED_A_Pin GPIO_PIN_8
#define LED_A_GPIO_Port GPIOG
#define POWER4_CTRL_Pin GPIO_PIN_5
#define POWER4_CTRL_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_7
#define LED_B_GPIO_Port GPIOG
#define LED_C_Pin GPIO_PIN_6
#define LED_C_GPIO_Port GPIOG
#define SERVO_PWM_B_Pin GPIO_PIN_12
#define SERVO_PWM_B_GPIO_Port GPIOH
#define LED_D_Pin GPIO_PIN_5
#define LED_D_GPIO_Port GPIOG
#define LED_E_Pin GPIO_PIN_4
#define LED_E_GPIO_Port GPIOG
#define LED_F_Pin GPIO_PIN_3
#define LED_F_GPIO_Port GPIOG
#define SERVO_PWM_C_Pin GPIO_PIN_11
#define SERVO_PWM_C_GPIO_Port GPIOH
#define SERVO_PWM_D_Pin GPIO_PIN_10
#define SERVO_PWM_D_GPIO_Port GPIOH
#define SERVO_PWM_E_Pin GPIO_PIN_15
#define SERVO_PWM_E_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOG
#define BUTTON_Pin GPIO_PIN_2
#define BUTTON_GPIO_Port GPIOB
#define LED_H_Pin GPIO_PIN_1
#define LED_H_GPIO_Port GPIOG
#define SERVO_PWM_F_Pin GPIO_PIN_14
#define SERVO_PWM_F_GPIO_Port GPIOD
#define SERVO_PWM_G_Pin GPIO_PIN_13
#define SERVO_PWM_G_GPIO_Port GPIOD
#define SERVO_PWM_T_Pin GPIO_PIN_1
#define SERVO_PWM_T_GPIO_Port GPIOA
#define SERVO_PWM_S_Pin GPIO_PIN_0
#define SERVO_PWM_S_GPIO_Port GPIOA
#define SERVO_PWM_E13_Pin GPIO_PIN_13
#define SERVO_PWM_E13_GPIO_Port GPIOE
#define SERVO_PWM_H_Pin GPIO_PIN_12
#define SERVO_PWM_H_GPIO_Port GPIOD
#define SERVO_PWM_U_Pin GPIO_PIN_2
#define SERVO_PWM_U_GPIO_Port GPIOA
#define OLED_MI_Pin GPIO_PIN_6
#define OLED_MI_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define SERVO_PWM_V_Pin GPIO_PIN_3
#define SERVO_PWM_V_GPIO_Port GPIOA
#define OLED_B0__Pin GPIO_PIN_0
#define OLED_B0__GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
#define SD_EXTI__Pin GPIO_PIN_15
#define SD_EXTI__GPIO_Port GPIOE
#define SYNC_OUT_Pin GPIO_PIN_14
#define SYNC_OUT_GPIO_Port GPIOB
#define SYNC_IN_Pin GPIO_PIN_15
#define SYNC_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
