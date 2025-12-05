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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define Vout_Poten_Pin GPIO_PIN_0
#define Vout_Poten_GPIO_Port GPIOA
#define L_Switch_Pin GPIO_PIN_1
#define L_Switch_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define R_Switch_Pin GPIO_PIN_4
#define R_Switch_GPIO_Port GPIOA
#define Backward_Pin GPIO_PIN_5
#define Backward_GPIO_Port GPIOA
#define Forward_Pin GPIO_PIN_6
#define Forward_GPIO_Port GPIOA
#define Auto_Pin GPIO_PIN_7
#define Auto_GPIO_Port GPIOA
#define Emergency_Pin GPIO_PIN_5
#define Emergency_GPIO_Port GPIOC
#define Lamp_Forward_Pin GPIO_PIN_10
#define Lamp_Forward_GPIO_Port GPIOB
#define Signal_B_Pin GPIO_PIN_11
#define Signal_B_GPIO_Port GPIOB
#define Signal_L_Pin GPIO_PIN_12
#define Signal_L_GPIO_Port GPIOB
#define L_Break_Pin GPIO_PIN_6
#define L_Break_GPIO_Port GPIOC
#define Teleop_Pin GPIO_PIN_7
#define Teleop_GPIO_Port GPIOC
#define R_Break_Pin GPIO_PIN_8
#define R_Break_GPIO_Port GPIOC
#define Lamp_Backward_Pin GPIO_PIN_8
#define Lamp_Backward_GPIO_Port GPIOA
#define Manual_Pin GPIO_PIN_9
#define Manual_GPIO_Port GPIOA
#define Lamp_Mode1_Pin GPIO_PIN_10
#define Lamp_Mode1_GPIO_Port GPIOA
#define Signal_R_Pin GPIO_PIN_11
#define Signal_R_GPIO_Port GPIOA
#define Signal_F_Pin GPIO_PIN_12
#define Signal_F_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Lamp_Mode2_Pin GPIO_PIN_3
#define Lamp_Mode2_GPIO_Port GPIOB
#define Lamp_Mode3_Pin GPIO_PIN_4
#define Lamp_Mode3_GPIO_Port GPIOB
#define Lamp_Mode4_Pin GPIO_PIN_5
#define Lamp_Mode4_GPIO_Port GPIOB
#define Joystick_Pin GPIO_PIN_6
#define Joystick_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
