/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Battery_Voltage_Pin GPIO_PIN_4
#define Battery_Voltage_GPIO_Port GPIOA
#define Bus_Voltage_Pin GPIO_PIN_5
#define Bus_Voltage_GPIO_Port GPIOA
#define Bridge_Voltage_Pin GPIO_PIN_6
#define Bridge_Voltage_GPIO_Port GPIOA
#define Gate_Driver_Enable_Pin GPIO_PIN_0
#define Gate_Driver_Enable_GPIO_Port GPIOB
#define Boost_H1_Pin GPIO_PIN_9
#define Boost_H1_GPIO_Port GPIOE
#define Boost_L1_Pin GPIO_PIN_11
#define Boost_L1_GPIO_Port GPIOE
#define Boost_H2_Pin GPIO_PIN_13
#define Boost_H2_GPIO_Port GPIOE
#define Boost_L2_Pin GPIO_PIN_14
#define Boost_L2_GPIO_Port GPIOE
#define Buck_L_Pin GPIO_PIN_10
#define Buck_L_GPIO_Port GPIOB
#define Buck_H_Pin GPIO_PIN_11
#define Buck_H_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_12
#define Buzzer_GPIO_Port GPIOB
#define Led_Status_Pin GPIO_PIN_8
#define Led_Status_GPIO_Port GPIOD
#define Bridge_ON_Pin GPIO_PIN_12
#define Bridge_ON_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
