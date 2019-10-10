/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define Current_Decimal_ON_Pin GPIO_PIN_14
#define Current_Decimal_ON_GPIO_Port GPIOC
#define Current_Decimal_OFF_Pin GPIO_PIN_15
#define Current_Decimal_OFF_GPIO_Port GPIOC
#define Voltage_Encoder_A_Pin GPIO_PIN_0
#define Voltage_Encoder_A_GPIO_Port GPIOF
#define Voltage_Encoder_B_Pin GPIO_PIN_1
#define Voltage_Encoder_B_GPIO_Port GPIOF
#define Output_enable_LED_Pin GPIO_PIN_0
#define Output_enable_LED_GPIO_Port GPIOC
#define CC_LED_Pin GPIO_PIN_1
#define CC_LED_GPIO_Port GPIOC
#define CV_LED_Pin GPIO_PIN_2
#define CV_LED_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define Vsense_Pin GPIO_PIN_1
#define Vsense_GPIO_Port GPIOA
#define Isense_Pin GPIO_PIN_2
#define Isense_GPIO_Port GPIOA
#define Tempsense_Pin GPIO_PIN_3
#define Tempsense_GPIO_Port GPIOA
#define Current_Encoder_A_Pin GPIO_PIN_6
#define Current_Encoder_A_GPIO_Port GPIOA
#define Current_Encoder_B_Pin GPIO_PIN_7
#define Current_Encoder_B_GPIO_Port GPIOA
#define Max_transient_Reset_ON_Pin GPIO_PIN_0
#define Max_transient_Reset_ON_GPIO_Port GPIOB
#define Max_transient_Reset_OFF_Pin GPIO_PIN_1
#define Max_transient_Reset_OFF_GPIO_Port GPIOB
#define Voltage_Decimal_ON_Pin GPIO_PIN_10
#define Voltage_Decimal_ON_GPIO_Port GPIOB
#define Voltage_Decimal_OFF_Pin GPIO_PIN_11
#define Voltage_Decimal_OFF_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define BuckPWM_Pin GPIO_PIN_8
#define BuckPWM_GPIO_Port GPIOA
#define Output_Enable_ON_Pin GPIO_PIN_11
#define Output_Enable_ON_GPIO_Port GPIOA
#define Output_Enable_OFF_Pin GPIO_PIN_12
#define Output_Enable_OFF_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
