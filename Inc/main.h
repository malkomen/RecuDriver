/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "cmsis_os.h"
//#include "sensorsTask.h"
#include "nextionDisplay.h"
#include "automationTask.h"
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
#define ADC_Bypass_CH1_IS_Pin GPIO_PIN_0
#define ADC_Bypass_CH1_IS_GPIO_Port GPIOC
#define ADC_Bypass_CH2_IS_Pin GPIO_PIN_1
#define ADC_Bypass_CH2_IS_GPIO_Port GPIOC
#define FAN1_PWM_uC_Pin GPIO_PIN_0
#define FAN1_PWM_uC_GPIO_Port GPIOA
#define FAN2_PWM_uC_Pin GPIO_PIN_1
#define FAN2_PWM_uC_GPIO_Port GPIOA
#define HEATER_uC_Pin GPIO_PIN_2
#define HEATER_uC_GPIO_Port GPIOA
#define Bypass_CH1_INH_Pin GPIO_PIN_4
#define Bypass_CH1_INH_GPIO_Port GPIOA
#define Bypass_CH2_INH_Pin GPIO_PIN_5
#define Bypass_CH2_INH_GPIO_Port GPIOA
#define TIM3_CH3_ENK_A_Pin GPIO_PIN_6
#define TIM3_CH3_ENK_A_GPIO_Port GPIOA
#define TIM3_CH4_ENK_B_Pin GPIO_PIN_7
#define TIM3_CH4_ENK_B_GPIO_Port GPIOA
#define ENK_SW_Pin GPIO_PIN_4
#define ENK_SW_GPIO_Port GPIOC
#define ADC_Bypass_current_Pin GPIO_PIN_0
#define ADC_Bypass_current_GPIO_Port GPIOB
#define ADC_230V_current_Pin GPIO_PIN_1
#define ADC_230V_current_GPIO_Port GPIOB
#define ESP_RESET_Pin GPIO_PIN_2
#define ESP_RESET_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_11
#define SPI2_CS1_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_12
#define SPI2_CS2_GPIO_Port GPIOB
#define SPI2_CS3_Pin GPIO_PIN_13
#define SPI2_CS3_GPIO_Port GPIOB
#define SPI2_CS4_Pin GPIO_PIN_14
#define SPI2_CS4_GPIO_Port GPIOB
#define SPI2_CS5_Pin GPIO_PIN_15
#define SPI2_CS5_GPIO_Port GPIOB
#define FAN1_TACHO_Pin GPIO_PIN_6
#define FAN1_TACHO_GPIO_Port GPIOC
#define FAN2_TACHO_Pin GPIO_PIN_7
#define FAN2_TACHO_GPIO_Port GPIOC
#define SPI2_CS6_Pin GPIO_PIN_8
#define SPI2_CS6_GPIO_Port GPIOC
#define SPI2_CS7_Pin GPIO_PIN_9
#define SPI2_CS7_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define USART1_TX_NEXTION_Pin GPIO_PIN_9
#define USART1_TX_NEXTION_GPIO_Port GPIOA
#define USART1_RX_NEXTION_Pin GPIO_PIN_10
#define USART1_RX_NEXTION_GPIO_Port GPIOA
#define RS485_ST_Pin GPIO_PIN_15
#define RS485_ST_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOC
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOC
#define UART5_TX_ESP_Pin GPIO_PIN_12
#define UART5_TX_ESP_GPIO_Port GPIOC
#define UART5_RX_ESP_Pin GPIO_PIN_2
#define UART5_RX_ESP_GPIO_Port GPIOD
#define Bypass_CH1_PWM_Pin GPIO_PIN_6
#define Bypass_CH1_PWM_GPIO_Port GPIOB
#define Bypass_CH2_PWM_Pin GPIO_PIN_7
#define Bypass_CH2_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
