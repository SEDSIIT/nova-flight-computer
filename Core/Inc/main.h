/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PYRO_F_Pin GPIO_PIN_13
#define PYRO_F_GPIO_Port GPIOC
#define SPI3_INT_TELE_GPIO_Pin GPIO_PIN_14
#define SPI3_INT_TELE_GPIO_GPIO_Port GPIOC
#define SPI3_CS_TELE_Pin GPIO_PIN_15
#define SPI3_CS_TELE_GPIO_Port GPIOC
#define PYRO_SENSE_F_Pin GPIO_PIN_0
#define PYRO_SENSE_F_GPIO_Port GPIOC
#define PYRO_SENSE_E_Pin GPIO_PIN_1
#define PYRO_SENSE_E_GPIO_Port GPIOC
#define PYRO_SENSE_D_Pin GPIO_PIN_2
#define PYRO_SENSE_D_GPIO_Port GPIOC
#define PYRO_SENSE_C_Pin GPIO_PIN_3
#define PYRO_SENSE_C_GPIO_Port GPIOC
#define PYRO_SENSE_B_Pin GPIO_PIN_0
#define PYRO_SENSE_B_GPIO_Port GPIOA
#define PYRO_SENSE_A_Pin GPIO_PIN_1
#define PYRO_SENSE_A_GPIO_Port GPIOA
#define BAT_SYS_VOLT_Pin GPIO_PIN_2
#define BAT_SYS_VOLT_GPIO_Port GPIOA
#define BAT_PYRO_VOLT_Pin GPIO_PIN_3
#define BAT_PYRO_VOLT_GPIO_Port GPIOA
#define SPI1_INT_MMC_GPIO_Pin GPIO_PIN_4
#define SPI1_INT_MMC_GPIO_GPIO_Port GPIOA
#define SPI1_SCK_SENS_Pin GPIO_PIN_5
#define SPI1_SCK_SENS_GPIO_Port GPIOA
#define SPI1_MISO_SENS_Pin GPIO_PIN_6
#define SPI1_MISO_SENS_GPIO_Port GPIOA
#define SPI1_MOSI_SENS_Pin GPIO_PIN_7
#define SPI1_MOSI_SENS_GPIO_Port GPIOA
#define SPI1_CS_ICM_GPIO_Pin GPIO_PIN_4
#define SPI1_CS_ICM_GPIO_GPIO_Port GPIOC
#define SPI1_CS_H3L_GPIO_Pin GPIO_PIN_5
#define SPI1_CS_H3L_GPIO_GPIO_Port GPIOC
#define SPI1_CS_MS5_GPIO_Pin GPIO_PIN_0
#define SPI1_CS_MS5_GPIO_GPIO_Port GPIOB
#define SPI1_CS_MMC_GPIO_Pin GPIO_PIN_1
#define SPI1_CS_MMC_GPIO_GPIO_Port GPIOB
#define SPI1_INT_ICM_GPIO_Pin GPIO_PIN_2
#define SPI1_INT_ICM_GPIO_GPIO_Port GPIOB
#define SPI1_INT_H3L_GPIO_Pin GPIO_PIN_10
#define SPI1_INT_H3L_GPIO_GPIO_Port GPIOB
#define PYRO_A_Pin GPIO_PIN_12
#define PYRO_A_GPIO_Port GPIOB
#define PYRO_B_Pin GPIO_PIN_13
#define PYRO_B_GPIO_Port GPIOB
#define PYRO_C_Pin GPIO_PIN_14
#define PYRO_C_GPIO_Port GPIOB
#define PYRO_D_Pin GPIO_PIN_15
#define PYRO_D_GPIO_Port GPIOB
#define USART6_TX_TELEMETRY_Pin GPIO_PIN_6
#define USART6_TX_TELEMETRY_GPIO_Port GPIOC
#define USART6_RX_TELEMETRY_Pin GPIO_PIN_7
#define USART6_RX_TELEMETRY_GPIO_Port GPIOC
#define PYRO_E_Pin GPIO_PIN_8
#define PYRO_E_GPIO_Port GPIOC
#define USART1_TX_GPS_Pin GPIO_PIN_9
#define USART1_TX_GPS_GPIO_Port GPIOA
#define USART1_RX_GPS_Pin GPIO_PIN_10
#define USART1_RX_GPS_GPIO_Port GPIOA
#define SPI3_FLASH_CS_GPIO_Pin GPIO_PIN_15
#define SPI3_FLASH_CS_GPIO_GPIO_Port GPIOA
#define SPI3_SCK_FLASH_Pin GPIO_PIN_10
#define SPI3_SCK_FLASH_GPIO_Port GPIOC
#define SPI3_MISO_FLASH_Pin GPIO_PIN_11
#define SPI3_MISO_FLASH_GPIO_Port GPIOC
#define SPI3_MOSI_FLASH_Pin GPIO_PIN_12
#define SPI3_MOSI_FLASH_GPIO_Port GPIOC
#define TIM2_BUZZER_Pin GPIO_PIN_3
#define TIM2_BUZZER_GPIO_Port GPIOB
#define TIM3_LED_BLUE_Pin GPIO_PIN_4
#define TIM3_LED_BLUE_GPIO_Port GPIOB
#define TIM3_LED_GREEN_Pin GPIO_PIN_5
#define TIM3_LED_GREEN_GPIO_Port GPIOB
#define TIM4_CH1_PWM_Pin GPIO_PIN_6
#define TIM4_CH1_PWM_GPIO_Port GPIOB
#define TIM4_CH2_PWM_Pin GPIO_PIN_7
#define TIM4_CH2_PWM_GPIO_Port GPIOB
#define TIM4_CH3_PWM_Pin GPIO_PIN_8
#define TIM4_CH3_PWM_GPIO_Port GPIOB
#define TIM4_CH4_PWM_Pin GPIO_PIN_9
#define TIM4_CH4_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
