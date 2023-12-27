/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TURN_ON_CHARGE_Pin GPIO_PIN_2
#define TURN_ON_CHARGE_GPIO_Port GPIOE
#define TURN_ON_DISCHARGE_Pin GPIO_PIN_3
#define TURN_ON_DISCHARGE_GPIO_Port GPIOE
#define EN_RELAY_Pin GPIO_PIN_4
#define EN_RELAY_GPIO_Port GPIOE
#define TURN_OFF_CHARGE_Pin GPIO_PIN_5
#define TURN_OFF_CHARGE_GPIO_Port GPIOE
#define SPI6_NSS_LTC_Pin GPIO_PIN_4
#define SPI6_NSS_LTC_GPIO_Port GPIOA
#define SPI6_SCK_LTC_Pin GPIO_PIN_5
#define SPI6_SCK_LTC_GPIO_Port GPIOA
#define SPI6_MISO_LTC_Pin GPIO_PIN_6
#define SPI6_MISO_LTC_GPIO_Port GPIOA
#define SPI6_MOSI_LTC_Pin GPIO_PIN_7
#define SPI6_MOSI_LTC_GPIO_Port GPIOA
#define FNC_BTN_Pin GPIO_PIN_7
#define FNC_BTN_GPIO_Port GPIOE
#define SPI4_NSS_EEPROM_Pin GPIO_PIN_9
#define SPI4_NSS_EEPROM_GPIO_Port GPIOE
#define WRPT_EEPROM_Pin GPIO_PIN_10
#define WRPT_EEPROM_GPIO_Port GPIOE
#define HOLD_EEPROM_Pin GPIO_PIN_11
#define HOLD_EEPROM_GPIO_Port GPIOE
#define LED_GREEN_OK_Pin GPIO_PIN_15
#define LED_GREEN_OK_GPIO_Port GPIOE
#define LED_YELLOW_DISCHARGE_Pin GPIO_PIN_10
#define LED_YELLOW_DISCHARGE_GPIO_Port GPIOB
#define LED_YELLOW_CHARGE_Pin GPIO_PIN_11
#define LED_YELLOW_CHARGE_GPIO_Port GPIOB
#define SPI4_NSS_FLASH_Pin GPIO_PIN_12
#define SPI4_NSS_FLASH_GPIO_Port GPIOB
#define SPI2_SCK_INA_Pin GPIO_PIN_13
#define SPI2_SCK_INA_GPIO_Port GPIOB
#define SPI2_MISO_INA_Pin GPIO_PIN_14
#define SPI2_MISO_INA_GPIO_Port GPIOB
#define SPI2_MOSI_INA_Pin GPIO_PIN_15
#define SPI2_MOSI_INA_GPIO_Port GPIOB
#define LED_RED_ALARM_Pin GPIO_PIN_8
#define LED_RED_ALARM_GPIO_Port GPIOD
#define SPI2_NSS_INA_Pin GPIO_PIN_9
#define SPI2_NSS_INA_GPIO_Port GPIOD
#define ALARM_INA_Pin GPIO_PIN_10
#define ALARM_INA_GPIO_Port GPIOD
#define ALARM_INA_EXTI_IRQn EXTI15_10_IRQn
#define TURN_ON_PRECHARGE_Pin GPIO_PIN_11
#define TURN_ON_PRECHARGE_GPIO_Port GPIOD
#define CAN_PWR_EN_Pin GPIO_PIN_12
#define CAN_PWR_EN_GPIO_Port GPIOC
#define LED_ALARM_NC3_Pin GPIO_PIN_2
#define LED_ALARM_NC3_GPIO_Port GPIOD
#define LED_DISCHARGE_NC1_Pin GPIO_PIN_3
#define LED_DISCHARGE_NC1_GPIO_Port GPIOD
#define USART2_DE_Pin GPIO_PIN_4
#define USART2_DE_GPIO_Port GPIOD
#define LED_GREEN_BUZEER_Pin GPIO_PIN_6
#define LED_GREEN_BUZEER_GPIO_Port GPIOB
#define LED_CHARGE_NC2_Pin GPIO_PIN_7
#define LED_CHARGE_NC2_GPIO_Port GPIOB
#define TURN_OFF_DISCHARGE_Pin GPIO_PIN_0
#define TURN_OFF_DISCHARGE_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define COMMAND_TYPE_ADDRES 15
#define COMMAND_DATA_ADDRES 17

/*
#define MAX_CELL_TEMPERATURE_DEFAULT 60
#define MIN_CELL_TEMPERATURE_DEFAULT -20
#define MAX_CHARGE_CURRENT_DEFAULT 5000
#define MAX_DISCHARGE_CURRENT_DEFAULT 10000
#define MAX_CELL_VOLTAGE_DEFAULT 3600
#define MIN_CELL_VOLTAGE_DEFAULT 2500
#define MAX_BATTERY_VOLTAGE_DEFAULT 48000
#define MIN_BATTERY_VOLTAGE_DEFAULT 36000
*/

#define BATTERY_CAPACITY_DEFAULT 10000 // 10Ah
#define RS485_RX_MAX_FRAME_LENGTH 255

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
