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
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


extern osMessageQueueId_t ebs_sm_event_queue_Handle;
extern osTimerId_t watchdog_timer_Handle;
extern osTimerId_t timeoutTimerHandle;
extern osMutexId_t ebs_status_can_message_mutex_Handle;


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

extern osSemaphoreId_t angleDataAvailableSemaphoreHandle;
extern osSemaphoreId_t angleBufferAvailableSemaphoreHandle;
extern osSemaphoreId_t BrakeNotActiveSemaphoreHandle;
extern osSemaphoreId_t pressureDmaCpltSemaphoreHandle;
extern CAN_HandleTypeDef hcan1;
extern osThreadId_t can_tx_Handle;
extern osMutexId_t can_message_mutex_Handle;

extern osThreadId_t angle_sensors_Handle;


typedef struct{
	uint8_t id;
	uint8_t status;
	uint16_t pressure;
}tank_t;

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
#define SDC_TO_REAR_Pin GPIO_PIN_2
#define SDC_TO_REAR_GPIO_Port GPIOE
#define SDC_FROM_REAR_Pin GPIO_PIN_3
#define SDC_FROM_REAR_GPIO_Port GPIOE
#define RELEASE_BRAKE_1_Pin GPIO_PIN_4
#define RELEASE_BRAKE_1_GPIO_Port GPIOE
#define RELEASE_BRAKE_2_Pin GPIO_PIN_5
#define RELEASE_BRAKE_2_GPIO_Port GPIOE
#define PS1_Pin GPIO_PIN_0
#define PS1_GPIO_Port GPIOC
#define TANK_1_ANALOG_Pin GPIO_PIN_1
#define TANK_1_ANALOG_GPIO_Port GPIOC
#define PS2_Pin GPIO_PIN_2
#define PS2_GPIO_Port GPIOC
#define TANK_2_ANALOG_Pin GPIO_PIN_3
#define TANK_2_ANALOG_GPIO_Port GPIOC
#define APPS_1_Pin GPIO_PIN_0
#define APPS_1_GPIO_Port GPIOA
#define APPS_2_Pin GPIO_PIN_1
#define APPS_2_GPIO_Port GPIOA
#define BPPS_1_Pin GPIO_PIN_2
#define BPPS_1_GPIO_Port GPIOA
#define BPPS_2_Pin GPIO_PIN_3
#define BPPS_2_GPIO_Port GPIOA
#define BRAKE_PRESSURE_1_Pin GPIO_PIN_4
#define BRAKE_PRESSURE_1_GPIO_Port GPIOA
#define BRAKE_PRESSURE_2_Pin GPIO_PIN_5
#define BRAKE_PRESSURE_2_GPIO_Port GPIOA
#define STEERING_ANGLE_1_Pin GPIO_PIN_6
#define STEERING_ANGLE_1_GPIO_Port GPIOA
#define STEERING_ANGLE_2_Pin GPIO_PIN_7
#define STEERING_ANGLE_2_GPIO_Port GPIOA
#define SYSTEM_OK_LED_Pin GPIO_PIN_9
#define SYSTEM_OK_LED_GPIO_Port GPIOE
#define SYSTEM_FAULT_LED_Pin GPIO_PIN_11
#define SYSTEM_FAULT_LED_GPIO_Port GPIOE
#define SDC_STATUS_LED_Pin GPIO_PIN_13
#define SDC_STATUS_LED_GPIO_Port GPIOE
#define GENERAL_LED_Pin GPIO_PIN_14
#define GENERAL_LED_GPIO_Port GPIOE
#define WATCHDOG_Pin GPIO_PIN_9
#define WATCHDOG_GPIO_Port GPIOB
#define WATCHDOG_READY_Pin GPIO_PIN_0
#define WATCHDOG_READY_GPIO_Port GPIOE
#define CLOSE_SDC_Pin GPIO_PIN_1
#define CLOSE_SDC_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* Tank Status Bitmask */

#define MAX_ITERATOR 		100

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
