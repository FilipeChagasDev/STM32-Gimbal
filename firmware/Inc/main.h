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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stm32f1xx_hal_i2c.h>
#include <stm32f1xx_hal_uart.h>
#include <stm32f1xx_hal_tim.h>
#include <stm32f1xx_ll_tim.h>
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

// System
void init_system();
void error_alert(char *msg);

// Servomotors
void init_servomotors();
void init_servomotors_motion();
void servo_update();

// Sensor
void init_position_sensor();
void test_sensor();
void update_position_sensor();
void sensor_calc_position(float *pitch, float *roll);

// Smoothing
void change_smoothing(size_t value);

// Messaging
void init_uart_messaging();
void uart_message_update();

typedef enum {SERVO_ANGLE_PACK, SENSOR_ANGLE_PACK, OFFSET_ANGLE_PACK} angle_pack_target_t;
typedef enum {FACTOR_VALUE_PACK, SMOOTHING_VALUE_PACK} value_pack_target_t;
void send_value_pack(value_pack_target_t target, float value);
void send_angle_pack(angle_pack_target_t target, float pitch, float roll);
void send_echo_pack(char *msg);
void send_message_pack(char *msg);

// Mailbox
void init_mailbox();
void mailbox_message_received(char *message);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UPDATE_LED_Pin GPIO_PIN_0
#define UPDATE_LED_GPIO_Port GPIOA
#define ERROR_LED_Pin GPIO_PIN_1
#define ERROR_LED_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
