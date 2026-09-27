/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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


/* 函数声明 */
void itoa(int num, char* str);
void readMotorSingleCircleAngle(UART_HandleTypeDef *huart, uint8_t id);
void sendMotorStatusToSerial5(void);
void TEST_readMotorAngle(UART_HandleTypeDef *huart, uint8_t id);
void debug_motor_response(uint8_t *response, uint8_t id);
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define MOTOR_ID_1 0x01  // 电机1 ID
#define MOTOR_ID_2 0x02  // 电机2 ID

/* 电机命令定义 */
#define CMD_MOTOR_RUN      0x88  // 电机运行命令
#define CMD_SINGLE_CIRCLE_POSITION_CONTROL 0xA5  // 单圈位置闭环控制命令
#define CMD_READ_SINGLE_CIRCLE_ANGLE 0x94  // 读取单圈角度命令
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
