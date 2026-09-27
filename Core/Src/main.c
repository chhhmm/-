/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>  //
#include <math.h>
#include "stdlib.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 电机控制参数定义 */
#define MOTOR_ID_1 0x01  // 电机1 ID
#define MOTOR_ID_2 0x02  // 电机2 ID

/* 电机命令定义 */
#define CMD_MOTOR_RUN      0x88  // 电机运行命令
#define CMD_SINGLE_CIRCLE_POSITION_CONTROL 0xA5  // 单圈位置闭环控制命令
#define CMD_READ_SINGLE_CIRCLE_ANGLE 0x94  // 读取单圈角度命令
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char debugBuffer[32] = {0};  // 用于存储待发送字符串
 char space = ' ';
volatile uint8_t needSend = 0;  // 发送标志
uint32_t lastInterruptTime = 0;
uint32_t interruptCount = 0;
// 串口5接收相关变量
/*uint8_t uart5RxBuffer[20];
uint8_t uart5RxIndex = 0;
uint8_t newDataAvailable = 0;
uint8_t rxData;  // 用于HAL_UART_Receive_IT*/
uint8_t uart5RxBuffer[6];  // 只需要6字节缓冲区
uint8_t newDataAvailable = 0;
/* 全局变量 - 用于存储电机角度（单位：0.01°） */
volatile uint16_t motor1_angle = 30;  // 10° * 100 = 1000
volatile uint16_t motor2_angle = 300;  // 10° * 100 = 1000
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* 发送电机命令函数 */
void sendMotorCommand(UART_HandleTypeDef *huart, uint8_t command, uint8_t id, uint8_t dataLength, uint8_t *data) {
    uint8_t frame[10]; // 最大帧长度
    uint8_t checksum = 0;
    
    // 帧头
    frame[0] = 0x3E;
    // 命令
    frame[1] = command;
    // ID
    frame[2] = id;
    // 数据长度
    frame[3] = dataLength;
    
    // 计算帧命令校验和
    for (int i = 0; i < 4; i++) {
        checksum += frame[i];
    }
    frame[4] = checksum & 0xFF;
    
    // 复制数据
    for (int i = 0; i < dataLength; i++) {
        frame[5 + i] = data[i];
    }
    
    // 计算帧数据校验和
    checksum = 0;
    for (int i = 0; i < dataLength; i++) {
        checksum += data[i];
    }
    if (dataLength > 0) {
        frame[5 + dataLength] = checksum & 0xFF;
    }
    
    // 发送帧
    HAL_UART_Transmit(huart, frame, 5 + dataLength + (dataLength > 0 ? 1 : 0), HAL_MAX_DELAY);
}

/* 设置单圈位置闭环控制（10°） */
void setSingleCirclePositionControl(UART_HandleTypeDef *huart, uint8_t id, uint16_t positionDegrees) {
    // 10° = 10 * 100 = 1000 LSB
    uint16_t positionLSB = positionDegrees * 100;
    
    // 位置控制数据
    uint8_t data[4];
    data[0] = 0x00; // 顺时针方向
    data[1] = positionLSB & 0xFF; // 低字节
    data[2] = (positionLSB >> 8) & 0xFF; // 高字节
    data[3] = 0x00; // NULL
    
    sendMotorCommand(huart, CMD_SINGLE_CIRCLE_POSITION_CONTROL, id, 4, data);
}


/* 电机上电自我校准（仅在上电时执行一次） */
void motorSelfCalibration() {
    // 电机1上电自我校准
    uint8_t runCommand1[5] = {0x3E, CMD_MOTOR_RUN, MOTOR_ID_1, 0x00, 0x00};
    uint8_t checksum1 = 0;
    for (int i = 0; i < 4; i++) {
        checksum1 += runCommand1[i];
    }
    runCommand1[4] = checksum1 & 0xFF;
    HAL_UART_Transmit(&huart2, runCommand1, 5, HAL_MAX_DELAY);
    
    // 等待电机1启动
    HAL_Delay(100);
    
    // 设置电机1默认位置为10°
    setSingleCirclePositionControl(&huart2, MOTOR_ID_1, 10);
    
    // 电机2上电自我校准
    uint8_t runCommand2[5] = {0x3E, CMD_MOTOR_RUN, MOTOR_ID_2, 0x00, 0x00};
    uint8_t checksum2 = 0;
    for (int i = 0; i < 4; i++) {
        checksum2 += runCommand2[i];
    }
    runCommand2[4] = checksum2 & 0xFF;
    HAL_UART_Transmit(&huart3, runCommand2, 5, HAL_MAX_DELAY);
    
    // 等待电机2启动
    HAL_Delay(100);
    
    // 设置电机2默认位置为10°
    setSingleCirclePositionControl(&huart3, MOTOR_ID_2, 10);
}



// 计算校验和（前n个字节）
uint8_t calculateChecksum(uint8_t *data, uint8_t length) {
    uint8_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

// 通过串口5打印字符串
void printToUART5(const char *str) {
    HAL_UART_Transmit(&huart5, (uint8_t*)str, strlen(str), 100);
}
//读取单圈角度
float readSingleCircleAngle(UART_HandleTypeDef *huart, uint8_t motorID) {
    uint8_t commandFrame[5] = {0x3E, 0x94, motorID, 0x00, 0x00};
    uint8_t responseFrame[10];
    uint32_t rawAngle = 0;
    float angle = -1.0f;
    
    // 计算校验和（前4字节）
    commandFrame[4] = 0;
    for(int i = 0; i < 4; i++) {
        commandFrame[4] += commandFrame[i];
    }
    
    // 发送命令
    HAL_UART_Transmit(huart, commandFrame, 5, 100);
    
    // 关键：100us以上延时（收发切换）
    for(volatile uint32_t i = 0; i < 200; i++) {  // ~200us延时
        __NOP();
        __NOP();
        __NOP();
    }
    
    // 接收响应（10字节）
    if(HAL_UART_Receive(huart, responseFrame, 10, 200) == HAL_OK) {
        // 直接解析角度：第5-8字节（小端格式）
        rawAngle = responseFrame[5] | 
                  (responseFrame[6] << 8) | 
                  (responseFrame[7] << 16) | 
                  (responseFrame[8] << 24);
        
        angle = rawAngle / 100.0f;
    }
    
    return angle;
}

// 读取并打印电机角度
void processMotorReading(void) {
    float angle1, angle2;
    char buffer[100];
    
    // 读取电机1
    printToUART5("\r\n--- Reading Motor 1 ---\r\n");
    angle1 = readSingleCircleAngle(&huart2, MOTOR_ID_1);
    HAL_Delay(10);  // 电机响应间隔
    
    // 读取电机2
    printToUART5("\r\n--- Reading Motor 2 ---\r\n");
    angle2 = readSingleCircleAngle(&huart3, MOTOR_ID_2);
    
    // 格式化输出
    if (angle1 >= 0 && angle2 >= 0) {
        snprintf(buffer, sizeof(buffer), "\r\n=== SUCCESS ===\r\nMotor 1: %.2f°\r\nMotor 2: %.2f°\r\n", angle1, angle2);
    } else {
        snprintf(buffer, sizeof(buffer), "\r\n=== ERROR ===\r\nMotor 1: %.2f°\r\nMotor 2: %.2f°\r\n", 
                 angle1 >= 0 ? angle1 : -1.0f, 
                 angle2 >= 0 ? angle2 : -1.0f);
    }
    
    // 通过串口5输出
    printToUART5(buffer);
    printToUART5("\r\n=== Reading Complete ===\r\n");
}

/**
  * @brief  将电机角度映射到-1000~+1000范围并通过串口5发送
  * @param  huart1: USART1句柄（用于Motor1）
  * @param  huart2: USART2句柄（用于Motor2）
  * @retval None
  */
/*
void sendMappedMotorAngles(UART_HandleTypeDef *huart5) {
   float angle1 = -1.0f;
    float angle2 = -1.0f;
    int16_t mappedAngle1 = -2000;  // -2000表示读取失败
    int16_t mappedAngle2 = -2000;
    uint8_t sendData[6];
    
    // 1. 读取电机1
    angle1 = readSingleCircleAngle(&huart2, MOTOR_ID_1);
    
    // 2. 关键延时
    HAL_Delay(10);
    
    // 3. 读取电机2
    angle2 = readSingleCircleAngle(&huart3, MOTOR_ID_2);
    
    // 4. 映射Motor1: 300-360° & 0-40° -> -1000~+1000 (正常方向)
    if (angle1 >= 0) {
        if (angle1 >= 300.0f && angle1 <= 360.0f) {
            // 300-360° -> -1000~0 (300°=-1000, 360°=0)
            mappedAngle1 = (int16_t)((angle1 - 300.0f) * (-1000.0f / 60.0f));
        } else if (angle1 >= 0.0f && angle1 <= 40.0f) {
            // 0-40° -> 0~+1000 (0°=0, 40°=+1000)
            mappedAngle1 = (int16_t)(angle1 * (1000.0f / 40.0f));
        } else {
            // 超出有效范围，保持边界值
            if (angle1 < 300.0f) {
                mappedAngle1 = -1000;  // 小于300度，保持-1000
            } else {
                mappedAngle1 = 1000;   // 大于40度，保持+1000
            }
        }
    }
    
    // 5. 映射Motor2: 330-360° & 0-45° -> +1000~0 & 0~-1000 (反转方向) ??
    if (angle2 >= 0) {
        if (angle2 >= 330.0f && angle2 <= 360.0f) {
            // 330-360° -> +1000~0 (330°=+1000, 360°=0) ?? 方向反转
            mappedAngle2 = (int16_t)((angle2 - 330.0f) * (-1000.0f / 30.0f) + 1000.0f);
        } else if (angle2 >= 0.0f && angle2 <= 45.0f) {
            // 0-45° -> 0~-1000 (0°=0, 45°=-1000) ?? 方向反转
            mappedAngle2 = (int16_t)(angle2 * (-1000.0f / 45.0f));
        } else {
            // 超出有效范围，保持边界值（方向已反转）
            if (angle2 < 330.0f) {
                mappedAngle2 = 1000;   // 小于330度，保持+1000（反转后）
            } else {
                mappedAngle2 = -1000;  // 大于45度，保持-1000（反转后）
            }
        }
    }
    
    // 6. 构建发送数据包
    sendData[0] = 0x02;  // 起始符
    sendData[1] = (uint8_t)((mappedAngle1 >> 8) & 0xFF);  // M1高字节
    sendData[2] = (uint8_t)(mappedAngle1 & 0xFF);         // M1低字节
    sendData[3] = (uint8_t)((mappedAngle2 >> 8) & 0xFF);  // M2高字节  
    sendData[4] = (uint8_t)(mappedAngle2 & 0xFF);         // M2低字节
    sendData[5] = 0x03;  // 结束符
    
    // 7. 通过串口5发送
    HAL_UART_Transmit(huart5, sendData, 6, 100);
    
 // 8. 调试输出
    char debug[250];
    snprintf(debug, sizeof(debug), 
             "\r\n--- MAPPED DATA (Motor2 Direction Fixed) ---\r\n"
             "Motor1 (%.2f°): %d  [300-360°: -1000~0, 0-40°: 0~+1000]\r\n"
             "Motor2 (%.2f°): %d  [330-360°: +1000~0, 0-45°: 0~-1000] ?? FIXED\r\n"
             "TX: %02X %02X %02X %02X %02X %02X\r\n",
             angle1, mappedAngle1,
             angle2, mappedAngle2,
             sendData[0], sendData[1], sendData[2], sendData[3], sendData[4], sendData[5]);
    printToUART5(debug);
		
}
*/

void sendMappedMotorAngles(UART_HandleTypeDef *huart5) {
    float angle1 = -1.0f;
    float angle2 = -1.0f;
    int16_t mappedAngle1 = -2000;  // -2000表示读取失败
    int16_t mappedAngle2 = -2000;
    uint8_t sendData[6];
    
    // 1. 读取电机1
    angle1 = readSingleCircleAngle(&huart2, MOTOR_ID_1);
    
    // 2. 增加关键延时
    HAL_Delay(20);
    
    // 3. 读取电机2
    angle2 = readSingleCircleAngle(&huart3, MOTOR_ID_2);
	
	
    
    // 4. 映射Motor1: 300-360° & 0-40° -> -1000~+1000 (正常方向)
   /* if (angle1 >= 0) {
        if (angle1 >= 300.0f && angle1 <= 360.0f) {
            // 300-360° -> -1000~0 (300°=-1000, 360°=0)
            mappedAngle1 = (int16_t)((angle1 - 300.0f) * (-1000.0f / 60.0f));
        } else if (angle1 >= 0.0f && angle1 <= 40.0f) {
            // 0-40° -> 0~+1000 (0°=0, 40°=+1000)
            mappedAngle1 = (int16_t)(angle1 * (1000.0f / 40.0f));
        } else {
            // 超出有效范围，保持边界值
            if (angle1 < 300.0f) {
                mappedAngle1 = -1000;  // 小于300度，保持-1000
            } else {
                mappedAngle1 = 1000;   // 大于40度，保持+1000
            }
        }
    }*/
		if (angle1 >= 0) {
    if (angle1 >= 300.0f && angle1 <= 360.0f) {
        // 修正：300°=-1000, 360°=0
        mappedAngle1 = (int16_t)((angle1 - 360.0f) * (1000.0f / 60.0f));
        // 验证：300°=(300-360)×(1000/60)=-60×16.67=-1000 
        //       360°=(360-360)×(1000/60)=0×16.67=0 
    } else if (angle1 >= 0.0f && angle1 <= 40.0f) {
        //  0-40° -> 0~+1000 (0°=0, 40°=+1000) 
        mappedAngle1 = (int16_t)(angle1 * (1000.0f / 40.0f));
    } else {
        // 超出有效范围，保持边界值
        if (angle1 < 300.0f) {
            mappedAngle1 = -1000;  // 小于300度，保持-1000
        } else {
            mappedAngle1 = 1000;   // 大于40度，保持+1000
        }
    }
}
    
    // 5. 映射Motor2: 330-360° & 0-45° -> +1000~0 & 0~-1000 (反转方向)
    if (angle2 >= 0) {
        if (angle2 >= 330.0f && angle2 <= 360.0f) {
            // 330-360° -> +1000~0 (330°=+1000, 360°=0) 方向反转
            mappedAngle2 = (int16_t)((angle2 - 330.0f) * (-1000.0f / 30.0f) + 1000.0f);
        } else if (angle2 >= 0.0f && angle2 <= 45.0f) {
            // 0-45° -> 0~-1000 (0°=0, 45°=-1000) 方向反转
            //mappedAngle2 = (int16_t)(angle2 * (-1000.0f / 45.0f));
						mappedAngle2=0;
        } else {
            // 超出有效范围，保持边界值
            if (angle2 < 330.0f) {
                mappedAngle2 = 1000;   // 小于330度，保持+1000
            } else {
                //mappedAngle2 = -1000;  // 大于45度，保持-1000
							mappedAngle2 = 0;
            }
        }
    }
    
    // 6. 构建发送数据包
    sendData[0] = 0x02;  // 起始符
    sendData[1] = (uint8_t)((mappedAngle1 >> 8) & 0xFF);  // M1高字节
    sendData[2] = (uint8_t)(mappedAngle1 & 0xFF);         // M1低字节
    sendData[3] = (uint8_t)((mappedAngle2 >> 8) & 0xFF);  // M2高字节  
    sendData[4] = (uint8_t)(mappedAngle2 & 0xFF);         // M2低字节
    sendData[5] = 0x03;  // 结束符
		
	  /*char debugPacket[100];
    snprintf(debugPacket, sizeof(debugPacket), 
             "\r\n=== TX PACKET ===\r\n"
             "Data: 02 %02X %02X %02X %02X 03\r\n"
             "M1 Value: %d (0x%04X)\r\n"
             "M2 Value: %d (0x%04X)\r\n", 
             sendData[1], sendData[2], sendData[3], sendData[4],
             mappedAngle1, mappedAngle1 & 0xFFFF,
             mappedAngle2, mappedAngle2 & 0xFFFF);
    HAL_UART_Transmit(huart5, (uint8_t*)debugPacket, strlen(debugPacket), 100);*/
    
    // 7. 等待串口空闲
    while (HAL_UART_GetState(huart5) != HAL_UART_STATE_READY) {
        // 等待串口发送完成
    }
    
    // 8. 直接发送原始数据包（02 00 2E FC C0 03 格式）
    HAL_UART_Transmit(huart5, sendData, 6, 100);
    
    // 9. 增加总延时，确保稳定
    HAL_Delay(30);
}

/**
  * @brief  处理接收到的串口5数据，控制电机转动
  * @note   此函数应在主循环中调用，处理中断接收到的数据
  * @retval None
  */
/*void processReceivedMotorCommand(void) {
    if (!newDataAvailable) {
        return;
    }
    
    newDataAvailable = 0;
    
    uint8_t parsedData[6] = {0};
    uint8_t byteIndex = 0;
    uint8_t charIndex = 0;
    
    // 调试：显示接收到的原始数据
    char debugRaw[30];
    snprintf(debugRaw, sizeof(debugRaw), "RX: [%s]\r\n", uart5RxBuffer);
    HAL_UART_Transmit(&huart5, (uint8_t*)debugRaw, strlen(debugRaw), 100);
    
    // 解析十六进制数据
    while (byteIndex < 6 && charIndex < strlen((char*)uart5RxBuffer)) {
        // 跳过所有非十六进制字符
        while (charIndex < strlen((char*)uart5RxBuffer)) {
            char c = uart5RxBuffer[charIndex];
            if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
                break;
            }
            charIndex++;
        }
        
        if (charIndex >= strlen((char*)uart5RxBuffer) - 1) break;
        
        char c1 = uart5RxBuffer[charIndex];
        char c2 = uart5RxBuffer[charIndex + 1];
        
        // 安全的十六进制转换
        uint8_t highNibble = (c1 >= 'a') ? (c1 - 'a' + 10) : 
                           (c1 >= 'A') ? (c1 - 'A' + 10) : (c1 - '0');
        uint8_t lowNibble = (c2 >= 'a') ? (c2 - 'a' + 10) : 
                          (c2 >= 'A') ? (c2 - 'A' + 10) : (c2 - '0');
        
        parsedData[byteIndex++] = (highNibble << 4) | lowNibble;
        charIndex += 2;
    }
    
    // 显示解析结果
    char debugParse[50];
    snprintf(debugParse, sizeof(debugParse), "PARSED: ");
    HAL_UART_Transmit(&huart5, (uint8_t*)debugParse, strlen(debugParse), 100);
    
    for (int i = 0; i < 6; i++) {
        char byteStr[5];
        snprintf(byteStr, sizeof(byteStr), "%02X ", parsedData[i]);
        HAL_UART_Transmit(&huart5, (uint8_t*)byteStr, strlen(byteStr), 100);
    }
    HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, 100);
    
    // 提取电机控制值
    int16_t mappedAngle1 = (int16_t)((parsedData[1] << 8) | parsedData[2]);
    int16_t mappedAngle2 = (int16_t)((parsedData[3] << 8) | parsedData[4]);
    
    // 反向映射Motor1
    float targetAngle1 = -1.0f;
    if (mappedAngle1 >= -1000 && mappedAngle1 <= 1000) {
        if (mappedAngle1 <= 0) {
            targetAngle1 = 300.0f + (mappedAngle1 * (60.0f / -1000.0f));
        } else {
            targetAngle1 = mappedAngle1 * (40.0f / 1000.0f);
        }
    }
    
    // 反向映射Motor2
    float targetAngle2 = -1.0f;
    if (mappedAngle2 >= -1000 && mappedAngle2 <= 1000) {
        if (mappedAngle2 >= 0) {
            targetAngle2 = 330.0f + ((1000.0f - mappedAngle2) * (30.0f / 1000.0f));
        } else {
            targetAngle2 = (0.0f - mappedAngle2) * (45.0f / 1000.0f);
        }
    }
    
    // 控制电机
    uint8_t motor1_success = 0;
    uint8_t motor2_success = 0;
    
    if (targetAngle1 >= 0) {
        setSingleCirclePositionControl(&huart2, MOTOR_ID_1, (uint16_t)targetAngle1);
        motor1_success = 1;
    }
    
    if (targetAngle2 >= 0) {
        setSingleCirclePositionControl(&huart3, MOTOR_ID_2, (uint16_t)targetAngle2);
        motor2_success = 1;
    }
    
    // 发送控制结果
    char result[60];
    snprintf(result, sizeof(result), "M1:%s(%.1f°) M2:%s(%.1f°)\r\n", 
            motor1_success ? "OK" : "FAIL", targetAngle1,
            motor2_success ? "OK" : "FAIL", targetAngle2);
    HAL_UART_Transmit(&huart5, (uint8_t*)result, strlen(result), 100);
}*/
void processReceivedMotorCommand(void) {
    if (!newDataAvailable) {
        return;
    }
    
    newDataAvailable = 0;
    
    // 直接提取角度值 - 无任何校验
    int16_t mappedAngle1 = (int16_t)((uart5RxBuffer[1] << 8) | uart5RxBuffer[2]);
    int16_t mappedAngle2 = (int16_t)((uart5RxBuffer[3] << 8) | uart5RxBuffer[4]);
    
    // 反向映射Motor1 - 无范围检查
    float targetAngle1 = 300.0f + (mappedAngle1 * (60.0f / -1000.0f));
    if (mappedAngle1 > 0) {
        targetAngle1 = mappedAngle1 * (40.0f / 1000.0f);
    }
    
    // 反向映射Motor2 - 无范围检查
    float targetAngle2 = 330.0f + ((1000.0f - (float)mappedAngle2) * (30.0f / 1000.0f));
    if (mappedAngle2 < 0) {
        targetAngle2 = (0.0f - (float)mappedAngle2) * (45.0f / 1000.0f);
    }
    
    // 直接控制电机 - 无条件执行
    setSingleCirclePositionControl(&huart2, MOTOR_ID_1, (uint16_t)targetAngle1);
    setSingleCirclePositionControl(&huart3, MOTOR_ID_2, (uint16_t)targetAngle2);
    
    // 可选：发送确认（可以完全移除）
    char ack[40];
    snprintf(ack, sizeof(ack), "M1:%.1f° M2:%.1f°\r\n", targetAngle1, targetAngle2);
    HAL_UART_Transmit(&huart5, (uint8_t*)ack, strlen(ack), 100);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);  // 优先级1（比0低）
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);  // 优先级1
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);  // 优先级0（最高）
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  MX_TIM6_Init();
  MX_UART5_Init();
	
 HAL_NVIC_SetPriority(UART5_IRQn, 2, 0);  // 设置UART5中断优先级
 HAL_NVIC_EnableIRQ(UART5_IRQn);          // 使能UART5中断
	
	HAL_TIM_Base_Start_IT(&htim6);  // 启动定时器中断
  /* USER CODE BEGIN 2 */
	
	  //  // ======== 串口5测试: 发送 "Hello World" ========
   //char hello[] = "Hello World from UART5!\r\n";
   //HAL_UART_Transmit(&huart5, (uint8_t*)hello, sizeof(hello)-1, HAL_MAX_DELAY);
    /* 电机上电自我校准 */
    motorSelfCalibration();
    HAL_Delay(100); 

// 启用UART5接收中断 - 直接接收6字节
//	HAL_UART_Receive_IT(&huart5, uart5RxBuffer, 6);

char initMsg[] = "RAW CONTROL MODE\r\n";
HAL_UART_Transmit(&huart5, (uint8_t*)initMsg, strlen(initMsg), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if (needSend) {
        needSend = 0;
     // 处理电机读取和打印
			sendMappedMotorAngles(&huart5);
     }
		
		 /* if (newDataAvailable) {
        newDataAvailable = 0;
	    
        processReceivedMotorCommand(); // 处理接收到的UART5数据
        
        // 在主循环中重新启动接收
        if (HAL_UART_GetState(&huart5) == HAL_UART_STATE_READY) {
            HAL_UART_Receive_IT(&huart5, uart5RxBuffer, 6);
        }
    }*/
		 

		//processReceivedMotorCommand();
		 
    HAL_Delay(10);  // 主循环延时
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
			// readAndPrintMotorAngles();
			
        needSend = 1;  // 设置标志，主循环会检测
        // 可选：调试用，确认中断是否触发
        // char debug[] = "TIM6 INT\r\n";
        //HAL_UART_Transmit(&huart5, (uint8_t*)debug, strlen(debug), 10);
    }
    /* USER CODE BEGIN Callback 1 */
    
    /* USER CODE END Callback 1 */
}

// 串口接收中断回调函数
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {  // 确保使用正确的串口实例
        uint8_t receivedChar = *(huart->pRxBuffPtr);  // 获取接收到的字符
        
        if (uart5RxIndex < sizeof(uart5RxBuffer) - 1) {
            uart5RxBuffer[uart5RxIndex++] = receivedChar;
            
            // 检测到行结束符或缓冲区满
            if (receivedChar == '\n' || receivedChar == '\r' || uart5RxIndex >= 18) {
                uart5RxBuffer[uart5RxIndex] = '\0';  // 确保字符串结束
                newDataAvailable = 1;
                uart5RxIndex = 0;
            }
        } else {
            // 缓冲区满，重置
            uart5RxIndex = 0;
        }
        
        // 重新启动中断接收
        uint8_t dummy;
        HAL_UART_Receive_IT(huart, &dummy, 1);
    }
}*/
// 串口中断回调函数 - 最简版本
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        newDataAvailable = 1;  // 标记新数据可用
        
        // 重新启动接收
      //  HAL_UART_Receive_IT(&huart5, uart5RxBuffer, 6);
    }
}




#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
