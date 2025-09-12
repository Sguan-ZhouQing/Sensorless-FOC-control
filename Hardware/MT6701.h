/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:19:39
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-12 16:53:48
 * @FilePath: \demo_STM32F103FocCode\Hardware\MT6701.h
 * @Description: 
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#ifndef __MT6701_H
#define __MT6701_H

#include "stm32f1xx_hal.h"
#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>
#include "fast_sin.h"
#include "filter.h"

// MT6701相关定义
#define MT6701_SPI_HANDLE    hspi1
#define MT6701_CS_GPIO       GPIOA
#define MT6701_CS_PIN        GPIO_PIN_4

// 角度相关定义
#define MT6701_ANGLE_MAX     0x3FFF  // 14位最大角度值
#define MT6701_ANGLE_RES     16384.0f // 14位分辨率

// 函数声明
void MT6701_Init(void);
bool MT6701_ReadAngle(float *angle_rad);
bool MT6701_ReadRawAngle(uint16_t *raw_angle);
uint8_t MT6701_CalculateCRC(uint32_t data);
void MT6701_SPI_CompleteCallback(SPI_HandleTypeDef *hspi);
void MT6701_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
bool MT6701_ReadMultiTurnAngle(float *angle_rad);
void MT6701_ResetMultiTurnCounter(void);
// 角速度检测
bool MT6701_CalculateAngularVelocity(float *angular_velocity_rads);
void MT6701_FilteredAngularVelocity(float *filtered_velocity_rads);

// 外部变量声明
extern int32_t mt6701_total_turns;
extern float mt6701_prev_angle_rad;
extern SPI_HandleTypeDef MT6701_SPI_HANDLE;


#endif // MT6701_H
