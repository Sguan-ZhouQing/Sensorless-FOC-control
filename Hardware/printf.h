/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:43:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-12 23:56:33
 * @FilePath: \demo_STM32F103FocCode\Hardware\printf.h
 * @Description: 
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#ifndef __PRINTF_H
#define __PRINTF_H

#include "stm32f1xx_hal.h"

#define f103_BUFFER_SIZE 256
extern uint8_t DataBuff[200];
extern uint16_t RxLine;

void UART_Init(void);
void UART_SendFloats(float *data, uint8_t count, uint8_t decimal_places);
float Get_Data(void);
void PID_Adjust(uint8_t Motor_n);


#endif // PRINTF_H

