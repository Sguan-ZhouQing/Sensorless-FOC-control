#ifndef __INA199A1_H
#define __INA199A1_H

#include "stm32f1xx_hal.h"

// ADC通道数量
#define ADC_CHANNEL_COUNT 2

// 外部声明ADC句柄（在adc.c中定义）
extern ADC_HandleTypeDef hadc1;

// 函数声明
void INA199A1_Init(void);
void INA199A1_Start(void);
void INA199A1_Stop(void);
float INA199A1_GetVoltage(uint8_t channel);
uint16_t INA199A1_GetRawValue(uint8_t channel);
float INA199A1_GetVoltageAverage(uint8_t channel, uint8_t sample_count);
uint8_t INA199A1_IsTransferComplete(void);
void INA199A1_ClearTransferFlag(void);


#endif // INA199A1_H
