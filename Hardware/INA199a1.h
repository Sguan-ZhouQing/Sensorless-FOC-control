#ifndef __INA199A1_H
#define __INA199A1_H

#include "stm32f1xx_hal.h"

// ADC通道数量
#define ADC_CHANNEL_COUNT 2

// 外部声明ADC句柄（在adc.c中定义）
extern ADC_HandleTypeDef hadc1;

// 函数声明
void MY_ADC_Init(void);
void MY_ADC_Start(void);
void MY_ADC_Stop(void);
float MY_ADC_GetVoltage(uint8_t channel);
uint16_t MY_ADC_GetRawValue(uint8_t channel);
float MY_ADC_GetVoltageAverage(uint8_t channel, uint8_t sample_count);


#endif // INA199A1_H
