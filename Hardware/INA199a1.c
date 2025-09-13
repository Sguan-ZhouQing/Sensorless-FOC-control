/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:18:33
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-13 16:25:12
 * @FilePath: \demo_STM32F103FocCode\Hardware\INA199a1.c
 * @Description: FOC硬件层的电流传感器代码开发
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "INA199a1.h"
#include "adc.h"
#include "dma.h"
#include <math.h>

// ADC采样缓冲区（2个通道）
static uint16_t adcBuffer[ADC_CHANNEL_COUNT];
// 电压参考值（STM32F103为3.3V）
#define VREF 3.3f
// ADC分辨率（12位 = 4096）
#define ADC_RESOLUTION 4096.0f
// DMA传输完成标志
static volatile uint8_t dmaTransferComplete = 0;


// 初始化ADC并启动DMA传输
void INA199A1_Init(void) {
    // 确保DMA和ADC已初始化
    // MX_DMA_Init();
    // MX_ADC1_Init();
    
    // 关闭DMA中断函数
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    // 仅启用ADC的DMA传输
    INA199A1_Start();
}

// 启动ADC DMA传输
void INA199A1_Start(void) {
    dmaTransferComplete = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_CHANNEL_COUNT);
}

// 停止ADC DMA传输
void INA199A1_Stop(void) {
    HAL_ADC_Stop_DMA(&hadc1);
}


/**
  * @brief  获取指定通道的原始ADC值
  * @param  channel: 通道编号 (0 或 1)
  * @retval 原始ADC值 (0-4095)
  */
uint16_t INA199A1_GetRawValue(uint8_t channel) {
    if (channel >= ADC_CHANNEL_COUNT) {
        return 0;
    }
    return adcBuffer[channel];
}


/**
  * @brief  获取指定通道的电压值
  * @param  channel: 通道编号 (0 或 1)
  * @retval 电压值 (0-3.3V)
  */
float INA199A1_GetVoltage(uint8_t channel) {
    if (channel >= ADC_CHANNEL_COUNT) {
        return 0.0f;
    }
    // 将原始值转换为电压：电压 = (原始值 / 4096) * 3.3V
    return (adcBuffer[channel] * VREF) / ADC_RESOLUTION;
}


/**
  * @brief  获取指定通道的平均电压值（多次采样取平均）
  * @param  channel: 通道编号 (0 或 1)
  * @param  sample_count: 采样次数
  * @retval 平均电压值
  */
float INA199A1_GetVoltageAverage(uint8_t channel, uint8_t sample_count) {
    if (channel >= ADC_CHANNEL_COUNT || sample_count == 0) {
        return 0.0f;
    }

    uint32_t sum = 0;
    for (uint8_t i = 0; i < sample_count; i++) {
        sum += adcBuffer[channel];
        HAL_Delay(1); // 短暂延迟，确保采样不同值
    }
    return (sum * VREF) / (ADC_RESOLUTION * sample_count);
}



// 检查DMA传输是否完成（1完成，0未完成）
uint8_t INA199A1_IsTransferComplete(void) {
    return dmaTransferComplete;
}

// 清除DMA传输完成标志
void INA199A1_ClearTransferFlag(void) {
    dmaTransferComplete = 0;
}


