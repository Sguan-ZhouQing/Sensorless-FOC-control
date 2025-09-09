/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:25:14
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-09 17:21:09
 * @FilePath: \demo_STM32F103FocCode\Software\Foc.c
 * @Description: FOC应用层代码开发
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Foc.h"

FOC_HandleTypeDef SguanFOC;

/**
 * @description: 初始化FOC控制器的底层硬件
 * @return {*}
 */
void FOC_Init(void)
{
    // 1.TIM时钟初始化
    HAL_TIM_Base_Start_IT(&htim1);
    Timer_Init();
    // 2.磁编码器初始化
    // 3.ADC相电流采样初始化
}

void FOC_PositionLoop(float rad)
{
    
}


