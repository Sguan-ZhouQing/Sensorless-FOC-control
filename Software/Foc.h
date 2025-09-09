/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:25:14
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-09 17:20:12
 * @FilePath: \demo_STM32F103FocCode\Software\Foc.h
 * @Description: 
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#ifndef __FOC_H
#define __FOC_H

#include "stm32f1xx_hal.h"
#include "Timer.h"
#include "fast_sin.h"
#include "svpwm.h"
#include "INA199a1.h"
#include "MT6701.h"

extern FOC_HandleTypeDef SguanFOC;

void FOC_Init(void);


#endif // FOC_H
