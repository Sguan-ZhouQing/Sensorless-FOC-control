#ifndef __FOC_H
#define __FOC_H

#include "stm32f1xx_hal.h"
#include <string.h>  // 添加这行，用于memset函数
#include <math.h>    // 添加这行，用于fabsf等数学函数
#include "Timer.h"
#include "fast_sin.h"
#include "svpwm.h"
#include "INA199a1.h"
#include "MT6701.h"
#include "printf.h"

extern FOC_HandleTypeDef SguanFOC;

void FOC_Init(void);
void FOC_OpenPosition_Loop(float angle_deg, float voltage);
void FOC_OpenVelocity_Loop(float velocity_rad_s, float voltage);
float FOC_Calculate_Iq(void);


#endif // FOC_H
