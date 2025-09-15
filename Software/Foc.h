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


typedef struct {
    struct {
        float Kp;    // 比例系数
        float Ki;    // 积分系数
        float Kd;    // 微分系数
        float integral;      // 积分项累加值
        float prev_error;    // 上一次误差（用于微分项）
        float output_limit;  // 输出限制
    } position_pid;
} FOC_HandleTypeDef;


// FOC控制的结构体变量(声明)
extern SVPWM_HandleTypeDef SguanSVPWM;
extern FOC_HandleTypeDef SguanFOC;

// 磁定向控制FOC函数声明汇总
void FOC_Init(void);
void FOC_EncoderAlignment(void);
float FOC_Calculate_Iq(void);
// 开环运行（位置环，速度环，电流环）
void FOC_OpenPosition_Loop(float angle_deg, float voltage);
void FOC_OpenVelocity_Loop(float velocity_rad_s, float voltage);
void FOC_OpenCurrent_Loop(float current_desired);
// 闭环运行(位置单环，速度单环，电流单环，三环串级PID)
void FOC_SetPositionPID(float Kp, float Ki, float Kd, float output_limit);
void FOC_CloseAbsolutePos_Loop(float target_angle_rad, float voltage);


#endif // FOC_H
