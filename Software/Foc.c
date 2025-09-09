/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:25:14
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-10 00:15:55
 * @FilePath: \demo_STM32F103FocCode\Software\Foc.c
 * @Description: FOC应用层代码开发
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Foc.h"

#define SguanFOC_ARR 2000   // 磁定向控制之中PWM份额值
#define Pole_Pairs 7        //电机的极对极数（通常为7）
#define Dead_Time 0.01f     //死区时间限幅（Low-High）
FOC_HandleTypeDef SguanFOC;


// 初始化FOC控制器的底层硬件
void FOC_Init(void) {
    // 初始化FOC结构体
    memset(&SguanFOC, 0, sizeof(FOC_HandleTypeDef));
    SguanFOC.u_d = 0.0f;     // d轴电压设为0
    SguanFOC.u_q = 0.3f;     // q轴电压（控制转矩，0.3是合理的启动值）
    
    // 1.硬件初始化
    HAL_TIM_Base_Start_IT(&htim1);
    Timer_Init();
    // 2.磁编码器初始化
    // 3.ADC相电流采样初始化
    // 4.串口初始化
    UART_Init();
    
    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}


// 添加constrain函数实现
static float constrain(float value, float min_val, float max_val) {
    if (value < min_val) {
        return min_val;
    } else if (value > max_val) {
        return max_val;
    } else {
        return value;
    }
}


// 设置三相PWM占空比
static void set_pwm_duty_cycle(float duty_a, float duty_b, float duty_c) {
    // 限制占空比在安全范围内
    duty_a = constrain(duty_a, Dead_Time, (1.0f - Dead_Time));
    duty_b = constrain(duty_b, Dead_Time, (1.0f - Dead_Time));
    duty_c = constrain(duty_c, Dead_Time, (1.0f - Dead_Time));
    
    uint16_t ccr_a = (uint16_t)(duty_a * SguanFOC_ARR);
    uint16_t ccr_b = (uint16_t)(duty_b * SguanFOC_ARR);
    uint16_t ccr_c = (uint16_t)(duty_c * SguanFOC_ARR);
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}


// 生成SVPWM波
static void generate_svpwm_waveforms(void) {
    ipark(&SguanFOC);      // 逆Park变换
    svpwm(&SguanFOC);      // SVPWM计算
    set_pwm_duty_cycle(SguanFOC.t_a, SguanFOC.t_b, SguanFOC.t_c); // 设置PWM
}


/**
 * @description: [Mode1]FOC开环位置控制
 * @param {float} angle_deg 机械角度度数(0到359度)
 * @param {int} pole_pairs 电机极对数
 * @param {float} voltage 电压幅值(0-1)
 */
void FOC_OpenPosition_Loop(float angle_deg, float voltage)
{
    // 1. 机械角度转电角度
    float mechanical_angle = (angle_deg / 360.0f) * 2.0f * PI;  // 机械角度转弧度
    SguanFOC.theta = mechanical_angle * Pole_Pairs;             // 机械角度转电角度
    
    // 2. 设置电压（控制转矩）
    SguanFOC.u_q = voltage;
    SguanFOC.u_d = 0.0f;
    
    // 3. 生成SVPWM波形
    generate_svpwm_waveforms();
}


// /**
//  * @description: [Mode2]FOC开环速度
//  * @param {float} speed 弧度每秒
//  * @return {*}
//  */
// void FOC_OpenSpeed_Loop(float speed)
// {
    
// }




