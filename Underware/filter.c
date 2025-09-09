/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:27:38
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-09 17:05:48
 * @FilePath: \demo_STM32F103FocCode\Underware\filter.c
 * @Description: FOC底层的滤波函数代码编写
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "filter.h"

/**
 * @description: 低通滤波
 * @param {float} input
 * @param {float} last_output
 * @param {float} alpha
 * @return {*}
 */
float low_pass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1.0 - alpha) * last_output;
}

/**
 * @description: 一阶卡尔曼滤波
 * @param {float} input
 * @param {float} r
 * @param {float} q
 * @return {*}
 */
float kalman_filter_std(float input, float r, float q)
{
    static float z;
    static float p = 1;
    float g = 0;
    p = p + q;
    g = p / (p + r);
    z = z + g * (input - z);
    p = (1 - g) * p;
    return z;
}


/**
 * @description: 低通滤波的使用方法
 * @return {*}
 */
// // 全局或静态变量，用于保存上一次的滤波结果
// static float filtered_value = 0.0f;
// // 滤波系数，根据实际需求调整
// // 常用范围: 0.01 - 0.3 (时间常数大，平滑好) 或 0.5 - 0.9 (响应快)
// #define ALPHA 0.2f
// void main_loop(void){
//     while(1) {
//         // 读取原始传感器数据
//         float raw_data = read_sensor();
//         // 应用低通滤波
//         filtered_value = low_pass_filter(raw_data, filtered_value, ALPHA);
//         // 使用滤波后的值
//         control_motor(filtered_value);
//         delay_ms(10); // 根据采样周期调整
//     }
// }

/**
 * @description: 一阶卡尔曼滤波的使用方式
 * @return {*}
 */
// // 定义噪声参数 - 需要根据实际系统调试
// #define MEASUREMENT_NOISE 10.0f    // R值，传感器噪声大则设大
// #define PROCESS_NOISE     0.001f   // Q值，系统变化快则设大
// void main_loop(void){
//     while(1) {
//         // 读取原始传感器数据
//         float raw_data = read_sensor();
//         // 应用卡尔曼滤波
//         float filtered_data = kalman_filter_std(raw_data, MEASUREMENT_NOISE, PROCESS_NOISE);
//         // 使用滤波后的值
//         control_motor(filtered_data);
//         delay_ms(10); // 根据采样周期调整
//     }
// }