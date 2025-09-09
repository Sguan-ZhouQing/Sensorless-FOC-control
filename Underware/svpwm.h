#ifndef __SVPWM_H
#define __SVPWM_H

#include <math.h>
#include <stdint.h>
#include "fast_sin.h"

/**
 * @description: FOC控制系统核心数据结构
 * 包含所有输入、输出和中间变量，方便在函数间传递数据
 */
typedef struct {
    /* 【输入】目标电压和角度（来自PID控制器） */
    float u_d;       // d轴电压指令（励磁分量，通常控制为0）
    float u_q;       // q轴电压指令（转矩分量，控制电机扭矩）
    float theta;     // 转子电角度（来自编码器或观测器）

    /* 【中间变量】α-β坐标系电压（逆Park变换结果） */
    float u_alpha;   // α轴电压
    float u_beta;    // β轴电压

    /* 【输出】三相PWM占空比（SVPWM计算结果） */
    float t_a;       // A相PWM占空比
    float t_b;       // B相PWM占空比
    float t_c;       // C相PWM占空比

    /* 【输入】三相采样电流（来自ADC） */
    float i_a;       // A相电流
    float i_b;       // B相电流
    float i_c;       // C相电流

    /* 【中间变量】α-β坐标系电流（Clark变换结果） */
    float i_alpha;   // α轴电流
    float i_beta;    // β轴电流

    /* 【输出】d-q坐标系电流（Park变换结果，用于PID反馈） */
    float i_d;       // d轴电流（反馈给d轴PID）
    float i_q;       // q轴电流（反馈给q轴PID）

    /* 【中间变量】三角函数值（避免重复计算） */
    float sine;      // sin(theta)
    float cosine;    // cos(theta)
    float k_svpwm;   // SVPWM调制系数（超调时使用）
} FOC_HandleTypeDef;

// 函数声明
void ipark(FOC_HandleTypeDef* foc);    // 逆Park变换（计算三角函数）
void ipark2(FOC_HandleTypeDef* foc);   // 逆Park变换（复用三角函数）
void clarke(FOC_HandleTypeDef* foc);   // Clark变换
void park(FOC_HandleTypeDef* foc);     // Park变换  
void svpwm(FOC_HandleTypeDef* foc);    // SVPWM核心算法

#endif  //SVPWM_H
