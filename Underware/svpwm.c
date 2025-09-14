/*
 * @Author: 星必尘Sguan
 * @Date: 2025-04-21 16:58:11
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-09 12:32:57
 * @FilePath: \demo_STM32F103FocCode\Underware\svpwm.c
 * @Description: 实现SVPWM波的生成;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "svpwm.h"


/**
 * @description: 【逆Park变换1】计算并更新u_alpha、u_beta，同时计算sinθ和cosθ
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：将旋转坐标系(dq)的电压转换回静止坐标系(αβ)
 */
void ipark(SVPWM_HandleTypeDef* foc) {
    // 计算当前角度的正弦和余弦值
    foc->sine = fast_sin(foc->theta);
    foc->cosine = fast_cos(foc->theta);
    
    // 逆Park变换公式：从dq坐标系转换到αβ坐标系
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_q * foc->cosine + foc->u_d * foc->sine;
}


/**
 * @description: 【逆Park变换2】仅计算u_alpha和u_beta，复用已有的sinθ和cosθ
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：与ipark()相同，但效率更高（避免重复计算三角函数）
 */
void ipark2(SVPWM_HandleTypeDef* foc) {
    // 直接使用预先计算好的三角函数值进行逆Park变换
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_q * foc->cosine + foc->u_d * foc->sine;
}


/**
 * @description: 【Clark变换】将三相电流(a-b-c)转换为两相静止坐标系(α-β)
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：简化三相系统为两相系统，便于后续处理
 */
void clarke(SVPWM_HandleTypeDef* foc) {
    // Clark变换公式（假设三相电流和为0：ia + ib + ic = 0）
    foc->i_alpha = foc->i_a;  // α轴电流等于A相电流
    foc->i_beta = (foc->i_a + 2 * foc->i_b) * 0.5773502691896257f;  // β轴电流 = (ia + 2*ib)/√3
}


/**
 * @description: 【Park变换】将静止坐标系(α-β)的电流转换到旋转坐标系(d-q)
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：将电流转换到随转子旋转的坐标系，便于解耦控制
 */
void park(SVPWM_HandleTypeDef* foc) {
    // 计算当前角度的正弦和余弦值
    foc->sine = fast_sin(foc->theta);
    foc->cosine = fast_cos(foc->theta);
    
    // Park变换公式：从αβ坐标系转换到dq坐标系
    foc->i_d = foc->i_alpha * foc->cosine + foc->i_beta * foc->sine;
    foc->i_q = foc->i_beta * foc->cosine - foc->i_alpha * foc->sine;
}


/**
 * @description: 【SVPWM核心算法】将α-β轴电压转换为三相PWM占空比
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：生成驱动三相逆变器的PWM信号，实现精确的电压矢量控制
 */
void svpwm(SVPWM_HandleTypeDef* foc) {
    const float ts = 1;  // PWM周期（归一化为1）
    // 步骤1：计算三个参考电压，用于扇区判断
    float u1 = foc->u_beta;  // Uβ
    float u2 = -0.8660254037844386f * foc->u_alpha - 0.5f * foc->u_beta;  // -√3/2 * Uα - 1/2 * Uβ
    float u3 = 0.8660254037844386f * foc->u_alpha - 0.5f * foc->u_beta;   // √3/2 * Uα - 1/2 * Uβ
    // 步骤2：通过符号判断法确定当前扇区（1-6）
    // 原理：根据u1、u2、u3的正负组合确定电压矢量所在的60°扇区
    uint8_t sector = (u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2);
    // 步骤3：根据不同扇区，计算基本矢量的作用时间
    // 每个扇区使用两个相邻的基本矢量合成目标电压矢量
    if (sector == 5) {
        // 扇区5：使用矢量U4(100)和U6(110)
        float t4 = u3;  // U4的作用时间
        float t6 = u1;  // U6的作用时间
        // 过调制处理：如果总时间超过周期，等比例缩小
        float sum = t4 + t6;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t4 = foc->k_svpwm * t4;
            t6 = foc->k_svpwm * t6;
        }
        // 计算零矢量时间（七段式对称分布）
        float t7 = (ts - t4 - t6) / 2;
        // 计算三相占空比（七段式序列：零-U4-U6-零-U6-U4-零）
        foc->t_a = t4 + t6 + t7;  // A相：U4 + U6 + 零矢量
        foc->t_b = t6 + t7;       // B相：U6 + 零矢量
        foc->t_c = t7;            // C相：零矢量
        
    } else if (sector == 1) {
        // 扇区1：使用矢量U2(010)和U6(110)
        float t2 = -u3;  // U2的作用时间
        float t6 = -u2;  // U6的作用时间
        
        float sum = t2 + t6;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t2 = foc->k_svpwm * t2;
            t6 = foc->k_svpwm * t6;
        }
        float t7 = (ts - t2 - t6) / 2;
        // 七段式序列：零-U6-U2-零-U2-U6-零
        foc->t_a = t6 + t7;          // A相：U6 + 零矢量
        foc->t_b = t2 + t6 + t7;     // B相：U2 + U6 + 零矢量
        foc->t_c = t7;               // C相：零矢量
        
    } 
    // ...（其他扇区的计算类似，只是使用的矢量和计算公式不同）
    else if (sector == 3) {
        // 扇区3：使用矢量U2(010)和U3(011)
        float t2 = u1;
        float t3 = u2;
        float sum = t2 + t3;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t2 = foc->k_svpwm * t2;
            t3 = foc->k_svpwm * t3;
        }
        float t7 = (ts - t2 - t3) / 2;
        foc->t_a = t7;
        foc->t_b = t2 + t3 + t7;
        foc->t_c = t3 + t7;
        
    } else if (sector == 2) {
        // 扇区2：使用矢量U1(001)和U3(011)
        float t1 = -u1;
        float t3 = -u3;
        float sum = t1 + t3;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t1 = foc->k_svpwm * t1;
            t3 = foc->k_svpwm * t3;
        }
        float t7 = (ts - t1 - t3) / 2;
        foc->t_a = t7;
        foc->t_b = t3 + t7;
        foc->t_c = t1 + t3 + t7;
        
    } else if (sector == 6) {
        // 扇区6：使用矢量U1(001)和U5(101)
        float t1 = u2;
        float t5 = u3;
        float sum = t1 + t5;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t1 = foc->k_svpwm * t1;
            t5 = foc->k_svpwm * t5;
        }
        float t7 = (ts - t1 - t5) / 2;
        foc->t_a = t5 + t7;
        foc->t_b = t7;
        foc->t_c = t1 + t5 + t7;
        
    } else if (sector == 4) {
        // 扇区4：使用矢量U4(100)和U5(101)
        float t4 = -u2;
        float t5 = -u1;
        float sum = t4 + t5;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t4 = foc->k_svpwm * t4;
            t5 = foc->k_svpwm * t5;
        }
        float t7 = (ts - t4 - t5) / 2;
        foc->t_a = t4 + t5 + t7;
        foc->t_b = t7;
        foc->t_c = t5 + t7;
    }
    // 至此，t_a, t_b, t_c 就是三相PWM的占空比，可以直接写入定时器寄存器
}
