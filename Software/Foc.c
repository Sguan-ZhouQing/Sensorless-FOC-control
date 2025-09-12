/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:25:14
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-12 21:22:41
 * @FilePath: \demo_STM32F103FocCode\Software\Foc.c
 * @Description: 
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:25:14
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-12 20:19:12
 * @FilePath: \demo_STM32F103FocCode\Software\Foc.c
 * @Description: FOC应用层代码开发
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Foc.h"

//电机参数设定（宏定义）
#define SguanFOC_ARR 2000   // 磁定向控制之中PWM份额值
#define Pole_Pairs 7        //电机的极对极数（通常为7）
#define Motor_Dir 1         //电机方向辨识（正负区分）
#define Dead_Time 0.01f     //死区时间限幅（Low-High）
// 磁定向控制的结构体变量
FOC_HandleTypeDef SguanFOC;
// 开环速度控制相关变量
static float electrical_angle = 0.0f;   // 电角度（弧度）
static uint32_t last_time = 0;          // 上次更新时间
// q轴电压计算值（INA199a1中间变量）
#define Sqrt3 1.732050807568877f        // 根号3的浮点值
#define Intermediate_Raw 1972           // 电流采样基准Raw数据
#define INA199A1_Num 50                 // INA199A1功放倍数
#define Shunt_Resistor 20               // 20毫伏的电流采样电阻
static float current_Iq = 0.0f;         // Iq电流滤波后的数据
// 卡尔曼滤波宏定义
#define M_NOISE     10.0f                // R值，传感器噪声大则设大
#define P_NOISE     0.01f              // Q值，系统变化快则设大


// 初始化FOC控制器的底层硬件
void FOC_Init(void) {
    // 初始化FOC结构体
    memset(&SguanFOC, 0, sizeof(FOC_HandleTypeDef));
    SguanFOC.u_d = 0.0f;    // d轴电压设为0
    SguanFOC.u_q = 0.3f;    // q轴电压（控制转矩，0.3是合理的启动值）
    current_Iq = 0.0f;     // 初始化电流Iq值
    
    // 1.硬件初始化
    HAL_TIM_Base_Start_IT(&htim1);
    Timer_Init();
    // 2.磁编码器初始化
    MT6701_Init();
    // 3.ADC相电流采样初始化
    INA199A1_Init();
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
    // 电机占空比计算（ABC三相电压）
    uint16_t FinalDuty_A,FinalDuty_B, ccr_a, ccr_b, ccr_c;
    ccr_a = (uint16_t)(duty_a * SguanFOC_ARR);
    ccr_b = (uint16_t)(duty_b * SguanFOC_ARR);
    ccr_c = (uint16_t)(duty_c * SguanFOC_ARR);
    // 根据电机方向调整相位
    if (Motor_Dir) {
        // 正转：正常输出
        FinalDuty_A = ccr_a;
        FinalDuty_B = ccr_b;
    } else {
        // 反转：交换A相和B相
        FinalDuty_A = ccr_b;
        FinalDuty_B = ccr_a;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FinalDuty_A);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FinalDuty_B);
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
void FOC_OpenPosition_Loop(float angle_deg, float voltage) {
    // 1. 机械角度转电角度
    float mechanical_angle = (angle_deg / 360.0f) * 2.0f * PI;  // 机械角度转弧度
    SguanFOC.theta = mechanical_angle * Pole_Pairs;             // 机械角度转电角度
    // 2. 设置电压（控制转矩）
    SguanFOC.u_q = voltage;
    SguanFOC.u_d = 0.0f;
    // 3. 生成SVPWM波形
    generate_svpwm_waveforms();
}


/**
 * @description: [Mode2]FOC开环速度控制
 * @param {float} velocity_rad_s 机械角速度（弧度/秒）
 * @param {float} voltage 电压幅值(0-1)
 */
void FOC_OpenVelocity_Loop(float velocity_rad_s, float voltage) {
    // 1. 计算时间差（毫秒）
    uint32_t current_time = HAL_GetTick();
    float delta_time_ms = (float)(current_time - last_time);
    last_time = current_time;
    // 2. 将机械角速度转换为电角速度
    float electrical_velocity = velocity_rad_s * Pole_Pairs;  // 电角速度（弧度/秒）
    // 3. 计算角度增量（积分得到角度）
    // delta_time_ms转换为秒：/ 1000.0f
    float angle_increment = electrical_velocity * (delta_time_ms / 1000.0f);
    // 4. 更新电角度（保持角度在0-2π范围内）
    electrical_angle += angle_increment;
    if (electrical_angle > 2.0f * PI) {
        electrical_angle -= 2.0f * PI;
    } else if (electrical_angle < 0) {
        electrical_angle += 2.0f * PI;
    }
    
    // 5. 设置电角度
    SguanFOC.theta = electrical_angle;
    // 6. 设置电压（控制转矩）
    SguanFOC.u_q = voltage;
    SguanFOC.u_d = 0.0f;
    // 7. 生成SVPWM波形
    generate_svpwm_waveforms();
}


/**
 * @description: 计算q轴电流Iq值
 * @note: 带低通滤波（计算后输出平稳曲线）
 * @return {float} Iq电流值
 */
float FOC_Calculate_Iq(void) {
    // 1. 采样三相电流
    int32_t Iu_Raw,Iv_Raw;
    Iu_Raw = INA199A1_GetRawValue(0) - Intermediate_Raw;  // U相电流（电压值）
    Iv_Raw = INA199A1_GetRawValue(1) - Intermediate_Raw;  // V相电流（电压值）
    // 将原始ADC值转换为实际电流值（单位：A）
    // 计算公式：电流(A) = (ADC原始值 * 3.3V / 4096) / (增益 * 采样电阻)
    // 简化后：电流(A) = ADC原始值 * (3.3 / (4096 * 50 * 0.02))
    // 计算系数：3.3 / (4096 * 50 * 0.02) = 3.3 / 4096 ≈ 0.00080566
    const float ADC_to_Current = 3.3f / 4096.0f / INA199A1_Num / (Shunt_Resistor / 1000.0f);
    float Iu = ADC_to_Current * (float)Iu_Raw;
    float Iv = ADC_to_Current * (float)Iv_Raw;
    // Iw = -Iu - Iv;                // W相电流（根据KCL定律）
    
    // 2. Clarke变换 (3相 → 2相)
    float I_alpha = Iu;
    float I_beta = (Iu + 2.0f * Iv) * (1.0f / Sqrt3);  // 1/√3
    // 3. Park变换 (静止 → 旋转)
    float sin_theta, cos_theta;
    fast_sin_cos(SguanFOC.theta,&sin_theta,&cos_theta);
    
    // Park变换公式: 
    // Id = I_alpha * cos(theta) + I_beta * sin(theta)
    // Iq = -I_alpha * sin(theta) + I_beta * cos(theta)
    float Raw_Iq = -I_alpha * sin_theta + I_beta * cos_theta;
    current_Iq = kalman_filter_std(Raw_Iq, M_NOISE, P_NOISE);
    return current_Iq;
}

