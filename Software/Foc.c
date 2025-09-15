/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:25:14
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-15 20:20:26
 * @FilePath: \demo_STM32F103FocCode\Software\Foc.c
 * @Description: FOC应用层代码开发
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Foc.h"


//电机参数设定（宏定义）
#define SguanFOC_ARR 2000   // 磁定向控制之中PWM份额值
#define Pole_Pairs 7        // 电机的极对极数（通常为7）
#define Motor_Dir 1         // 电机方向辨识（正负区分）
#define Dead_Time 0.01f     // 死区时间限幅（Low-High）
#define Motor_Vbus 12.0f    // 设置电机驱动电源的大小

// 磁定向控制的结构体变量
SVPWM_HandleTypeDef SguanSVPWM;
FOC_HandleTypeDef SguanFOC;

// 开环速度控制相关变量
static float electrical_angle = 0.0f;   // 电角度（弧度）
static uint32_t last_time = 0;          // 上次更新时间

// 初始"电角度"和"机械角度"对齐变量
static float alignment_angle_offset = 0.0f;

// q轴电压计算值（INA199a1中间变量）
#define Sqrt3 1.732050807568877f        // 根号3的浮点值
#define Intermediate_Raw 1972           // 电流采样基准Raw数据
#define INA199A1_Num 50                 // INA199A1功放倍数
#define Shunt_Resistor 20               // 20毫伏的电流采样电阻
static float current_Iq = 0.0f;         // Iq电流滤波后的数据

// 卡尔曼滤波宏定义
#define M_NOISE     10.0f               // R值,传感器噪声大则设大
#define P_NOISE     0.01f               // Q值,系统变化快则设大

// 电流开环所需的电机参数
#define MOTOR_RESISTANCE 11.1f          // 电机相电阻（Ω）
// static float MOTOR_L = 0.0053f;      // 电机相电感（H）
// #define MOTOR_KV 120.0f                 // 电机KV值（RPM/V）
// #define MOTOR_KE (60.0f / (2.0f * PI * MOTOR_KV))  // 反电动势常数（V/(rad/s)）



// 初始化FOC控制器的底层硬件
void FOC_Init(void) {
    // 初始化FOC结构体
    memset(&SguanSVPWM, 0, sizeof(SVPWM_HandleTypeDef));
    current_Iq = 0.0f;     // 初始化电流Iq值
    
    // 1.定时器初始化
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
    // 电角度对齐(end)
    FOC_EncoderAlignment();
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
    ipark(&SguanSVPWM);      // 逆Park变换
    svpwm(&SguanSVPWM);      // SVPWM计算
    set_pwm_duty_cycle(SguanSVPWM.t_a, SguanSVPWM.t_b, SguanSVPWM.t_c); // 设置PWM
}


// 角度归一化，将角度限制在0-2π范围内
static float normalize_angle(float angle) {
    // 一次性处理所有情况
    angle = fmodf(angle, 2.0f * PI);
    return angle < 0 ? angle + 2.0f * PI : angle;
}


/**
 * @brief 编码器零位自动校准
 * @description: 通过注入d轴电流将转子拉到d轴位置，记录此时的编码器位置作为偏移量
 */
void FOC_EncoderAlignment(void) {
    // 1. 注入d轴电流将转子拉到d轴位置
    SguanSVPWM.u_d = 0.0f;  // 设置d轴电压
    SguanSVPWM.u_q = 0.3f;  // 注入q轴电压
    SguanSVPWM.theta = 0.0f; // 假设电角度为0时q轴对齐
    // 生成SVPWM，将转子拉到d轴位置
    generate_svpwm_waveforms();
    HAL_Delay(800);  // 等待800ms让转子稳定
    
    // 2. 读取此时的编码器机械角度
    float mechanical_angle_rad;
    if (MT6701_ReadAngle(&mechanical_angle_rad)) {
        // 计算偏移量：电角度 = (机械角度 + 偏移量) * 极对数
        alignment_angle_offset = -mechanical_angle_rad;
    }
    // 3. 停止注入电流
    SguanSVPWM.u_d = 0.0f;
    SguanSVPWM.u_q = 0.0f;
    generate_svpwm_waveforms();
}


/**
 * @description: 计算q轴电流Iq值
 * @note: 带一阶卡尔曼滤波（计算后输出平稳曲线）
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
    fast_sin_cos(SguanSVPWM.theta,&sin_theta,&cos_theta);
    
    // Park变换公式: 
    // Id = I_alpha * cos(theta) + I_beta * sin(theta)
    // Iq = -I_alpha * sin(theta) + I_beta * cos(theta)
    float Raw_Iq = -I_alpha * sin_theta + I_beta * cos_theta;
    current_Iq = kalman_filter_std(Raw_Iq, M_NOISE, P_NOISE);
    return current_Iq;
}


/**
 * @description: [Mode1]FOC开环位置控制
 * @param {float} angle_deg 机械角度度数(0到359度)
 * @param {int} pole_pairs 电机极对数
 * @param {float} voltage 电压幅值(0-1)
 */
void FOC_OpenPosition_Loop(float angle_deg, float voltage) {
    // 1. 机械角度转电角度
    float mechanical_angle = (angle_deg / 360.0f) * 2.0f * PI;
    SguanSVPWM.theta = normalize_angle(mechanical_angle * Pole_Pairs);
    // 2. 设置电压（控制转矩）
    SguanSVPWM.u_q = voltage;
    SguanSVPWM.u_d = 0.0f;
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
    electrical_angle = normalize_angle(electrical_angle);
    
    // 5. 设置电角度
    SguanSVPWM.theta = electrical_angle;
    // 6. 设置电压（控制转矩）
    SguanSVPWM.u_q = voltage;
    SguanSVPWM.u_d = 0.0f;
    // 7. 生成SVPWM波形
    generate_svpwm_waveforms();
}


/**
 * @description: [Mode3]FOC开环电流控制（使用编码器速度进行前馈补偿）
 * @param {float} target_current_iq 期望的q轴电流（单位：A），正负代表扭矩方向
 * @note 这是一个开环函数。它输出一个电压以期产生目标电流，但实际电流会有误差。
 *        电机最终达到的速度由目标电流和负载共同决定。
 */
void FOC_OpenCurrent_Loop(float target_current_iq) {
    // 1. 获取当前机械角度
    float mechanical_angle_rad;
    if (!MT6701_ReadAngle(&mechanical_angle_rad)) {
        return;
    }
    // 2. 计算电角度（考虑对齐偏移）
    float raw_electrical_angle = (mechanical_angle_rad + alignment_angle_offset) * Pole_Pairs;
    SguanSVPWM.theta = normalize_angle(raw_electrical_angle);
    
    // 3. 获取角速度（用于前馈补偿）
    float mechanical_velocity_rads;
    MT6701_FilteredAngularVelocity(&mechanical_velocity_rads);
    float electrical_velocity_rads = mechanical_velocity_rads * Pole_Pairs;
    
    // 4. 使用电机模型计算所需的q轴电压
    float u_q_feedforward;
    if (target_current_iq == 0.0f) {
        u_q_feedforward = 0.0f;
    } else {
        u_q_feedforward = target_current_iq * MOTOR_RESISTANCE;
    }
    // 5. 限制输出电压（使用标幺值）
    float u_q_max = 0.95f;
    u_q_feedforward = constrain(u_q_feedforward / Motor_Vbus, -u_q_max, u_q_max);
    
    // 6. 更新SVPWM指令
    SguanSVPWM.u_q = u_q_feedforward;
    SguanSVPWM.u_d = 0.0f;
    
    // 7. 生成SVPWM波形
    generate_svpwm_waveforms();
}



// PID参数设置的函数
void FOC_SetPositionPID(float Kp, float Ki, float Kd, float output_limit) {
    SguanFOC.position_pid.Kp = Kp;
    SguanFOC.position_pid.Ki = Ki;
    SguanFOC.position_pid.Kd = Kd;
    SguanFOC.position_pid.output_limit = output_limit;
    // 重置积分项和上一次误差
    SguanFOC.position_pid.integral = 0.0f;
    SguanFOC.position_pid.prev_error = 0.0f;
}


/**
 * @description: [Mode4]FOC闭环绝对位置控制（带多圈功能，使用完整PID）
 * @param {float} target_angle_rad 目标绝对角度（弧度），可以超过2π
 * @param {float} voltage 基准电压幅值(0-1)，PID输出会在此基础上调整
 */
void FOC_CloseAbsolutePos_Loop(float target_angle_rad, float voltage) {
    static float current_electrical_angle = 0.0f;
    static uint32_t prev_time_ms = 0;
    
    // 1. 获取当前时间并计算时间差
    uint32_t current_time_ms = HAL_GetTick();
    float dt_ms = (float)(current_time_ms - prev_time_ms);
    prev_time_ms = current_time_ms;
    
    // 避免除零错误
    if (dt_ms == 0) dt_ms = 1.0f;
    float dt = dt_ms / 1000.0f; // 转换为秒
    
    // 2. 读取当前编码器角度（多圈）
    float current_angle_rad;
    if (!MT6701_ReadMultiTurnAngle(&current_angle_rad)) {
        // 如果读取失败，使用上一次的角度
        return;
    }
    
    // 3. 计算角度误差（考虑多圈）
    float error = target_angle_rad - current_angle_rad;
    // 4. 处理角度误差的周期性（归一化到[-π, π]范围）
    if (error > PI) {
        error -= 2.0f * PI;
    } else if (error < -PI) {
        error += 2.0f * PI;
    }
    
    // 5. 计算PID三项
    // 比例项
    float P = SguanFOC.position_pid.Kp * error;
    // 积分项（抗积分饱和）
    SguanFOC.position_pid.integral += error * dt;
    // 限制积分项防止windup
    SguanFOC.position_pid.integral = constrain(SguanFOC.position_pid.integral, 
                                              -SguanFOC.position_pid.output_limit / SguanFOC.position_pid.Ki, 
                                              SguanFOC.position_pid.output_limit / SguanFOC.position_pid.Ki);
    float I = SguanFOC.position_pid.Ki * SguanFOC.position_pid.integral;
    // 微分项
    float derivative = (error - SguanFOC.position_pid.prev_error) / dt;
    float D = SguanFOC.position_pid.Kd * derivative;
    SguanFOC.position_pid.prev_error = error;

    // 6. 计算PID总输出
    float pid_output = P + I + D;
    // 7. 限制输出幅度
    pid_output = constrain(pid_output, -SguanFOC.position_pid.output_limit, SguanFOC.position_pid.output_limit);
    // 8. 更新电角度（PID输出作为角度增量）
    current_electrical_angle += pid_output;
    // 9. 保持电角度在0-2π范围内
    current_electrical_angle = normalize_angle(current_electrical_angle);
    
    // 10. 设置电角度和电压
    SguanSVPWM.theta = current_electrical_angle;
    SguanSVPWM.u_q = voltage;  // 基准电压 + PID调整
    SguanSVPWM.u_d = 0.0f;
    // 11. 生成SVPWM波形
    generate_svpwm_waveforms();
}
