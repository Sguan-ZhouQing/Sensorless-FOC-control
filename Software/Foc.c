/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:25:14
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-24 18:44:51
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

// 默认模式与目标量
FOC_Mode_t FOC_Mode = FOC_MODE_NONE;
float FOC_Target_Position = 0.0f;
float FOC_Target_Speed = 0.0f;
float FOC_Target_Current = 0.0f;
float FOC_Target_Voltage = 0.2f;

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
    SguanSVPWM.u_q = 0.0f;
    SguanSVPWM.u_d = voltage;
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



void FOC_OpenVelocityAAA_Loop(float voltage) {
    float mech_angle_rad;
    if (MT6701_ReadAngle(&mech_angle_rad)) {
        SguanSVPWM.theta = normalize_angle((mech_angle_rad + alignment_angle_offset) * Pole_Pairs);
    }
    // 6. 设置电压（控制转矩）
    SguanSVPWM.u_q = 0.0f;
    SguanSVPWM.u_d = voltage;
    // 7. 生成SVPWM波形
    generate_svpwm_waveforms();
}



/**
 * @description: [Mode3]FOC位置单环闭环控制
 * @param {float} target_angle_rad  目标多圈位置 (弧度)
 */
void FOC_Position_SingleLoop(float target_angle_rad) {
    float actual_angle_rad;
    if (!MT6701_ReadMultiTurnAngle(&actual_angle_rad)) {
        return; // 读取失败直接退出
    }

    // --- 1. 误差计算 ---
    float error = target_angle_rad - actual_angle_rad;

    // --- 2. PID控制器 ---
    float *Kp = &SguanFOC.Position_PID.Kp;
    float *Ki = &SguanFOC.Position_PID.Ki;
    float *Kd = &SguanFOC.Position_PID.Kd;
    float *Integral = &SguanFOC.Position_PID.Integral;
    float *Prev_error = &SguanFOC.Position_PID.Prev_error;
    float limit = SguanFOC.Position_PID.Output_limit;

    *Integral += error;
    // 防止积分过大
    if (*Integral > limit) *Integral = limit;
    if (*Integral < -limit) *Integral = -limit;

    float derivative = error - *Prev_error;
    *Prev_error = error;

    float pid_output = (*Kp * error) + (*Ki * (*Integral)) + (*Kd * derivative);

    // --- 3. 限幅 ---
    pid_output = constrain(pid_output, -limit, limit);

    // --- 4. 设置FOC电压 ---
    SguanSVPWM.u_q = 0.0f; // 误差越大，扭矩越大
    SguanSVPWM.u_d = pid_output;

    // 实际电角度 = 机械角度 * 极对数 + 偏移
    SguanSVPWM.theta = normalize_angle((actual_angle_rad + alignment_angle_offset) * Pole_Pairs);

    // --- 5. 输出SVPWM ---
    generate_svpwm_waveforms();
}


/**
 * @description: [Mode4]FOC速度单环闭环控制
 * @param {float} target_speed_rad_s 目标速度 (rad/s)
 */
void FOC_Velocity_SingleLoop(float target_speed_rad_s) {
    float actual_speed_rad_s;
    MT6701_FilteredAngularVelocity(&actual_speed_rad_s);

    // --- 1. 误差计算 ---
    float error = target_speed_rad_s - actual_speed_rad_s;

    // --- 2. PID控制器 ---
    float *Kp = &SguanFOC.Velocity_PID.Kp;
    float *Ki = &SguanFOC.Velocity_PID.Ki;
    float *Kd = &SguanFOC.Velocity_PID.Kd;
    float *Integral = &SguanFOC.Velocity_PID.Integral;
    float *Prev_error = &SguanFOC.Velocity_PID.Prev_error;
    float limit = SguanFOC.Velocity_PID.Output_limit;

    *Integral += error;
    if (*Integral > limit) *Integral = limit;
    if (*Integral < -limit) *Integral = -limit;

    float derivative = error - *Prev_error;
    *Prev_error = error;

    float pid_output = (*Kp * error) + (*Ki * (*Integral)) + (*Kd * derivative);

    // --- 3. 限幅 ---
    pid_output = constrain(pid_output, -limit, limit);

    // --- 4. 设置FOC电压 ---
    SguanSVPWM.u_q = 0.0f;
    SguanSVPWM.u_d = pid_output;

    // 实际电角度 = 机械角度 * 极对数 + 偏移
    float mech_angle_rad;
    if (MT6701_ReadAngle(&mech_angle_rad)) {
        SguanSVPWM.theta = normalize_angle((mech_angle_rad + alignment_angle_offset) * Pole_Pairs);
    }

    // --- 5. 输出SVPWM ---
    generate_svpwm_waveforms();
}


/**
 * @description: [Mode5]FOC电流单环闭环控制
 * @param {float} target_iq 目标q轴电流 (A)
 */
void FOC_Current_SingleLoop(float target_iq) {
    // --- 1. 获取实际Iq ---
    float actual_iq = FOC_Calculate_Iq();

    // --- 2. 误差计算 ---
    float error = target_iq - actual_iq;

    // --- 3. PID控制器 ---
    float *Kp = &SguanFOC.Current_PID.Kp;
    float *Ki = &SguanFOC.Current_PID.Ki;
    float *Kd = &SguanFOC.Current_PID.Kd;
    float *Integral = &SguanFOC.Current_PID.Integral;
    float *Prev_error = &SguanFOC.Current_PID.Prev_error;
    float limit = SguanFOC.Current_PID.Output_limit;

    *Integral += error;
    if (*Integral > limit) *Integral = limit;
    if (*Integral < -limit) *Integral = -limit;

    float derivative = error - *Prev_error;
    *Prev_error = error;

    float pid_output = (*Kp * error) + (*Ki * (*Integral)) + (*Kd * derivative);

    // --- 4. 限幅 ---
    pid_output = constrain(pid_output, -limit, limit);

    // --- 5. 设置FOC电压 ---
    SguanSVPWM.u_q = pid_output;
    SguanSVPWM.u_d = 0.0f;

    // 实际电角度 = (机械角度 + 偏移) * 极对数
    float mech_angle_rad;
    if (MT6701_ReadAngle(&mech_angle_rad)) {
        SguanSVPWM.theta = normalize_angle((mech_angle_rad + alignment_angle_offset) * Pole_Pairs);
    }

    // --- 6. 输出SVPWM ---
    generate_svpwm_waveforms();
}



/**
 * @description: [Mode6] FOC速度-电流串级控制（电流环比速度环快7倍）
 * @param {float} target_speed_rad_s 目标速度 (rad/s)
 */
void FOC_Velocity_Current_Cascade_FastInner(float target_speed_rad_s) {
    static uint8_t speed_loop_counter = 0;
    static float Iq_ref = 0.0f;  // 缓存速度环计算出的目标电流

    // --- 1. 外环速度PID (每7次执行一次) ---
    if (speed_loop_counter == 0) {
        float actual_speed_rad_s;
        MT6701_FilteredAngularVelocity(&actual_speed_rad_s);

        float v_error = target_speed_rad_s - actual_speed_rad_s;

        float *vKp = &SguanFOC.Velocity_Current_Cascade.Velocity.Kp;
        float *vKi = &SguanFOC.Velocity_Current_Cascade.Velocity.Ki;
        float *vKd = &SguanFOC.Velocity_Current_Cascade.Velocity.Kd;
        float *vIntegral = &SguanFOC.Velocity_Current_Cascade.Velocity.Integral;
        float *vPrev_error = &SguanFOC.Velocity_Current_Cascade.Velocity.Prev_error;
        float v_limit = SguanFOC.Velocity_Current_Cascade.Velocity.Output_limit;

        *vIntegral += v_error;
        if (*vIntegral > v_limit) *vIntegral = v_limit;
        if (*vIntegral < -v_limit) *vIntegral = -v_limit;

        float v_derivative = v_error - *vPrev_error;
        *vPrev_error = v_error;

        Iq_ref = (*vKp * v_error) + (*vKi * (*vIntegral)) + (*vKd * v_derivative);

        // 限幅目标电流
        Iq_ref = constrain(Iq_ref, -SguanFOC.Velocity_Current_Cascade.Current.Output_limit,
                                    SguanFOC.Velocity_Current_Cascade.Current.Output_limit);
    }

    // --- 2. 内环电流PID (每次都执行) ---
    float actual_iq = FOC_Calculate_Iq();
    float c_error = Iq_ref - actual_iq;

    float *cKp = &SguanFOC.Velocity_Current_Cascade.Current.Kp;
    float *cKi = &SguanFOC.Velocity_Current_Cascade.Current.Ki;
    float *cKd = &SguanFOC.Velocity_Current_Cascade.Current.Kd;
    float *cIntegral = &SguanFOC.Velocity_Current_Cascade.Current.Integral;
    float *cPrev_error = &SguanFOC.Velocity_Current_Cascade.Current.Prev_error;
    float c_limit = SguanFOC.Velocity_Current_Cascade.Current.Output_limit;

    *cIntegral += c_error;
    if (*cIntegral > c_limit) *cIntegral = c_limit;
    if (*cIntegral < -c_limit) *cIntegral = -c_limit;

    float c_derivative = c_error - *cPrev_error;
    *cPrev_error = c_error;

    float pid_output_current = (*cKp * c_error) + (*cKi * (*cIntegral)) + (*cKd * c_derivative);

    // 限幅
    pid_output_current = constrain(pid_output_current, -c_limit, c_limit);

    // --- 3. 设置FOC电压 ---
    SguanSVPWM.u_q = pid_output_current;
    SguanSVPWM.u_d = 0.0f;

    float mech_angle_rad;
    if (MT6701_ReadAngle(&mech_angle_rad)) {
        SguanSVPWM.theta = normalize_angle((mech_angle_rad + alignment_angle_offset) * Pole_Pairs);
    }

    generate_svpwm_waveforms();

    // --- 4. 更新速度环计数器 ---
    speed_loop_counter++;
    if (speed_loop_counter >= 7) {
        speed_loop_counter = 0;
    }
}


/**
 * @description: [Mode7] FOC位置-速度串级控制（速度环比位置环快7倍）
 * @param {float} target_angle_rad 目标位置 (rad，多圈角度)
 */
void FOC_Position_Velocity_Cascade_FastInner(float target_angle_rad) {
    static uint8_t pos_loop_counter = 0;
    static float velocity_ref = 0.0f;  // 缓存位置环计算出的目标速度

    // --- 1. 外环位置PID (每7次执行一次) ---
    if (pos_loop_counter == 0) {
        float actual_angle_rad;
        if (!MT6701_ReadMultiTurnAngle(&actual_angle_rad)) {
            return; // 如果读取失败，直接退出
        }

        float p_error = target_angle_rad - actual_angle_rad;

        float *pKp = &SguanFOC.Position_Velocity_Cascade.Position.Kp;
        float *pKi = &SguanFOC.Position_Velocity_Cascade.Position.Ki;
        float *pKd = &SguanFOC.Position_Velocity_Cascade.Position.Kd;
        float *pIntegral = &SguanFOC.Position_Velocity_Cascade.Position.Integral;
        float *pPrev_error = &SguanFOC.Position_Velocity_Cascade.Position.Prev_error;
        float p_limit = SguanFOC.Position_Velocity_Cascade.Position.Output_limit;

        *pIntegral += p_error;
        if (*pIntegral > p_limit) *pIntegral = p_limit;
        if (*pIntegral < -p_limit) *pIntegral = -p_limit;

        float p_derivative = p_error - *pPrev_error;
        *pPrev_error = p_error;

        velocity_ref = (*pKp * p_error) + (*pKi * (*pIntegral)) + (*pKd * p_derivative);

        // 限幅目标速度
        velocity_ref = constrain(velocity_ref,
                                 -SguanFOC.Position_Velocity_Cascade.Velocity.Output_limit,
                                  SguanFOC.Position_Velocity_Cascade.Velocity.Output_limit);
    }

    // --- 2. 内环速度PID (每次执行) ---
    float actual_speed_rad_s;
    MT6701_FilteredAngularVelocity(&actual_speed_rad_s);

    float v_error = velocity_ref - actual_speed_rad_s;

    float *vKp = &SguanFOC.Position_Velocity_Cascade.Velocity.Kp;
    float *vKi = &SguanFOC.Position_Velocity_Cascade.Velocity.Ki;
    float *vKd = &SguanFOC.Position_Velocity_Cascade.Velocity.Kd;
    float *vIntegral = &SguanFOC.Position_Velocity_Cascade.Velocity.Integral;
    float *vPrev_error = &SguanFOC.Position_Velocity_Cascade.Velocity.Prev_error;
    float v_limit = SguanFOC.Position_Velocity_Cascade.Velocity.Output_limit;

    *vIntegral += v_error;
    if (*vIntegral > v_limit) *vIntegral = v_limit;
    if (*vIntegral < -v_limit) *vIntegral = -v_limit;

    float v_derivative = v_error - *vPrev_error;
    *vPrev_error = v_error;

    float pid_output_velocity = (*vKp * v_error) + (*vKi * (*vIntegral)) + (*vKd * v_derivative);

    // 限幅
    pid_output_velocity = constrain(pid_output_velocity, -v_limit, v_limit);

    // --- 3. 设置FOC电压 ---
    SguanSVPWM.u_q = pid_output_velocity;
    SguanSVPWM.u_d = 0.0f;

    float mech_angle_rad;
    if (MT6701_ReadAngle(&mech_angle_rad)) {
        SguanSVPWM.theta = normalize_angle((mech_angle_rad + alignment_angle_offset) * Pole_Pairs);
    }

    generate_svpwm_waveforms();

    // --- 4. 更新位置环计数器 ---
    pos_loop_counter++;
    if (pos_loop_counter >= 7) {
        pos_loop_counter = 0;
    }
}


/**
 * @description: [Mode8] FOC位置-速度-电流三环串级控制
 *               电流环比速度环快5倍，速度环比位置环快5倍
 * @param {float} target_angle_rad 目标位置 (rad，多圈角度)
 */
void FOC_Position_Velocity_Current_Cascade_Triple(float target_angle_rad) {
    static uint8_t pos_counter = 0;   // 位置环计数器
    static uint8_t vel_counter = 0;   // 速度环计数器
    static float velocity_ref = 0.0f; // 缓存位置环输出
    static float Iq_ref = 0.0f;       // 缓存速度环输出

    // --- 1. 位置环 (每25次执行1次) ---
    if (pos_counter == 0) {
        float actual_angle_rad;
        if (MT6701_ReadMultiTurnAngle(&actual_angle_rad)) {
            float p_error = target_angle_rad - actual_angle_rad;

            float *pKp = &SguanFOC.Position_Velocity_Current_Cascade.Position.Kp;
            float *pKi = &SguanFOC.Position_Velocity_Current_Cascade.Position.Ki;
            float *pKd = &SguanFOC.Position_Velocity_Current_Cascade.Position.Kd;
            float *pIntegral = &SguanFOC.Position_Velocity_Current_Cascade.Position.Integral;
            float *pPrev_error = &SguanFOC.Position_Velocity_Current_Cascade.Position.Prev_error;
            float p_limit = SguanFOC.Position_Velocity_Current_Cascade.Position.Output_limit;

            *pIntegral += p_error;
            if (*pIntegral > p_limit) *pIntegral = p_limit;
            if (*pIntegral < -p_limit) *pIntegral = -p_limit;

            float p_derivative = p_error - *pPrev_error;
            *pPrev_error = p_error;

            velocity_ref = (*pKp * p_error) + (*pKi * (*pIntegral)) + (*pKd * p_derivative);

            // 限幅目标速度
            velocity_ref = constrain(velocity_ref,
                                     -SguanFOC.Position_Velocity_Current_Cascade.Velocity.Output_limit,
                                      SguanFOC.Position_Velocity_Current_Cascade.Velocity.Output_limit);
        }
    }

    // --- 2. 速度环 (每5次执行1次) ---
    if (vel_counter == 0) {
        float actual_speed_rad_s;
        MT6701_FilteredAngularVelocity(&actual_speed_rad_s);

        float v_error = velocity_ref - actual_speed_rad_s;

        float *vKp = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Kp;
        float *vKi = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Ki;
        float *vKd = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Kd;
        float *vIntegral = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Integral;
        float *vPrev_error = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Prev_error;
        float v_limit = SguanFOC.Position_Velocity_Current_Cascade.Velocity.Output_limit;

        *vIntegral += v_error;
        if (*vIntegral > v_limit) *vIntegral = v_limit;
        if (*vIntegral < -v_limit) *vIntegral = -v_limit;

        float v_derivative = v_error - *vPrev_error;
        *vPrev_error = v_error;

        Iq_ref = (*vKp * v_error) + (*vKi * (*vIntegral)) + (*vKd * v_derivative);

        // 限幅目标电流
        Iq_ref = constrain(Iq_ref,
                           -SguanFOC.Position_Velocity_Current_Cascade.Current.Output_limit,
                            SguanFOC.Position_Velocity_Current_Cascade.Current.Output_limit);
    }

    // --- 3. 电流环 (每次都执行) ---
    float actual_iq = FOC_Calculate_Iq();
    float c_error = Iq_ref - actual_iq;

    float *cKp = &SguanFOC.Position_Velocity_Current_Cascade.Current.Kp;
    float *cKi = &SguanFOC.Position_Velocity_Current_Cascade.Current.Ki;
    float *cKd = &SguanFOC.Position_Velocity_Current_Cascade.Current.Kd;
    float *cIntegral = &SguanFOC.Position_Velocity_Current_Cascade.Current.Integral;
    float *cPrev_error = &SguanFOC.Position_Velocity_Current_Cascade.Current.Prev_error;
    float c_limit = SguanFOC.Position_Velocity_Current_Cascade.Current.Output_limit;

    *cIntegral += c_error;
    if (*cIntegral > c_limit) *cIntegral = c_limit;
    if (*cIntegral < -c_limit) *cIntegral = -c_limit;

    float c_derivative = c_error - *cPrev_error;
    *cPrev_error = c_error;

    float pid_output_current = (*cKp * c_error) + (*cKi * (*cIntegral)) + (*cKd * c_derivative);

    // 限幅
    pid_output_current = constrain(pid_output_current, -c_limit, c_limit);

    // --- 4. 设置FOC电压 ---
    SguanSVPWM.u_q = pid_output_current;
    SguanSVPWM.u_d = 0.0f;

    float mech_angle_rad;
    if (MT6701_ReadAngle(&mech_angle_rad)) {
        SguanSVPWM.theta = normalize_angle((mech_angle_rad + alignment_angle_offset) * Pole_Pairs);
    }

    generate_svpwm_waveforms();

    // --- 5. 更新计数器 ---
    vel_counter++;
    if (vel_counter >= 5) vel_counter = 0;

    pos_counter++;
    if (pos_counter >= 25) pos_counter = 0;
}


// 电机多环调控运行函数
void FOC_LoopHandler(void) {
    switch (FOC_Mode) {
        // --- 开环 ---
        case FOC_MODE_OPEN_POSITION:
            // 注意：开环角度用度数作为输入
            FOC_OpenPosition_Loop((FOC_Target_Position / (2.0f * PI)) * 360.0f, FOC_Target_Voltage);
            break;
        case FOC_MODE_OPEN_VELOCITY:
            FOC_OpenVelocity_Loop(FOC_Target_Speed, FOC_Target_Voltage);
            break;

        // --- 单环闭环 ---
        case FOC_MODE_POSITION_SINGLE:
            FOC_Position_SingleLoop(FOC_Target_Position);
            break;
        case FOC_MODE_VELOCITY_SINGLE:
            FOC_Velocity_SingleLoop(FOC_Target_Speed);
            break;
        case FOC_MODE_CURRENT_SINGLE:
            FOC_Current_SingleLoop(FOC_Target_Current);
            break;

        // --- 双环闭环 ---
        case FOC_MODE_POSITION_VELOCITY_CASCADE:
            FOC_Position_Velocity_Cascade_FastInner(FOC_Target_Position);
            break;
        case FOC_MODE_VELOCITY_CURRENT_CASCADE:
            FOC_Velocity_Current_Cascade_FastInner(FOC_Target_Speed);
            break;

        // --- 三环闭环 ---
        case FOC_MODE_POSITION_VELOCITY_CURRENT_CASCADE:
            FOC_Position_Velocity_Current_Cascade_Triple(FOC_Target_Position);
            break;
        default:
            // 默认空闲：关电机
            SguanSVPWM.u_q = 0.0f;
            SguanSVPWM.u_d = 0.0f;
            generate_svpwm_waveforms();
            break;
    }
}


/**
 * @description: 设置指定环路的PID参数
 * @param loop   "pos", "vel", "cur",
 *               "pos_vel", "vel_cur", "pos_vel_cur"
 * @param kp     比例系数
 * @param ki     积分系数
 * @param kd     微分系数
 * @param limit  输出限幅
 */
void FOC_SetPIDParams(const char *loop, float kp, float ki, float kd, float limit) {
    if (strcmp(loop, "pos") == 0) {
        SguanFOC.Position_PID.Kp = kp;
        SguanFOC.Position_PID.Ki = ki;
        SguanFOC.Position_PID.Kd = kd;
        SguanFOC.Position_PID.Output_limit = limit;
    }
    else if (strcmp(loop, "vel") == 0) {
        SguanFOC.Velocity_PID.Kp = kp;
        SguanFOC.Velocity_PID.Ki = ki;
        SguanFOC.Velocity_PID.Kd = kd;
        SguanFOC.Velocity_PID.Output_limit = limit;
    }
    else if (strcmp(loop, "cur") == 0) {
        SguanFOC.Current_PID.Kp = kp;
        SguanFOC.Current_PID.Ki = ki;
        SguanFOC.Current_PID.Kd = kd;
        SguanFOC.Current_PID.Output_limit = limit;
    }
    else if (strcmp(loop, "pos_vel") == 0) {
        SguanFOC.Position_Velocity_Cascade.Position.Kp = kp;
        SguanFOC.Position_Velocity_Cascade.Position.Ki = ki;
        SguanFOC.Position_Velocity_Cascade.Position.Kd = kd;
        SguanFOC.Position_Velocity_Cascade.Position.Output_limit = limit;
    }
    else if (strcmp(loop, "vel_cur") == 0) {
        SguanFOC.Velocity_Current_Cascade.Velocity.Kp = kp;
        SguanFOC.Velocity_Current_Cascade.Velocity.Ki = ki;
        SguanFOC.Velocity_Current_Cascade.Velocity.Kd = kd;
        SguanFOC.Velocity_Current_Cascade.Velocity.Output_limit = limit;
    }
    else if (strcmp(loop, "pos_vel_cur") == 0) {
        SguanFOC.Position_Velocity_Current_Cascade.Position.Kp = kp;
        SguanFOC.Position_Velocity_Current_Cascade.Position.Ki = ki;
        SguanFOC.Position_Velocity_Current_Cascade.Position.Kd = kd;
        SguanFOC.Position_Velocity_Current_Cascade.Position.Output_limit = limit;
    }
}
