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
    // 1.（单环）位置闭环
    struct {
        float Kp;           // 比例系数
        float Ki;           // 积分系数
        float Kd;           // 微分系数
        float Integral;     // 积分项累加值
        float Prev_error;   // 上一次误差（用于微分项）
        float Output_limit; // 输出限制
    } Position_PID;
    // 2.（单环）速度闭环
    struct {
        float Kp;
        float Ki;
        float Kd;
        float Integral;
        float Prev_error;
        float Output_limit;
    } Velocity_PID;
    // 3.（单环）电流闭环
    struct {
        float Kp;
        float Ki;
        float Kd;
        float Integral;
        float Prev_error;
        float Output_limit;
    } Current_PID;

    // 4.速度-电流双环串级控制(速度控制常选)
    struct {
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Velocity;             // 速度环
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Current;              // 电流环
    } Velocity_Current_Cascade;
    
    // 5.位置-速度双环串级控制(位置控制常选)
    struct {
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Position;             // 位置环
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Velocity;             // 速度环
    } Position_Velocity_Cascade;

    // 6.位置-速度-电流三环串级控制(主流)
    struct {
        struct {
            float Kp;           // 比例系数
            float Ki;           // 积分系数
            float Kd;           // 微分系数
            float Integral;     // 积分项累加值
            float Prev_error;   // 上一次误差
            float Output_limit; // 输出限制
        } Position;             // 位置环
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Velocity;             // 速度环
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Current;              // 电流环
    } Position_Velocity_Current_Cascade;   
} FOC_HandleTypeDef;


// 控制模式枚举
typedef enum {
    FOC_MODE_NONE = 0,
    // 开环
    FOC_MODE_OPEN_POSITION,
    FOC_MODE_OPEN_VELOCITY,
    // 单环闭环
    FOC_MODE_POSITION_SINGLE,
    FOC_MODE_VELOCITY_SINGLE,
    FOC_MODE_CURRENT_SINGLE,
    // 双环闭环
    FOC_MODE_POSITION_VELOCITY_CASCADE,
    FOC_MODE_VELOCITY_CURRENT_CASCADE,
    // 三环闭环
    FOC_MODE_POSITION_VELOCITY_CURRENT_CASCADE
} FOC_Mode_t;


// FOC控制的结构体变量(声明)
extern SVPWM_HandleTypeDef SguanSVPWM;
extern FOC_HandleTypeDef SguanFOC;
// 全局模式变量 & 目标量
extern FOC_Mode_t FOC_Mode;
extern float FOC_Target_Position; // rad
extern float FOC_Target_Speed;    // rad/s
extern float FOC_Target_Current;  // A
extern float FOC_Target_Voltage;  // 0-1 (开环时使用)

// 磁定向控制FOC函数声明汇总
void FOC_Init(void);
void FOC_EncoderAlignment(void);
float FOC_Calculate_Iq(void);
// 开环运行（位置开环，速度开环）
void FOC_OpenPosition_Loop(float angle_deg, float voltage);
void FOC_OpenVelocity_Loop(float velocity_rad_s, float voltage);
// 闭环运行(位置单环，速度单环，电流单环)
void FOC_Position_SingleLoop(float target_angle_rad);
void FOC_Velocity_SingleLoop(float target_speed_rad_s);
void FOC_Current_SingleLoop(float target_iq);
// 串级闭环(“速度-电流环”，“位置-速度环”，“位置-速度-电流环”)
void FOC_Velocity_Current_Cascade_FastInner(float target_speed_rad_s);
void FOC_Position_Velocity_Cascade_FastInner(float target_angle_rad);
void FOC_Position_Velocity_Current_Cascade_Triple(float target_angle_rad);
// 多环运行调度函数
void FOC_LoopHandler(void);
// PID参数调节函数
void FOC_SetPIDParams(const char *loop, float kp, float ki, float kd, float limit);


#endif // FOC_H
