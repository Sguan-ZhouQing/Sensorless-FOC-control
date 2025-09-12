/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:19:39
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-12 16:53:28
 * @FilePath: \demo_STM32F103FocCode\Hardware\MT6701.c
 * @Description: FOC硬件层的编码器程序开发
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "MT6701.h"

// MT6701接收数据缓冲区
static uint8_t mt6701_rx_data[3];
static bool data_ready = false;
static uint16_t current_angle_raw = 0;
static bool angle_valid = false;
// 添加全局变量用于绕圈检测
static float mt6701_prev_angle_rad = 0.0f;
static int32_t mt6701_total_turns = 0;
static bool mt6701_first_reading = true;
// 角速度计算相关变量
static float mt6701_prev_multi_angle_rad = 0.0f;
static uint32_t mt6701_prev_timestamp_ms = 0;
// 卡尔曼滤波宏定义
#define MEASUREMENT_NOISE 8.0f    // R值，传感器噪声大则设大
#define PROCESS_NOISE     0.008f   // Q值，系统变化快则设大


/**
 * @brief 初始化MT6701编码器
 */
void MT6701_Init(void)
{
    // 确保CS引脚为高电平（不选中）
    HAL_GPIO_WritePin(MT6701_CS_GPIO, MT6701_CS_PIN, GPIO_PIN_SET);
    
    // 启动第一次DMA传输
    HAL_GPIO_WritePin(MT6701_CS_GPIO, MT6701_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&MT6701_SPI_HANDLE, mt6701_rx_data, mt6701_rx_data, 3);
}

/**
 * @brief CRC计算函数
 * @param data: 要计算CRC的数据
 * @return CRC校验值
 */
uint8_t MT6701_CalculateCRC(uint32_t data)
{
    uint8_t crc = 0;
    uint32_t polynomial = 0x43; // (X^6 + X + 1)
    
    for (int i = 17; i >= 0; i--)
    {
        uint8_t bit = (data >> i) & 1;
        crc <<= 1;
        if ((crc >> 6) ^ bit)
            crc ^= polynomial;
        crc &= 0x3F;
    }
    return crc;
}

/**
 * @brief 读取原始角度值
 * @param raw_angle: 输出的原始角度值(0-16383)
 * @return true: 读取成功, false: 读取失败
 */
bool MT6701_ReadRawAngle(uint16_t *raw_angle)
{
    if (angle_valid)
    {
        *raw_angle = current_angle_raw;
        return true;
    }
    return false;
}

/**
 * @brief 读取角度值（弧度）
 * @param angle_rad: 输出的角度值（弧度）
 * @return true: 读取成功, false: 读取失败
 */
bool MT6701_ReadAngle(float *angle_rad)
{
    if (angle_valid)
    {
        *angle_rad = (2.0f * PI * current_angle_raw) / MT6701_ANGLE_RES;
        return true;
    }
    return false;
}

/**
 * @brief SPI传输完成回调函数
 * @param hspi: SPI句柄
 */
void MT6701_SPI_CompleteCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        // 拉高CS引脚
        HAL_GPIO_WritePin(MT6701_CS_GPIO, MT6701_CS_PIN, GPIO_PIN_SET);
        // 提取角度数据
        uint16_t angle_raw = (mt6701_rx_data[1] >> 2) | (mt6701_rx_data[0] << 6);
        // CRC校验
        uint8_t crc_raw = mt6701_rx_data[2] & 0x3F;
        uint32_t crc_data = (mt6701_rx_data[0] << 16 | mt6701_rx_data[1] << 8 | mt6701_rx_data[2]) >> 6;
        if (MT6701_CalculateCRC(crc_data) == crc_raw) {
            current_angle_raw = angle_raw;
            angle_valid = true;
            data_ready = true;
            // 绕圈检测逻辑
            float current_angle_rad = (2.0f * PI * current_angle_raw) / MT6701_ANGLE_RES;
            if (!mt6701_first_reading) {
                // 检测角度跳变（接近2π的边界）
                float angle_diff = current_angle_rad - mt6701_prev_angle_rad;
                // 处理角度跳变（归一化到[-π, π]范围）
                if (angle_diff > PI) {
                    angle_diff -= 2.0f * PI;
                    mt6701_total_turns--;
                }
                else if (angle_diff < -PI) {
                    angle_diff += 2.0f * PI;
                    mt6701_total_turns++;
                }
                // Code更新总圈数（如果需要精确的圈数计数）
            }
            else {
                mt6701_first_reading = false;
            } 
            mt6701_prev_angle_rad = current_angle_rad;
        }
        else {
            angle_valid = false;
        }
        // 启动下一次传输
        HAL_GPIO_WritePin(MT6701_CS_GPIO, MT6701_CS_PIN, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_DMA(&MT6701_SPI_HANDLE, mt6701_rx_data, mt6701_rx_data, 3);
    }
}

/**
 * @brief SPI错误回调函数
 * @param hspi: SPI句柄
 */
void MT6701_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        printf("MT6701 SPI Error: %d\n", hspi->ErrorCode);
        
        // 确保CS引脚状态
        HAL_GPIO_WritePin(MT6701_CS_GPIO, MT6701_CS_PIN, GPIO_PIN_SET);
        
        // 重新启动传输
        HAL_Delay(1);
        HAL_GPIO_WritePin(MT6701_CS_GPIO, MT6701_CS_PIN, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_DMA(&MT6701_SPI_HANDLE, mt6701_rx_data, mt6701_rx_data, 3);
    }
}

/**
 * @brief 检查是否有新数据可用
 * @return true: 有新数据, false: 无新数据
 */
bool MT6701_IsDataReady(void)
{
    if (data_ready)
    {
        data_ready = false;
        return true;
    }
    return false;
}


/**
 * @brief 读取多圈角度值（弧度）
 * @param angle_rad: 输出的多圈角度值（弧度）
 * @return true: 读取成功, false: 读取失败
 */
bool MT6701_ReadMultiTurnAngle(float *angle_rad)
{
    if (angle_valid)
    {
        // 计算当前单圈角度
        float current_angle_rad = (2.0f * PI * current_angle_raw) / MT6701_ANGLE_RES;
        
        // 加上圈数对应的角度
        *angle_rad = current_angle_rad + (2.0f * PI * mt6701_total_turns);
        
        return true;
    }
    return false;
}


// 添加重置函数（可选）
/**
 * @brief 重置多圈计数器
 */
void MT6701_ResetMultiTurnCounter(void)
{
    mt6701_total_turns = 0;
    mt6701_first_reading = true;
    mt6701_prev_angle_rad = 0.0f;
}


/**
 * @brief 计算角速度（弧度/秒）
 * @param angular_velocity_rads: 输出的角速度值
 * @return true: 计算成功, false: 计算失败
 */
bool MT6701_CalculateAngularVelocity(float *angular_velocity_rads)
{
    if (!angle_valid) return false;

    float current_angle_rad;
    if (!MT6701_ReadMultiTurnAngle(&current_angle_rad)) return false;

    uint32_t current_time_ms = HAL_GetTick();
    uint32_t time_diff_ms = current_time_ms - mt6701_prev_timestamp_ms;

    if (time_diff_ms == 0) return false;

    // 直接计算角度差（多圈角度已经是连续的）
    float angle_diff_rad = current_angle_rad - mt6701_prev_multi_angle_rad;

    // 计算角速度（弧度/秒）
    *angular_velocity_rads = angle_diff_rad / (time_diff_ms / 1000.0f);

    // 更新上一帧数据
    mt6701_prev_multi_angle_rad = current_angle_rad;
    mt6701_prev_timestamp_ms = current_time_ms;
    return true;
}


void MT6701_FilteredAngularVelocity(float *filtered_velocity_rads)
{
    float angular_velocity;
    MT6701_CalculateAngularVelocity(&angular_velocity);
    float filtered_data = kalman_filter_std(angular_velocity, MEASUREMENT_NOISE, PROCESS_NOISE);
    *filtered_velocity_rads = filtered_data;
}



/**
 * @description: 回调函数（使用DMA进行连续的数据传输）
 * @param {SPI_HandleTypeDef} *hspi
 * @return {*}
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    MT6701_SPI_CompleteCallback(hspi);  // SPI传输完成回调函数
}

// // /**
// //  * @description: 回调函数（SPI错误时重新启动DMA传输）
// //  * @param {SPI_HandleTypeDef} *hspi
// //  * @return {*}
// //  */
// void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
// {
//     MT6701_SPI_ErrorCallback(hspi); // SPI传输错误回调函数
// }
