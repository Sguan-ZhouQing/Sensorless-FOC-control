/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-26 15:32:26
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-09-13 14:26:21
 * @FilePath: \demo_STM32F103FocCode\Hardware\printf.c
 * @Description: 使用USART串口收发和数据处理
 * @Key_GPIO: Many;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "printf.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// 定义串口缓存区
extern UART_HandleTypeDef huart2;
uint8_t f103_readBuffer[f103_BUFFER_SIZE];
// 串口接收缓冲数组
uint8_t RxBuffer[1];        // 串口接收缓冲
uint16_t RxLine = 0;        // 指令长度
uint8_t DataBuff[200];      // 指令内容
// 全局可调变量
float Adjustable_Data;


// 支持printf函数，而不需要选择MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE { 
	int handle; 
}; 
// 支持printf函数，而不需要选择MicroLIB
FILE __stdout;
//定义_sys_exit避免使用半主机模式
void _sys_exit(int x) { 
	x = x; 
} 
#endif
//串口重定向函数printf，不使用到MicroLIB
int fputc(int ch,FILE *f) {
	HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}


//串口主程序初始化
void UART_Init(void)
{
    // 重新启动接收，使用Ex函数，接收不定长数据
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, f103_readBuffer, sizeof(f103_readBuffer));
    // 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}


/**
 * @brief 通过 UART2 发送多个 float 数据
 * @param data float 数组指针
 * @param count 数据个数
 * @param decimal_places 小数位数（如 2 表示保留 2 位小数）
 */
void UART_SendFloats(float *data, uint8_t count, uint8_t decimal_places) {
    char buffer[128];  // 缓冲区（根据数据量调整大小）
    char *ptr = buffer;  // 指针指向缓冲区起始位置
    // 遍历所有 float 数据
    for (uint8_t i = 0; i < count; i++) {
        // 格式化 float 到字符串（如 "1.23"）
        ptr += snprintf(ptr, sizeof(buffer) - (ptr - buffer), 
                      "%.*f", decimal_places, data[i]);
        // 添加逗号分隔（最后一个数据不加）
        if (i < count - 1) {
            ptr += snprintf(ptr, sizeof(buffer) - (ptr - buffer), ", ");
        }
    }
    ptr += snprintf(ptr, sizeof(buffer) - (ptr - buffer), "\n");    // 添加换行符
    // 通过 HAL_UART_Transmit 发送
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, ptr - buffer, HAL_MAX_DELAY);
}


// 数据解析函数
static float Get_Data(void) {
    uint8_t data_Start_Num = 0;
    uint8_t data_End_Num = 0;
    uint8_t minus_Flag = 0;
    float data_return = 0;
    // 查找等号和问号的位置
    for(uint8_t i = 0; i < 200; i++) {
        if(DataBuff[i] == '=') data_Start_Num = i + 1;
        if(DataBuff[i] == '?') {
            data_End_Num = i - 1;
            break;
        }
    }
    
    if(DataBuff[data_Start_Num] == '-') {
        data_Start_Num += 1;
        minus_Flag = 1;
    }
    // 简化数据处理逻辑
    data_return = 0;
    uint8_t decimal_point = 0;
    for(uint8_t i = data_Start_Num; i <= data_End_Num; i++) {
        if(DataBuff[i] == '.') {
            decimal_point = i;
            continue;
        }
        
        if(decimal_point == 0) {
            data_return = data_return * 10 + (DataBuff[i] - '0');
        }
        else {
            float decimal_place = 0.1f;
            for(uint8_t j = decimal_point + 1; j <= i; j++)
            {
                decimal_place *= 0.1f;
            }
            data_return += (DataBuff[i] - '0') * decimal_place;
        }
    }
    if(minus_Flag == 1) data_return = -data_return;
    return data_return;
}


// PID调整函数（需要根据你的实际PID结构体定义进行调整）
static void PID_Adjust(uint8_t Motor_n) {
    float data_Get = Get_Data();
    // 这里需要根据你的实际PID结构体进行修改
    // 示例代码：
    if(Motor_n == 1) {
        if(DataBuff[0]=='A' && DataBuff[1]=='1')
        {
            // PID_Pos.P = data_Get;
            Adjustable_Data = data_Get;
        }
        // 其他PID参数调整逻辑...
    }
}


/**
 * @description: 处理接收到的数据（回调函数）
 * @param {UART_HandleTypeDef} *huart
 * @param {uint16_t} Size
 * @return {*}
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        // 处理接收到的数据
        for(uint16_t i = 0; i < Size; i++) {
            RxLine++;
            if(RxLine <= sizeof(DataBuff)) {
                DataBuff[RxLine-1] = f103_readBuffer[i];
            }
            if(f103_readBuffer[i] == 0x3F) { // 结束标志
                PID_Adjust(1);
                memset(DataBuff, 0, sizeof(DataBuff));
                RxLine = 0;
                break;
            }
        }
        // 重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, f103_readBuffer, sizeof(f103_readBuffer));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}

