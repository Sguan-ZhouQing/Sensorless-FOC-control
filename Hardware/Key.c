/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-04 15:04:29
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-08-29 16:23:32
 * @FilePath: \demo_STM32F103FocCode\Hardware\Key.c
 * @Description: TIM定时中断，触发更新Key_Num
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Key.h"

#define Key_Dissipate_Time 20
uint8_t Key_Num;

//GPIO引脚的宏定义，方便代码管理
#define Key1_GPIO_Port GPIOA
#define Key1_Pin GPIO_PIN_12


/**
 * @description: 读取GPIO_Input的不同引脚的数值;
 * @return {*}
 */
uint8_t Key_GetState(void)
{
    if (HAL_GPIO_ReadPin(Key1_GPIO_Port,Key1_Pin) == 0){
        return 1;
    }
    return 0;
}

/**
 * @description: 更新Key_Num的数值
 * @return {*}
 */
uint8_t Key_GetNum(void)
{
    uint8_t Temp;
    if (Key_Num)
    {
        Temp = Key_Num;
        Key_Num = 0;
        return Temp;
    }
    return 0;
}

/**
 * @description: TIM定时中断的写入函数（可设1ms中断）
 * @return {*}
 */
void Key_Tick(void)
{
    static uint8_t Key_Count;
    static uint8_t CurrState, PrevState;
    Key_Count++;
    if (Key_Count >= Key_Dissipate_Time)
    {
        Key_Count = 0;

        PrevState = CurrState;
        CurrState = Key_GetState();
        if (CurrState == 0 && PrevState != 0)
        {
            Key_Num = PrevState;
        }
    }
}

/**
 * @description: TIM2中断回调函数，1ms的定时器定时中断
 * @param {TIM_HandleTypeDef*} htim
 * @return {*}
 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim == &htim2) {  // 1kHz中断
//         Key_Tick();
//     }
// }

