#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"

uint8_t Key_GetState(void);
uint8_t Key_GetNum(void);
void Key_Tick(void);


#endif //Key_H

