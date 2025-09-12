#ifndef __PRINTF_H
#define __PRINTF_H

#include "stm32f1xx_hal.h"

#define f103_BUFFER_SIZE 256
extern uint8_t DataBuff[200];
extern uint16_t RxLine;

void UART_Init(void);
void UART_SendFloats(float *data, uint8_t count, uint8_t decimal_places);


#endif // PRINTF_H

