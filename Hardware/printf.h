#ifndef __PRINTF_H
#define __PRINTF_H

#include "stm32f1xx_hal.h"

#define f103_BUFFER_SIZE 256

void UART_SendFloats(float *data, uint8_t count, uint8_t decimal_places);


#endif // PRINTF_H

