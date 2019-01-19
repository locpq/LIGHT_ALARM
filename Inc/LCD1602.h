#ifndef __LCD1602_H
#define __LCD1602_H
#include "stm32f1xx_hal.h"
#include "gpio.h"
void Data_LCD(unsigned char data);
void Cmd_LCD(unsigned char cmd);
void Init_LCD(void);
void StringDisplay_LCD(unsigned char linen, unsigned char position, const unsigned char *data);
void Clear_LCD(void);
#endif /*__ LCD1602_H */
