#ifndef __KEYADC_H
#define __KEYADC_H
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "LCD1602.h"
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
uint8_t ScanKey (void);
#endif /*__ LCD1602_H */
