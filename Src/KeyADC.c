#include "KeyADC.h"
volatile uint16_t ADC_Value;
char datalcd[17];
uint8_t ScanKey (void)
{
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_Value,1);
	HAL_Delay(100);
	HAL_ADC_Stop_DMA(&hadc1);
	if (ADC_Value < 50)   return btnRIGHT;  
	if (ADC_Value < 700)  return btnUP; 
	if (ADC_Value < 1200)  return btnDOWN; 
	if (ADC_Value < 1900)  return btnLEFT; 
	if (ADC_Value < 2800)  return btnSELECT;   
 return btnNONE;  // when all others fail, return this..
}
