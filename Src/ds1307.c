#include "ds1307.h"
volatile uint8_t	ds1307ReadBuff[8];
volatile uint8_t ds1307WriteBuff[8];
extern char DataLCD[17];
time time1;
void ReadDs1307(uint8_t *data)
{
	HAL_I2C_Mem_Read_DMA(&hi2c1,SLAVE_ADDRESS<<1,0x00,I2C_MEMADD_SIZE_16BIT,data,7);
	HAL_Delay(200);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == hi2c1.Instance)
	{
		time1.seconds = ds1307ReadBuff[0];
		time1.minutes = ds1307ReadBuff[1];
		time1.hours = ds1307ReadBuff[2];
		time1.day_of_week = ds1307ReadBuff[3];
		time1.date = ds1307ReadBuff[4];
		time1.month = ds1307ReadBuff[5];
		time1.year = ds1307ReadBuff[6];
//		sprintf(DataLCD,"%02X:%02X:%02X",time1.hours,time1.minutes,time1.seconds);
//		StringDisplay_LCD(2,5,DataLCD);
	}
}
