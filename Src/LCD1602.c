#include "LCD1602.h"
//////////////////////////
// Connect LCD          //
// PD.0 <--> RS         //
// PD.1 <--> /E         //
// PC.4-PC.7 <--> D4-D7 //
//////////////////////////
void Data_LCD(unsigned char data)
{
	//high 4 bits
  HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_SET);//set Enable
	HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,GPIO_PIN_SET);//Rs = 1 ->Data
	HAL_GPIO_WritePin(Data4_GPIO_Port,Data4_Pin,(GPIO_PinState)((data&0x10)&&0x10));
	HAL_GPIO_WritePin(Data5_GPIO_Port,Data5_Pin,(GPIO_PinState)((data&0x20)&&0x20));
	HAL_GPIO_WritePin(Data6_GPIO_Port,Data6_Pin,(GPIO_PinState)((data&0x40)&&0x40));
	HAL_GPIO_WritePin(Data7_GPIO_Port,Data7_Pin,(GPIO_PinState)((data&0x80)&&0x80));
	HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_RESET);//Clear Enable
  HAL_Delay(1);
  //low 4 bits
  HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_SET);//set Enable
	HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,GPIO_PIN_SET);//Rs = 1 ->Data
	HAL_GPIO_WritePin(Data4_GPIO_Port,Data4_Pin,(GPIO_PinState)((data&0x01)&&0x01));
	HAL_GPIO_WritePin(Data5_GPIO_Port,Data5_Pin,(GPIO_PinState)((data&0x02)&&0x02));
	HAL_GPIO_WritePin(Data6_GPIO_Port,Data6_Pin,(GPIO_PinState)((data&0x04)&&0x04));
	HAL_GPIO_WritePin(Data7_GPIO_Port,Data7_Pin,(GPIO_PinState)((data&0x08)&&0x08));
	HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_RESET);//Clear Enable
  HAL_Delay(1);
}
void Cmd_LCD(unsigned char cmd)
{
    //high 4 bits
	HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_SET);//set Enable
	HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,GPIO_PIN_RESET);//Rs = 0 ->Command
	HAL_GPIO_WritePin(Data4_GPIO_Port,Data4_Pin,(GPIO_PinState)((cmd&0x10)&&0x10));
	HAL_GPIO_WritePin(Data5_GPIO_Port,Data5_Pin,(GPIO_PinState)((cmd&0x20)&&0x20));
	HAL_GPIO_WritePin(Data6_GPIO_Port,Data6_Pin,(GPIO_PinState)((cmd&0x40)&&0x40));
	HAL_GPIO_WritePin(Data7_GPIO_Port,Data7_Pin,(GPIO_PinState)((cmd&0x80)&&0x80));
	HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_RESET);//Clear Enable
  HAL_Delay(1);
  //low 4 bits
  HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_SET);//set Enable
	HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,GPIO_PIN_RESET);//Rs = 0 ->Command
	HAL_GPIO_WritePin(Data4_GPIO_Port,Data4_Pin,(GPIO_PinState)((cmd&0x01)&&0x01));
	HAL_GPIO_WritePin(Data5_GPIO_Port,Data5_Pin,(GPIO_PinState)((cmd&0x02)&&0x02));
	HAL_GPIO_WritePin(Data6_GPIO_Port,Data6_Pin,(GPIO_PinState)((cmd&0x04)&&0x04));
	HAL_GPIO_WritePin(Data7_GPIO_Port,Data7_Pin,(GPIO_PinState)((cmd&0x08)&&0x08));
	HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_RESET);//Clear Enable
  HAL_Delay(1);
}
void Init_LCD(void)
{	 
  Cmd_LCD(0x33);
  Cmd_LCD(0x32);
  Cmd_LCD(0x28);
  Cmd_LCD(0x06);
  Cmd_LCD(0x0C);
  Cmd_LCD(0x01);
	HAL_Delay(1);
}
////////////////////////////
// line is 1, 2, 3 or 4   //
// position is 1 to 20    //
// *data is display string//
////////////////////////////
void StringDisplay_LCD(unsigned char linen, unsigned char position, const unsigned char *data)
{
  unsigned char temp=0;
  while(data[++temp]!='\0');
  switch (linen)
  {
    case 1:
         Cmd_LCD(0x80+position);
         break;
    case 2:
         Cmd_LCD(0xC0+position);
         break;
    case 3:
         Cmd_LCD(0x90+position);
         break;
    case 4:
         Cmd_LCD(0xD0+position);
         break;
    default:
         break;
  }
  linen=0;
  while(data[linen]!='\0')
    Data_LCD(data[linen++]);
}
//////////////////
void Clear_LCD(void)
{
	Cmd_LCD(0x01);
	HAL_Delay(1);
}
