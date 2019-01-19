
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f1xx_flash.h"
//#include "uart.h"
#include "stm32f1xx_hal_gpio_ex.h"
#include "stm32f1xx_it.h"
#include "LCD1602.h"
#include "ds1307.h"
#include "KeyADC.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t	ds1307ReadBuff[8];
extern volatile uint8_t  ds1307WriteBuff[8];
extern time time1;
char DataLCD[17];
RTC_TimeTypeDef rtcTime1;
RTC_DateTypeDef rtcDate1;
RTC_AlarmTypeDef rtcAlarm1,rtcAlarm2,rtcAlarm3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t	temp32;
	unsigned char temp8[2048]={1,0x00,0x01,0x20,1,0x00,0x02,0x20,1,0x00,0x03,0x20};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	__HAL_AFIO_REMAP_SWJ_ENABLE();
	HAL_Delay(1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	Init_LCD();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(BK_En_GPIO_Port,BK_En_Pin,GPIO_PIN_SET);
	StringDisplay_LCD(1,2,"Hello NaBeau!");
	HAL_Delay(1000);
	StringDisplay_LCD(2,2,"How are you?");
	HAL_Delay(1000);
	Cmd_LCD(0x01);
//	StringDisplay_LCD(1,2,"               ");
//	StringDisplay_LCD(2,2,"               ");
	StringDisplay_LCD(1,2,"How are you? ");
	StringDisplay_LCD(2,2,"Fine! Thanks");
	if(HAL_RTC_GetState(& hrtc)== HAL_RTC_STATE_RESET)//if rtc reset then configure default time
	{
		rtcTime1.Hours = 0x23;
		rtcTime1.Minutes = 0x59;
		rtcTime1.Seconds = 0x50;
		HAL_RTC_SetTime(&hrtc,&rtcTime1,RTC_FORMAT_BCD);
		rtcDate1.Date = 0x31;
		rtcDate1.Month = 0x12;
		rtcDate1.WeekDay = 0x01;
		rtcDate1.Year = 0x99;
		HAL_RTC_SetDate(&hrtc,&rtcDate1,RTC_FORMAT_BCD);
	}
	////////////////////////
//	ds1307WriteBuff[0]=0x19;
//	ds1307WriteBuff[1]=0;
//	ds1307WriteBuff[2]=0;
//	ds1307WriteBuff[3]=0;
//	ds1307WriteBuff[4]=0;
//	ds1307WriteBuff[5]=0;
//	ds1307WriteBuff[6]=0;
//	ds1307WriteBuff[7]=0x10;
//	HAL_I2C_Mem_Write_IT(&hi2c1,SLAVE_ADDRESS<<1,0x00,I2C_MEMADD_SIZE_16BIT,ds1307WriteBuff,8);
	HAL_Delay(5000);
	Cmd_LCD(0x01);
//	/////////////////////////////////
//	if(WriteFlash(temp8)!=0)
//	{
//		StringDisplay_LCD(1,1,"ERROR Flash!!");
//	}
//	/////////////////////////////////
	HAL_RTC_GetTime(&hrtc,&rtcTime1,RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc,&rtcDate1,RTC_FORMAT_BCD);
	if(rtcDate1.Year>0x99)
		rtcDate1.Year = 0;
	uint8_t tempdat[4];
	dayDisplay(tempdat,rtcDate1.WeekDay);
	sprintf(DataLCD,"%02X/%02X/%02X-%s",rtcDate1.Year,rtcDate1.Month,rtcDate1.Date,tempdat);
	StringDisplay_LCD(1,2,DataLCD);
	sprintf(DataLCD,"%02X:%02X:%02X",rtcTime1.Hours,rtcTime1.Minutes,rtcTime1.Seconds);
	StringDisplay_LCD(2,4,DataLCD);
	HAL_RTCEx_SetSecond_IT(&hrtc);
	///////////////////////////////
	temp32 = ReadFlash(FLASH_USER_START_ADDR);
	rtcAlarm1.Alarm	= temp32>>24;
	rtcAlarm1.AlarmTime.Hours = temp32>>16;
	rtcAlarm1.AlarmTime.Minutes = temp32>>8;
	rtcAlarm1.AlarmTime.Seconds = temp32;
	temp32 = ReadFlash(FLASH_USER_START_ADDR+4);
	rtcAlarm2.Alarm	= temp32>>24;
	rtcAlarm2.AlarmTime.Hours = temp32>>16;
	rtcAlarm2.AlarmTime.Minutes = temp32>>8;
	rtcAlarm2.AlarmTime.Seconds = temp32;
	temp32 = ReadFlash(FLASH_USER_START_ADDR+8);
	rtcAlarm3.Alarm	= temp32>>24;
	rtcAlarm3.AlarmTime.Hours = temp32>>16;
	rtcAlarm3.AlarmTime.Minutes = temp32>>8;
	rtcAlarm3.AlarmTime.Seconds = temp32;
	//HAL_RTC_SetAlarm_IT(&hrtc,&rtcAlarm1,RTC_FORMAT_BCD);
	///////////////////////////////////////////
	static uint8_t	Mode = 0,LeftRight = 0,UpDown = 0;
//	Cmd_LCD(0x80);
//	Cmd_LCD(0x0F);
  while (1)
  {
		printf("Loop here!\r\n");
//		HAL_RTC_GetTime(&hrtc,&rtcTime1,RTC_FORMAT_BCD);
//		sprintf(DataLCD,"%02X:%02X:%02X",rtcTime1.Hours,rtcTime1.Minutes,rtcTime1.Seconds);
//		StringDisplay_LCD(2,4,DataLCD);
//		ReadDs1307(ds1307ReadBuff);
//		sprintf(DataLCD,"%02X:%02X:%02X",time1.hours,time1.minutes,time1.seconds);
//		StringDisplay_LCD(2,5,DataLCD);
		switch(ScanKey())
		{
			case 	btnRIGHT:
				if(Mode == 1)
				{
					Cmd_LCD(0x82);
					Cmd_LCD(0x0F);
					LeftRight ++;
					switch(LeftRight)
					{
						case 1:
							
							break;
						case 2:
							break;
					}
				}
				break;
			case	btnLEFT:
				if(Mode == 1)
				{
					Cmd_LCD(0x82);
					Cmd_LCD(0x0F);
					LeftRight --;
				}
				break;
			case btnDOWN:
				UpDown --;
				__HAL_RTC_SECOND_DISABLE_IT(&hrtc,RTC_IT_SEC);
				switch(UpDown)
				{
					case 1://Setup mode
						StringDisplay_LCD(1,0,"1.View Alarm    ");
						StringDisplay_LCD(2,0,"Press SELECT?   ");
						break;
					case 2://
						StringDisplay_LCD(1,0,"2.Real Time     ");
						StringDisplay_LCD(2,0,"Press SELECT?   ");
						break;
					case 3:
						StringDisplay_LCD(1,0,"3.Alarm Time    ");
						StringDisplay_LCD(2,0,"Press SELECT?   ");
						break;
					case 0:
						UpDown = 4;
					case 4:
						StringDisplay_LCD(1,0,"4.Exit          ");
						StringDisplay_LCD(2,0,"Press SELECT?   ");
						break;
					default:
						UpDown = 1;
						break;
				}
				break;
			case btnUP:
				UpDown ++;
				__HAL_RTC_SECOND_DISABLE_IT(&hrtc,RTC_IT_SEC);
				switch(UpDown)
				{
					case 5:
						UpDown = 1;
					case 1://Setup mode
						StringDisplay_LCD(1,0,"1.View Alarm    ");
						StringDisplay_LCD(2,0,"Press SELECT?   ");
						break;
					case 2://
						StringDisplay_LCD(1,0,"2.Real Time     ");
						StringDisplay_LCD(2,0,"Press SELECT?   ");
						break;
					case 3:
						StringDisplay_LCD(1,0,"3.Alarm Time    ");
						StringDisplay_LCD(2,0,"Press SELECT?   ");
						break;
					case 4:
						StringDisplay_LCD(1,0,"4.Exit          ");
						StringDisplay_LCD(2,0,"Press SELECT?   ");
						break;
					default:
						UpDown = 1;
						break;
				}
				break;
			case btnSELECT://Mode function	
				switch(UpDown)
				{
					case 1:
						break;
					case 2:
						Cmd_LCD(0x01);
						HAL_RTC_GetTime(&hrtc,&rtcTime1,RTC_FORMAT_BCD);
						HAL_RTC_GetDate(&hrtc,&rtcDate1,RTC_FORMAT_BCD);
						if(rtcDate1.Year>0x99)
							rtcDate1.Year = 0;
						dayDisplay(tempdat,rtcDate1.WeekDay);
						sprintf(DataLCD,"%02X/%02X/%02X-%s",rtcDate1.Year,rtcDate1.Month,rtcDate1.Date,tempdat);
						StringDisplay_LCD(1,2,DataLCD);
						sprintf(DataLCD,"%02X:%02X:%02X",rtcTime1.Hours,rtcTime1.Minutes,rtcTime1.Seconds);
						StringDisplay_LCD(2,4,DataLCD);
						Mode = 1;
						break;
					case 3:
						break;
					case 4:
						Cmd_LCD(0x01);
						HAL_RTC_GetTime(&hrtc,&rtcTime1,RTC_FORMAT_BCD);
						HAL_RTC_GetDate(&hrtc,&rtcDate1,RTC_FORMAT_BCD);
						if(rtcDate1.Year>0x99)
							rtcDate1.Year = 0;
						dayDisplay(tempdat,rtcDate1.WeekDay);
						sprintf(DataLCD,"%02X/%02X/%02X-%s",rtcDate1.Year,rtcDate1.Month,rtcDate1.Date,tempdat);
						StringDisplay_LCD(1,2,DataLCD);
						sprintf(DataLCD,"%02X:%02X:%02X",rtcTime1.Hours,rtcTime1.Minutes,rtcTime1.Seconds);
						StringDisplay_LCD(2,4,DataLCD);
						__HAL_RTC_SECOND_ENABLE_IT(&hrtc,RTC_IT_SEC);
						break;
				}
				break;
			case btnNONE:
//				StringDisplay_LCD(2,1,"N");
				break;
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
  /* RTC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RTC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
