/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "LCD1602.h"
extern char DataLCD[17];
extern RTC_TimeTypeDef rtcTime1;
extern RTC_DateTypeDef rtcDate1;
extern RTC_AlarmTypeDef rtcAlarm1,rtcAlarm2,rtcAlarm3;
static	uint8_t	tempSecond, TempData[4];
void dayDisplay(uint8_t *WeekDays,char dat)
{
	switch(dat)
	{
		case 1:
			WeekDays[0] = 'S';
			WeekDays[1] = 'u';
			WeekDays[2] = 'n';
			WeekDays[3] = NULL;
			break;
		case 2:
			WeekDays[0] = 'M';
			WeekDays[1] = 'o';
			WeekDays[2] = 'n';
			WeekDays[3] = NULL;
			break;
		case 3:
			WeekDays[0] = 'T';
			WeekDays[1] = 'u';
			WeekDays[2] = 'e';
			WeekDays[3] = NULL;
			break;
		case 4:
			WeekDays[0] = 'W';
			WeekDays[1] = 'e';
			WeekDays[2] = 'd';
			WeekDays[3] = NULL;
			break;
		case 5:
			WeekDays[0] = 'T';
			WeekDays[1] = 'h';
			WeekDays[2] = 'u';
			WeekDays[3] = NULL;
			break;
		case 6:
			WeekDays[0] = 'F';
			WeekDays[1] = 'r';
			WeekDays[2] = 'i';
			WeekDays[3] = NULL;
			break;
		case 7:
			WeekDays[0] = 'S';
			WeekDays[1] = 'a';
			WeekDays[2] = 't';
			WeekDays[3] = NULL;
			break;
	}
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RTC global interrupt.
*/
void RTC_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_IRQn 0 */

  /* USER CODE END RTC_IRQn 0 */
  HAL_RTCEx_RTCIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_IRQn 1 */
	HAL_RTC_GetTime(&hrtc,&rtcTime1,RTC_FORMAT_BCD);
	if(tempSecond!=rtcTime1.Seconds)
	{
		tempSecond = rtcTime1.Seconds;
		if(!rtcTime1.Seconds)
		{
			if(!rtcTime1.Minutes)
			{
				if(!rtcTime1.Hours)
				{
					HAL_RTC_GetDate(&hrtc,&rtcDate1,RTC_FORMAT_BCD);
					if(rtcDate1.Date == 0x01)
					{
						if(rtcDate1.Month == 0x01)
						{
							if(rtcDate1.Year>0x99)
								rtcDate1.Year = 0;
							dayDisplay(TempData,rtcDate1.WeekDay);
							sprintf(DataLCD,"%02X/%02X/%02X-%s",rtcDate1.Year,rtcDate1.Month,rtcDate1.Date,TempData);
							StringDisplay_LCD(1,2,DataLCD);
							sprintf(DataLCD,"%02X:%02X:%02X",rtcTime1.Hours,rtcTime1.Minutes,rtcTime1.Seconds);
							StringDisplay_LCD(2,4,DataLCD);
						}
						else
						{
							dayDisplay(TempData,rtcDate1.WeekDay);
							sprintf(DataLCD,"%02X/%02X-%s",rtcDate1.Month,rtcDate1.Date,TempData);
							StringDisplay_LCD(1,5,DataLCD);
							sprintf(DataLCD,"%02X:%02X:%02X",rtcTime1.Hours,rtcTime1.Minutes,rtcTime1.Seconds);
							StringDisplay_LCD(2,4,DataLCD);
						}
					}
					else
					{
						dayDisplay(TempData,rtcDate1.WeekDay);
						sprintf(DataLCD,"%02X-%s",rtcDate1.Date,TempData);
						StringDisplay_LCD(1,8,DataLCD);
						sprintf(DataLCD,"%02X:%02X:%02X",rtcTime1.Hours,rtcTime1.Minutes,rtcTime1.Seconds);
						StringDisplay_LCD(2,4,DataLCD);
					}
				}
				else
				{
					sprintf(DataLCD,"%02X:%02X:%02X",rtcTime1.Hours,rtcTime1.Minutes,rtcTime1.Seconds);
					StringDisplay_LCD(2,4,DataLCD);
				}
			}
			else
			{
				sprintf(DataLCD,"%02X:%02X",rtcTime1.Minutes,rtcTime1.Seconds);
				StringDisplay_LCD(2,7,DataLCD);
			}
		}
		else
		{
			sprintf(DataLCD,"%02X",tempSecond);
			StringDisplay_LCD(2,10,DataLCD);
		}
	}
	//////////////////////////////////////////////
	//Alarm here!!!
	//////////////////////////////////////////////
	if((rtcTime1.Hours == rtcAlarm1.AlarmTime.Hours)&&(rtcTime1.Minutes == rtcAlarm1.AlarmTime.Minutes)&&(rtcTime1.Seconds == rtcAlarm1.AlarmTime.Seconds))
	{
//		HAL_GPIO_WritePin(BK_En_GPIO_Port,BK_En_Pin,GPIO_PIN_RESET);//Turn off backlight
		HAL_GPIO_WritePin(RELAY5_GPIO_Port,RELAY5_Pin,GPIO_PIN_RESET);//Turn on Light spa
		HAL_GPIO_WritePin(RELAY12_GPIO_Port,RELAY12_Pin,GPIO_PIN_RESET);//Turn on Light Table
	}
	else if((rtcTime1.Hours == rtcAlarm2.AlarmTime.Hours)&&(rtcTime1.Minutes == rtcAlarm2.AlarmTime.Minutes)&&(rtcTime1.Seconds == rtcAlarm2.AlarmTime.Seconds))
	{
//		HAL_GPIO_WritePin(BK_En_GPIO_Port,BK_En_Pin,GPIO_PIN_SET);//Turn on backlight
		HAL_GPIO_WritePin(RELAY5_GPIO_Port,RELAY5_Pin,GPIO_PIN_RESET);//Turn on Light spa
		HAL_GPIO_WritePin(RELAY12_GPIO_Port,RELAY12_Pin,GPIO_PIN_SET);//Turn off Light Table
	}
	else if((rtcTime1.Hours == rtcAlarm3.AlarmTime.Hours)&&(rtcTime1.Minutes == rtcAlarm3.AlarmTime.Minutes)&&(rtcTime1.Seconds == rtcAlarm2.AlarmTime.Seconds))
	{
//		HAL_GPIO_WritePin(BK_En_GPIO_Port,BK_En_Pin,GPIO_PIN_RESET);//Turn off backlight
		HAL_GPIO_WritePin(RELAY5_GPIO_Port,RELAY5_Pin,GPIO_PIN_SET);//Turn off Light spa
		HAL_GPIO_WritePin(RELAY12_GPIO_Port,RELAY12_Pin,GPIO_PIN_SET);//Turn off Light Table
	}
  /* USER CODE END RTC_IRQn 1 */
}

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
	
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel7 global interrupt.
*/
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
