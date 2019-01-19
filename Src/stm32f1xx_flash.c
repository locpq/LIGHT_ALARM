#include "stm32f1xx_flash.h"
/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;
//////////////////////////////////////////////
//Output: 0: ok
//				1: Erase fail
//				2: Write fail
//				3: Verify fail
//////////////////////////////////////////////
unsigned char WriteFlash(unsigned char *StrData)
{
	uint32_t Address = 0, PageError = 0;
	__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
	/* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
  EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PageError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
    /* Infinite loop */
		return 1;//Erase Fail
  }
  /* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  Address = FLASH_USER_START_ADDR;

  while (Address < FLASH_USER_END_ADDR)
  {
		MemoryProgramStatus = ((uint32_t)StrData[Address - FLASH_USER_START_ADDR])<<24;
		MemoryProgramStatus |= ((uint32_t)StrData[Address - FLASH_USER_START_ADDR+1])<<16;
		MemoryProgramStatus |= ((uint32_t)StrData[Address - FLASH_USER_START_ADDR+2])<<8;
		MemoryProgramStatus |= ((uint32_t)StrData[Address - FLASH_USER_START_ADDR+3]);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, MemoryProgramStatus) == HAL_OK)
    {
      Address = Address + 4;
    }
    else
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
			return 2; //WriteFlash Fail
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  /* Check if the programmed data is OK
      MemoryProgramStatus = 0: data programmed correctly
      MemoryProgramStatus != 0: number of words not programmed correctly ******/
  Address = FLASH_USER_START_ADDR;
  MemoryProgramStatus = 0x0;

  while (Address < FLASH_USER_END_ADDR)
  {
		MemoryProgramStatus = ((uint32_t)StrData[Address - FLASH_USER_START_ADDR])<<24;
		MemoryProgramStatus |= ((uint32_t)StrData[Address - FLASH_USER_START_ADDR+1])<<16;
		MemoryProgramStatus |= ((uint32_t)StrData[Address - FLASH_USER_START_ADDR+2])<<8;
		MemoryProgramStatus |= ((uint32_t)StrData[Address - FLASH_USER_START_ADDR+3]);
		
    data32 = *(__IO uint32_t *)Address;//Read flash
    if (data32 != MemoryProgramStatus)
    {
      return 3;
    }
    Address = Address + 4;
  }
	return 0;//Ok
}
//////////////////////////////////
////
//////////////////////////////////
//void LoadSpeedSpindle(uint16_t load)
//{
//	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin,(GPIO_PinState)((load & 0x0001) && 0x0001));
//	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin,(GPIO_PinState)((load & 0x0002) && 0x0002));
//	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin,(GPIO_PinState)((load & 0x0004) && 0x0004));
//	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin,(GPIO_PinState)((load & 0x0008) && 0x0008));
//	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin,(GPIO_PinState)((load & 0x0010) && 0x0010));
//	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin,(GPIO_PinState)((load & 0x0020) && 0x0020));
//	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin,(GPIO_PinState)((load & 0x0040) && 0x0040));
//	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin,(GPIO_PinState)((load & 0x0080) && 0x0080));
//	HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin,(GPIO_PinState)((load & 0x0100) && 0x0100));
//	HAL_GPIO_WritePin(D9_GPIO_Port, D9_Pin,(GPIO_PinState)((load & 0x0200) && 0x0200));
//	HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin,(GPIO_PinState)((load & 0x0400) && 0x0400));
//	HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin,(GPIO_PinState)((load & 0x0800) && 0x0800));
//}
