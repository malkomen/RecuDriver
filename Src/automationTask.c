/*
 * automationTask.c
 *
 *  Created on: 14.02.2022
 *      Author: Slawek
 */

#include "main.h"
#include "circularbuffer.h"
#include "automationTask.h"

#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"

#define FLASH_ADDES_SETUP 0x08020000
#define FLASH_ADDES_SETUP_CHECKSUM 0x08040000

extern CRC_HandleTypeDef hcrc;

timeTablePlanner_t timeTablePlanner = {0};
uint32_t setupCheckSum = 0;

timeTablePlanner_t* GetTimeTablePlanner() {return &timeTablePlanner;}

/*
 *
 *
 */
HAL_StatusTypeDef FlashWrite(uint32_t address, uint8_t* ptr, uint16_t lenght)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	uint32_t timeoutFlash = 50000;
	uint16_t i;

	HAL_FLASH_Unlock();

	status = FLASH_WaitForLastOperation((uint32_t)timeoutFlash);

	if(status == HAL_OK)
	{
		if (address == FLASH_ADDES_SETUP)
		{
			FLASH_Erase_Sector(FLASH_SECTOR_5, VOLTAGE_RANGE_3);
		}
		else if(address == FLASH_ADDES_SETUP_CHECKSUM)
		{
			FLASH_Erase_Sector(FLASH_SECTOR_6, VOLTAGE_RANGE_3);
		}

		status = FLASH_WaitForLastOperation((uint32_t)timeoutFlash);

		if(status == HAL_OK)
		{
			for (i=0; i<lenght; i++)
			{
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, ptr[i])!=HAL_OK)
				{
					return status;
				}
			}
			HAL_FLASH_Lock();
		}
	}
	return status;
}

HAL_StatusTypeDef SaveSetting()
{
	HAL_StatusTypeDef status = HAL_ERROR;
	uint32_t length = 0;
	uint16_t size = 0;

	size = sizeof(timeTablePlanner_t);

	if(size%4==0)
		length = size/4;
	else
		length=size/4 + 1;

	 setupCheckSum = 0x00;
	 setupCheckSum = HAL_CRC_Calculate(&hcrc, (uint32_t*)(&timeTablePlanner), length);

	 status = FlashWrite(FLASH_ADDES_SETUP, (uint8_t*)&timeTablePlanner, sizeof(timeTablePlanner_t));
	 if(status != HAL_OK)
	 {
		 return status;
	 }

	 status = FlashWrite(FLASH_ADDES_SETUP_CHECKSUM, (uint8_t*)&setupCheckSum, sizeof(setupCheckSum));

	 return status;
}

void readSetting()
{
	uint32_t checkSumSaved = 0;
	uint32_t checkSumCalc = 0;
	uint32_t length = 0;
	uint16_t size = 0;

	size = sizeof(timeTablePlanner_t);

	if(size%4==0)
		length = size/4;
	else
		length=size/4 + 1;

	//checkSumeSaved = memcpy(&checkSumSaved, (void*)(FLASH_ADDES_SETUP) + sizeof(Setup_t) + 16, sizeof(uint32_t));
	//odczyt w flash wartosci zapisanej sumy kontrolnej wyliczonej z Setup
	checkSumSaved = *(uint32_t*)(FLASH_ADDES_SETUP_CHECKSUM);

	memcpy(&timeTablePlanner, (void*)(FLASH_ADDES_SETUP), sizeof(timeTablePlanner_t));

	checkSumCalc = (HAL_CRC_Calculate(&hcrc, (uint32_t*)(&timeTablePlanner), length));

	if(checkSumSaved != checkSumCalc)
	{
		//jesli suma kontrolna sie nie zgada to przypisuje paramtry domyslne
		memset(&timeTablePlanner, 0,  sizeof(timeTablePlanner_t));
	}

}


/*
 *
 */
void StartAutomationTask(void const * argument)
{

	readSetting();

	while(1)
	{
		osDelay(100);
	}

}
