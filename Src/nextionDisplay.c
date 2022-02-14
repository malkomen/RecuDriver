/*
 * nextionDisplay.c
 *
 *  Created on: 28.01.2022
 *      Author: Slawek
 */

//#include "stm32f4xx_hal_uart.h"

#include "main.h"
#include "circularbuffer.h"
#include "sensorsTask.h"

#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"


static char NextionDisplay_uartRxBuf[NEXTION_DISPLAY_BUFFERSIZE_UART];///< Buffer for circular uart rx.
CircularBufferObject_t rxBufferObject;
uint8_t RxDataUsart1;
//FlagStatus FlagRxDataNextion = RESET;
//uint8_t rxDataNextion[NEXTION_RX_BUF_SIZE] = {0};

extern UART_HandleTypeDef huart1;

/*
 *
 */
/*uint8_t* getRxDataNextion()									{return rxDataNextion;}

static void clearRxDataNextion()							{memset(getRxDataNextion(),0,NEXTION_RX_BUF_SIZE);}

static void setFlagRxDataNextion(FlagStatus stat)			{FlagRxDataNextion = stat;}

FlagStatus getFlagRxDataNextion()							{return FlagRxDataNextion;}*/

/*
 *
 */
void nextionTrace(const char* format, ...)
{
	int len = 100;
	va_list args;
	uint8_t txData[100]={0};
	va_start(args, format);
	len = vsnprintf((char*)txData, len, format, args);

	char* ptr = strchr((char*)txData, '\0');
	*ptr = 0xff;
	*(ptr+1) = 0xff;
	*(ptr+2) = 0xff;

	HAL_UART_Transmit(&huart1, (uint8_t*)txData, len+3,10);

	HAL_UART_Receive_IT(&huart1, &RxDataUsart1, 1);
}

/*
 *
 */
/*FlagStatus nextionWaitForFrame(uint32_t Timeout)
{
	uint32_t tickstart = 0U;

	tickstart = HAL_GetTick();

	while(getFlagRxDataNextion() != SET)
	{
		if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
		{
			return RESET;	//zwraca w przypadku przekroczenie czasu
		}
	}

	return SET;	//zwraca w przypadku odebrania poprawnych danych
}*/

/*
 *
 */
/*void nextionDisplayHandler()
{
	FlagStatus status = RESET;
	uint8_t dataNextion[NEXTION_RX_BUF_SIZE] = {0};

	status = nextionWaitForFrame(100);

	if(status == SET)
	{
		memcpy(dataNextion, getRxDataNextion(), NEXTION_RX_BUF_SIZE);
		setFlagRxDataNextion(RESET);
		clearRxDataNextion();

		switch(dataNextion[0])
		{
			case 0x30:	//wiadomosc uzytkownika, z poza protokolu nextion
			break;

			default:
			break;
		}
	}
}*/

/*
 *
 */
void odczytHarmonogramWdomu(char* buf)
{
	timeTablePlanner_t* tTP = GetTimeTablePlanner();
	memcpy(&(tTP->plan[planner_wDomu]), buf, 24*7);

	SaveSetting();
}

/*
 *
 */
void odczytHarmonogramWbiurze(char* buf)
{
	timeTablePlanner_t* tTP = GetTimeTablePlanner();
	memcpy(&(tTP->plan[planner_wBiurze]), buf, 24*7);

	SaveSetting();
}

/*
 *
 */
void odczytHarmonogramTydzWpracy(char* buf)
{
	timeTablePlanner_t* tTP = GetTimeTablePlanner();
	memcpy(&(tTP->plan[planner_tydzienWPracy]), buf, 24*7);

	SaveSetting();
}

/*
 *
 */
void odczytHarmonogramUzytkownika(char* buf)
{
	timeTablePlanner_t* tTP = GetTimeTablePlanner();
	memcpy(&(tTP->plan[planner_uzytkownia]), buf, 24*7);

	SaveSetting();
}

/*
 *
 */
void wyslanieHarmonogramWdomu()
{
	timeTablePlanner_t* tTP = GetTimeTablePlanner();

	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+2), tTP->plan[planner_wDomu].pon[i] + 3);	// + 3 id = 3 to indeks pierwsze obrazka odpowiadajacego za kolor szary
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+28), tTP->plan[planner_wDomu].wt[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+53), tTP->plan[planner_wDomu].sr[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+78), tTP->plan[planner_wDomu].czw[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+103), tTP->plan[planner_wDomu].pt[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+128), tTP->plan[planner_wDomu].sob[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+153), tTP->plan[planner_wDomu].niedz[i] + 3);
	}
}

/*
 *
 */
void wyslanieHarmonogramWbiurze()
{
	timeTablePlanner_t* tTP = GetTimeTablePlanner();

	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+2), tTP->plan[planner_wBiurze].pon[i] + 3);	// + 3 id = 3 to indeks pierwsze obrazka odpowiadajacego za kolor szary
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+28), tTP->plan[planner_wBiurze].wt[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+53), tTP->plan[planner_wBiurze].sr[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+78), tTP->plan[planner_wBiurze].czw[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+103), tTP->plan[planner_wBiurze].pt[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+128), tTP->plan[planner_wBiurze].sob[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+153), tTP->plan[planner_wBiurze].niedz[i] + 3);
	}
}

/*
 *
 */
void wyslanieHarmonogramTydzWpracy()
{
	timeTablePlanner_t* tTP = GetTimeTablePlanner();

	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+2), tTP->plan[planner_tydzienWPracy].pon[i] + 3);	// + 3 id = 3 to indeks pierwsze obrazka odpowiadajacego za kolor szary
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+28), tTP->plan[planner_tydzienWPracy].wt[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+53), tTP->plan[planner_tydzienWPracy].sr[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+78), tTP->plan[planner_tydzienWPracy].czw[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+103), tTP->plan[planner_tydzienWPracy].pt[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+128), tTP->plan[planner_tydzienWPracy].sob[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+153), tTP->plan[planner_tydzienWPracy].niedz[i] + 3);
	}
}

/*
 *
 */
void wyslanieHarmonogramUzytkownika()
{
	timeTablePlanner_t* tTP = GetTimeTablePlanner();

	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+2), tTP->plan[planner_uzytkownia].pon[i] + 3);	// + 3 id = 3 to indeks pierwsze obrazka odpowiadajacego za kolor szary
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+28), tTP->plan[planner_uzytkownia].wt[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+53), tTP->plan[planner_uzytkownia].sr[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+78), tTP->plan[planner_uzytkownia].czw[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+103), tTP->plan[planner_uzytkownia].pt[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+128), tTP->plan[planner_uzytkownia].sob[i] + 3);
	}
	for (int i = 0; i<24; i++)
	{
		nextionTrace("p%d.pic=%d", (i+153), tTP->plan[planner_uzytkownia].niedz[i] + 3);
	}
}




/*
 *
 */
void nextionDisplayHandler()
{
	static uint8_t ms=0;
	uint16_t unreadSize;
	static uint8_t parsingBuf[NEXTION_RX_BUF_SIZE] = {0};
	static uint8_t index;
	char* ptr;
	uint32_t value;
	char mark = 0;
	static nextionCyclicalTrace_t nextionCyclicalTrace = 0;


	//uint8_t mark = 0;
	static uint32_t startTickTrace = 0;


	switch (ms)
	{
		case 0:
			CircularBuffer_init(&rxBufferObject, NULL, 0);

			// Initialize the Rx buffer.
			CircularBuffer_init(&rxBufferObject, (uint8_t * const)NextionDisplay_uartRxBuf, NEXTION_DISPLAY_BUFFERSIZE_UART_2N);
			//start odbioru danych w przerwaniu
			HAL_UART_Receive_IT(&huart1, &RxDataUsart1, 1);


			startTickTrace = HAL_GetTick();

			ms=1;

		break;

		//pobranie linni danych
		case 1:

			unreadSize = CircularBuffer_getUnreadSize(&rxBufferObject);

			//kopiowanie linni danych z pominieciem znakow usunietych prze Backspace
			while(unreadSize)
			{
				CircularBuffer_popFrontByte(&rxBufferObject, &parsingBuf[index]);
				unreadSize-=1;

				//jesli odebrano 0xFFFFFF to znacz ze koniec wiadomosci z nextion
				if((index > 3) && (parsingBuf[index] == 0xFF) && (parsingBuf[index-1] == 0xFF) && (parsingBuf[index-2] == 0xFF))
				{
					parsingBuf[index] = 0;
					parsingBuf[index-1] = 0;
					parsingBuf[index-2] = 0;

					ms = 2;
					break;
				}
				else
				{
					index+=1;
				}
			}
		break;

		//parsowanie linni danych
		case 2:
		{
			//szukanmie znakow '=' lub '?'
			if((ptr = strchr((const char *)parsingBuf, '=')) != NULL)
			{
				//value = atoi(ptr+1);
				mark = '=';
				*ptr = 0;
			}
			else if((ptr = strchr((const char *)parsingBuf, '?')) != NULL)
			{
				mark = '?';
				*ptr = 0;
			}

			switch(parsingBuf[0])
			{
				case 0x30:	//wiadomosc uzytkownika, z poza protokolu nextion

					if(stricmp("test", (const char *)(&parsingBuf[1])) == 0)
					{
						value = ptr[1];
					}
					else if(stricmp("fan", (const char *)(&parsingBuf[1])) == 0)
					{
						value = ptr[1];
						TIM2->CCR2 = value * 10;
					}
					else if(stricmp("OdczytCzujnikow", (const char *)&parsingBuf[1]) == 0)
					{
						nextionCyclicalTrace = odczytCzujnikow;
					}
					else if(stricmp("Harm. w domu", (const char *)&parsingBuf[1]) == 0)
					{
						odczytHarmonogramWdomu(&ptr[1]);
					}
					else if(stricmp("Akt. Harm. w domu", (const char *)&parsingBuf[1]) == 0)
					{
						wyslanieHarmonogramWdomu();
					}
					else if(stricmp("Harm. w biurze", (const char *)&parsingBuf[1]) == 0)
					{
						odczytHarmonogramWbiurze(&ptr[1]);
					}
					else if(stricmp("Akt. Harm. w biurze", (const char *)&parsingBuf[1]) == 0)
					{
						wyslanieHarmonogramWbiurze();
					}
					else if(stricmp("Harm. tydz. w pracy", (const char *)&parsingBuf[1]) == 0)
					{
						odczytHarmonogramTydzWpracy(&ptr[1]);
					}
					else if(stricmp("Akt. Harm. tydz. w pracy", (const char *)&parsingBuf[1]) == 0)
					{
						wyslanieHarmonogramTydzWpracy();
					}
					else if(stricmp("Harm. uzytkownika", (const char *)&parsingBuf[1]) == 0)
					{
						odczytHarmonogramUzytkownika(&ptr[1]);
					}
					else if(stricmp("Akt. Harm. uzytkownika", (const char *)&parsingBuf[1]) == 0)
					{
						wyslanieHarmonogramUzytkownika();
					}

					else
					{
						//Trace("ERROR");
					}

				break;

				default:
				break;
			}



			ms = 1;
			memset(parsingBuf, 0, NEXTION_RX_BUF_SIZE);
			index = 0;
		}

		break;
	}

	//cyliczne generowanie trace
	switch (nextionCyclicalTrace)
	{
		case odczytCzujnikow:
			if((HAL_GetTick() - startTickTrace) >= 1000)
			{
				startTickTrace = HAL_GetTick();

				BME280_t* bme = GetBME280Sensors();

				for(int i=0 ; i < MAX_NUMBER_OF_SENSORS ; i++)
				{
					if(bme[i].typeOfSensor == Unidentified_type)
					{
						nextionTrace("t%d.txt=\"-\"", (i*3)+8);
						osDelay(5);
						nextionTrace("t%d.txt=\"-\"", (i*3)+9);
						osDelay(5);
						nextionTrace("t%d.txt=\"-\"", (i*3)+10);
						osDelay(5);
					}
					else if (bme[i].typeOfSensor == BMP_280_type)
					{
						nextionTrace("t%d.txt=\"%2.1f °C\"", (i*3)+8, bme[i].temp);
						osDelay(5);
						nextionTrace("t%d.txt=\"%d hPa\"", (i*3)+9, (int)(bme[i].pres/1000));
						osDelay(5);
						nextionTrace("t%d.txt=\"-\"", (i*3)+10);
						osDelay(5);
					}
					else if (bme[i].typeOfSensor == BME_280_type)
					{
						nextionTrace("t%d.txt=\"%2.1f °C\"", (i*3)+8, bme[i].temp);
						osDelay(5);
						nextionTrace("t%d.txt=\"%d hPa\"", (i*3)+9, (int)(bme[i].pres/1000));
						osDelay(5);
						nextionTrace("t%d.txt=\"%d %RH\"", (i*3)+10, (int)(bme[i].humi));
						osDelay(5);
					}

				}
				//nextionTrace("page 2");
				//dupa i kamieni kupa
				//dupa sama

			}

			break;
		default:
			break;
	}


}

//timeTablePlanner_t* GetTimeTablePlanner();

/*
 *
 */
void StartNextionDisplayTask(void const * argument)
{
	HAL_UART_Receive_IT(&huart1, &RxDataUsart1, 1);

	while(1)
	{
		nextionDisplayHandler();	//funkcja blokujaca, dlatego poznienie tylko 1ms
		osDelay(50);
	}

}

/*
 *
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//static uint8_t rxBuf[NEXTION_RX_BUF_SIZE] = {0};
	//static int16_t i = 0;
	//static uint32_t Tickstart = 0;


	//LedSet(LED3, off);
	if(huart->Instance == huart1.Instance)
	{
		CircularBuffer_pushBackByte(&rxBufferObject, RxDataUsart1);

		//obsluga z wykozystaniem funkcji blokujacej
/*		//jesli czas oczekiwania na kolejny zak dluzszy niz TIMEOUT_UART_CHAR to licz od nowa
		if ((HAL_GetTick() - Tickstart) > TIMEOUT_UART_CHAR)
		{
			i = 0;
			memset(rxBuf, 0, NEXTION_RX_BUF_SIZE);
		}
		Tickstart = HAL_GetTick();


		rxBuf[i] = RxDataUsart1;

		if((i>2) && (rxBuf[i]== 0xFF))
		{
			if((rxBuf[i-1]==0xFF) & (rxBuf[i-2]==0xFF))
			{
				memcpy(getRxDataNextion(), rxBuf, i-2);
				setFlagRxDataNextion(SET);
				i = 0;
				memset(rxBuf, 0, NEXTION_RX_BUF_SIZE);
			}
			else
			{
				i++;	//ikrementue wskaznik na buforze
			}
		}
		else
		{
			i++;	//ikrementue wskaznik na buforze
		}*/

		HAL_UART_Receive_IT(&huart1, &RxDataUsart1, 1);
	}

}
