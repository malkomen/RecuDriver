/*
 * sensorsTask.c
 *
 *  Created on: 04.12.2021
 *      Author: Slawek
 */

#include "sensorsTask.h"
#include "stm_bme_280.h"



uint8_t activeSensorBME280 = 0;


void BME280_Handler();
void BME280_SetSpiMode();
void BME280_SetStatus();

BME280_t BME280[MAX_NUMBER_OF_SENSORS] = {0};
uint8_t bme280_activeSensor = 1;


void StartSensorsTask(void const * argument)
{
	BME280_SetSpiMode();
	BME280_SetStatus();

	/* Infinite loop */
	for(;;)
	{
		osDelay(200);
		BME280_Handler();
	}
}

void BME280_Handler()
{
	static uint32_t startTickTrace = 0;
	uint8_t activeSensor = GetActiveSensorBME280();


	if(BME280[activeSensor].typeOfSensor == Unidentified_type)
	{
		BME280[activeSensor].typeOfSensor = BME280_Initial(activeSensor,
												BME280_STANDBY_MS_10,
												BME280_FILTER_X8,
												BME280_TEMP_OVERSAMPLING_X8,
												BME280_PRES_OVERSAMPLING_X8,
												BME280_HUM_OVERSAMPLING_X8,
												BME280_MODE_NORMAL, 100);

		if(BME280[activeSensor].typeOfSensor != Unidentified_type)
			BME280[activeSensor].timeout = SENSORS_TIMEOUT;

	}
	else if(BME280[activeSensor].typeOfSensor == BME_280_type)
	{
		BME280[activeSensor].temp = BME280_ReadTemperature(activeSensor);
		BME280[activeSensor].pres = BME280_ReadPressure(activeSensor);
		BME280[activeSensor].humi = BME280_ReadHumidity(activeSensor);

		if(BME280[activeSensor].temp > -50.0)	//jesli wartosc mniejsza niz czujnik przesta³ odpowiadac, wartosc ponizej 50 stopni wynika ze wspolczynnikow korekcyjnych
			BME280[activeSensor].timeout = SENSORS_TIMEOUT;
	}
	else if(BME280[activeSensor].typeOfSensor == BMP_280_type)
	{
		BME280[activeSensor].temp = BME280_ReadTemperature(activeSensor);
		BME280[activeSensor].pres = BME280_ReadPressure(activeSensor);

		if(BME280[activeSensor].temp > -50.0)	//jesli wartosc mniejsza niz czujnik przesta³ odpowiadac, wartosc ponizej 50 stopni wynika ze wspolczynnikow korekcyjnych
			BME280[activeSensor].timeout = SENSORS_TIMEOUT;
	}

	IncrementActiveSensorBME280();

	//timeout handle
	if((HAL_GetTick() - startTickTrace) >= 1000)
	{
		startTickTrace = HAL_GetTick();
		for(int i = 0; i < MAX_NUMBER_OF_SENSORS ; i++)
		{
			if(BME280[i].timeout > 0) BME280[i].timeout--;
			if(BME280[i].timeout == 0) BME280[i].typeOfSensor = Unidentified_type;

		}
	}


}


/*void BME280_Handler2()
{
	static uint8_t ms = 0;


	BME280.statusBME = BME280_ERROR;

	switch (ms)
	{
	case 0:
		BME280.statusBME = BME280_Initial(BME280_STANDBY_MS_10,
											 BME280_FILTER_X8,
											 BME280_TEMP_OVERSAMPLING_X8,
											 BME280_PRES_OVERSAMPLING_X8,
											 BME280_HUM_OVERSAMPLING_X8,
											 BME280_MODE_NORMAL, 100);

		if(BME280.statusBME == BME280_OK) {ms = 1;}

	break;

	case 1:
		BME280.temp = BME280_ReadTemperature();
		BME280.pres = BME280_ReadPressure();
		BME280.humi = BME280_ReadHumidity();
	break;
	}
}*/

void BME280_SetSpiMode()
{
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_RESET); //SPI CS DOWN
	HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_RESET); //SPI CS DOWN
	HAL_GPIO_WritePin(SPI2_CS3_GPIO_Port, SPI2_CS3_Pin, GPIO_PIN_RESET); //SPI CS DOWN
	HAL_GPIO_WritePin(SPI2_CS4_GPIO_Port, SPI2_CS4_Pin, GPIO_PIN_RESET); //SPI CS DOWN
	HAL_GPIO_WritePin(SPI2_CS5_GPIO_Port, SPI2_CS5_Pin, GPIO_PIN_RESET); //SPI CS DOWN
	HAL_GPIO_WritePin(SPI2_CS6_GPIO_Port, SPI2_CS6_Pin, GPIO_PIN_RESET); //SPI CS DOWN
	HAL_GPIO_WritePin(SPI2_CS7_GPIO_Port, SPI2_CS7_Pin, GPIO_PIN_RESET); //SPI CS DOWN

	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET); //SPI CS UP
	HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_SET); //SPI CS UP
	HAL_GPIO_WritePin(SPI2_CS3_GPIO_Port, SPI2_CS3_Pin, GPIO_PIN_SET); //SPI CS UP
	HAL_GPIO_WritePin(SPI2_CS4_GPIO_Port, SPI2_CS4_Pin, GPIO_PIN_SET); //SPI CS UP
	HAL_GPIO_WritePin(SPI2_CS5_GPIO_Port, SPI2_CS5_Pin, GPIO_PIN_SET); //SPI CS UP
	HAL_GPIO_WritePin(SPI2_CS6_GPIO_Port, SPI2_CS6_Pin, GPIO_PIN_SET); //SPI CS UP
	HAL_GPIO_WritePin(SPI2_CS7_GPIO_Port, SPI2_CS7_Pin, GPIO_PIN_SET); //SPI CS UP

}

BME280_t* GetBME280Sensors()
{
	return BME280;
}

void BME280_SetStatus()
{
	for(int i = 0; i < MAX_NUMBER_OF_SENSORS; i++)
	{
		BME280[i].typeOfSensor = Unidentified_type;
	}
}

uint8_t GetActiveSensorBME280()
{
	return activeSensorBME280;
}

void SetActiveSensorBME280(uint8_t sensor)
{
	activeSensorBME280 = sensor;
}

void IncrementActiveSensorBME280(void)
{
	uint8_t tempActiveSensor = GetActiveSensorBME280();
	tempActiveSensor += 1;
	if(tempActiveSensor >= MAX_NUMBER_OF_SENSORS)
		tempActiveSensor = 0;
	SetActiveSensorBME280(tempActiveSensor);
}
