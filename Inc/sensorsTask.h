/*
 * sensorsTask.h
 *
 *  Created on: 04.12.2021
 *      Author: Slawek
 */

#ifndef __SENSORSTASK_H
#define __SENSORSTASK_H

#include "main.h"
//#include "cmsis_os.h"
#include "stm_bme_280.h"

#define MAX_NUMBER_OF_SENSORS 7
#define SENSORS_TIMEOUT 15 //sekund

/*typedef enum {
	Unidentified_type	 = 0,
	BMP_280_type		 = 1,
	BME_280_type 		 = 2
}BME280_typeOfSensor_t;*/

typedef struct
{
	float temp;
	float pres;
	float humi;
	BME280_typeOfSensor_t typeOfSensor;
	int timeout;
}BME280_t;




uint8_t GetActiveSensorBME280();
void SetActiveSensorBME280(uint8_t sensor);
void IncrementActiveSensorBME280(void);
void StartSensorsTask(void const * argument);
BME280_t* GetBME280Sensors();








#endif /* __SENSORSTASK_H */
