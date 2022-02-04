#include "stm_bme_280.h"
//#include "stm32f4xx_hal_i2c.h"
//#include "stm32f4xx_hal_spi.h"
//#include "main.h"
#include "sensorsTask.h"

//#define MAX_NUMBER_OF_SENSORS 7


#define SPI 1
//#define I2C

#ifdef I2C
extern I2C_HandleTypeDef 	hi2c2;

#elif SPI
extern SPI_HandleTypeDef hspi2;

#else
	#error Zdefiniuj interface
#endif
BME280_CalibData CalibData[MAX_NUMBER_OF_SENSORS];
BME280_Settings BME280_Set[MAX_NUMBER_OF_SENSORS];
int32_t tFineValue[MAX_NUMBER_OF_SENSORS] = {0};
//-------------------------------------------------------------------------------
//I2C Communication functions
static inline void errorHandler()
{
	/* Error handler */
}

//-------------------------------SPI-------------------------------------------
#ifdef SPI

/*
 *
 */
static void SPIx_CsDown()
{
	uint8_t tempActiveSensor = GetActiveSensorBME280();

	switch(tempActiveSensor)
	{
		case 0: HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_RESET); break;
		case 1: HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_RESET); break;
		case 2: HAL_GPIO_WritePin(SPI2_CS3_GPIO_Port, SPI2_CS3_Pin, GPIO_PIN_RESET); break;
		case 3: HAL_GPIO_WritePin(SPI2_CS4_GPIO_Port, SPI2_CS4_Pin, GPIO_PIN_RESET); break;
		case 4: HAL_GPIO_WritePin(SPI2_CS5_GPIO_Port, SPI2_CS5_Pin, GPIO_PIN_RESET); break;
		case 5: HAL_GPIO_WritePin(SPI2_CS6_GPIO_Port, SPI2_CS6_Pin, GPIO_PIN_RESET); break;
		case 6: HAL_GPIO_WritePin(SPI2_CS7_GPIO_Port, SPI2_CS7_Pin, GPIO_PIN_RESET); break;
	}
}

/*
 *
 */
static void SPIx_CsUp()
{
	HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS3_GPIO_Port, SPI2_CS3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS4_GPIO_Port, SPI2_CS4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS5_GPIO_Port, SPI2_CS5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS6_GPIO_Port, SPI2_CS6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS7_GPIO_Port, SPI2_CS7_Pin, GPIO_PIN_SET);

}
/*
 *
 */
static inline void SPIx_WriteData(uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef operationStatus = HAL_ERROR;

  uint8_t writeData[2] = {Reg & 0x7F, Value};

  //operationStatus = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
  SPIx_CsDown();
  operationStatus = HAL_SPI_Transmit(&hspi2, writeData, 2, 10);
  SPIx_CsUp();

  if(operationStatus != HAL_OK)
  {
	  errorHandler();
  }
}

/*
 *
 */
static inline uint8_t SPIx_ReadData(uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t writeData[2] = {Reg, 0};
  uint8_t readData[2] = {0};

  //uint8_t readedValue = 0;

  //status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &readedValue, 1, 0x10000);
  SPIx_CsDown();
  status = HAL_SPI_TransmitReceive(&hspi2, writeData, readData, 2, 10);
  SPIx_CsUp();

  if(status != HAL_OK)
  {
	  errorHandler();
  }
  return readData[1];
  //return readedValue;
}

/*
 *
 */
static void SPIx_ReadData16(uint8_t Reg, uint16_t *Value)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t writeData[3] = {Reg, 0, 0};
  uint8_t readData[3] = {0};

  //status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 2, 0x10000);
  SPIx_CsDown();
  status = HAL_SPI_TransmitReceive(&hspi2, writeData, readData, 3, 10);
  SPIx_CsUp();

  if(status != HAL_OK)
  {
	  errorHandler();
  }

  //Value = (uint16_t*)&readData[1];
  *Value = readData[2]<<8 | readData[1];
}

/*
 *
 */
static void SPIx_ReadData24(uint8_t Reg, uint32_t *Value)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t writeData[4] = {Reg, 0, 0, 0};
  uint8_t readData[4] = {0};

  //status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 3, 0x10000);
  SPIx_CsDown();
  status = HAL_SPI_TransmitReceive(&hspi2, writeData, readData, 4, 10);
  SPIx_CsUp();

  if(status != HAL_OK)
  {
	  errorHandler();
  }
  //Value = (uint32_t*)&readData[1];
  *Value = readData[3]<<16 | readData[2]<<8 | readData[1];
}

#elif I2C
//-------------------------------I2C-------------------------------------------
static inline void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef operationStatus = HAL_ERROR;

  //operationStatus = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);

  if(operationStatus != HAL_OK)
  {
	  errorHandler();
  }
}

static inline uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  uint8_t readedValue = 0;

  //status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &readedValue, 1, 0x10000);

  if(status != HAL_OK)
  {
	  errorHandler();
  }
  return readedValue;
}

static void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  //status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 2, 0x10000);
  if(status != HAL_OK)
  {
	  errorHandler();
  }
}

static void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  //status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 3, 0x10000);
  if(status != HAL_OK)
  {
	  errorHandler();
  }
}
#endif
//-------------------------------------------------------------------------------
/* Static declarations */
static void bme280_WriteReg(uint8_t Reg, uint8_t Value);
static uint8_t bme280_ReadReg(uint8_t Reg);
static void bme280_ReadRegPtr(uint8_t readRegister, uint8_t *ptrReadedValue);
static uint8_t bme280_ReadStatus(void);
static void bme280_ReadData16(uint8_t readRegister, uint16_t *ptrReadedValue);
static void bme280_ReadSignedData16(uint8_t readRegister, int16_t *ptrReadedValue);
static void bme280_ReadSignedData16_Convert(uint8_t readRegister, int16_t *ptrReadedValue);
static void bme280_ReadRegDataConvert24(uint8_t readRegister, uint32_t *ptrReadedValue);
static void bme280_ReadCoefficients(uint8_t sensor);
static void bmp280_ReadCoefficients(uint8_t sensor);
static void bme280_SetStandby(BME280_standby_Time_E standByTime);
static void bme280_SetFilter(BME280_filter_E filter);
static void bme280_SetOversamplingTemper(BME280_overSamplingTemp_E tempOversampl);
static void bme280_SetOversamplingPressure(BME280_overSamplingPres_E presOversampl);
static void bme280_SetOversamplingHum(BME280_overSamplingHum_E humOversampl);
static void bme280_SetMode(BME280_mode_E mode);
//-------------------------------------------------------------------------------
BME280_typeOfSensor_t BME280_Initial(uint8_t activeSensor, BME280_standby_Time_E standbyTime, BME280_filter_E filter,
					  BME280_overSamplingTemp_E tempOversampl, BME280_overSamplingPres_E presOversampl,
					  BME280_overSamplingHum_E humOversampl, BME280_mode_E sensMode, uint16_t timeout)
{
	uint32_t tickstart = HAL_GetTick();


	BME280_typeOfSensor_t tempTypeOfSensor;

	BME280_Set[activeSensor].sensorID = bme280_ReadReg(BME280_REG_ID);

	if(BME280_Set[activeSensor].sensorID == BME280_ID)
	{
		BME280_Set[activeSensor].standbyTime = standbyTime;
		BME280_Set[activeSensor].filter = filter;
		BME280_Set[activeSensor].tempOversampl = tempOversampl;
		BME280_Set[activeSensor].presOversampl = presOversampl;
		BME280_Set[activeSensor].humOversampl = humOversampl;
		BME280_Set[activeSensor].sensMode = sensMode;

		bme280_WriteReg(BME280_REG_SOFTRESET, BME280_SOFTRESET_VALUE);

		while (bme280_ReadStatus() & BME280_STATUS_IM_UPDATE)
		{
			if(HAL_GetTick() - tickstart > timeout)
				return BME280_ERROR;
		}

		bme280_ReadCoefficients(activeSensor);

		bme280_SetStandby(BME280_Set[activeSensor].standbyTime);
		bme280_SetFilter(BME280_Set[activeSensor].filter);

		bme280_SetOversamplingTemper(BME280_Set[activeSensor].tempOversampl);
		bme280_SetOversamplingPressure(BME280_Set[activeSensor].presOversampl);
		bme280_SetOversamplingHum(BME280_Set[activeSensor].humOversampl);

		BME280_Set[activeSensor].measurementStatus = bme280_ReadReg(BME280_REG_CTRL_MEAS);
		BME280_Set[activeSensor].measurementStatus = bme280_ReadReg(BME280_REG_CTRL_HUM) << 8;

		BME280_Set[activeSensor].tempOn = (BME280_Set[activeSensor].measurementStatus & BME280_OSRS_T_MSK) ? 1 : 0;
		BME280_Set[activeSensor].presOn = (BME280_Set[activeSensor].measurementStatus & BME280_OSRS_P_MSK) ? 1 : 0;
		BME280_Set[activeSensor].humiOn = ((BME280_Set[activeSensor].measurementStatus >> 8) & BME280_OSRS_H_MSK) ? 1 : 0;

		bme280_SetMode(BME280_Set[activeSensor].sensMode);

		tempTypeOfSensor = BME_280_type;
	}
	else if(BME280_Set[activeSensor].sensorID == BMP280_ID)
	{
		BME280_Set[activeSensor].standbyTime = standbyTime;
		BME280_Set[activeSensor].filter = filter;
		BME280_Set[activeSensor].tempOversampl = tempOversampl;
		BME280_Set[activeSensor].presOversampl = presOversampl;
		//BME280_Set.humOversampl = humOversampl;
		BME280_Set[activeSensor].sensMode = sensMode;

		bme280_WriteReg(BME280_REG_SOFTRESET, BME280_SOFTRESET_VALUE);

		while (bme280_ReadStatus() & BME280_STATUS_IM_UPDATE)
		{
			if(HAL_GetTick() - tickstart > timeout)
				return BME280_ERROR;
		}

		bmp280_ReadCoefficients(activeSensor);

		bme280_SetStandby(BME280_Set[activeSensor].standbyTime);
		bme280_SetFilter(BME280_Set[activeSensor].filter);

		bme280_SetOversamplingTemper(BME280_Set[activeSensor].tempOversampl);
		bme280_SetOversamplingPressure(BME280_Set[activeSensor].presOversampl);
		//bme280_SetOversamplingHum(BME280_Set.humOversampl);

		BME280_Set[activeSensor].measurementStatus = bme280_ReadReg(BME280_REG_CTRL_MEAS);
		//BME280_Set.measurementStatus = bme280_ReadReg(BME280_REG_CTRL_HUM) << 8;

		BME280_Set[activeSensor].tempOn = (BME280_Set[activeSensor].measurementStatus & BME280_OSRS_T_MSK) ? 1 : 0;
		BME280_Set[activeSensor].presOn = (BME280_Set[activeSensor].measurementStatus & BME280_OSRS_P_MSK) ? 1 : 0;
		//BME280_Set.humiOn = ((BME280_Set.measurementStatus >> 8) & BME280_OSRS_H_MSK) ? 1 : 0;

		bme280_SetMode(BME280_Set[activeSensor].sensMode);

		tempTypeOfSensor = BMP_280_type;
	}
	else
	{
		tempTypeOfSensor = Unidentified_type;

	}

	return tempTypeOfSensor;
}


float BME280_ReadTemperature(uint8_t sensor)
{
	float readTemp = 0.0f;
	int32_t readRawData = 0;

	bme280_ReadRegDataConvert24(BME280_REGISTER_TEMPDATA, (uint32_t*)&readRawData);

	if(readRawData == 0x800000)
	{
		return 0xFFFF;
	}

	readRawData >>= 4;

	int32_t tmp_1 = ((((readRawData>>3) - ((int32_t)CalibData[sensor].tempValue.dig_T1 <<1))) *
		((int32_t)CalibData[sensor].tempValue.dig_T2)) >> 11;

	int32_t tmp_2 = (((((readRawData>>4) - ((int32_t)CalibData[sensor].tempValue.dig_T1)) *
		((readRawData>>4) - ((int32_t)CalibData[sensor].tempValue.dig_T1))) >> 12) *
		((int32_t)CalibData[sensor].tempValue.dig_T3)) >> 14;

	tFineValue[sensor] = tmp_1 + tmp_2;
	readTemp = ((tFineValue[sensor] * 5 + 128) >> 8);
	readTemp /= 100.0f;

	return readTemp;
}


float BME280_ReadPressure(uint8_t sensor)
{
	float pressFloat = 0.0f;
	int64_t presureInt = 0;
	int32_t presureRaw = 0;
	uint32_t presUint = 0;
	int64_t tmp_1 = 0;
	int64_t tmp_2 = 0;

	//BME280_ReadTemperature();
	bme280_ReadRegDataConvert24(BME280_REGISTER_PRESSUREDATA, (uint32_t*)&presureRaw);

    if (presureRaw == 0x800000)
    {
    	return 0xFFFF;
    }

    presureRaw >>= 4;

    tmp_1 = ((int64_t) tFineValue[sensor]) - 128000;
    tmp_2 = tmp_1 * tmp_1 * (int64_t)CalibData[sensor].presureValue.dig_P6;
    tmp_2 = tmp_2 + ((tmp_1 * (int64_t)CalibData[sensor].presureValue.dig_P5) << 17);
    tmp_2 = tmp_2 + ((int64_t)CalibData[sensor].presureValue.dig_P4 << 35);
	tmp_1 = ((tmp_1 * tmp_1 * (int64_t)CalibData[sensor].presureValue.dig_P3) >> 8) + ((tmp_1 * (int64_t)CalibData[sensor].presureValue.dig_P2) << 12);
	tmp_1 = (((((int64_t)1) << 47) + tmp_1)) * ((int64_t)CalibData[sensor].presureValue.dig_P1) >> 33;

	if (tmp_1 == 0) {
		return 0;
	}

	presureInt = 1048576 - presureRaw;
	presureInt = (((presureInt << 31) - tmp_2) * 3125) / tmp_1;

	tmp_1 = (((int64_t)CalibData[sensor].presureValue.dig_P9) * (presureInt >> 13) * (presureInt >> 13)) >> 25;
	tmp_2 = (((int64_t)CalibData[sensor].presureValue.dig_P8) * presureInt) >> 19;

	presureInt = ((presureInt + tmp_1 + tmp_2) >> 8) + ((int64_t)CalibData[sensor].presureValue.dig_P7 << 4);

	presUint = ((presureInt >> 8) * 1000) + (((presureInt & 0xff) * 390625) / 100000);
	//presUint = (uint32_t)(presureInt);
	pressFloat = presUint / 100.0f;

	return pressFloat;
}


float BME280_ReadHumidity(uint8_t sensor)
{
	float humidConverted = 0.0f;
	int16_t humidRawValue = 0;
	int32_t humidRaw32 = 0;
	int32_t tmpValue = 0;

	//BME280_ReadTemperature(sensor);
	bme280_ReadSignedData16_Convert(BME280_REGISTER_HUMIDDATA, &humidRawValue);

	if(humidRawValue == 0x8000)
	{
		return 0xFFFF;
	}

	humidRaw32 = ((int32_t)humidRawValue)&0x0000FFFF;

	tmpValue = (tFineValue[sensor] - ((int32_t)76800));
	tmpValue = (((((humidRaw32 << 14) - (((int32_t)CalibData[sensor].humidValue.dig_H4) << 20) -
		(((int32_t)CalibData[sensor].humidValue.dig_H5) * tmpValue)) + ((int32_t)16384)) >> 15) *
		(((((((tmpValue * ((int32_t)CalibData[sensor].humidValue.dig_H6)) >> 10) *
		(((tmpValue * ((int32_t)CalibData[sensor].humidValue.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
		((int32_t)2097152)) * ((int32_t)CalibData[sensor].humidValue.dig_H2) + 8192) >> 14));

	tmpValue = (tmpValue - (((((tmpValue >> 15) * (tmpValue >> 15)) >> 7) *
		((int32_t)CalibData[sensor].humidValue.dig_H1)) >> 4));
	tmpValue = (tmpValue < 0) ? 0 : tmpValue;
	tmpValue = (tmpValue > 419430400) ? 419430400 : tmpValue;

	humidConverted = (tmpValue>>12);
	humidConverted /= 1024.0f;

	return humidConverted;
}


float BME280_ReadAltitude(float seaLevel, uint8_t sensor)
{
	float altitude = 0.0f;
	float presure = BME280_ReadPressure(sensor);

	altitude = 44330.0 * (1.0 - pow(presure/seaLevel, 0.1903));

	return altitude;
}


float BME280_ReadAltitudeDefSeaLevel(uint8_t sensor)
{
	float altitude = 0.0f;
	float presure = BME280_ReadPressure(sensor);

	altitude = 44330.0 * (1.0 - pow(presure/kSEA_LEVEL_PRESURE_PA, 0.1903));

	return altitude;
}

//Communication with BME280
static void bme280_WriteReg(uint8_t readRegister, uint8_t valueToWrite)
{
#ifdef SPI
	SPIx_WriteData(readRegister, valueToWrite);
#elif I2C
	I2Cx_WriteData(BME280_ADDRESS, readRegister, valueToWrite);
#endif

}
//------------------------------------------------
static uint8_t bme280_ReadReg(uint8_t readRegister)
{
#ifdef SPI
	uint8_t readedStatus = SPIx_ReadData(readRegister);
#elif I2C
	uint8_t readedStatus = I2Cx_ReadData(BME280_ADDRESS, readRegister);
#endif

  return readedStatus;
}
//------------------------------------------------
static void bme280_ReadRegPtr(uint8_t readRegister, uint8_t *ptrReadedValue)
{
#ifdef SPI
	 *(uint8_t *)ptrReadedValue = SPIx_ReadData(readRegister);
#elif I2C
	 *(uint8_t *)ptrReadedValue = I2Cx_ReadData(BME280_ADDRESS, readRegister);
#endif



}
//------------------------------------------------
static uint8_t bme280_ReadStatus(void)
{
  uint8_t res = bme280_ReadReg(BME280_REGISTER_STATUS) & 0x09;
  return res;
}
//------------------------------------------------
static void bme280_ReadData16(uint8_t readRegister, uint16_t *ptrReadedValue)
{
#ifdef SPI
	SPIx_ReadData16(readRegister, ptrReadedValue);
#elif I2C
	 I2Cx_ReadData16(BME280_ADDRESS, readRegister, ptrReadedValue);
#endif

}
//------------------------------------------------
static void bme280_ReadSignedData16(uint8_t readRegister, int16_t *ptrReadedValue)
{
#ifdef SPI
	SPIx_ReadData16(readRegister, (uint16_t*)ptrReadedValue);
#elif I2C
	I2Cx_ReadData16(BME280_ADDRESS, readRegister, (uint16_t*)ptrReadedValue);
#endif

}
//------------------------------------------------
static void bme280_ReadSignedData16_Convert(uint8_t readRegister, int16_t *ptrReadedValue)
{
#ifdef SPI
	  SPIx_ReadData16(readRegister, (uint16_t*)ptrReadedValue);
	  *(uint16_t *)ptrReadedValue = convert16BitData(*(uint16_t *)ptrReadedValue);
#elif I2C
	  I2Cx_ReadData16(BME280_ADDRESS, readRegister, (uint16_t*)ptrReadedValue);
	  *(uint16_t *)ptrReadedValue = convert16BitData(*(uint16_t *)ptrReadedValue);
#endif


}
//------------------------------------------------
static void bme280_ReadRegDataConvert24(uint8_t readRegister, uint32_t *ptrReadedValue)
{
#ifdef SPI
	SPIx_ReadData24(readRegister, ptrReadedValue);
	*(uint32_t *) ptrReadedValue = convert24BitData(*(uint32_t *) ptrReadedValue) & 0x00FFFFFF;
#elif I2C
	I2Cx_ReadData24(BME280_ADDRESS, readRegister, ptrReadedValue);
	*(uint32_t *) ptrReadedValue = convert24BitData(*(uint32_t *) ptrReadedValue) & 0x00FFFFFF;
#endif


}
//------------------------------------------------
/*
 * @brief: read factory set coeficiency
 */
static void bme280_ReadCoefficients_Temp(uint8_t sensor)
{
	bme280_ReadData16(BME280_REGISTER_DIG_T1, &CalibData[sensor].tempValue.dig_T1);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_T2, &CalibData[sensor].tempValue.dig_T2);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_T3, &CalibData[sensor].tempValue.dig_T3);
}

static void bme280_ReadCoefficients_Pres(uint8_t sensor)
{
	bme280_ReadData16(BME280_REGISTER_DIG_P1, &CalibData[sensor].presureValue.dig_P1);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_P2, &CalibData[sensor].presureValue.dig_P2);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_P3, &CalibData[sensor].presureValue.dig_P3);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_P4, &CalibData[sensor].presureValue.dig_P4);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_P5, &CalibData[sensor].presureValue.dig_P5);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_P6, &CalibData[sensor].presureValue.dig_P6);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_P7, &CalibData[sensor].presureValue.dig_P7);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_P8, &CalibData[sensor].presureValue.dig_P8);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_P9, &CalibData[sensor].presureValue.dig_P9);
}

static void bme280_ReadCoefficients_Hum(uint8_t sensor)
{
	bme280_ReadRegPtr(BME280_REGISTER_DIG_H1, &CalibData[sensor].humidValue.dig_H1);
	bme280_ReadSignedData16(BME280_REGISTER_DIG_H2, &CalibData[sensor].humidValue.dig_H2);
	bme280_ReadRegPtr(BME280_REGISTER_DIG_H3, &CalibData[sensor].humidValue.dig_H3);

	CalibData[sensor].humidValue.dig_H4 = (bme280_ReadReg(BME280_REGISTER_DIG_H4) << 4) | (bme280_ReadReg(BME280_REGISTER_DIG_H4+1) & 0xF);
	CalibData[sensor].humidValue.dig_H5 = (bme280_ReadReg(BME280_REGISTER_DIG_H5+1) << 4) | (bme280_ReadReg(BME280_REGISTER_DIG_H5) >> 4);
	CalibData[sensor].humidValue.dig_H6 = (int8_t)bme280_ReadReg(BME280_REGISTER_DIG_H6);
}

static void bme280_ReadCoefficients(uint8_t sensor)
{
	bme280_ReadCoefficients_Temp(sensor);

	bme280_ReadCoefficients_Pres(sensor);

	bme280_ReadCoefficients_Hum(sensor);
}

static void bmp280_ReadCoefficients(uint8_t sensor)
{
	bme280_ReadCoefficients_Temp(sensor);

	bme280_ReadCoefficients_Pres(sensor);
}
//------------------------------------------------
static void bme280_SetStandby(BME280_standby_Time_E standByTime)
{
  uint8_t registerValue = 0;

  registerValue = bme280_ReadReg(BME280_REG_CONFIG) & ~BME280_STBY_MSK;
  registerValue |= standByTime & BME280_STBY_MSK;

  bme280_WriteReg(BME280_REG_CONFIG, registerValue);
}

static void bme280_SetFilter(BME280_filter_E filter)
{
	uint8_t registerValue = 0;

	registerValue = bme280_ReadReg(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;
	registerValue |= filter & BME280_FILTER_MSK;

	bme280_WriteReg(BME280_REG_CONFIG, registerValue);
}

static void bme280_SetOversamplingTemper(BME280_overSamplingTemp_E tempOversampl)
{
	uint8_t registerValue = 0;

	registerValue = bme280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;
	registerValue |= tempOversampl & BME280_OSRS_T_MSK;

	bme280_WriteReg(BME280_REG_CTRL_MEAS, registerValue);
}

static void bme280_SetOversamplingPressure(BME280_overSamplingPres_E presOversampl)
{
	uint8_t registerValue = 0;

	registerValue = bme280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;
	registerValue |= presOversampl & BME280_OSRS_P_MSK;

	bme280_WriteReg(BME280_REG_CTRL_MEAS,registerValue);
}

static void bme280_SetOversamplingHum(BME280_overSamplingHum_E humOversampl)
{
	uint8_t registerValue = 0;

	registerValue = bme280_ReadReg(BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;
	registerValue |= humOversampl & BME280_OSRS_H_MSK;
	bme280_WriteReg(BME280_REG_CTRL_HUM,registerValue);

	/* Reewrite setting to change oversamplig efectivly */
	registerValue = bme280_ReadReg(BME280_REG_CTRL_MEAS);
	bme280_WriteReg(BME280_REG_CTRL_MEAS,registerValue);
}

static void bme280_SetMode(BME280_mode_E mode)
{
	uint8_t registerValue = 0;

	registerValue = bme280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;
	registerValue |= mode & BME280_MODE_MSK;

	bme280_WriteReg(BME280_REG_CTRL_MEAS, registerValue);
}
