// ina219_hal.h
#ifndef INA219_HAL_H
#define INA219_HAL_H
#include "stm32g0xx_hal.h"
// یا سری میکروکنترلر خودت
typedef enum {
	INA219_SHUNT_40MV = 0,
	INA219_SHUNT_80MV,
	INA219_SHUNT_160MV,
	INA219_SHUNT_320MV
} INA219_ShuntVoltageRange;

typedef enum {
	INA219_BUS_RANGE_16V = 0, INA219_BUS_RANGE_32V
} INA219_BusVoltageRange;

typedef enum {
	INA219_GAIN_1_40MV = 0,
	INA219_GAIN_2_80MV,
	INA219_GAIN_4_160MV,
	INA219_GAIN_8_320MV
} INA219_PGAGain;

typedef enum {
	INA219_ADC_RES_9BIT = 0b0000,
	INA219_ADC_RES_10BIT = 0b0001,
	INA219_ADC_RES_11BIT = 0b0010,
	INA219_ADC_RES_12BIT = 0b1000,
	INA219_ADC_RES_12BIT_Default = 0b0011,
	INA219_ADC_RES_12BIT_2S = 0b1001,
	INA219_ADC_RES_12BIT_4S = 0b1010,
	INA219_ADC_RES_12BIT_8S = 0b1011,
	INA219_ADC_RES_12BIT_16S = 0b1100,
	INA219_ADC_RES_12BIT_32S = 0b1101,
	INA219_ADC_RES_12BIT_64S = 0b1110,
	INA219_ADC_RES_12BIT_128S = 0b1111
} INA219_ADCResolution;

typedef enum {
	INA219_MODE_POWER_DOWN = 0,
	INA219_MODE_SHUNT_TRIGGERED,
	INA219_MODE_BUS_TRIGGERED,
	INA219_MODE_SHUNT_BUS_TRIGGERED,
	INA219_MODE_ADC_OFF,
	INA219_MODE_SHUNT_CONTINUOUS,
	INA219_MODE_BUS_CONTINUOUS,
	INA219_MODE_SHUNT_BUS_CONTINUOUS
} INA219_MeasurementMode;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_address;
	float shunt_resistance;
	float current_lsb;
	float power_lsb;
	int16_t calibration_value;
	INA219_MeasurementMode mode;
	INA219_PGAGain gain;
} INA219_HandleTypeDef;

HAL_StatusTypeDef INA219_Init(INA219_HandleTypeDef *dev);
HAL_StatusTypeDef INA219_Reset(INA219_HandleTypeDef *dev);
HAL_StatusTypeDef INA219_ReadCurrent(INA219_HandleTypeDef *dev,
		float *current_mA);
HAL_StatusTypeDef INA219_ReadVoltage(INA219_HandleTypeDef *dev,
		float *voltage_V);
HAL_StatusTypeDef INA219_SetCalibration(INA219_HandleTypeDef *dev,
		float max_expected_current);
HAL_StatusTypeDef INA219_ReadShuntVoltage(INA219_HandleTypeDef *dev,
		float *shunt_V);
HAL_StatusTypeDef INA219_SetMode(INA219_HandleTypeDef *ina219,
		INA219_MeasurementMode mode);
HAL_StatusTypeDef INA219_SetBusVoltageRange(INA219_HandleTypeDef *ina219,
		INA219_BusVoltageRange range);
HAL_StatusTypeDef INA219_SetGain(INA219_HandleTypeDef *ina219,
		INA219_PGAGain gain);
HAL_StatusTypeDef INA219_SetBusADCResolution(INA219_HandleTypeDef *ina219,
		INA219_ADCResolution adc);
HAL_StatusTypeDef INA219_SetShuntADCResolution(INA219_HandleTypeDef *ina219,
		INA219_ADCResolution adc);

#endif
