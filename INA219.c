// ina219_hal.c
#include "INA219.h"

#define INA219_REG_CONFIG 0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

static HAL_StatusTypeDef INA219_WriteRegister(INA219_HandleTypeDef *dev, uint8_t reg,
		uint16_t value) {
	uint8_t data[3];
	data[0] = reg;
	data[1] = (value >> 8) & 0xFF;
	data[2] = value & 0xFF;
	return HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_address << 1, data, 3,
			HAL_MAX_DELAY);
}

static HAL_StatusTypeDef INA219_ReadRegister(INA219_HandleTypeDef *dev, uint8_t reg,
		uint16_t *value) {
	uint8_t data[2];
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(dev->hi2c,
			dev->i2c_address << 1, &reg, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK)
		return ret;
	ret = HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_address << 1, data, 2,
			HAL_MAX_DELAY);
	if (ret != HAL_OK)
		return ret;
	*value = (data[0] << 8) | data[1];
	return HAL_OK;
}

HAL_StatusTypeDef INA219_Init(INA219_HandleTypeDef *dev) {
	return INA219_Reset(dev);
}

HAL_StatusTypeDef INA219_Reset(INA219_HandleTypeDef *dev) {
	return INA219_WriteRegister(dev, INA219_REG_CONFIG, 0x8000);
}

// =============================
//      Configuration APIs
// =============================

/**
 * @brief Set operating mode without affecting other configuration bits.
 */
HAL_StatusTypeDef INA219_SetMode(INA219_HandleTypeDef *ina219, INA219_MeasurementMode mode)
{
    uint16_t configReg;
    if (INA219_ReadRegister(ina219, INA219_REG_CONFIG, &configReg) != HAL_OK)
        return HAL_ERROR;

    configReg &= ~(0x07);                 // Clear bits 0-2
    configReg |= (mode & 0x07);           // Set new mode
    return INA219_WriteRegister(ina219, INA219_REG_CONFIG, configReg);
}

/**
 * @brief Set Bus Voltage Range (bit 13) without affecting other bits.
 */
HAL_StatusTypeDef INA219_SetBusVoltageRange(INA219_HandleTypeDef *ina219, INA219_BusVoltageRange range)
{
    uint16_t configReg;
    if (INA219_ReadRegister(ina219, INA219_REG_CONFIG, &configReg) != HAL_OK)
        return HAL_ERROR;

    configReg &= ~(0x01 << 13);                         // Clear bit 13
    configReg |= ((range & 0x01) << 13);                // Set new range
    return INA219_WriteRegister(ina219, INA219_REG_CONFIG, configReg);
}

/**
 * @brief Set PGA Gain (bits 11-12) without affecting other config bits.
 */
HAL_StatusTypeDef INA219_SetGain(INA219_HandleTypeDef *ina219, INA219_PGAGain gain)
{
    uint16_t configReg;
    if (INA219_ReadRegister(ina219, INA219_REG_CONFIG, &configReg) != HAL_OK)
        return HAL_ERROR;

    configReg &= ~(0x03 << 11);                         // Clear bits 11-12
    configReg |= ((gain & 0x03) << 11);                 // Set gain
    return INA219_WriteRegister(ina219, INA219_REG_CONFIG, configReg);
}

/**
 * @brief Set Bus ADC Resolution/Averaging (bits 7-10) without affecting other bits.
 */
HAL_StatusTypeDef INA219_SetBusADCResolution(INA219_HandleTypeDef *ina219, INA219_ADCResolution adc)
{
    uint16_t configReg;
    if (INA219_ReadRegister(ina219, INA219_REG_CONFIG, &configReg) != HAL_OK)
        return HAL_ERROR;

    configReg &= ~(0x0F << 7);                          // Clear bits 7-10
    configReg |= ((adc & 0x0F) << 7);                   // Set resolution
    return INA219_WriteRegister(ina219, INA219_REG_CONFIG, configReg);
}

/**
 * @brief Set Shunt ADC Resolution/Averaging (bits 3-6) without affecting other bits.
 */
HAL_StatusTypeDef INA219_SetShuntADCResolution(INA219_HandleTypeDef *ina219, INA219_ADCResolution adc)
{
    uint16_t configReg;
    if (INA219_ReadRegister(ina219, INA219_REG_CONFIG, &configReg) != HAL_OK)
        return HAL_ERROR;

    configReg &= ~(0x0F << 3);                          // Clear bits 3-6
    configReg |= ((adc & 0x0F) << 3);                   // Set resolution
    return INA219_WriteRegister(ina219, INA219_REG_CONFIG, configReg);
}



HAL_StatusTypeDef INA219_SetCalibration(INA219_HandleTypeDef *dev, float max_current) {
	dev->current_lsb = max_current / 32767.0f;
	dev->power_lsb = dev->current_lsb * 20.0f;
	dev->calibration_value = (int16_t) (0.04096f
			/ (dev->current_lsb * dev->shunt_resistance));
	return INA219_WriteRegister(dev, INA219_REG_CALIBRATION,
			dev->calibration_value);
}

HAL_StatusTypeDef INA219_ReadCurrent(INA219_HandleTypeDef *dev, float *current_mA) {
	uint16_t raw;
	HAL_StatusTypeDef ret = INA219_WriteRegister(dev, INA219_REG_CALIBRATION,
			dev->calibration_value);
	if (ret != HAL_OK)
		return ret;
	ret = INA219_ReadRegister(dev, INA219_REG_CURRENT, &raw);
	if (ret != HAL_OK)
		return ret;
	*current_mA = (int16_t) raw * dev->current_lsb * 1000.0f;
	return HAL_OK;
}

HAL_StatusTypeDef INA219_ReadVoltage(INA219_HandleTypeDef *dev, float *voltage_V) {
	uint16_t raw;
	HAL_StatusTypeDef ret = INA219_ReadRegister(dev, INA219_REG_BUS_VOLTAGE,
			&raw);
	if (ret != HAL_OK)
		return ret;
	*voltage_V = ((raw >> 3) * 4) / 1000.0f;
	return HAL_OK;
}

HAL_StatusTypeDef INA219_ReadShuntVoltage(INA219_HandleTypeDef *dev, float *shunt_V) {
	uint16_t raw;
	HAL_StatusTypeDef ret = INA219_ReadRegister(dev, INA219_REG_SHUNT_VOLTAGE,
			&raw);
	if (ret != HAL_OK)
		return ret;
	*shunt_V = (int16_t) raw * 0.00001f;
	return HAL_OK;
}

/*
 *
 *
 main.c:
 	INA219_HandleTypeDef ina219 = { .hi2c = &hi2c2, .i2c_address = 0x40,
		.shunt_resistance = 0.03f };
		float current, voltage;

	INA219_Init(&ina219);
	INA219_SetCalibration(&ina219, 1.0f);
	INA219_SetBusADCResolution(&ina219, INA219_ADC_RES_12BIT_128S);
	INA219_SetBusVoltageRange(&ina219, INA219_BUS_RANGE_32V);
	INA219_SetGain(&ina219, INA219_GAIN_1_40MV);
	INA219_SetMode(&ina219, INA219_MODE_SHUNT_BUS_CONTINUOUS);
	INA219_SetShuntADCResolution(&ina219, INA219_ADC_RES_12BIT_128S);


	INA219_ReadCurrent(&ina219, &temporaryCurrent);
	INA219_ReadVoltage(&ina219, &voltage);

 *
 *
 */

