

/*
 * file: lsm6dso.cpp
 *
 *  Created on: Jan 29, 2026
 *      Author: Jad Dina
*/

/* Includes ------------------------------------------------------------------*/
#include "lsm6dso.hpp"

LSM6DSO_Driver::LSM6DSO_Driver()
{
}

void LSM6DSO_Driver::Init(SPI_HandleTypeDef *hspi_, uint8_t cs_pin_, GPIO_TypeDef *cs_gpio_)
{
	hspi = hspi_;
	cs_pin = cs_pin_;
	cs_gpio = cs_gpio_;
	CSHigh();

	// read from WHO_AM_I reg to confirm initialization

	uint8_t ID = getRegister(LSM6DSO_REG::WHO_AM_I);

	SOAR_PRINT("WHO_AM_I %d\n", ID);
	if (ID != LSM6DSO_ID)
	{
		return;
	}

	/*set CTRL3_C reg disable continuous update for predictable reads, as well enable
	 * automatic increment of register addresses with multiple byte reads*/
	setRegister(LSM6DSO_REG::CTRL3_C, 0b01000100);
	// set CTRL1_XL reg 208Hz and 2g for Accel
	setRegister(LSM6DSO_REG::CTRL1_XL, 0b01010100);
	// set CTRL2_G reg 208Hz and 250dps for Gyro
	setRegister(LSM6DSO_REG::CTRL2_G, 0b01010000);
	// bypass FIFO mode
	setRegister(LSM6DSO_REG::FIFO_CTRL_4, 0b00000000);
}

void LSM6DSO_Driver::setRegister(LSM6DSO_REGISTER_t reg, uint8_t val)
{

	uint8_t tx[2] = {(uint8_t)(0b00000000 | (0x7f & reg)), val}; // write MSB must be 0 ensures MSB is 0
	// transmit spi message
	CSLow();
	HAL_StatusTypeDef result = HAL_SPI_Transmit(hspi, tx, 2, 1000);
	CSHigh();
	if (result == HAL_OK)
	{
		return;
	}
	else
	{
		SOAR_PRINT("set register error");
	}
}

uint8_t LSM6DSO_Driver::getRegister(LSM6DSO_REGISTER_t reg)
{
	uint8_t tx[2] = {(uint8_t)(0b10000000 | (0x7f & reg)), 0x00}; // read MSB must be 1 ensures MSB is 1
	uint8_t rx[2] = {0, 0};
	// transmit address of reg and recieve reg data spi message
	CSLow();
	HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, 1000);
	CSHigh();
	if (HAL_OK == result)
	{
		return rx[1];
	}
	return 0;
}

void LSM6DSO_Driver::readRegisters(uint8_t startreg, uint8_t *out, uint16_t numBytes)
{
	uint8_t tx[numBytes + 1];
	uint8_t rx[numBytes + 1];
	tx[0] = (0b10000000 | (0x7f & startreg)); // first 8 bits must be R and start reg

	for (uint16_t i = 1; i < numBytes + 1; i++)
	{
		tx[i] = 0x00; // fill tx with spi dummy bytes
	}

	CSLow();
	HAL_SPI_TransmitReceive(hspi, tx, rx, numBytes + 1, 1000);
	CSHigh();

	for (uint16_t i = 0; i < numBytes; i++)
	{
		out[i] = rx[i + 1]; // copy back i+1 into out since first byte is command garbage
	}
}

IMUData LSM6DSO_Driver::bytesToStruct(const uint8_t *raw_bytes, bool accel, bool gyro, bool temp)
{

	IMUData out = {};
	uint8_t i = 0;
	int16_t rawTemp = 0;
	int16_t rawGx = 0;
	int16_t rawGy = 0;
	int16_t rawGz = 0;
	int16_t rawAx = 0;
	int16_t rawAy = 0;
	int16_t rawAz = 0;

	auto scaleToInt16 = [](int16_t raw, int32_t num, int32_t den) -> int16_t
	{
		int32_t scaled = static_cast<int32_t>(raw) * num;
		// Round to nearest integer while staying in integer arithmetic.
		scaled = (scaled >= 0) ? ((scaled + (den / 2)) / den) : ((scaled - (den / 2)) / den);

		if (scaled > 32767)
		{
			return static_cast<int16_t>(32767);
		}
		if (scaled < -32768)
		{
			return static_cast<int16_t>(-32768);
		}
		return static_cast<int16_t>(scaled);
	};

	// litte endian (L to H) so shift high byte up 1 byte and fill lower byte
	if (temp)
	{

		rawTemp = (int16_t)(raw_bytes[i + 1] << 8 | raw_bytes[i]);
		i += 2;
	}

	if (gyro)
	{
		rawGx = (int16_t)(raw_bytes[i + 1] << 8 | raw_bytes[i]);
		rawGy = (int16_t)(raw_bytes[i + 3] << 8 | raw_bytes[i + 2]);
		rawGz = (int16_t)(raw_bytes[i + 5] << 8 | raw_bytes[i + 4]);
		i += 6;
	}

	if (accel)
	{
		rawAx = (int16_t)(raw_bytes[i + 1] << 8 | raw_bytes[i]);
		rawAy = (int16_t)(raw_bytes[i + 3] << 8 | raw_bytes[i + 2]);
		rawAz = (int16_t)(raw_bytes[i + 5] << 8 | raw_bytes[i + 4]);
	}

	if (accel)
	{
		// Store acceleration as integer mg.
		out.accel.x = scaleToInt16(rawAx, 488, 1000);
		out.accel.y = scaleToInt16(rawAy, 488, 1000);
		out.accel.z = scaleToInt16(rawAz, 488, 1000);
	}

	if (gyro)
	{
		// Store angular rate as integer mdps.
		out.gyro.x = scaleToInt16(rawGx, 875, 100);
		out.gyro.y = scaleToInt16(rawGy, 875, 100);
		out.gyro.z = scaleToInt16(rawGz, 875, 100);
	}

	if (temp)
	{
		// Store temperature as integer degC.
		out.temp = static_cast<int16_t>(25 + ((rawTemp >= 0) ? ((rawTemp + 128) / 256) : ((rawTemp - 128) / 256)));
	}

	return out;
}

void LSM6DSO_Driver::readSensors(uint8_t *out)
{
	readRegisters(LSM6DSO_REG::OUT_TEMP_L, out, 14);
}

void LSM6DSO_Driver::CSHigh()
{

	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);
}

void LSM6DSO_Driver::CSLow()
{

	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);
}
