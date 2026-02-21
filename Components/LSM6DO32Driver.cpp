/*
 * IMUDriver.cpp
 *
 *  Created on: Aug 31, 2024
 *      Author: goada
 */

#include "LSM6DO32Driver.h"


/* @brief Initialize the driver. Must be called before any other functions can be used.
 * @param hspi_ Pointer to the SPI handle
 * @param cs_gpio_ GPIO port for the chip select pin (GPIOA, GPIOB ...)
 * @param cs_pin_ Pin number for the chip select
 */
void LSM6DO32_Driver::Init(SPI_HandleTypeDef* hspi_, GPIO_TypeDef* cs_gpio_, uint16_t cs_pin_) {
	hspi = hspi_;
	initialized = true;
	SetCSPin(cs_gpio_, cs_pin_);
	CSHigh();

	uint8_t ID = GetRegister(LSM6DSO32_REG::WHO_AM_I);
	if(ID != LSM6DSO32_ID) {
		// couldn't get chip ID
		initialized = false;
		return;
	}



	SetRegister(LSM6DSO32_REG::CTRL3_C, 0b01000100);
	SetRegister(LSM6DSO32_REG::CTRL1_XL,0b01011100);
	SetRegister(LSM6DSO32_REG::CTRL2_G,0b01010000);
	SetRegister(LSM6DSO32_REG::FIFO_CTRL4, 0b00000000);
}

/* @brief Sets a single 8-bit register.
 * @param reg The register to set. Constants contained in ICM20948_REG namespace.
 * @param val Value to set the register to.
 * @return Success
 */
bool LSM6DO32_Driver::SetRegister(LSM6DSO32_REGISTER_t reg, uint8_t val) {
	assert(initialized);

	uint8_t data[2] = {(uint8_t)(0b00000000 | (reg&0x7F)),val};
	CSLow();
	HAL_StatusTypeDef result = HAL_SPI_Transmit(hspi, data, 2, 1000);
	CSHigh();
	return result == HAL_OK;

}

/* @brief Gets a single 8-bit register.
 * @param reg The register to get. Constants contained in ICM20948_REG namespace.
 * @return Value read from the register.
 */
uint8_t LSM6DO32_Driver::GetRegister(LSM6DSO32_REGISTER_t reg) {
	assert(initialized);
	uint8_t data[2] = {(uint8_t)(0b10000000 | (reg&0x7F)),SPI_DUMMY_BYTE};
	uint8_t incoming[2] = {0,0}; 				// first byte is dummy byte
	CSLow();
	HAL_SPI_TransmitReceive(hspi, data, incoming, 2, 1000); // second incoming byte is response
	CSHigh();
	return incoming[1];
}

/* @brief Reads multiple successive registers in a row.
 * @param startreg The register that the readings should start at.
 * @param numBytes Number of bytes to read.
 * @param out Address of buffer to receive data. Must be numBytes long.
 */
void LSM6DO32_Driver::GetMultipleRegisters(LSM6DSO32_REGISTER_t startreg, int numBytes,
		uint8_t *out) {
	assert(initialized);

	uint8_t transmit[numBytes+1] = {SPI_DUMMY_BYTE};
	transmit[0] = (uint8_t)(0b10000000 | startreg);

	CSLow();
	HAL_SPI_TransmitReceive(hspi, transmit, out, numBytes+1, 1000);
	CSHigh();
}

/* @brief Repeatedly reads the top two bytes of each of the x, y, and z FIFOs in little-endian format.
 * @param numReads Number of repetitive reads to perform.
 * @param out Buffer to receive data in. Must be numReads*6 long.
 */
void LSM6DO32_Driver::SampleFIFOs(int numReads, uint8_t *out, size_t outBufferSize) {
	assert(initialized);
	assert(outBufferSize >= (size_t)numReads * 6);
	
	for(int i = 0; i < numReads; i++) {
		GetMultipleRegisters(LSM6DSO32_REG::FIFO_DATA_OUT_X_L, 6, out+i*6);
	}
}

/* @brief Extract IMU data from a raw byte buffer (such as from ReadAllSensorRegs) into a struct.
 * @param buf Input buffer containing raw data.
 * @param accel Buffer includes accel data
 * @param gyro Buffer includes gyroscope data
 * @param temp Buffer includes temperature data
 * @return Struct containing extracted data
 */
const IMUData LSM6DO32_Driver::ConvertRawMeasurementToStruct(const uint8_t *buf, bool accel, bool gyro, bool temp) {
	 IMUData out;
	size_t i = 0;
	if(temp){
		out.temp = (int16_t)((uint16_t)buf[i] | ((uint16_t)buf[i+1] << 8));
		i+=2;
	}

	if(gyro){
		out.gyro.x = (int16_t)((uint16_t)buf[i] | ((uint16_t)buf[i+1] << 8));
		out.gyro.y = (int16_t)((uint16_t)buf[i+2] | ((uint16_t)buf[i+3] << 8));
		out.gyro.z = (int16_t)((uint16_t)buf[i+4] | ((uint16_t)buf[i+5] << 8));
		i+=6;
	}
	if(accel){
		out.accel.x = (int16_t)((uint16_t)buf[i] | ((uint16_t)buf[i+1] << 8));
		out.accel.y = (int16_t)((uint16_t)buf[i+2] | ((uint16_t)buf[i+3] << 8));
		out.accel.z = (int16_t)((uint16_t)buf[i+4] | ((uint16_t)buf[i+5] << 8));
	}
	out.temp = 25.0f + out.temp / 256.0f;

	out.accel.x *= 0.732;
	out.accel.y *= 0.732;
	out.accel.z *= 0.732;

	out.gyro.x *= 8.75;
	out.gyro.y *= 8.75;
	out.gyro.z *= 8.75;


	return out;

}

/* @brief Reads all 14 sensor registers in order. (temp, gyro, accel)
 * @param out Raw bytes read from registers. Must be 14 bytes long
 */
void LSM6DO32_Driver::ReadSensors(uint8_t* out) {
	GetMultipleRegisters(LSM6DSO32_REG::OUT_TEMP_L, 14, out);
}


LSM6DO32_Driver::LSM6DO32_Driver() {
}

LSM6DO32_Driver::~LSM6DO32_Driver() {
}

void LSM6DO32_Driver::CSLow() {
	assert(initialized);
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);
}

void LSM6DO32_Driver::SetCSPin(GPIO_TypeDef* gpio, uint16_t pin) {
	cs_gpio = gpio;
	cs_pin = pin;
}

void LSM6DO32_Driver::CSHigh() {
	assert(initialized);
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);
}

/* @brief Set the output data rate of the accelerometer
 * @param speed Output speed
 */
void LSM6DO32_Driver::SetAccelODR(LSM6D032_SAMPLE_SPEED speed) {
	assert(initialized);
	SetRegister(LSM6DSO32_REG::CTRL1_XL, (GetRegister(LSM6DSO32_REG::CTRL1_XL) & 0b00001111) | (speed<<4));
}

/* @brief Set the output data rate of the gyroscope
 * @param speed Output speed
 */
void LSM6DO32_Driver::SetGyroODR(LSM6D032_SAMPLE_SPEED speed) {
	assert(initialized);
	SetRegister(LSM6DSO32_REG::CTRL2_G, (GetRegister(LSM6DSO32_REG::CTRL2_G) & 0b00001111) | (speed<<4));
}

/* @brief Set the full-scale range of the accelerometer
 * @param scale Range, in plus/minus G
 */
void LSM6DO32_Driver::SetAccelFullScaleRange(LSM6D032_ACCEL_SCALE_SELECT scale) {
	assert(initialized);
	SetRegister(LSM6DSO32_REG::CTRL1_XL, (GetRegister(LSM6DSO32_REG::CTRL1_XL) & 0b11110011) | (scale<<2));
}

/* @brief Set the full-scale range of the gyroscope
 * @param scale Range, in plus/minus dps
 */
void LSM6DO32_Driver::SetGyroFullScaleRange(LSM6D032_GYRO_SCALE_SELECT scale) {
	assert(initialized);
	SetRegister(LSM6DSO32_REG::CTRL2_G, (GetRegister(LSM6DSO32_REG::CTRL2_G) & 0b11110011) | (scale<<2));

}

