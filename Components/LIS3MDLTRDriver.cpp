/*
 * IMUDriver.cpp
 *
 *  Created on: Aug 31, 2024
 *      Author: goada
 */

#include <LIS3MDLTRDriver.h>

/* @brief Initialize the driver. Must be called before any other functions can be used.
 * @param hspi_ Pointer to the SPI handle
 * @param cs_gpio_ GPIO port for the chip select pin (GPIOA, GPIOB ...)
 * @param cs_pin_ Pin number for the chip select
 */
void LIS3MDLTR_Driver::Init(SPI_HandleTypeDef* hspi_, GPIO_TypeDef* cs_gpio_, uint16_t cs_pin_) {
	hspi = hspi_;
	initialized = true;
	SetCSPin(cs_gpio_, cs_pin_);
	CSHigh();

	uint8_t ID = GetRegister(LIS3MDLTR_REG::WHO_AM_I);
	if(ID != LIS3MDLTR_ID) {
		// couldn't get chip ID
		initialized = false;
		return;
	}
	// Enable temp and max speed
	SetRegister(LIS3MDLTR_REG::CTRL_REG1, 0b11111110);

	// Leave power-down mode
	SetRegister(LIS3MDLTR_REG::CTRL_REG3, 0b00000000);

	// High-performance z axis
	SetRegister(LIS3MDLTR_REG::CTRL_REG4, 0b00001100);

}

/* @brief Sets a single 8-bit register.
 * @param reg The register to set. Constants contained in ICM20948_REG namespace.
 * @param val Value to set the register to.
 * @return Success
 */
bool LIS3MDLTR_Driver::SetRegister(LIS3MDLTR_REGISTER_t reg, uint8_t val) {
	assert(initialized);

	uint8_t data[2] = {(uint8_t)(0b00000000 | (reg&0x3F)),val};
	CSLow();
	HAL_StatusTypeDef r = HAL_SPI_Transmit(hspi, data, 2, 1000);
	CSHigh();
	return r == HAL_OK;

}

/* @brief Gets a single 8-bit register.
 * @param reg The register to get. Constants contained in ICM20948_REG namespace.
 * @return Value read from the register.
 */
uint8_t LIS3MDLTR_Driver::GetRegister(LIS3MDLTR_REGISTER_t reg) {
	assert(initialized);
	uint8_t data[2] = {(uint8_t)(0b10000000 | (reg&0x3F)),SPI_DUMMY_BYTE};
	uint8_t incoming[2] = {0,0};
	CSLow();
	HAL_SPI_TransmitReceive(hspi, data, incoming, 2, 1000);
	CSHigh();
	return incoming[1];
}

/* @brief Reads multiple successive registers in a row.
 * @param startreg The register that the readings should start at.
 * @param numBytes Number of bytes to read.
 * @param out Address of buffer to receive data. Must be numBytes long.
 */
void LIS3MDLTR_Driver::GetMultipleRegisters(LIS3MDLTR_REGISTER_t startreg, int numBytes,
		uint8_t *out) {
	assert(initialized);

	uint8_t transmit[numBytes+1] = {SPI_DUMMY_BYTE};
	transmit[0] = (uint8_t)(0b11000000 | (startreg&0x3f));

	CSLow();
	HAL_SPI_TransmitReceive(hspi, transmit, out, numBytes+1, 1000);
	CSHigh();
}


/* @brief Extract IMU data from a raw byte buffer (such as from ReadAllSensorRegs) into a struct.
 * @param temp Buffer includes temperature data
 * @return Struct containing extracted data
 */
const MagData2 LIS3MDLTR_Driver::GetDataFromBuf(const uint8_t *buf, bool mag, bool temp) {
	MagData2 out;
	size_t i = 0;

	//both little-endian
	if(mag) {
		out.mag.x = (buf[i]) | (buf[1+1]<<8);
		out.mag.y = (buf[i+2]) | (buf[1+3]<<8);
		out.mag.z = (buf[i+4]) | (buf[1+5]<<8);
	}
	if(temp) {
		out.temp = (buf[i]) | (buf[i+1]<<8);
		i += 2;
	}

	return out;

}

/* @brief Reads all 8 sensor registers in order. (mag, temp)
 * @param out Raw bytes read from registers. Must be 8 bytes long
 */
void LIS3MDLTR_Driver::GetMeasurements(uint8_t* out) {
	GetMultipleRegisters(LIS3MDLTR_REG::OUTX_L, 8, out);
}


LIS3MDLTR_Driver::LIS3MDLTR_Driver() {
}

LIS3MDLTR_Driver::~LIS3MDLTR_Driver() {
}

void LIS3MDLTR_Driver::CSLow() {
	assert(initialized);
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);
}

void LIS3MDLTR_Driver::SetCSPin(GPIO_TypeDef* gpio, uint16_t pin) {
	cs_gpio = gpio;
	cs_pin = pin;
}

void LIS3MDLTR_Driver::EnableTemp() {
	SetRegister(LIS3MDLTR_REG::CTRL_REG1, GetRegister(LIS3MDLTR_REG::CTRL_REG1) | 0b10000000);
}

void LIS3MDLTR_Driver::DisableTemp() {
	SetRegister(LIS3MDLTR_REG::CTRL_REG1, GetRegister(LIS3MDLTR_REG::CTRL_REG1) & 0b01111111);
}

void LIS3MDLTR_Driver::CSHigh() {
	assert(initialized);
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);
}
/* @brief Gets the temperature. Must have temperature enabled with EnableTemp.
 * @return Temperature in Celsius.
 */
uint16_t GetTemp() {
	uint8_t tempbyte[2];
	GetMultipleRegisters(LIS3MDLTR_REG::OUT_TEMP_L,2,tempbyte);
	uint16_t temp = (tempbyte[0])|(tempbyte[1]<<8);
	return 25+temp/8;
}
