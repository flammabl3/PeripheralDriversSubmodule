/*
 * IMUDriver.h
 *
 *  Created on: Aug 31, 2024
 *      Author: goada
 */

#ifndef LIS3MDLTRDRIVER_H_
#define LIS3MDLTRDRIVER_H_

#include "stm32h7xx_hal_gpio.h"
#include "SensorDataTypes.hpp"

constexpr uint8_t SPI_DUMMY_BYTE = 0x00;
constexpr uint8_t LIS3MDLTR_ID = 0b00111101;

typedef uint8_t LIS3MDLTR_REGISTER_t;



/* @brief Singleton SPI driver for the LIS3MDLTR magnetometer.
 * Must call Init before using any other functions besides SetCSPin.
 * SPI max clock speed 10MHz.
 */
class LIS3MDLTR_Driver {
public:
	LIS3MDLTR_Driver();
	~LIS3MDLTR_Driver();

	static LIS3MDLTR_Driver& Inst() {
		static LIS3MDLTR_Driver inst;
		return inst;
	}

	void Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_gpio_, uint16_t cs_pin_);

	bool SetRegister(LIS3MDLTR_REGISTER_t reg, uint8_t val);
	uint8_t GetRegister(LIS3MDLTR_REGISTER_t reg);
	void GetMultipleRegisters(LIS3MDLTR_REGISTER_t startreg, int numBytes, uint8_t* out);
	void GetMeasurements(uint8_t* out);

	void EnableTemp();
	void DisableTemp();

	uint16_t GetTemp();

	LIS3MDLTR_Driver(const LIS3MDLTR_Driver&) = delete;
	LIS3MDLTR_Driver& operator=(const LIS3MDLTR_Driver&) = delete;

	const MagData2 GetDataFromBuf(const uint8_t *buf, bool mag = true, bool temp = true);

	void SetCSPin(GPIO_TypeDef* gpio, uint16_t pin);

private:
	bool initialized = false;
	SPI_HandleTypeDef* hspi = nullptr;
	GPIO_TypeDef* cs_gpio;
	uint16_t cs_pin;

	void CSLow();
	void CSHigh();

};

// Constant addresses for registers in the LIS3MDLTR magnetometer
namespace LIS3MDLTR_REG {

	constexpr LIS3MDLTR_REGISTER_t WHO_AM_I  = 0x0F;

	constexpr LIS3MDLTR_REGISTER_t CTRL_REG1  = 0x20;
	constexpr LIS3MDLTR_REGISTER_t CTRL_REG2  = 0x21;
	constexpr LIS3MDLTR_REGISTER_t CTRL_REG3  = 0x22;
	constexpr LIS3MDLTR_REGISTER_t CTRL_REG4  = 0x23;
	constexpr LIS3MDLTR_REGISTER_t CTRL_REG5  = 0x24;

	constexpr LIS3MDLTR_REGISTER_t STATUS_REG    = 0x27;

	constexpr LIS3MDLTR_REGISTER_t OUT_TEMP_L    = 0x2E;
	constexpr LIS3MDLTR_REGISTER_t OUT_TEMP_H    = 0x2F;

	constexpr LIS3MDLTR_REGISTER_t OUTX_L    = 0x28;
	constexpr LIS3MDLTR_REGISTER_t OUTX_H    = 0x29;
	constexpr LIS3MDLTR_REGISTER_t OUTY_L    = 0x2A;
	constexpr LIS3MDLTR_REGISTER_t OUTY_H    = 0x2B;
	constexpr LIS3MDLTR_REGISTER_t OUTZ_L    = 0x2C;
	constexpr LIS3MDLTR_REGISTER_t OUTZ_H    = 0x2D;

}

#endif /* LIS3MDLTRDRIVER_H_ */
