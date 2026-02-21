

/**
 * @file lsm6dso.hpp
 * @brief Implementation of the LSM6DSO driver.
 * @author Jad Dina
 * @version 1.0
 * @date 2026-01-28
*/


#ifndef LSM6DSO_DRIVER_LSM6DSO_HPP_
#define LSM6DSO_DRIVER_LSM6DSO_HPP_




#include "stm32h7xx_hal.h"
#include "SensorDataTypes.hpp"
#include "CubeDefines.hpp"

constexpr uint8_t LSM6DSO_ID = 0x6C;
typedef uint8_t LSM6DSO_REGISTER_t;


class LSM6DSO_Driver{
	
	public:
		LSM6DSO_Driver();
		enum LSM6DS0_SAMPLE_SPEED {
			FREQ_12p5_HZ=0b0001,
			FREQ_26_HZ=0b0010,
			FREQ_52_HZ=0b0011,
			FREQ_104_HZ=0b0100,
			FREQ_208_HZ=0b0101,
			FREQ_417_HZ=0b0110,
			FREQ_833_HZ=0b0111,
			FREQ_1667_HZ=0b1000,
			FREQ_3333_HZ=0b1001,
			FREQ_6667_HZ=0b1010
		};
		enum LSM6DS0_ACCEL_SCALE_SELECT {
			SCALE_2g = 0b00,
			SCALE_16g = 0b01,
			SCALE_4g = 0b10,
			SCALE_8g = 0b11
		};

		enum LSM6DS0_GYRO_SCALE_SELECT {
			SCALE_250dps = 0b00,
			SCALE_500dps = 0b01,
			SCALE_1000dps = 0b10,
			SCALE_2000dps = 0b11
		};

		void Init(SPI_HandleTypeDef* hspi_, uint8_t cs_pin_, GPIO_TypeDef* cs_gpio_);
		void setRegister(LSM6DSO_REGISTER_t reg, uint8_t val);
		uint8_t getRegister(LSM6DSO_REGISTER_t reg);
		void readRegisters(uint8_t startreg, uint8_t *out, uint16_t numBytes);
		void readSensors(uint8_t *out);
		IMUData bytesToStruct(const uint8_t *raw_bytes, bool accel, bool gyro, bool temp);


	private:
		SPI_HandleTypeDef* hspi = nullptr;
		GPIO_TypeDef* cs_gpio;
		uint16_t cs_pin;

		void CSHigh();
		void CSLow();

};

namespace LSM6DSO_REG{

	constexpr LSM6DSO_REGISTER_t FIFO_CTRL_2 = 0x08;
	constexpr LSM6DSO_REGISTER_t FIFO_CTRL_3 = 0x09;
	constexpr LSM6DSO_REGISTER_t FIFO_CTRL_4 = 0x0A;
	
	constexpr LSM6DSO_REGISTER_t WHO_AM_I = 0x0F;

	constexpr LSM6DSO_REGISTER_t CTRL1_XL  = 0x10;
	constexpr LSM6DSO_REGISTER_t CTRL2_G   = 0x11;
	constexpr LSM6DSO_REGISTER_t CTRL3_C = 0x12;

	constexpr LSM6DSO_REGISTER_t STATUS_REG   = 0x1E;

	constexpr LSM6DSO_REGISTER_t OUT_TEMP_L = 0x20; // Temperature [7:0]
	constexpr LSM6DSO_REGISTER_t OUT_TEMP_H = 0x21; // Temperature [15:8]

	//Gyroscope
	//X-axis
	constexpr LSM6DSO_REGISTER_t OUTX_L_G = 0x22;
	constexpr LSM6DSO_REGISTER_t OUTX_H_G = 0x23;
	//Y-axis
	constexpr LSM6DSO_REGISTER_t OUTY_L_G = 0x24;
	constexpr LSM6DSO_REGISTER_t OUTY_H_G = 0x25;
	//Z-axis
	constexpr LSM6DSO_REGISTER_t OUTZ_L_G = 0x26;
	constexpr LSM6DSO_REGISTER_t OUTZ_H_G = 0x27;

	//Accelerometer
	//X-axis
	constexpr LSM6DSO_REGISTER_t OUTX_L_A = 0x28;
	constexpr LSM6DSO_REGISTER_t OUTX_H_A = 0x29;
	//Y-axis
	constexpr LSM6DSO_REGISTER_t OUTY_L_A = 0x2A;
	constexpr LSM6DSO_REGISTER_t OUTY_H_A = 0x2B;
	//Z-axis
	constexpr LSM6DSO_REGISTER_t OUTZ_L_A = 0x2C;
	constexpr LSM6DSO_REGISTER_t OUTZ_H_A  = 0x2D;
}
#endif
