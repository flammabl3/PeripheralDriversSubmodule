/** ********************************************************************************
 * @file    MS5611Driver.cpp
 * @author  root
 * @date    Nov 9, 2024
 * @brief	This driver gets pressure and temperature readings from the MS5611 barometers.
 ******************************************************************************** */
/************************************ * INCLUDES ************************************/
#include "MS5611Driver.hpp"
#include "stm32h7xx_hal_gpio.h"
/************************************ * PRIVATE MACROS AND DEFINES ************************************/
/************************************ * VARIABLES ************************************/
// Barometer Commands (should not be modified, non-const due to HAL and C++ strictness)
static uint8_t ADC_D1_512_CONV_CMD = 0x42;
static uint8_t ADC_D2_512_CONV_CMD = 0x52;
static uint8_t ADC_READ_CMD = 0x00;
static uint8_t PROM_READ_SENS_CMD = 0xA2;
static uint8_t PROM_READ_OFF_CMD = 0xA4;
static uint8_t PROM_READ_TCS_CMD = 0xA6;
static uint8_t PROM_READ_TCO_CMD = 0xA8;
static uint8_t PROM_READ_TREF_CMD = 0xAA;
static uint8_t PROM_READ_TEMPSENS_CMD = 0xAC;
static uint8_t READ_BYTE_CMD = 0x00;
static uint8_t RESET_CMD = 0x1E;
/************************************ * FUNCTION DECLARATIONS ************************************/
/************************************ * FUNCTION DEFINITIONS ************************************/

MS5611_Driver::MS5611_Driver(SPI_HandleTypeDef* hspi_, GPIO_TypeDef* cs_gpio_, uint16_t cs_pin_){
	hspi = hspi_;
	cs_gpio = cs_gpio_;
	cs_pin = cs_pin_;

}

/**
 * @brief gets a single sample of barometer data
 * @returns a barometer data structure consisting of a 'temp' and 'pressure' variable
 */
BaroData MS5611_Driver::getSample(){
	/**
	 * Variable Descriptions from MS5607-02BA03 Data Sheet:
	 *
	 * C1 (SENSt1)      - Pressure sensitivity
	 * C2 (OFFt1)       - Pressure offset
	 * C3 (TCS)         - Temperature coefficient of pressure sensitivity
	 * C4 (TCO)         - Temperature coefficient of pressure offset
	 * C5 (Tref)        - Reference temperature
	 * C6 (TEMPSENS)    - Temperature coefficient of the temperature
	 *
	 * D1   - Digital pressure value
	 * D2   - Digital temperature value
	 *
	 * dT   - Difference between actual and reference temperature
	 *          dT = D2 - Tref = D2 - (C5 * 2^8)
	 * TEMP - Actual temperature (-40...85�C with 0.01�C resolution)
	 *          TEMP = 20�C + (dT * TEMPSENS) = 2000 + (dT * C6)/2^23
	 * OFF  - Offset at actual temperature
	 *          OFF = OFFt1 + (TCO * dT) = (C2 * 2^17) + (C4 * dT)/2^6
	 * SENS - Sensitivity at actual temperature
	 *          SENS = SENSt1 + (TCS * dT) = (C1 * 2^16) + (C3 * dT)/2^7
	 * P    - Temperature compensated pressure (10...1200mbar with 0.01mbar resolution)
	 *          P = (D1 * SENS) - OFF = ((D1 * SENS)/2^21 - OFF)/2^15
	 */

	// Variables
	uint32_t pressureReading = 0;    // Stores a 24 bit value
	uint32_t temperatureReading = 0;    // Stores a 24 bit value
	uint8_t dataInBuffer;
	BaroData data;

	// Reset the barometer
	resetBarometer();

	// Read PROM for calibration coefficients
	uint16_t c1Sens = readCalibrationCoefficients(PROM_READ_SENS_CMD);
	uint16_t c2Off = readCalibrationCoefficients(PROM_READ_OFF_CMD);
	uint16_t c3Tcs = readCalibrationCoefficients(PROM_READ_TCS_CMD);
	uint16_t c4Tco = readCalibrationCoefficients(PROM_READ_TCO_CMD);
	uint16_t c5Tref = readCalibrationCoefficients(PROM_READ_TREF_CMD);
	uint16_t c6Tempsens = readCalibrationCoefficients(PROM_READ_TEMPSENS_CMD);

	/**
	 * Repeatedly read digital pressure and temperature.
	 * Convert these values into their calibrated counterparts.
	 * Finally, update the data globally if the mutex is available.
	 */
	/* Read Digital Pressure (D1) ----------------------------------------*/
	pressureReading = getPressureReading();

	/* Read Digital Temperature (D2) -------------------------------------*/
	temperatureReading = getTemperatureReading();

	/* Calculate First-Order Temperature and Parameters ------------------*/

	// Calibration coefficients need to be type cast to int64_t to avoid overflow during intermediate calculations
	int32_t dT = temperatureReading - ((int32_t)c5Tref << 8);
	int32_t temp = 2000 + ((dT * (int64_t)c6Tempsens) >> 23); // Divide this value by 100 to get degrees Celsius
	int64_t off = ((int64_t)c2Off << 16) + ((dT * (int64_t)c4Tco) >> 7);
	int64_t sens = ((int64_t)c1Sens << 15) + ((dT * (int64_t)c3Tcs) >> 8);

	/* Calculate Second-Order Temperature and Pressure -------------------*/

	if (temp < TEMP_LOW)    // If the temperature is below 20C
	{
		int32_t t2 = ((int64_t)dT * dT) >> 31;
		int64_t off2 = 5 * (((int64_t)(temp - 2000) * (temp - 2000)) >> 1);
		int64_t sens2 = 5 * (((int64_t)(temp - 2000) * (temp - 2000)) >> 2);

		if (temp < TEMP_VERY_LOW)   // If the temperature is below -15�C
		{
			off2 = off2 + (7 * ((int64_t)(temp + 1500) * (temp + 1500)));
			sens2 = sens2 + ((11 * ((int64_t)(temp + 1500) * (temp + 1500)) >> 1));
		}

		temp = temp - t2;
		off = off - off2;
		sens = sens - sens2;
	}

	int32_t p = (((pressureReading * sens) >> 21) - off) >> 15;   // Divide this value by 100 to get millibars

	/* Store Data --------------------------------------------------------*/
	data.pressure = p;
	data.temp = temp;

	return data;

	// All equations provided by MS5607-02BA03 data sheet

	// PRESSURE AND TEMPERATURE VALUES ARE STORED AT 100x VALUE TO MAINTAIN
	// 2 DECIMAL POINTS OF PRECISION AS AN INTEGER!
	// E.x. The value 1234 should be interpreted as 12.34
}

/**
 * @brief   This function reads and returns a 16-bit coefficient from the barometer.
 * @param   PROM_READ_CMD   The command to send in order to read the desired
 *                          coefficient. See the data sheet for the commands.
 * @return                  The read coefficient.
 */
uint16_t MS5611_Driver::readCalibrationCoefficients(uint8_t PROM_READ_CMD)
{

    uint8_t dataInBuffer;

    HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(hspi, &PROM_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

    // Read the first byte (bits 15-8)
    HAL_SPI_TransmitReceive(hspi, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    uint16_t coefficient = dataInBuffer << 8;

    // Read the second byte (bits 7-0)
    HAL_SPI_TransmitReceive(hspi, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient += dataInBuffer;

    HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);

    return coefficient;
}

/**
 * @brief   This function reads and returns a 32-bit temperature reading from the barometer (without converting it to a workable format).
 * @return                  The read temperature
 */
uint32_t MS5611_Driver::getTemperatureReading()
{
	// Variables
	uint32_t temperatureReading = 0;    // Stores a 24 bit value
	uint8_t dataInBuffer;

	// Tell the barometer to convert the temperature to a digital value with an over-sampling ratio of 512
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &ADC_D2_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);

	osDelay(2); // 1.17ms max conversion time for an over-sampling ratio of 512

	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspi, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

	// Read the first byte (bits 23-16)
	HAL_SPI_TransmitReceive(hspi, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
	temperatureReading = dataInBuffer << 16;

	// Read the second byte (bits 15-8)
	HAL_SPI_TransmitReceive(hspi, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
	temperatureReading += dataInBuffer << 8;

	// Read the third byte (bits 7-0)
	HAL_SPI_TransmitReceive(hspi, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
	temperatureReading += dataInBuffer;

	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);

	return temperatureReading;
}

/**
 * @brief   This function reads and returns a 32-bit pressure reading from the barometer (without converting it to a workable format).
 * @return                  The read pressure
 */
uint32_t MS5611_Driver::getPressureReading()
{
	// Variables
	uint32_t pressureReading = 0;    // Stores a 24 bit value
	uint8_t dataInBuffer;

	// Tell the barometer to convert the pressure to a digital value with an over-sampling ratio of 512
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &ADC_D1_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);

	osDelay(2); // 1.17ms max conversion time for an over-sampling ratio of 512

	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspi, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

	// Read the first byte (bits 23-16)
	HAL_SPI_TransmitReceive(hspi, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
	pressureReading = dataInBuffer << 16;

	// Read the second byte (bits 15-8)
	HAL_SPI_TransmitReceive(hspi, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
	pressureReading += dataInBuffer << 8;

	// Read the third byte (bits 7-0)
	HAL_SPI_TransmitReceive(hspi, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
	pressureReading += dataInBuffer;

	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);

	return pressureReading;
}


/**
 * @brief   This function resets the barometer prior to sampling
 */
void MS5611_Driver::resetBarometer()
{
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
	osDelay(4); // 2.8ms reload after Reset command
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);
}


