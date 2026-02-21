/** ********************************************************************************
 * @file    MS5611Driver.hpp
 * @author  Noah
 * @date    Nov 9, 2024
 * @brief	This driver gets pressure and temperature readings from the MS5611 barometers.
 ******************************************************************************** */
#ifndef SOARDRIVERS_MS5611DRIVER_INC_MS5611DRIVER_HPP_
#define SOARDRIVERS_MS5611DRIVER_INC_MS5611DRIVER_HPP_
/************************************ * INCLUDES ************************************/
#include "SystemDefines.hpp"
#include "cmsis_os.h"
#include "SensorDataTypes.hpp"
/************************************ * MACROS AND DEFINES ************************************/

/************************************ * TYPEDEFS ************************************/

/************************************ * CLASS DEFINITIONS ************************************/
/**
 * @brief the driver for MS5611 barometers
 * see datasheet here: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036
 */
class MS5611_Driver{
public:
	MS5611_Driver(SPI_HandleTypeDef* hspi_, GPIO_TypeDef* cs_gpio_, uint16_t cs_pin_);

	BaroData getSample();

private:
	// constants
	GPIO_TypeDef* cs_gpio;
	uint16_t cs_pin;
	SPI_HandleTypeDef* hspi;

	// helper functions
	uint16_t readCalibrationCoefficients(uint8_t PROM_READ_CMD);
	uint32_t getTemperatureReading();
	uint32_t getPressureReading();
	void resetBarometer();
};

/************************************ * FUNCTION DECLARATIONS ************************************/
#endif /* EXAMPLE_TASK_HPP_ */
