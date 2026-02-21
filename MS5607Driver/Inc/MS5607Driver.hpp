/** ********************************************************************************
 * @file    MS5607Driver.hpp
 * @author  Noah
 * @date    Nov 16, 2024
 * @brief	This driver gets pressure and temperature readings from the MS5607 barometers.
 ******************************************************************************** */
#ifndef SOARDRIVERS_MS5607DRIVER_INC_MS5607DRIVER_HPP_
#define SOARDRIVERS_MS5607DRIVER_INC_MS5607DRIVER_HPP_
/************************************ * INCLUDES ************************************/
#include "SystemDefines.hpp"
#include "cmsis_os.h"
#include "SensorDataTypes.hpp"
/************************************ * MACROS AND DEFINES ************************************/

/************************************ * TYPEDEFS ************************************/

/************************************ * CLASS DEFINITIONS ************************************/
/**
 * @brief the driver for MS5607 barometers
 * see datasheet here: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5607-02BA03%7FB%7Fpdf%7FEnglish%7FENG_DS_MS5607-02BA03_B.pdf%7FCAT-BLPS0035
 */
class MS5607_Driver{
public:
	MS5607_Driver(SPI_HandleTypeDef* hspi_, GPIO_TypeDef* cs_gpio_, uint16_t cs_pin_):
		hspi(hspi_), cs_gpio(cs_gpio_), cs_pin(cs_pin_){}

	BaroData getSample();
private:

	//constants
	GPIO_TypeDef* cs_gpio;
	uint16_t cs_pin;
	SPI_HandleTypeDef* hspi;

	uint16_t readCalibrationCoefficients(uint8_t PROM_READ_CMD);
	uint32_t getTemperatureReading();
	uint32_t getPressureReading();
	void resetBarometer();
};

/************************************ * FUNCTION DECLARATIONS ************************************/
#endif /* EXAMPLE_TASK_HPP_ */
