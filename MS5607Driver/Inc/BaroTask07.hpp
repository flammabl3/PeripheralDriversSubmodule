/*
 * BaroTask07.hpp
 *
 *  Created on: Jan 26, 2026
 *      Author: jaddina
 */

#ifndef PERIPHERALDRIVERSSUBMODULE_MS5607DRIVER_INC_BAROTASK07_HPP_
#define PERIPHERALDRIVERSSUBMODULE_MS5607DRIVER_INC_BAROTASK07_HPP_
#include "Task.hpp"
#include "MS5607Driver.hpp"
#include "SensorDataTypes.hpp"
#include "main.h"

extern SPI_HandleTypeDef hspi4;
/************************************
 * MACROS AND DEFINES
 ************************************/


/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/
enum BARO07_TASK_COMMANDS {
	BARO07_NONE,
	BARO07_SAMPLE_AND_LOG

};

/************************************
 * FUNCTION DECLARATIONS
 ************************************/
class BaroTask07: public Task
{
	public:
		static BaroTask07& Inst() {
			static BaroTask07 inst;
			return inst;
		}

		void InitTask();



	protected:
		static void RunTask(void* pvParams) { BaroTask07::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
		void Run(void * pvParams); // Main run code
		void HandleCommand(Command& cm);
		void HandleRequestCommand(uint16_t taskCommand);
		BaroData data;

		GPIO_TypeDef* MS5607_CS_PORT = BARO07_CS_GPIO_Port;
		const uint16_t MS5607_CS_PIN = BARO07_CS_Pin; //adjust when needed
		SPI_HandleTypeDef* hspi_= &hspi4;// adjust this when needed
		MS5607_Driver barometer{hspi_, MS5607_CS_PORT, MS5607_CS_PIN};




	private:
		// Private Functions
		BaroTask07();        // Private constructor
		BaroTask07(const BaroTask07&);                        // Prevent copy-construction
		BaroTask07& operator=(const BaroTask07&);														// Prevent assignment
		void LogData();
};

#endif /* PERIPHERALDRIVERSSUBMODULE_MS5607DRIVER_INC_BAROTASK07_HPP_ */
