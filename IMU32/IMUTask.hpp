/*
 * IMUTask.hpp
 *
 *  Created on: Jan 27, 2026
 *      Author: jaddina
 */

#ifndef PERIPHERALDRIVERSSUBMODULE_COMPONENTS_IMUTASK_HPP_
#define PERIPHERALDRIVERSSUBMODULE_COMPONENTS_IMUTASK_HPP_

#include <LSM6DO32Driver.h>
#include "SensorDataTypes.hpp"
#include "Task.hpp"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/
class IMUTask : public Task
{
public:
	static IMUTask &Inst()
	{
		static IMUTask inst;
		return inst;
	}

	void InitTask();

	enum IMU_TASK_COMMANDS
	{
		IMU_NONE,
		IMU_SAMPLE_AND_LOG,
		IMU_START_CONTINUOUS_PRINT,
		IMU_STOP_CONTINUOUS_PRINT,
	};

protected:
	static void RunTask(void *pvParams) { IMUTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
	void Run(void *pvParams);											   // Main run code
	void HandleCommand(Command &cm);
	void HandleRequestCommand(uint16_t taskCommand);
	void CalibrateGyroBias();
	void ApplyGyroBias();
	LSM6DO32_Driver imu;
	IMUData imu_data;
	GYRO_t gyro_bias = {0, 0, 0};
	bool continuous_print_enabled = false;
	uint8_t data[14];
	GPIO_TypeDef *LSM6DSO32_CS_PORT = IMU32_CS_GPIO_Port;
	const uint16_t LSM6DSO32_CS_PIN = IMU32_CS_Pin; // adjust when needed
	SPI_HandleTypeDef *hspi_ = &hspi2;				// adjust this when needed

private:
	// Private Functions
	IMUTask();							 // Private constructor
	IMUTask(const IMUTask &);			 // Prevent copy-construction
	IMUTask &operator=(const IMUTask &); // Prevent assignment
	void SampleAndPrint();
	void LogData();
};

#endif /* PERIPHERALDRIVERSSUBMODULE_COMPONENTS_IMUTASK_HPP_ */
