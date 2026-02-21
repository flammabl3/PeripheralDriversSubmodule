 /**
 ********************************************************************************
 * @file    MMC5983MATask.cpp
 * @author  Javier
 * @date    2026-01-10
 * @brief   Implementation of the MMC5983MA task handling.
 ********************************************************************************
 */

 /************************************
 * INCLUDES
 ************************************/
#include "mmc5983Task.hpp"
#include "main.h"
#include "DataBroker.hpp"
#include "LoggingTask.hpp"


// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

 /************************************
 * MACROS AND DEFINES
 ************************************/
/* temp definitions if not SystemDefines.hpp*/
#ifndef MMC_TASK_QUEUE_DEPTH
#define MMC_TASK_QUEUE_DEPTH 10
#endif

#ifndef MMC_TASK_STACK_DEPTH
#define MMC_TASK_STACK_DEPTH 512 // Adjust..
#endif

#ifndef MMC_TASK_PRIORITY
#define MMC_TASK_PRIORITY 3
#endif

 /************************************
 * FUNCTION DEFINITIONS
 ************************************/

 // Constructor
 MMC5983MATask::MMC5983MATask() : Task(MMC_TASK_QUEUE_DEPTH)
{

}




void MMC5983MATask::InitTask() // RTOS Task Init
{
    // Make sure dependencies are set


	magnetometer.Init(_hspi, MMC_CS_PORT, MMC_CS_PIN);



    // Assert Task not already created
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize MMC5983MA task twice");
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)MMC5983MATask::RunTask,
            (const char*)"MMC5983MATask",
            (uint16_t)MMC_TASK_STACK_DEPTH,
            (void*)this,
            (UBaseType_t)MMC_TASK_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

     SOAR_ASSERT(rtValue == pdPASS, "MMC5983MATask::InitTask() - xTaskCreate() failed");
}


void MMC5983MATask::Run(void * pvParams)  // Instance Run loop for task
{
        // Handle incoming commands (optional)
        // receive with 0 timeout to poll
        Command cm;
        if (qEvtQueue->Receive(cm, 0)) {
            HandleCommand(cm);
        }

}

void MMC5983MATask::HandleCommand(Command & cm)
{
    switch (cm.GetTaskCommand())
    {
    // TODO: Add command cases (Gain change, calibration, etc.) IF NEEDED
    
    case MMC5983MA_Commands::MMC_CMD_START_READ: // Start Readings

        SOAR_PRINT("MMC5983MATask: Enabled Readings.\n");
        break;

    case MMC5983MA_Commands::MMC_CMD_STOP_READ: // Stop Readings

        SOAR_PRINT("MMC5983MATask: Disabled Readings.\n");
        break;

    case MMC5983MA_Commands::MMC_CMD_ENABLE_LOG: // Enable Logging
		while(1){
			magnetometer.triggerMeasurement();
			vTaskDelay(pdMS_TO_TICKS(10));
			magnetometer.readData(magData);
			LogData();
			osDelay(1000);
			SOAR_PRINT("Data Sent to LoggingTask\n");
		}

        break;

    case MMC5983MA_Commands::MMC_CMD_DISABLE_LOG: // Disable Logging

        SOAR_PRINT("MMC5983MATask: Disabled Logging.\n");
        break;

    default:
        SOAR_PRINT("MMC5983MATask: Received Unsupported Command {%d}.\n", cm.GetTaskCommand());
        break;
    }

    cm.Reset();
}

void MMC5983MATask::LogData(){

	SOAR_PRINT("Mag rawX: %d\n", magData.rawX);
	SOAR_PRINT("Mag rawY: %d\n", magData.rawY);
	SOAR_PRINT("Mag rawZ: %d\n", magData.rawZ);

	SOAR_PRINT("Mag scaledX: %f\n", magData.scaledX);
	SOAR_PRINT("Mag scaledY: %f\n", magData.scaledY);
	SOAR_PRINT("Mag scaledZ: %f\n", magData.scaledZ);

	DataBroker::Publish<MagData>(&magData);
	Command logCommand(DATA_BROKER_COMMAND, static_cast<uint16_t>(DataBrokerMessageTypes::MAG_DATA));
	LoggingTask::Inst().GetEventQueue()->Send(logCommand);

}
