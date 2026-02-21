/*
 * BaroTask07.cpp
 *
 *  Created on: Jan 23, 2026
 *      Author: jaddina
 */


/**
 ********************************************************************************
 * @file    BaroTask07.cpp
 * @author  jaddina
 * @date    Sep 13, 2025
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "BaroTask07.hpp"
#include "SystemDefines.hpp"
#include "Command.hpp"
#include "LoggingService.hpp"
#include "LoggingTask.hpp"

#include "DataBroker.hpp"
#include "Task.hpp"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * VARIABLES
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/



/************************************
 * FUNCTION DEFINITIONS
 ************************************/
BaroTask07::BaroTask07():Task(TASK_LOGGING_QUEUE_DEPTH_OBJS)
{

}

/**
 * @brief Initialize the BaroTask07
 *        Do not modify this function aside from adding the task name
 */
void BaroTask07::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)BaroTask07::RunTask,
            (const char*)"BaroTask07",
            (uint16_t)TASK_LOGGING_QUEUE_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_LOGGING_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

                SOAR_ASSERT(rtValue == pdPASS, "BaroTask07::InitTask() - xTaskCreate() failed");
}

void BaroTask07::Run(void * pvParams){


    while (1) {
        /* Process commands in blocking mode */
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if(res){

        	HandleCommand(cm);
        }
    }
}

void BaroTask07::HandleCommand(Command& cm){
	switch(cm.GetCommand()){
	case DATA_COMMAND:
		HandleRequestCommand(cm.GetTaskCommand());
		break;

	case TASK_SPECIFIC_COMMAND:
		break;
	}

	cm.Reset();

}
void BaroTask07::HandleRequestCommand(uint16_t taskCommand){
	switch(taskCommand){
	case BARO07_SAMPLE_AND_LOG:

		data = barometer.getSample();
		data.id = 0;
		LogData();

	default:
		break;
	}


}

void BaroTask07::LogData(){


DataBroker::Publish<BaroData>(&data);
	osDelay(10);
	Command logCommand(DATA_BROKER_COMMAND, static_cast<uint16_t>(DataBrokerMessageTypes::BARO_DATA));
	LoggingTask::Inst().GetEventQueue()->Send(logCommand);

	SOAR_PRINT("Data Sent to LoggingTask\n");

}



