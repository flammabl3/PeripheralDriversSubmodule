/**
 ********************************************************************************
 * @file    MMC5983MATask.hpp
 * @author  Javier
 * @brief   FreeRTOS Task Wrapper for the MMC5983MA Magnetometer SPI Driver
 ********************************************************************************
 */

#ifndef MMC5983MA_TASK_HPP_
#define MMC5983MA_TASK_HPP_

/************************************
 * INCLUDES
 ************************************/
#include "main.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// system includes
#include "SystemDefines.hpp"
#include "Task.hpp"
#include "SensorDataTypes.hpp"

// Driver Includes
#include "mmc5983ma.hpp"
#include "spi_wrapper.hpp"

extern SPI_HandleTypeDef hspi2;
/************************************
 * CLASS DEFINITIONS
 ************************************/
class MMC5983MATask : public Task
{
public:
    static MMC5983MATask& Inst(){
        static MMC5983MATask inst;
        return inst;
    }
    
    // Command Definitions:
    enum MMC5983MA_Commands {
        MMC_CMD_START_READ   = 1, // Start reading data
        MMC_CMD_STOP_READ    = 2, // Stop reading data
        MMC_CMD_DISABLE_LOG  = 3, // Disable logging
        MMC_CMD_ENABLE_LOG   = 4  // Enable logging
    };


    void InitTask();

protected:
    static void RunTask(void* pvParams) { MMC5983MATask::Inst().Run(pvParams); }
    void Run(void* pvParams);
    void HandleCommand(Command& cm);


private:
    // Constructors
    MMC5983MATask();
    MMC5983MATask(const MMC5983MATask&);
    MMC5983MATask& operator=(const MMC5983MATask&);
    void LogData();

    SPI_HandleTypeDef* _hspi = &hspi2;
    GPIO_TypeDef* MMC_CS_PORT = MAG_CS_GPIO_Port;
    const uint16_t MMC_CS_PIN = MAG_CS_Pin; // Adjust as needed

    // Obj to allow delayed inits


    MMC5983MA magnetometer;
    MagData magData;


    // Chip Select Port




};

/*
 * TODO: FINISH
*/

#endif /* MMC5983MA_TASK_HPP_ */
