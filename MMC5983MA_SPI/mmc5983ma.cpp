/*
 * mmc5983ma.cpp
 *
 * Implementation of the MMC5983MA driver.
 */

#include "mmc5983ma.hpp"
#include "mmc5983ma_regs.hpp"
#include "spi_wrapper.hpp" 
#include "stm32h7xx_hal_gpio.h"
#include "SensorDataTypes.hpp"
#include "main.h"
#include <cstdint>

using std::uint8_t;
using std::uint16_t;
using std::uint32_t;

extern SPI_HandleTypeDef hspi2;
/**
 * @brief Constructor
 */
MMC5983MA::MMC5983MA()
{

}

MMC5983MA_Status MMC5983MA::Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin)
{
	_hspi = hspi;
	_csPort = csPort;
	_csPin  = csPin;

    HAL_Delay(10);
    // Set the chip select pin HIGH (idle) by default.
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    uint8_t productID = getProductID();

   if (productID == MMC5983MA_PRODUCT_ID_VALUE) {
	   return MMC5983MA_Status::OK;
   }
   else {
	   return MMC5983MA_Status::ERR_INVALID_ARG;
   }
   //default 200Hz sampling rate
   writeRegister(MMC5983MA_IT_CONTROL1, MMC5983MA_BW_200HZ);
}



uint8_t MMC5983MA::getProductID(){
    // (P ID at 0x2F)
    return (readRegister(MMC5983MA_P_ID));
}


MMC5983MA_Status MMC5983MA::triggerMeasurement(){
    writeRegister(MMC5983MA_IT_CONTROL0, MMC5983MA_TM_M);
    return MMC5983MA_Status::OK;
}

MMC5983MA_Status MMC5983MA::performSet(){
    writeRegister(MMC5983MA_IT_CONTROL0, MMC5983MA_SET);
    return MMC5983MA_Status::OK;
}


MMC5983MA_Status MMC5983MA::performReset(){
    writeRegister(MMC5983MA_IT_CONTROL0, MMC5983MA_RESET);
    return MMC5983MA_Status::OK;
}

MMC5983MA_Status MMC5983MA::setOffsets(float offsetX, float offsetY, float offsetZ) {
    _offsetX = offsetX;
    _offsetY = offsetY;
    _offsetZ = offsetZ;
    return MMC5983MA_Status::OK;
}

MMC5983MA_Status MMC5983MA::calibrateOffset(){
    MagData setReading;
    MagData resetReading;

    // --- 1. SET Sequence ---
    // 1. Perform SET
    if (performSet() != MMC5983MA_Status::OK) {
        return MMC5983MA_Status::ERR_I2C;
    }
    HAL_Delay(1); // Wait for measurement to complete
    
    // Measure
    triggerMeasurement();
    HAL_Delay(10); // Wait for measurement (~8ms for 100Hz BW)
    
    // Read SET data
    if (readData(setReading) != MMC5983MA_Status::OK) {
        return MMC5983MA_Status::ERR_I2C;
    }
    
    // --- 2.  RESET Sequence ---
    // 1. Perform RESET
    if (performReset() != MMC5983MA_Status::OK) {
        return MMC5983MA_Status::ERR_I2C;
    }
    HAL_Delay(1); // Wait for measurement to complete

    // Measure
    triggerMeasurement();
    HAL_Delay(10); // Wait for measurement (~8ms for 100Hz BW)

    // Read RESET data
    if (readData(resetReading) != MMC5983MA_Status::OK) {
        return MMC5983MA_Status::ERR_I2C;
    }

    // --- 3. Calculate Null Field Offset ---
    // Out1 (SET) = +H + Offset
    // Out2 (RESET) = -H + Offset
    // Offset = (Out1 + Out2) / 2

    float newOffsetX = (setReading.rawX + resetReading.rawX) / 2.0f;
    float newOffsetY = (setReading.rawY + resetReading.rawY) / 2.0f;
    float newOffsetZ = (setReading.rawZ + resetReading.rawZ) / 2.0f;

    // Update Internal member Variables
    setOffsets(newOffsetX, newOffsetY, newOffsetZ);

    return MMC5983MA_Status::OK;
}


MMC5983MA_Status MMC5983MA::readTemperature(float& tempOut) {
    // Trigger temperature measurement
    writeRegister(MMC5983MA_IT_CONTROL0, MMC5983MA_TM_T);
    
    HAL_Delay(10);
    // data ready Poll status
    uint8_t status = readRegister(MMC5983MA_STATUS);
    if (!(status & MMC5983MA_MEAS_T_DONE)) {
        return MMC5983MA_Status::ERR_NOT_READY; // Data not ready
    }

    // Read Reg 0x07
    uint8_t RawTemp = readRegister(MMC5983MA_TOUT);

    // Convert -75C to +125C range
    tempOut = -75.0f + ((float)RawTemp * 0.8f); // Each LSB = 0.8C
    
    return MMC5983MA_Status::OK;
}

MMC5983MA_Status MMC5983MA::readData(MagData& data) {
    // Read status register to check if data is ready
    uint8_t status = readRegister(MMC5983MA_STATUS);
    
    // Check if measurement done bit is set
    if (!(status & MMC5983MA_MEAS_M_DONE)) {
        return MMC5983MA_Status::ERR_NOT_READY; // Data not ready
    }
    
    // Data Ready. Read all 7 measurement regs at once.
    uint8_t buffer[7];
    readRegisters(MMC5983MA_XOUT0, buffer, 7);

    data.rawX = ((uint32_t)buffer[0] << 10) |
                ((uint32_t)buffer[1] << 2)  |
                ((uint32_t)(buffer[6] & 0xC0) >> 6);

    data.rawY = ((uint32_t)buffer[2] << 10) |
                ((uint32_t)buffer[3] << 2)  |
                ((uint32_t)(buffer[6] & 0x30) >> 4);

    data.rawZ = ((uint32_t)buffer[4] << 10) |
                ((uint32_t)buffer[5] << 2)  |
                ((uint32_t)(buffer[6] & 0x0C) >> 2);

        // Apply scaling factors
        data.scaledX = ((float)data.rawX - _offsetX) / _countsPerGauss;
        data.scaledY = ((float)data.rawY - _offsetY) / _countsPerGauss;
        data.scaledZ = ((float)data.rawZ - _offsetZ) / _countsPerGauss;

        return MMC5983MA_Status::OK;
}



/* ========================================================================*/
/* ========================PRIVATE HELPER FUNCTIONS========================*/
/* ========================================================================*/

void MMC5983MA::writeRegister(std::uint8_t reg, std::uint8_t value) {
    // Write : R/W bit (0) == 0
    uint8_t cmd_byte = (0x00|(reg & 0x7f));
    uint8_t txBuffer[2] = { cmd_byte, value };

    // Pull cd Low to select the chip
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);

    // Use our wrapper to transmit the 2 bytes
    HAL_SPI_Transmit(_hspi, txBuffer, 2, 150);

    // Pull CS High to deselect the chip
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}

uint8_t MMC5983MA::readRegister(uint8_t reg){
    // Read : R/W bit (0) == 1
    // Shift address left 2 bits, then OR with 0x01 to set the Read bit
    uint8_t cmd_byte[] = {(0x80 | (reg & 0x7f)), 0x00};
    uint8_t rx_data[] = {0, 0};

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(_hspi, cmd_byte, rx_data, 2, 1000);
	HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);


	return rx_data[1];


}

/**
 * @brief Reads multiple bytes from the sensor.
 */
void MMC5983MA::readRegisters(std::uint8_t reg, std::uint8_t* buffer, std::uint8_t len) {
    // 1. Create the command byte (same as readRegister)
	uint8_t tx[len+1];
	uint8_t rx[len+1];
	tx[0] = (0b10000000 | (0x7f & reg));// first 8 bits must be R and start reg

	for(uint8_t i = 1; i < len + 1; i++){
		tx[i] = 0x00; //fill tx with spi dummy bytes
	}
    // Pull CS Low
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);

    // 2. Send the command/address byte
    HAL_SPI_TransmitReceive(_hspi, tx, rx, len + 1, 150);
    
    // Pull CS High
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);


	for(uint16_t i = 0; i < len; i++){
		buffer[i] = rx[i+1];  //copy back i+1 into out since first byte is command garbage
	}


}
/* MMC5983MA_CPP */
