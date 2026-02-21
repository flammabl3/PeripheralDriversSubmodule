

/*
 * file: lsm6dso.cpp
 *
 *  Created on: Jan 29, 2026
 *      Author: Jad Dina
*/



/* Includes ------------------------------------------------------------------*/
#include "lsm6dso.hpp"

LSM6DSO_Driver::LSM6DSO_Driver(){

}

void LSM6DSO_Driver::Init(SPI_HandleTypeDef* hspi_, uint8_t cs_pin_, GPIO_TypeDef* cs_gpio_){
	hspi = hspi_;
	cs_pin = cs_pin_;
	cs_gpio = cs_gpio_;
	CSHigh();

	//read from WHO_AM_I reg to confirm initialization

	uint8_t ID = getRegister(LSM6DSO_REG::WHO_AM_I);

	SOAR_PRINT("WHO_AM_I %d", ID);
	if(ID != LSM6DSO_ID){
		return;
	}

	/*set CTRL3_C reg disable continuous update for predictable reads, as well enable
	 * automatic increment of register addresses with multiple byte reads*/
	setRegister(LSM6DSO_REG::CTRL3_C,0b01000100);
	//set CTRL1_XL reg 208Hz and 2g for Accel
	setRegister(LSM6DSO_REG::CTRL1_XL,0b01010100);
	//set CTRL2_G reg 208Hz and 250dps for Gyro
	setRegister(LSM6DSO_REG::CTRL2_G,0b01010000);
	//bypass FIFO mode
	setRegister(LSM6DSO_REG::FIFO_CTRL_4,0b00000000);

}

void LSM6DSO_Driver::setRegister(LSM6DSO_REGISTER_t reg, uint8_t val){

	uint8_t tx[2] = {(uint8_t)(0b00000000 | (0x7f & reg)), val};// write MSB must be 0 ensures MSB is 0
	//transmit spi message
	CSLow();
	HAL_StatusTypeDef result = HAL_SPI_Transmit(hspi, tx, 2, 1000);
	CSHigh();
	if(result == HAL_OK){
		return;
	}
	else{
		SOAR_PRINT("set register error");
	}

}

uint8_t LSM6DSO_Driver::getRegister(LSM6DSO_REGISTER_t reg){
	uint8_t tx[2] = {(uint8_t)(0b10000000 | (0x7f & reg)), 0x00};  //read MSB must be 1 ensures MSB is 1
	uint8_t rx[2]= {0,0};
	//transmit address of reg and recieve reg data spi message
	CSLow();
	HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, 1000);
	CSHigh();
	if(HAL_OK == result){
		return rx[1];
	}
	return 0;
}

void LSM6DSO_Driver::readRegisters(uint8_t startreg, uint8_t *out, uint16_t numBytes){
	uint8_t tx[numBytes+1];
	uint8_t rx[numBytes+1];
	tx[0] = (0b10000000 | (0x7f & startreg));// first 8 bits must be R and start reg

	for(uint16_t i = 1; i < numBytes + 1; i++){
		tx[i] = 0x00; //fill tx with spi dummy bytes
	}


	CSLow();
	HAL_SPI_TransmitReceive(hspi, tx, rx, numBytes+1, 1000);
	CSHigh();

	for(uint16_t i = 0; i < numBytes; i++){
		out[i] = rx[i+1]; //copy back i+1 into out since first byte is command garbage
	}

}

IMUData LSM6DSO_Driver::bytesToStruct(const uint8_t *raw_bytes, bool accel, bool gyro, bool temp){

	IMUData out;
	uint8_t i = 0;
	//litte endian (L to H) so shift high byte up 1 byte and fill lower byte
	if(temp){

		out.temp = (int16_t)(raw_bytes[i+ 1] << 8 | raw_bytes[i]);
		i+=2;
	}

	if(gyro){
		out.gyro.x = (int16_t)(raw_bytes[i+ 1] << 8 | raw_bytes[i]);
		out.gyro.y = (int16_t)(raw_bytes[i+ 3] << 8 | raw_bytes[i+ 2]);
		out.gyro.z = (int16_t)(raw_bytes[i+ 5] << 8 | raw_bytes[i+ 4]);
		i+=6;
	}

	if(accel){
		out.accel.x = (int16_t)(raw_bytes[i+ 1] << 8 | raw_bytes[i]);
		out.accel.y = (int16_t)(raw_bytes[i+ 3] << 8 | raw_bytes[i+ 2]);
		out.accel.z = (int16_t)(raw_bytes[i+ 5] << 8 | raw_bytes[i+ 4]);
	}

	out.accel.x *= 0.732; // mg/LSB
	out.accel.y *= 0.732;
	out.accel.z *= 0.732;

	out.gyro.x *= 8.75; //mdps/LSB
	out.gyro.y *= 8.75;
	out.gyro.z *= 8.75;

	out.temp = 25.0f + (out.temp / 256.0f);

	return out;
}

void LSM6DSO_Driver::readSensors(uint8_t *out){
	readRegisters(LSM6DSO_REG::OUT_TEMP_L, out, 14);
}



void LSM6DSO_Driver::CSHigh(){

	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);
}


void LSM6DSO_Driver::CSLow(){

	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);
}
