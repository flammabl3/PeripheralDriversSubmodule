/**
 ********************************************************************************
 * @file    MX66L1G45GMI.cpp
 * @author  shiva
 * @date    Mar 26, 2025
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "MX66L1G45GMI.hpp"
#include "main.h"
#include <cstdint>
#include <cstring>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "SystemDefines.hpp"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
extern SPI_HandleTypeDef hspi1;

#define MX66_SPI hspi1

/************************************
 * VARIABLES
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

// Wrap all implementations in extern "C" so they can be called from C code
// #ifdef __cplusplus
// extern "C"
//{
// #endif

//-------------------------------------------------------------------------------------------------
// STM32 SPI Driver
//-------------------------------------------------------------------------------------------------

// Create a static instance of the config
static MX66_Config io_driver = {nullptr, nullptr, nullptr, nullptr};

// Init function
void MX66_Init(MX66_Config* config) {
    if(config != nullptr) {
        io_driver = *config;
    }
	else {
		SOAR_PRINT("MX66_Init: config is nullptr!\n");
	}
}

// Internal Helper Wrappers


void MX66_Delay(uint32_t ms) {
	if(io_driver.Delay) {
		io_driver.Delay(ms);
	} else {
		SOAR_PRINT("[MX66 ERROR] MX66_Delay called before MX66_Init!\n");
	}
}


void csLOW(void) {
	if(io_driver.SetCS) {
		io_driver.SetCS(false);
	} else {
		SOAR_PRINT("[MX66 ERROR] csLOW called before MX66_Init!\n");
	}
}


void csHIGH(void) {
	if(io_driver.SetCS) {
		io_driver.SetCS(true);
	} else {
		SOAR_PRINT("[MX66 ERROR] csHIGH called before MX66_Init!\n");
	}
}


void SPI_Write(uint8_t *data, uint16_t len) {
	if(io_driver.Write) {
		io_driver.Write(data, len);
	} else {
		SOAR_PRINT("[MX66 ERROR] SPI_Write called before MX66_Init!\n");
	}
}


void SPI_Read(uint8_t *data, uint16_t len) {
	if(io_driver.Read) {
		io_driver.Read(data, len);
	} else {
		SOAR_PRINT("[MX66 ERROR] SPI_Read called before MX66_Init!\n");
	}
}

/************************************
 * FUNCTION DEFINITIONS
 ************************************/
uint32_t MX66_ReadID(void)
{
	uint8_t tData = 0x9F;
	uint8_t rData[3];
	csLOW();
	SPI_Write(&tData, 1);
	SPI_Read(rData, 3);
	csHIGH();
	uint32_t ret = ((rData[0] << 16) | (rData[1] << 8) | rData[2]);
	return ret;
}

uint8_t MX66_ReadStatus(int reg)
{
	uint8_t tData, rData;
	switch (reg)
	{
	case 1:
		tData = 0x05;
		break;
		//		case 2: tData = 0x35; break;
		//		case 3: tData = 0x15; break;
	default:
		SOAR_PRINT("Invalid status register 0\n");
		return 0;
	}
	csLOW();
	SPI_Write(&tData, 1);
	SPI_Read(&rData, 1);
	csHIGH();
	return (rData);
}

void MX66_WriteStatus(int reg, uint8_t newstatus)
{
	uint8_t tData[2];
	switch (reg)
	{
	case 1:
		tData[0] = 0x01;
		break;
		//		case 2: tData[0] = 0x31; break;
		//		case 3: tData[0] = 0x11; break;
	default:
		return;
	}

	tData[1] = newstatus;
	write_enable();
	csLOW();
	SPI_Write(tData, 2);
	csHIGH();
	write_disable();
}

void MX66_ReadSFDP(uint8_t *rData)
{
	uint8_t tData[5] = {0x5A, 0, 0, 0, 0};
	csLOW();
	SPI_Write(tData, 5);
	SPI_Read(rData, 256); //	Why 256???
	csHIGH();
}

void MX66_Read(uint32_t block, uint16_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (block * FS_SECTOR_SIZE) + offset;

	tData[0] = 0x03;
	tData[1] = (memAddr >> 16) & 0xFF;
	tData[2] = (memAddr >> 8) & 0xFF;
	tData[3] = (memAddr) & 0xFF;

	csLOW();
	SPI_Write(tData, 4);
	SPI_Read(rData, size);
	csHIGH();
}

void MX66_FastRead(uint32_t block, uint16_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (block * FS_SECTOR_SIZE) + offset;

	tData[0] = 0x0B;
	tData[1] = (memAddr >> 16) & 0xFF;
	tData[2] = (memAddr >> 8) & 0xFF;
	tData[3] = (memAddr) & 0xFF;
	tData[4] = 0;

	csLOW();
	SPI_Write(tData, 5);
	SPI_Read(rData, size);
	csHIGH();
}

void write_enable(void)
{
	uint8_t tData = 0x06;
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	MX66_Delay(5);
}

void write_disable(void)
{
	uint8_t tData = 0x04;
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	MX66_Delay(5);
}

void MX66_Erase_Chip(void)
{
	uint8_t tData = 0x60;

	write_enable();
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	write_disable();

	while (MX66_ReadStatus(1) & 0x01)
		; // ????
}

void MX66_Erase_Sector(uint16_t numsector)
{
	uint8_t tData[6];
	uint32_t memAddr = numsector * FS_SECTOR_SIZE; // Each sector contains 16 pages * 256 bytes

	write_enable();

	tData[0] = 0x21;
	tData[1] = (memAddr >> 16) & 0xFF;
	tData[2] = (memAddr >> 8) & 0xFF;
	tData[3] = (memAddr) & 0xFF;

	csLOW();
	SPI_Write(tData, 4);
	csHIGH();

	while (MX66_ReadStatus(1) & 0x01)
		;

	write_disable();
}

void MX66_Erase_Block(uint32_t block)
{
	uint8_t tData[6];
	uint32_t memAddr = block * FS_BLOCK_SIZE; // Each sector contains 16 pages * 256 bytes

	write_enable();

	tData[0] = 0xD8;
	tData[1] = (memAddr >> 16) & 0xFF;
	tData[2] = (memAddr >> 8) & 0xFF;
	tData[3] = (memAddr) & 0xFF;

	csLOW();
	SPI_Write(tData, 4);
	csHIGH();

	while (MX66_ReadStatus(1) & 0x01)
		;

	write_disable();
}

void MX66_Write_Page(uint32_t page, uint16_t offset, uint32_t size, const uint8_t *data)
{
	uint8_t tData[266];
	uint32_t memAddr = (page * FS_PAGE_SIZE) + offset;
	uint32_t indx = 0;

	SOAR_PRINT("MX66_Write(page=%ld, offset=%d, memaddr=%ld) memAddr=%08lx\n", page, offset, size, memAddr);

	write_enable();

	tData[0] = 0x02;				   // block program
	tData[1] = (memAddr >> 16) & 0xFF; // MSB of the memory Address
	tData[2] = (memAddr >> 8) & 0xFF;
	tData[3] = (memAddr) & 0xFF; // LSB of the memory Address
	indx = 4;

	memcpy(&tData[indx], data, size);

	csLOW();
	SPI_Write(tData, indx + size);
	csHIGH();

	while (MX66_ReadStatus(1) & 0x01)
		; // Check BUSY is low, if not wait, TODO add timeout?
	write_disable();
}

void MX66_Write_Block(uint32_t block, uint16_t offset, uint32_t size, const uint8_t *data)
{
	uint32_t startpage = ((block * FS_SECTOR_SIZE) + offset) / FS_PAGE_SIZE;
	uint32_t bytesleft = size;
	uint32_t newoff = offset % 256;
	uint32_t bufptr = 0;

	SOAR_PRINT("MX66_Write_block(%ld,%d,%ld)  startpage=%ld newoff=%ld\n", block, offset, size, startpage, newoff);

	SOAR_PRINT("First %ld,%04ld,%03ld ", startpage, newoff, (FS_PAGE_SIZE - newoff));
	MX66_Write_Page(startpage, newoff, (FS_PAGE_SIZE - newoff), data); // First block
	bufptr = (FS_PAGE_SIZE - newoff);
	bytesleft -= (FS_PAGE_SIZE - newoff);

	startpage++;
	while (bytesleft)
	{

		if (bytesleft > 256)
		{
			SOAR_PRINT("Page %ld,%04d,%03d ", startpage, 0, FS_PAGE_SIZE);
			MX66_Write_Page(startpage, 0, FS_PAGE_SIZE, &data[bufptr]);
			bytesleft -= FS_PAGE_SIZE;
			bufptr += FS_PAGE_SIZE;
		}
		else
		{
			if (newoff)
			{
				SOAR_PRINT("Last  %ld,%04d,%03ld ", startpage, 0, newoff);
				MX66_Write_Page(startpage, 0, newoff, &data[bufptr]); // Last block
			}
			else
			{
				SOAR_PRINT("Last  %ld,%04d,%03ld ", startpage, 0, bytesleft);
				MX66_Write_Page(startpage, 0, bytesleft, &data[bufptr]);
				bufptr += bytesleft;
			}
			bytesleft = 0;
		}
		startpage++;
	}
	SOAR_PRINT("\n");
}

bool isWriteProtected(void)
{
	return false;
}

// #ifdef __cplusplus
// }
// #endif
