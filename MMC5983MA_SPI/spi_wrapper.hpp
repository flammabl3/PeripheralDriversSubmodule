/*
 * spi_wrapper.hpp
 */

#ifndef SPI_WRAPPER_HPP
#define SPI_WRAPPER_HPP

#include <cstdint>
//#include <array>

extern "C" {
    #include "stm32h7xx_hal.h"
}

#define COMM_ERROR 5

class SPI_Wrapper {
public:
    /**
     * @brief Constructor.
     * @param hspi Pointer to the HAL SPI handle (e.g., &hspi1)
     */

    SPI_Wrapper(SPI_HandleTypeDef* hspi);

    /**
     * @brief Transmits and receives a single byte.
     * @param txByte The byte to send.
     * @return The byte received.
     */
    std::uint8_t transfer(std::uint8_t txByte);

    /**
     * @brief Transmits a block of data.
     * @param data Pointer to the data to send.
     * @param size Number of bytes to send.
     */
    void transmit(std::uint8_t* data, std::uint16_t size);

private:
    SPI_HandleTypeDef* _hspi;
};

#endif // SPI_WRAPPER_HPP
