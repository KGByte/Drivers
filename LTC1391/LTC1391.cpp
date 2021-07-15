/*
 * LTC1391.cpp
 *
 *  Created on: Mar 16, 2021
 *      Author: kyleg
 */

#include <Drivers/LTC1391.h>

LTC1391::LTC1391(SPIInterface *spi_interface)
    :
    spi{spi_interface}
{}

LTC1391::~LTC1391()
{
    // TODO Auto-generated destructor stub
}

void LTC1391::SelectChannel(uint8_t input, bool enable)
{
    uint32_t command = (enable << 3) | input;

    tx_data_array[0] = (command & 0xFF);

    spi->Write(tx_data_array, 1);
}
