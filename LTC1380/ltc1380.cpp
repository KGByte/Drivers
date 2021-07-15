/*
 * LTC1380.cpp
 *
 *  Created on: Jul 28, 2020
 *      Author: kgarland
 */

#include <ltc1380.h>

LTC1380::LTC1380(I2CInterface *i2c_interface, LTCDeviceAddress addressIn)
    : address{addressIn}, i2c{i2c_interface}
{
}

LTC1380::~LTC1380()
{
}

void LTC1380::writeCommand(LTCInput input, bool enable)
{
    uint8_t command = (enable << 3) | (uint8_t)input;
    // Set slave address, false for writing from master to slave
    //    I2CMasterSlaveAddrSet(I2C1_BASE, (uint8_t) address, false);
    //
    //    // Put data into the FIFO
    //    I2CMasterDataPut(I2C1_BASE, command);
    //
    //    // Initiate multi byte transfer
    //    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    //    while (I2CMasterBusy (I2C1_BASE));
    TxDataArray[0] = command;
    i2c->Write(TxDataArray, 1, address);
}
