/*
 * MCP47A1.cpp
 *
 *  Created on: Jul 30, 2020
 *      Author: kgarland
 */

#include <mcp47a1.h>

MCP47A1::MCP47A1(I2CInterface *i2c_interface)
    : i2c{i2c_interface}
{
}

MCP47A1::~MCP47A1()
{
    // TODO Auto-generated destructor stub
}

uint8_t MCP47A1::getWiperValue()
{
    i2c->Read(RxDataArray, 1, deviceAddress, 0x00);
    wiperRegData = RxDataArray[0];
    return wiperRegData;
}
void MCP47A1::setValue(uint8_t data)
{
    TxDataArray[0] = 0x00;
    TxDataArray[1] = data;
    i2c->Write(TxDataArray, 2, deviceAddress);
}
