/*
 * tca6408.cpp
 *
 *  Created on: Dec 8, 2020
 *      Author: kyleg
 */

#include <Devices/tca6408.h>

TCA6408::TCA6408(I2CInterface *i2c_interface)
    : i2c{i2c_interface}
{
}

TCA6408::~TCA6408()
{
}

uint8_t TCA6408::getInputRegister()
{
    //    I2CRead(INPUT_PORT);
    //    inputRegData = RxDataArray[0];
    //    return inputRegData;
    i2c->Read(RxDataArray, 1, deviceAddress, INPUT_PORT);
    inputRegData = RxDataArray[0];
    return inputRegData;
}

uint8_t TCA6408::getOutputRegister()
{
    i2c->Read(RxDataArray, 1, deviceAddress, OUTPUT_PORT);
    outputRegData = RxDataArray[0];
    return outputRegData;
}

//if pin is an input, set to 1
//if pin is an output, set to 0
void TCA6408::configureIO(ControlRegisters registers, uint8_t data)
{
    //I2CMultipleWrite(CONFIGURATION, data);
    TxDataArray[0] = registers;
    TxDataArray[1] = data;
    i2c->Write(TxDataArray, 2, deviceAddress);
}
