/*
 * OPT3001.cpp
 *
 *  Created on: Jun 30, 2020
 *      Author: kgarland
 */

#include <opt3001.h>

OPT3001::OPT3001(I2CInterface* i2c_interface)
{
    i2c = i2c_interface;
}

OPT3001::~OPT3001()
{
    // TODO Auto-generated destructor stub
}

uint16_t OPT3001::getResult()
{
    i2c->Read(RxDataArray, 2, deviceAddress, RESULT);
    conversionData = (uint16_t) (RxDataArray[0] << 8 | RxDataArray[1]);
    return conversionData;
}

uint16_t OPT3001::getDeviceId()
{
    i2c->Read(RxDataArray, 2, deviceAddress, DEVICE_ID);
    deviceId = (uint16_t) (RxDataArray[0] << 8 | RxDataArray[1]);
    return deviceId;

}

void OPT3001::deviceConfig()
{
    TxDataArray[0] = CONFIGURATION;
    TxDataArray[1] = 0xCE;
    TxDataArray[2] = 0x10;
    i2c->Write(TxDataArray, 3, deviceAddress);
    setIntThreshold(); //163d
}

float OPT3001::convertResult(uint16_t data)
{
    float result = 0.0;
    int power = 0.0;
    uint16_t fractional = 0, exponent = 0;

    fractional = data & 0x0FFF;
    exponent = (data >> 12) & 0x000F;
    power = pow_int(2, exponent);

    result = 0.01 * (float) power * (float) fractional;
    return result;
}

void OPT3001::setIntThreshold()
{
    TxDataArray[0] = HIGH_LIMIT;
    TxDataArray[1] = 0xA0;
    TxDataArray[2] = 0x010;
    i2c->Write(TxDataArray, 3, deviceAddress);
}





