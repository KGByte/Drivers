/*
 * ADS1115.cpp
 *
 *  Created on: Jun 16, 2020
 *      Author: kgarland
 */

#include "ads1115.h"

ADS1115::ADS1115(I2CInterface* i2c_interface, ADS1115DeviceAddress addressIn) : address(addressIn)
{
    i2c = i2c_interface;
}

ADS1115::~ADS1115()
{
    // TODO Auto-generated destructor stub
}

int16_t ADS1115::readResult()
{
    i2c->Read(RxDataArray, 2, deviceAddress, CONVERSION_REGISTER);

    conversionData = (int16_t) (RxDataArray[0] << 8 | RxDataArray[1]);

    return conversionData;
}

void ADS1115::deviceConfig(inputChannel input)
{
    //window comparator, active low, assert after one conversion, 800SPS, single shot FSR+- 4.096
    uint16_t data = (1 << 15) | (uint8_t) input << 12 | 0x3F0;

    uint8_t msb = (uint8_t)(data >> 8) & 0x00FF;
    uint8_t lsb = (uint8_t) data & 0x00FF;
    TxDataArray[0] = CONFIG_REGISTER;
    TxDataArray[1] = msb;
    TxDataArray[2] = lsb;

    i2c->Write(TxDataArray, 3, deviceAddress);
}

void ADS1115::setIntThreshold(uint16_t hiThresholdVal, uint16_t loThresholdVal)
{
    TxDataArray[0] = HI_THRESHOLD_REGISTER;
    TxDataArray[1] = (hiThresholdVal >> 8) & 0x00FF;
    TxDataArray[2] = hiThresholdVal & 0x00FF;
    i2c->Write(TxDataArray, 3, deviceAddress);

    TxDataArray[0] = LOW_THRESHOLD_REGISTER;
    TxDataArray[1] = (loThresholdVal >> 8) & 0x00FF;
    TxDataArray[2] = loThresholdVal & 0x00FF;
    i2c->Write(TxDataArray, 3, deviceAddress);
}
