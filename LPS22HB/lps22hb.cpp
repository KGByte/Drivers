/*
 * LPS22HB.cpp
 *
 *  Created on: Jun 19, 2020
 *      Author: kgarland
 */

#include <lps22hb.h>

LPS22HB::LPS22HB(I2CInterface *i2c_interface)
{
    i2c = i2c_interface;
}

LPS22HB::~LPS22HB()
{
    // TODO Auto-generated destructor stub
}

int32_t LPS22HB::getPressureValue()
{
    i2c->Read(RxDataArray, 3, deviceAddress, PRESS_OUT_XL | multipleRead);
    pressureData = (int32_t)(RxDataArray[2] << 16 | RxDataArray[1] << 8 | RxDataArray[0]);

    return pressureData;
}

float LPS22HB::convertResult(int32_t pressureValue)
{
    float result = 0;

    result = (float)(pressureValue / scalingFactor); //result in Pa
    result /= 10.0;                                  //scale for kPa

    return result;
}

uint8_t LPS22HB::getWhoAmI()
{
    i2c->Read(RxDataArray, 1, deviceAddress, WHO_AM_I);
    whoAmI = RxDataArray[0];
    return whoAmI;
}

void LPS22HB::deviceConfig()
{
    TxDataArray[0] = CTRL_REG1;
    TxDataArray[1] = BIT_6;
    i2c->Write(TxDataArray, 2, deviceAddress);

    TxDataArray[0] = CTRL_REG2;
    TxDataArray[1] = BIT_4;
    i2c->Write(TxDataArray, 2, deviceAddress);

    TxDataArray[0] = CTRL_REG3;
    TxDataArray[1] = BIT_0 | BIT_2;
    i2c->Write(TxDataArray, 2, deviceAddress);
}

void LPS22HB::interruptConfig()
{
    TxDataArray[0] = INTERRUPT_CFG;
    TxDataArray[1] = BIT_0 | BIT_1 | BIT_3;
    i2c->Write(TxDataArray, 2, deviceAddress);
}

void LPS22HB::setIntThreshold(uint16_t value)
{
    uint8_t msb = (uint8_t)(value >> 8) & 0x00FF;
    uint8_t lsb = (uint8_t)value & 0x00FF;

    TxDataArray[0] = THS_P_H;
    TxDataArray[1] = msb;
    i2c->Write(TxDataArray, 2, deviceAddress);

    TxDataArray[0] = THS_P_L;
    TxDataArray[1] = lsb;
    i2c->Write(TxDataArray, 2, deviceAddress);
}
