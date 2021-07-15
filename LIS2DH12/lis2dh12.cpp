/*
 * LISDH12.cpp
 *
 *  Created on: Jul 21, 2020
 *      Author: kgarland
 */

#include <lis2dh12.h>

LIS2DH12::LIS2DH12(SPIInterface *spi_interface)
    :
    spi(spi_interface)
{}

LIS2DH12::~LIS2DH12()
{
    // TODO Auto-generated destructor stub
}

/************************************************************************
 Check the value of the device identifier register

 Parameters:
 - None

 Returns:
 - Hexadecimal value of the who_am_i register

 ************************************************************************/
uint8_t LIS2DH12::getWhoAmI()
{
    uint8_t whoAmI;
    TxDataArray[0] = (WHO_AM_I | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    whoAmI = RxDataArray[1]; //should be in index 1 of array
    return whoAmI;
}

uint8_t LIS2DH12::getintSource()
{
    uint8_t source;
    TxDataArray[0] = (INT1_SRC | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    source = RxDataArray[1]; //should be in index 1 of array
    return source;
}

int16_t LIS2DH12::getSingleX()
{
    uint8_t lowX = 0, highX = 0;
    int16_t xValue = 0;

    TxDataArray[0] = (OUT_X_L | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    lowX = (uint8_t) (RxDataArray[1] & 0x000000FF); //should be in index 1 of array
    RxDataArray[1] = 0;

    TxDataArray[0] = (OUT_X_H | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    highX = (uint8_t) (RxDataArray[1] & 0x000000FF);
    RxDataArray[1] = 0;

    //SysCtlDelay(100 * (SysCtlClockGet() / 3 / 1000));
    xValue = (int16_t) ((highX << 8) | lowX);
    return xValue;
}

int16_t LIS2DH12::getSingleY()
{
    uint8_t lowY = 0, highY = 0;
    int16_t yValue = 0;

    TxDataArray[0] = (OUT_Y_L | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    lowY = (uint8_t) (RxDataArray[1] & 0x000000FF); //should be in index 1 of array
    RxDataArray[1] = 0;

    TxDataArray[0] = (OUT_Y_H | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    highY = (uint8_t) (RxDataArray[1] & 0x000000FF);
    RxDataArray[1] = 0;

    //SysCtlDelay(100 * (SysCtlClockGet() / 3 / 1000));
    yValue = (int16_t) ((highY << 8) | lowY);
    return yValue;
}

int16_t LIS2DH12::getSingleZ()
{
    uint8_t lowZ = 0, highZ = 0;
    int16_t zValue = 0;

    TxDataArray[0] = (OUT_Z_L | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    lowZ = (uint8_t) (RxDataArray[1] & 0x000000FF); //should be in index 1 of array
    RxDataArray[1] = 0;

    TxDataArray[0] = (OUT_Z_H | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    highZ = (uint8_t) (RxDataArray[1] & 0x000000FF);
    RxDataArray[1] = 0;

    //SysCtlDelay(100 * (SysCtlClockGet() / 3 / 1000));
    zValue = (int16_t) ((highZ << 8) | lowZ);
    return zValue;
}

int16_t LIS2DH12::getXPosition()
{
    //uint16_t xPos;
    TxDataArray[0] = (OUT_X_L | addReadBit | incrementRegisterBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    accelXData = (int16_t)((RxDataArray[2] << 8) | RxDataArray[1]); //MSB in index 2 of array
    return accelXData;
}

int16_t LIS2DH12::getYPosition()
{
    //int16_t yPos;
    TxDataArray[0] = (OUT_Y_L | addReadBit | incrementRegisterBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    accelYData = (int16_t)((RxDataArray[2] << 8) | RxDataArray[1]); //MSB in index 2 of array
    return accelYData;
}
int16_t LIS2DH12::getZPosition()
{
    //int16_t zPos;
    TxDataArray[0] = (OUT_Z_L | addReadBit | incrementRegisterBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    accelZData = (int16_t)((RxDataArray[2] << 8) | RxDataArray[1]); //MSB in index 2 of array
    return accelZData;
}

uint8_t LIS2DH12::getDeviceStatus()
{
    TxDataArray[0] = (STATUS_REG | addReadBit);
    TxDataArray[1] = 0x00;
    spi->Transfer(TxDataArray, RxDataArray, 2);
    status_register = RxDataArray[1];
    return status_register;
}


//  TODO finish this for the correct data format
/************************************************************************
 Write to the configuration registers to correctly configure the device
 for the operation we want

 Parameters:
 - None

 Returns:
 - None

 ************************************************************************/
void LIS2DH12::configReg0()
{
    TxDataArray[0] = (CTRL_REG0);
    TxDataArray[1] = 0x10;
    spi->Write(TxDataArray, 2);
    //write(0x10, 2, CTRL_REG0); //x, y, z axis enabled
}

//set output data rate to 100Hz and enable x, y, and z axes
void LIS2DH12::configReg1()
{
    TxDataArray[0] = (CTRL_REG1);
    TxDataArray[1] = 0x57;
    spi->Write(TxDataArray, 2);
    //write(0x57, 2, CTRL_REG1); //x, y, z axis enabled
}

//FIFO configuration. Ignore this for now
//void LIS2DH12::configReg2()
//{
//    GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, 0);
//    write(0x47, 2, CTRL_REG1); //x, y, z axis enabled
//    GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_1, GPIO_PIN_1);
//}

//int 1 is shock - this configured shock interrupt register
void LIS2DH12::configReg3()
{
    TxDataArray[0] = (CTRL_REG3);
    TxDataArray[1] = 0x40;
    spi->Write(TxDataArray, 2);
    //write(0x40, 2, CTRL_REG3); //x, y, z axis enabled
}
void LIS2DH12::configReg4()
{
    TxDataArray[0] = (CTRL_REG4);
    TxDataArray[1] = 0x80;
    spi->Write(TxDataArray, 2);
    //write(0x80, 2, CTRL_REG4); //4 wire, lsb at lower address, SDU bit set
}

//this configures the interrupt as a latching interrupt. Come back to this later
void LIS2DH12::configReg5()
{
    TxDataArray[0] = (CTRL_REG5);
    TxDataArray[1] = 0x00;
    spi->Write(TxDataArray, 2);
    //write(0x00, 2, CTRL_REG5); //x, y, z axis enabled
}

//int 2 is freefall - this configured freefall interrupt register
void LIS2DH12::configReg6()
{
    TxDataArray[0] = (CTRL_REG6);
    TxDataArray[1] = 0x20;
    spi->Write(TxDataArray, 2);
    //write(0x20, 2, CTRL_REG6); //x, y, z axis enabled
}

//configures intterupt 1 pin (shock)
void LIS2DH12::configInterrupt1()
{
    TxDataArray[0] = (INT1_CFG);
    TxDataArray[1] = 0x2A;
    spi->Write(TxDataArray, 2);
    //write(0x2A, 2, INT1_CFG); //x, y, z axis enabled
}

//configures intterupt 2 pin (freefall)
void LIS2DH12::configInterrupt2()
{
    TxDataArray[0] = (INT2_CFG);
    TxDataArray[1] = 0x95;
    spi->Write(TxDataArray, 2);
    //write(0x95, 2, INT2_CFG); //x, y, z axis enabled
}
//1.2g
void LIS2DH12::setInt1Threshold()
{
    TxDataArray[0] = (INT1_THS);
    TxDataArray[1] = 0x70;
    spi->Write(TxDataArray, 2);
    //write(0x70, 2, INT1_THS); //x, y, z axis enabled
}

//0.35g
void LIS2DH12::setInt2Threshold()
{
    TxDataArray[0] = (INT1_THS);
    TxDataArray[1] = 0x16;
    spi->Write(TxDataArray, 2);
    //write(0x16, 2, INT2_THS); //x, y, z axis enabled
}

//20ms
void LIS2DH12::setInt1Duration()
{
    TxDataArray[0] = (INT1_DURATION);
    TxDataArray[1] = 0x02;
    spi->Write(TxDataArray, 2);
    //write(0x02, 2, INT1_DURATION); //x, y, z axis enabled
}

//30ms
void LIS2DH12::setInt2Duration()
{
    TxDataArray[0] = (INT1_DURATION);
    TxDataArray[1] = 0x03;
    spi->Write(TxDataArray, 2);
    //write(0x03, 2, INT2_DURATION); //x, y, z axis enabled
}

float LIS2DH12::convertResult(int16_t position)
{
    float result = 0;

    result = (float) ((2.0 / conversionFactor) * position);

    return result;
}
