/*
 * SHTC3.cpp
 *
 *  Created on: Jun 9, 2020
 *      Author: kgarland
 */

#include <Devices/shtc3.h>

SHTC3::SHTC3(I2CInterface *i2c_interface)
    : i2c{i2c_interface}
{
}

SHTC3::~SHTC3()
{
    // TODO Auto-generated destructor stub
}

/************************************************************************
Utility method to check for transmission error, and sends a stop condition
if there was an error

Parameters:
    - None
Returns:
    - True or False

************************************************************************/
bool SHTC3::checkSendError()
{
    //check for error, returns 0 if no error
    if (I2CMasterErr(I2C1_BASE) != 0)
    {
        //send stop condition if error
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
        //delay until transmission completes
        while (I2CMasterBusBusy(I2C1_BASE))
        {
        }
    }
    return false;
}

/************************************************************************
Utility method to convert the raw value to the actual humidity.
Performs error checking.

Parameters:
    - pointer to the raw humidity value from the I2C read
Returns:
    - floating point true humidity value

************************************************************************/
float SHTC3::convertHumidity(uint16_t humidity)
{
    float humidOut = 100.0 * (humidity / 65536.0);
    //bounds checking. Humidity should be a value between 0%RH to 100%RH
    if (humidOut > 100.0 || humidOut < 0.0)
        return 0.0;
    else
        return humidOut;
}

/************************************************************************
Utility method to convert the raw value to the actual temperature.
Performs error checking.

Parameters:
    - pointer to the raw temperature value from the I2C read
Returns:
    - floating point true temperature value

************************************************************************/
float SHTC3::convertTemp(uint16_t temp)
{
    //temp conversion results in Celsius
    float tempOut = (-45.0 + 175.0 * (temp / 65536.0));
    //bounds checking. Temperature should be a value between -40C and 125C
    if (tempOut > 125.0 || tempOut < -40.0)
        return 0.0;
    else
        return tempOut;
}

/************************************************************************
Sends the wake-up command using the I2C write utility method

Parameters:
    - None
Returns:
    - None
************************************************************************/
void SHTC3::sendWakeup()
{
    TxDataArray[0] = (wakeup >> 8) & 0x00FF;
    TxDataArray[1] = wakeup & 0x00FF;
    //write(wakeup);
    i2c->Write(TxDataArray, 2, deviceAddress);
}

/************************************************************************
Sends the sleep command using the I2C write utility method

Parameters:
    - None
Returns:
    - None
************************************************************************/
void SHTC3::sendSleep()
{
    //write(sleep);
    TxDataArray[0] = (sleep >> 8) & 0x00FF;
    TxDataArray[1] = sleep & 0x00FF;
    //write(wakeup);
    i2c->Write(TxDataArray, 2, deviceAddress);
}

/************************************************************************
Sends the measure command using the I2C write utility method

Parameters:
    - None
Returns:
    - None
************************************************************************/
void SHTC3::sendMeasureCmd()
{
    //write(readHumidity);
    TxDataArray[0] = (readHumidity >> 8) & 0x00FF;
    TxDataArray[1] = readHumidity & 0x00FF;
    //write(wakeup);
    i2c->Write(TxDataArray, 2, deviceAddress);
}

/************************************************************************
gets the raw humdity and temp value using the I2C read function and converts
to the actual value.

Parameters:
    - None
Returns:
    - None
************************************************************************/

void SHTC3::getHumidityAndTemp(float *humid, float *temp)
{
    i2c->ReadOnly(RxDataArray, 6, deviceAddress);
    uint16_t dataOutHumidity = (RxDataArray[0] << 8) | RxDataArray[1];
    uint16_t dataOutTemp = (RxDataArray[3] << 8) | RxDataArray[4];

    *humid = convertHumidity(dataOutHumidity);
    *temp = convertTemp(dataOutTemp);
}
