/*
 * ADS1115.h
 *
 *  Created on: Jun 16, 2020
 *      Author: kgarland
 */

#ifndef ADS1115_H_
#define ADS1115_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "I2C_Interface.h"

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

class ADS1115
{
public:

    enum ADS1115DeviceAddress
    {
        DEVICE_1 = 0x48,
        DEVICE_2 = 0x49
    };

    enum inputChannel
    {
        WATER_1A = 0x04,
		WATER_1B,
		WATER_2A,
		WATER_2B
    };

    enum addressRegisters
    {
        CONVERSION_REGISTER, //16 bit conversion result
        CONFIG_REGISTER,     //configuring register
        LOW_THRESHOLD_REGISTER,
        HI_THRESHOLD_REGISTER
    };

    // Constructor with I2C Interface Defined
    ADS1115(I2CInterface* i2c_interface, ADS1115DeviceAddress addressIn);

    // Default Destructor
    ~ADS1115();

    int16_t readResult();
    //setup config register
    void deviceConfig(inputChannel input);

    void setIntThreshold(uint16_t hiThresholdVal, uint16_t loThresholdVal);

private:
    //If address pin is connected to ground. Use this way for now.
    static constexpr uint8_t deviceAddress = 0x48; //x1001000

    ADS1115DeviceAddress address;

    int16_t conversionData;

    uint8_t RxDataArray[4];
    uint8_t TxDataArray[4];

    I2CInterface *i2c;
};

#endif /* ADS1115_H_ */
