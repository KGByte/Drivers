/*
 * OPT3001.h
 *
 *  Created on: Jun 30, 2020
 *      Author: kgarland
 */

#ifndef OPT3001_H_
#define OPT3001_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "I2C_Interface.h"

#include "Utilities/utility.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"


class OPT3001
{
public:

    enum Registers
    {
        RESULT = 0x00,
        CONFIGURATION = 0x01,
        LOW_LIMIT = 0x02,
        HIGH_LIMIT = 0x03,
        MANUFACTURER_ID = 0x7E, //will read 0x5449 if setup correctly
        DEVICE_ID = 0x7F        //will read 3001 if setup correctly
    };

    // Constructor with I2C Interface
    OPT3001(I2CInterface* i2c_interface);

    // Default Destructor
    ~OPT3001();

    uint16_t getResult();
    //setup config register

    uint16_t getDeviceId();

    void deviceConfig(); //done
    void setIntThreshold(); //done

    float convertResult(uint16_t data); //done

private:
    static constexpr uint8_t deviceAddress = 0x44;

    uint16_t conversionData;
    uint16_t deviceId;
    uint8_t RxDataArray[4];
    uint8_t TxDataArray[4];

    I2CInterface *i2c;
};

#endif /* OPT3001_H_ */
