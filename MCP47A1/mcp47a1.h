/*
 * MCP47A1.h
 *
 *  Created on: Jul 30, 2020
 *      Author: kgarland
 */

#ifndef DEVICES_MCP47A1_H_
#define DEVICES_MCP47A1_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "I2C_Interface.h"

class MCP47A1
{
public:
    // Constructor with I2C Interface
    MCP47A1(I2CInterface *i2c_interface);

    // Default Destructor
    ~MCP47A1();

    //This method really writes 2 bytes instead of one, but the command code is 0x00 always
    //Only the data changes
    //    void I2CWrite(uint8_t data);
    //    //used in Read function
    //    void I2CSingleWrite(uint8_t data);
    //Write one byte first then read one byte (from wiper register) should be 0x20 at startup
    //uint8_t I2CRead(uint8_t address);
    void setValue(uint8_t data);
    uint8_t getWiperValue();

private:
    static constexpr uint8_t deviceAddress = 0x2E;
    uint8_t wiperRegData;
    uint8_t RxDataArray[2];
    uint8_t TxDataArray[2];
    I2CInterface *i2c;
};

#endif /* DEVICES_MCP47A1_H_ */
