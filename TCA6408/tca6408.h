/*
 * tca6408.h
 *
 *  Created on: Dec 8, 2020
 *      Author: kyleg
 */

#ifndef DEVICES_TCA6408_H_
#define DEVICES_TCA6408_H_

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

class TCA6408
{
public:
    // Constructor with I2C Interface
    TCA6408(I2CInterface* i2c_interface);

    // Default Destructor
    ~TCA6408();

    enum ControlRegisters
    {
        INPUT_PORT = 0x00,
        OUTPUT_PORT,
        POLARITY_INVERSION,
        CONFIGURATION
    };

    void configureIO(ControlRegisters registers, uint8_t data);
    uint8_t getInputRegister();
    uint8_t getOutputRegister();

private:
    static constexpr uint8_t deviceAddress = 0x20;
    uint8_t RxDataArray[1];
    uint8_t TxDataArray[2];
    uint8_t inputRegData;
    uint8_t outputRegData;

//Old I2C methods, not part of i2c class
//    void I2CSingleWrite(uint8_t data);
//    void I2CMultipleWrite(uint8_t registerAddr, uint8_t data);
//    void I2CRead(uint8_t registerAddress);

    //pointer to i2c_interface object
    I2CInterface *i2c;
};


#endif /* DEVICES_TCA6408_H_ */
