/*
 * LPS22HB.h
 *
 *  Created on: Jun 19, 2020
 *      Author: kgarland
 */

#ifndef LPS22HB_H_
#define LPS22HB_H_

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

class LPS22HB
{
public:
    enum Registers
    {
        INTERRUPT_CFG = 0x0B,
        THS_P_L,
        THS_P_H,
        WHO_AM_I = 0x0F,
        CTRL_REG1,
        CTRL_REG2,
        CTRL_REG3,
        FIFO_CTRL = 0x14,
        REF_P_XL,
        REF_P_L,
        REF_P_H,
        RPDS_L,
        RPDS_H,
        RES_CONF,
        INT_SOURCE = 0x25,
        FIFO_STATUS,
        STATUS,
        PRESS_OUT_XL,
        PRESS_OUT_L,
        PRESS_OUT_H,
        TEMP_OUT_L,
        TEMP_OUT_H,
        LPFP_RES = 0x33
    };

    enum Bits
    {
        BIT_0 = 1 << 0,
        BIT_1 = 1 << 1,
        BIT_2 = 1 << 2,
        BIT_3 = 1 << 3,
        BIT_4 = 1 << 4,
        BIT_5 = 1 << 5,
        BIT_6 = 1 << 6,
        BIT_7 = 1 << 7
    };

    // Constructor with I2C Interface
    LPS22HB(I2CInterface* i2c_interface);

    // Default Destructor
    ~LPS22HB();


    int32_t getPressureValue();
    float convertResult(int32_t pressureValue);

    //write to ctrl registers to configure device
    void deviceConfig();
    //configure interrupt register
    void interruptConfig();

    void setIntThreshold(uint16_t value);

    uint8_t getWhoAmI();

private:
    int32_t pressureData;
    uint8_t whoAmI;
    const float scalingFactor = 4096.0f;
    uint8_t RxDataArray[4];
    uint8_t TxDataArray[4];

    //A0 pin connected to ground
    static constexpr uint8_t deviceAddress = 0x5D;
    static constexpr uint8_t multipleRead = BIT_7;

    I2CInterface *i2c;
};

#endif /* LPS22HB_H_ */
