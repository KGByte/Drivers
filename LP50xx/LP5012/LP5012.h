/*
 * LP5012.h
 *
 *  Created on: May 17, 2021
 *      Author: kyleg
 */

#ifndef DRIVERS_LP5012_H_
#define DRIVERS_LP5012_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "Library/I2c_Interface.h"

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

class LP5012
{
public:

public:
    enum LED_t
    {
        LED0 = 0,
        LED1,
        LED2,
        LED3,
        NUM_LEDS
    };

    enum Registers
    {
        DEVICE_CONFIG0      = 0x00,
        DEVICE_CONFIG1,
        LED_CONFIG0,
        BANK_BRIGHTNESS,
        BANK_A_COLOR,
        BANK_B_COLOR,
        BANK_C_COLOR,
        LED0_BRIGHTNESS,
        LED1_BRIGHTNESS,
        LED2_BRIGHTNESS,
        LED3_BRIGHTNESS,
        OUT0_COLOR,
        OUT1_COLOR,
        OUT2_COLOR,
        OUT3_COLOR,
        OUT4_COLOR,
        OUT5_COLOR,
        OUT6_COLOR,
        OUT7_COLOR,
        OUT8_COLOR,
        OUT9_COLOR,
        OUT10_COLOR,
        OUT11_COLOR,
        RESET
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

    LP5012(I2CInterface* i2c_interface);
    virtual ~LP5012();

    void setLED(uint8_t ledNum, uint8_t colorA, uint8_t colorB, uint8_t colorC, uint8_t brightness);
    void setBrightnes(uint8_t ledNum, uint8_t brightness);

    void deviceConfig();
    void resetDevice();

    void SetLED1();
    void SetLED2();
    void SetLED3();
    void SetLED4();
    void ClearAll();

private:
    static constexpr uint8_t device_address {0x14};
//    void I2CWrite(uint8_t registerAddr, uint8_t data);
//    void I2CRead(uint8_t registerAddr);
//    //used in read function
//    void I2CSingleWrite(uint8_t registerAddr);

    //Structure for associating LED register addresses to a specific LED output channel
    struct ledRegisterAddr_s
    {
        uint8_t registerColorA;
        uint8_t registerColorB;
        uint8_t registerColorC;
        uint8_t registerBrightness;
    };

    //Array used to retrieve the color and brightness registers associated with a specific LED channel
    static const ledRegisterAddr_s led[];

    uint8_t RxDataArray[4];
    uint8_t TxDataArray[4];

    I2CInterface *i2c;
};

#endif /* DRIVERS_LP5012_H_ */
