/*
 * LP5018.h
 *
 *  Created on: Aug 4, 2020
 *      Author: kgarland
 */

#ifndef DEVICES_LP5018_H_
#define DEVICES_LP5018_H_

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

class LP5018
{
public:
    enum LED_t
    {
        LED0 = 0,
        LED1,
        LED2,
        LED3,
        LED4,
        NUM_LEDS
    };

    enum Registers
    {
        DEVICE_CONFIG0 		= 0x00,
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
        LED4_BRIGHTNESS,
        OUT0_COLOR 			= 0x0F,
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
        OUT12_COLOR,
        OUT13_COLOR,
        OUT14_COLOR,
        RESET 				= 0x27
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
    LP5018(I2CInterface* i2c_interface);

    // Default Destructor
    ~LP5018();

    void setLED(uint8_t ledNum, uint8_t colorA, uint8_t colorB, uint8_t colorC, uint8_t brightness);
    void setBrightnes(uint8_t ledNum, uint8_t brightness);

    void deviceConfig();
    void resetDevice();

private:
    static const uint8_t deviceAddress = 0x28;
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

#endif /* DEVICES_LP5018_H_ */
