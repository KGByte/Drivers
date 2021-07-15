/*
 * LP5012.cpp
 *
 *  Created on: May 17, 2021
 *      Author: kyleg
 */

#include <Drivers/LP5012.h>

// Map LED Channels to LED Driver Outputs
const LP5012::ledRegisterAddr_s LP5012::led[] =
{
    {OUT0_COLOR,   OUT1_COLOR,     OUT2_COLOR,     LED0_BRIGHTNESS}, //LED0
    {OUT3_COLOR,   OUT4_COLOR,     OUT5_COLOR,     LED1_BRIGHTNESS}, //LED1
    {OUT6_COLOR,   OUT7_COLOR,     OUT8_COLOR,     LED2_BRIGHTNESS}, //LED2
    {OUT9_COLOR,   OUT10_COLOR,    OUT11_COLOR,    LED3_BRIGHTNESS}, //LED3
};

LP5012::LP5012(I2CInterface* i2c_interface) :
        i2c{i2c_interface}
{}

LP5012::~LP5012()
{
    // TODO Auto-generated destructor stub
}

void LP5012::setLED(uint8_t ledNum, uint8_t colorA, uint8_t colorB, uint8_t colorC, uint8_t brightness)
{
    if (ledNum < NUM_LEDS)
    {
        //Set color and brightness to an individual LED
        TxDataArray[0] = led[ledNum].registerBrightness;
        TxDataArray[1] = brightness;
        i2c->Write(TxDataArray, 2, device_address);

        // Utilize auto increment
        TxDataArray[0] = led[ledNum].registerColorA;
        TxDataArray[1] = colorA;
        TxDataArray[2] = colorB;
        TxDataArray[3] = colorC;
        i2c->Write(TxDataArray, 4, device_address);
    }
}

void LP5012::setBrightnes(uint8_t ledNum, uint8_t brightness)
{
    //I2CWrite(led[ledNum].registerBrightness, brightness);
    TxDataArray[0] = led[ledNum].registerBrightness;
    TxDataArray[1] = brightness;
    i2c->Write(TxDataArray, 2, device_address);
}

void LP5012::deviceConfig()
{
    //Bit 6: chip enable
    //all other bits: reserved
    TxDataArray[0] = DEVICE_CONFIG0;
    TxDataArray[1] = BIT_6;
    i2c->Write(TxDataArray, 2, device_address);
    //Set:
    //Bit 7: Reserved
    //Bit 6: Reserved
    //Bit 5: Log Scale = Enabled
    //Bit 4: Power Save = Enabled
    //Bit 3: Auto Increment = Enabled
    //Bit 2: PWM Dithering = Enabled
    //Bit 1: Max Current Option = 0 (25.5 mA)
    //Bit 0: LED Global Off = 0 (normal operation)
    TxDataArray[0] = DEVICE_CONFIG1;
    TxDataArray[1] = BIT_2 | BIT_3 | BIT_4 | BIT_5;
    i2c->Write(TxDataArray, 2, device_address);
    //disable bank control (want to control each LED individually for this application
    TxDataArray[0] = LED_CONFIG0;
    TxDataArray[1] = 0x00;
    i2c->Write(TxDataArray, 2, device_address);
}
void LP5012::resetDevice()
{
    //I2CWrite(RESET, 0xFF);

    TxDataArray[0] = RESET;
    TxDataArray[1] = 0xFF;
    i2c->Write(TxDataArray, 2, device_address);
}

void LP5012::SetLED1()
{
    setLED(0, 255, 255, 255, 255);
}
void LP5012::SetLED2()
{
    setLED(1, 0, 0, 255, 255);
}
void LP5012::SetLED3()
{
    setLED(2, 0, 255, 255, 255);
}
void LP5012::SetLED4()
{
    setLED(3, 255, 255, 0, 255);
}
void LP5012::ClearAll()
{
    setLED(0, 0, 0, 0, 0);
    setLED(1, 0, 0, 0, 0);
    setLED(2, 0, 0, 0, 0);
    setLED(3, 0, 0, 0, 0);
}


