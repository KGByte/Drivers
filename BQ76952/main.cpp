/*
 * main.cpp
 *
 *  Created on: Apr 15, 2021
 *      Author: kyleg
 */

#include "TivaPin.h"
#include "TivaI2C.h" //here
#include "TivaSPI.h"
#include "BQ76952.h"

//#include <stdarg.h>
//#include <stdbool.h>
//#include <stdint.h>
//
//#include "inc/hw_ssi.h"
//#include "inc/hw_i2c.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "inc/hw_gpio.h"
//
//#include "driverlib/i2c.h"
//#include "driverlib/ssi.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/gpio.h"
//#include "driverlib/pin_map.h"


void InitGPIO();
void InitSPI();
void InitI2C();

//TivaPin CS(5, GPIO_PORTA_BASE, GPIO_PIN_3, Pin::kOutput);
TivaPin SDA(7, GPIO_PORTA_BASE, GPIO_PIN_7, Pin::kOutput);
TivaPin SCL(6, GPIO_PORTA_BASE, GPIO_PIN_6, Pin::kOutput);
TivaPin CS(5, GPIO_PORTA_BASE, GPIO_PIN_3, Pin::kOutput);
TivaSPI spi(SSI0_BASE, &CS, 1000000, 0, 0, 0);
TivaI2C i2c(I2C1_BASE, &SDA, &SCL);
BQ76952 bms(&spi, &i2c);

int main()
{
    uint16_t id;
    InitGPIO();
    InitI2C();
    InitSPI();
    CS.setIOType(Pin::kOutput);
    spi.Initialize();

    CS.SetStateOnPin(Pin::kHigh);

    //I2C testing and switch to SPI logic
    bms.GetDataMemoryI2C(BQ76952::DataMemory::kCommType);
    id = bms.GetDeviceId();
    bms.GetAllVoltages();
    bms.GetDataMemoryI2C(BQ76952::DataMemory::kCommType);

    bms.EnterConfigMode();
    delayUs(2500);
    bms.GetDataMemoryI2C(BQ76952::DataMemory::kEnableProtA);
    bms.GetDataMemoryI2C(BQ76952::DataMemory::kCommType);

    bms.SPIConfigure();
    bms.GetDataMemoryI2C(BQ76952::DataMemory::kCommType);
    bms.ExitConfigMode();
    delayUs(1500);
    bms.SendSubcommandI2C(BQ76952::SubCommands::kSwapCommMode);  //switch comms to what is prpogrammed in Comm type register?
    delayMs(1000);
    id = bms.GetDeviceId();

    bms.WakeupOScillator(10);

    //bms.SendSubcommand(BQ76952::SubCommands::kSleepEnable);

    bms.GetAllVoltages(false);
    delayUs(500);
    //bms.SendSubcommand(BQ76952::SubCommands::kSleepDisable);
    delayUs(500);
    id = bms.GetDeviceId(false);
    delayUs(2000);







    return 0;
}

void InitGPIO()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(5);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    return;
}

void InitSPI()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    // Wait for SPI Peripheral to Power Up
    SysCtlDelay(5);
}

void InitI2C()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    // Wait for I2C Peripheral to Power Up
    SysCtlDelay(5);

    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true); //true for 400kHz
}



