/*
 * ADS1118.h
 *
 *  Created on: Jul 22, 2020
 *      Author: kgarland
 */

#ifndef DEVICES_ADS1118_H_
#define DEVICES_ADS1118_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

class ADS1118
{
public:
    // Constructor
    ADS1118();
    ~ADS1118();

    // ADC Input Selection
    enum Input
    {
        PRECHARGE_VOLTAGE 	= 0x04,
        BATTERY_VOLTAGE 	= 0x05,
        HG_CURRENT 			= 0x06,
        LG_CURRENT 			= 0x07
    };

    bool SPIWrite(uint8_t numBytes);
    bool SPITransfer(uint8_t numBytes);
    bool writeConfigRegister(Input in);
    int16_t getConversionData();

private:
    // Buffers for SPI Communication
    uint8_t TxDataArray[4];
    uint32_t RxDataArray[4];
    int16_t conversionData;
};

#endif /* DEVICES_ADS1118_H_ */
