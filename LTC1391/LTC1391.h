/*
 * LTC1391.h
 *
 *  Created on: Mar 16, 2021
 *      Author: kyleg
 */

#ifndef LTC1391_H_
#define LTC1391_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include <Library/SPI_Interface.h>

class LTC1391
{
public:
    // LTC1380 Analog Inputs
    enum class Channel:uint8_t
    {
        kS0 = 0,
        kS1,
        kS2,
        kS3,
        kS4,
        kS5,
        kS6,
        kS7,
        kNumChannels
    };

    LTC1391(SPIInterface *spi_interface);
    virtual ~LTC1391();

    void SelectChannel(uint8_t input, bool enable);

private:
    SPIInterface *spi;
    uint32_t tx_data_array[1];
};

#endif /* LTC1391_H_ */
