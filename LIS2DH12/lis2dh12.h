/*
 * LISDH12.h
 *
 *  Created on: Jul 21, 2020
 *      Author: kgarland
 */

#ifndef LIS2DH12_H_
#define LISDH12_H_

#include "SPI_Interface.h"

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

class LIS2DH12
{
public:

    enum Registers
    {
        STATUS_REG_AUX = 0x07,
        OUT_TEMP_L = 0x0C,
        OUT_TEMP_H,
        WHO_AM_I = 0x0F,
        CTRL_REG0 = 0x1E,
        TEMP_CFG_REG,
        CTRL_REG1,
        CTRL_REG2,
        CTRL_REG3,
        CTRL_REG4,
        CTRL_REG5,
        CTRL_REG6,
        REFERENCE,
        STATUS_REG,
        OUT_X_L,
        OUT_X_H,
        OUT_Y_L,
        OUT_Y_H,
        OUT_Z_L,
        OUT_Z_H,
        FIFO_CTRL_REG,
        FIFO_SRC_REG,
        INT1_CFG,
        INT1_SRC,
        INT1_THS,
        INT1_DURATION,
        INT2_CFG,
        INT2_SRC,
        INT2_THS,
        INT2_DURATION,
        CLICK_CFG,
        CLICK_SRC,
        CLICK_THS,
        TIME_LIMIT,
        TIME_LATENCY,
        TIME_WINDOW,
        ACT_THS,
        ACT_DUR
    };

    LIS2DH12(SPIInterface *spi_interface);
    virtual ~LIS2DH12();

    //setup control registers 1-5 for specific device operation
    void configReg0();
    void configReg1();
    void configReg2();
    void configReg3();
    void configReg4();
    void configReg5();
    void configReg6();

    void configInterrupt1();
    void configInterrupt2();

    void setInt1Threshold();
    void setInt2Threshold();

    void setInt1Duration();
    void setInt2Duration();

    int16_t getXPosition();
    int16_t getYPosition();
    int16_t getZPosition();

    int16_t getSingleX();
    int16_t getSingleY();
    int16_t getSingleZ();

    //checks whether new data has overrun old data, or if new data is available
    //read each bit and set flags
    uint8_t getDeviceStatus();

    //returns value in g's
    float convertResult(int16_t position);

    uint8_t getWhoAmI();

    uint8_t getintSource();
private:
    uint32_t RxDataArray[10];
    uint32_t TxDataArray[10];

    int16_t accelXData;
    int16_t accelYData;
    int16_t accelZData;

    uint8_t status_register;

    const uint8_t incrementRegisterBit = 0x40;
    const float conversionFactor = 32768.0;
    static constexpr uint8_t addReadBit = 0x80;

    SPIInterface *spi;
};

#endif /* DEVICES_LISDH12_H_ */
