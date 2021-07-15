/*
 * LTC1380.h
 *
 *  Created on: Jul 8, 2020
 *      Author: kgarland
 *
 *
 Packet info for LTC1380 Analog Mux
 SDA - Pin 14
 SCL - Pin 15
 Send Byte Protocol -> S 1 0 0 1 1 A1 A0 W A X X X X En C2 C1 C0 A P
 | Address Byte  |   |    Command Byte   |
 S -> Start bit ()
 A1, A0 -> Address bits (0x90, 0x92, 0x94, 0x96)
 W -> Write command bit (set to 0)
 A -> Acknowledge bit from the LTC1380
 En, C2-C0 -> Multiplexer control bits (Controls S0-S7, if En is low, all are low -> see truth table)
 P -> Stop bit (The first stop bit after a successful command byte updates the multiplexer control latch)
 */


#ifndef LTC1380_H_
#define LTC1380_H_

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


class LTC1380
{
public:
    // Device addresses
    enum LTCDeviceAddress
    {
        DEVICE_1 = 0x48,
		DEVICE_2 = 0x49,
		DEVICE_3 = 0x4A,
		DEVICE_4 = 0x4B
    };

    // LTC1380 Analog Inputs
    enum LTCInput
    {
        S0 = 0,
		S1,
		S2,
		S3,
		S4,
		S5,
		S6,
		S7,
		NUM_CHANNELS
    };

    // Constructor with I2C Interface
    LTC1380(I2CInterface* i2c_interface, LTCDeviceAddress addressIn);

    // Default Destructor
    ~LTC1380();

    // This function chooses which Mux input channel to be selected
    void writeCommand(LTCInput input, bool enable);

private:
    // EN data bit. Must be 1 for any channels to be selected
    bool outputEnabled;
    LTCDeviceAddress address;
    uint8_t TxDataArray[1];

    // Pointer to I2C Interface
    I2CInterface *i2c;
};

#endif /* LTC1380_H_ */
