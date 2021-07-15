/*
 * SHTC3.h
 *
 *  Created on: Jun 9, 2020
 *      Author: kgarland
 */

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

#ifndef SHTC3_H_
#define SHTC3_H_

class SHTC3
{
public:
    SHTC3(I2CInterface* i2c_interface);
    virtual ~SHTC3();

    bool checkSendError();

    void sendWakeup();
    void sendMeasureCmd();
    void sendSleep();

    void getHumidityAndTemp(float *humid, float *temp);

private:
    const uint8_t deviceAddress = 0x70; //0000 0000 0111�0000
    const uint16_t sleep = 0xB098;       //1011�0000�1001�1000
    const uint16_t wakeup = 0x3517;      //0011�0101�0001�0111

    //measurement commands - clock stretching disabled
    const uint16_t readHumidity = 0x58E0;
    const uint16_t readTemp = 0x7866;

    //measurement commands - clock stretching enabled
    const uint16_t readHumidityCS = 0x5C24;
    const uint16_t readTempCS = 0x7CA2;

    uint8_t RxDataArray[6];
    uint8_t TxDataArray[4];
    uint8_t checkSum = 0;

    uint16_t humidityData = 0;
    uint16_t tempData = 0;

//    void read(uint16_t *dataOutHumidty, uint16_t *dataOutTemp,
//              uint8_t checkSum);
//    void write(uint16_t data);

    float convertHumidity(uint16_t humidity);
    float convertTemp(uint16_t temp);

    I2CInterface *i2c;

};

#endif /* SHTC3_H_ */
