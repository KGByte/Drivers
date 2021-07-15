/*
 * ADS1118.cpp
 *
 *  Created on: Jul 22, 2020
 *      Author: kgarland
 */
#include "ads1118.h"

ADS1118::ADS1118()
{

}


ADS1118::~ADS1118()
{

}

/************************************************************************
 Performs SPI write.  SPI CS must be handled by the caller of this method.

 Parameters:
 - numBytes: Number of bytes to be transmitted.

 Returns:
 - bytesProcessed: Number of bytes processed out of "numBytes."

 ************************************************************************/
bool ADS1118::SPIWrite(uint8_t numBytes)
{
    uint8_t bytesProcessed = 0;

    while (bytesProcessed < numBytes)
    {
        SSIDataPut(SSI1_BASE, TxDataArray[bytesProcessed]);

        bytesProcessed++;
    }

    while (SSIBusy(SSI1_BASE))
        ; //look into this

    //TODO: add fail condition
    return true;
}

/************************************************************************
 Perform SPI write and read.  SPI CS must be handled by the caller of this
 method.

 Parameters:
 - numBytes: Sum of bytes to be transmitted and received.

 Returns:
 - bytesProcessed: Number of bytes processed out of "numBytes."

 ************************************************************************/
bool ADS1118::SPITransfer(uint8_t numBytes)
{
    uint8_t bytesProcessed = 0;
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    while (SSIDataGetNonBlocking(SSI1_BASE, &RxDataArray[0]))
        ;
    //SSIDataGet(SSI1_BASE, &RxDataArray[0]);
    while (bytesProcessed < numBytes)
    {
        //put data into the Tx FIFO

        SSIDataPut(SSI1_BASE, TxDataArray[bytesProcessed]);
        //chill out for a bit
        while (SSIBusy(SSI1_BASE))
            ; //look into this

        SSIDataGet(SSI1_BASE, &RxDataArray[bytesProcessed]);

        bytesProcessed++;
    }

    //TODO: add fail condition
    return true;
}

bool ADS1118::writeConfigRegister(Input in)
{
    //config data to be sent. Single shot, start conversion, choose channel to select
    TxDataArray[0] = (0x01 << 7 | (uint8_t) in << 4 | 0x02);
    TxDataArray[1] = 0x8B;
    TxDataArray[2] = 0x00;
    TxDataArray[3] = 0x00;

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    SPITransfer(4);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // Check that Register was Written Correctly
//    if (RxDataArray[2] == (TxDataArray[0] & 0x45)
//            && (RxDataArray[3] & 0x8B) == TxDataArray[1])
//    {
//        // Register Matches
//        return true;
//    }
//    else
//    {
//        // Register Data does not Match
//        return false;
//    }

    return true;
}

int16_t ADS1118::getConversionData()
{
    TxDataArray[0] = 0x00;
    TxDataArray[1] = 0x00;

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    SPITransfer(2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

    conversionData = RxDataArray[0] << 8 | RxDataArray[1];

    return conversionData;
}

for(uint32_t index = 0; index < max; index++)
{
    //stuff
    while (!success)
    {
        //keep trying
    }
}