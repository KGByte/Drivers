/*
 * LTC6811.cpp
 *
 *  Created on: Jun 29, 2020
 *      Author: kgarland
 */

#include <Devices/ltc6811.h>

LTC6811::LTC6811()
{
    // TODO Auto-generated constructor stub

}

LTC6811::~LTC6811()
{
    // TODO Auto-generated destructor stub
}

/************************************************************************
 Initializes the LTC6804 device.

 Parameters:
 - None

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::initialize()
{
    initPEC15Table();

    //call wakeup function here
    sendWakeup();
    delayUs(200);

    configureADCOPT(0);
    configureREFON(1);
    configureGPIO(0x1F);
    configureVUV(2200);
    configureVOV(4200);
    configureDCC(0x0000);
    configureDCTO(0);

    writeConfigRegisters();
}

/************************************************************************
 Configure ADCOPT value.  This method merely prepares the data to be sent to
 the battery monitor chip.  A call to writeConfigRegisters() must be made
 to actually set this value.

 Parameters:
 - value (1 bit): This value should be 1 or 0
 0 = selects 27 kHz, 7 kHz, or 26 Hz mode; MD bits must also be set (default)
 1 = selects 14 kHz, 3 kHz, or 2 kHz mode; MD bits must also be set

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::configureADCOPT(uint8_t value)
{
    // Isolate only the relevant information
    value = value & 0x01;

    // Clear previous data
    configRegister[0] = configRegister[0] & ~0x01;

    // Append new data
    configRegister[0] = configRegister[0] | value;
}

/************************************************************************
 Configure REFON value.  This method merely prepares the data to be sent to
 the battery monitor chip.  A call to writeConfigRegisters() must be made
 to actually set this value.

 Parameters:
 - value (1 bit): This value should be 1 or 0
 0 = reference shuts down after conversions (default)
 1 = reference remains powered up until WDT expires

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::configureREFON(uint8_t value)
{
    // Isolate only the relevant information
    value = value & 0x01;

    // Clear previous data
    configRegister[0] = configRegister[0] & ~0x04;

    // Append new data
    configRegister[0] = configRegister[0] | (value << 2);
}

/************************************************************************
 Configure GPIO Pin Control values.  This method merely prepares the data
 to be sent to the battery monitor chip.  A call to writeConfigRegisters()
 must be made to actually set this value.

 Parameters:
 - value (5 bits): This value sets each of the 5 bits of the GPIO Pin Control.
 Each bit may be set to the following:
 0 = GPIO pin pull-down ON
 1 = GPIO pin pull-down OFF (default)

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::configureGPIO(uint8_t value)
{
    // Isolate only the relevant information
    value = value & 0x1F;

    // Clear previous data
    configRegister[0] = configRegister[0] & ~0xF8;

    // Append new data
    configRegister[0] = configRegister[0] | (value << 3);
}

/************************************************************************
 Configure VUV value.  This method merely prepares the data to be sent to
 the battery monitor chip.  A call to writeConfigRegisters() must be made
 to actually set this value.

 Parameters:
 - underVoltage_mV (12 bits): Undervoltage value in mV.  A value
 greater than or equal to 3.2 mV should be used.

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::configureVUV(float underVoltage_mV)
{
    //Convert scaled value to integer expected by the device
    int16_t vuv = ((underVoltage_mV * 10.0f) / 16.0f) - 1.0f;

    // Limit to max value of 12 bits
    if (vuv > 4095)
    {
        vuv = 4095;
    }

    // Limit to positive values.  The LTC6811 ADC rounds negative
    // readings to 0.
    if (vuv < 0)
    {
        vuv = 0;
    }

    // Clear previous data
    configRegister[1] = configRegister[1] & ~0xFF;
    configRegister[2] = configRegister[2] & ~0x0F;

    // Append new data
    configRegister[1] = configRegister[1] | (uint8_t)(vuv & 0x00FF);
    configRegister[2] = configRegister[2] | (uint8_t)((vuv & 0x0F00) >> 8);
}

/************************************************************************
 Configure VOV value.  This method merely prepares the data to be sent to
 the battery monitor chip.  A call to writeConfigRegisters() must be made
 to actually set this value.

 Parameters:
 - overVoltage_mV (12 bits): Overvoltage value in mV.

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::configureVOV(float overVoltage_mV)
{
    //Convert scaled value to integer expected by the device
    uint16_t vov = ((overVoltage_mV * 10.0f) / 16.0f);

    // Limit to max value of 12 bits
    if (vov > 4095)
    {
        vov = 4095;
    }

    // Clear previous data
    configRegister[2] = configRegister[2] & ~0xF0;
    configRegister[3] = configRegister[3] & ~0xFF;

    // Append new data
    configRegister[2] = configRegister[2] | (uint8_t)((vov & 0x000F) << 4);
    configRegister[3] = configRegister[3] | (uint8_t)((vov & 0x0FF0) >> 4);
}

/************************************************************************
 Configure DCC value.  This method merely prepares the data to be sent to
 the battery monitor chip.  A call to writeConfigRegisters() must be made
 to actually set this value.

 Parameters:
 - value (12 bits): This value sets each of the DCC bits for each of the cells
 0 = Turn OFF shorting switch for the cell (default)
 1 = Turn ON shorting switch for the cell

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::configureDCC(uint16_t value)
{
    // Isolate only the relevant information
    value = value & 0x0FFF;

    // Clear previous data
    configRegister[4] = configRegister[4] & ~0xFF;
    configRegister[5] = configRegister[5] & ~0x0F;

    // Append new data
    configRegister[4] = configRegister[4] | (uint8_t)(value & 0x00FF);
    configRegister[5] = configRegister[5] | (uint8_t)((value & 0x0F00) >> 8);
}

/************************************************************************
 Configure DCTO value.  This method merely prepares the data to be sent to
 the battery monitor chip.  A call to writeConfigRegisters() must be made
 to actually set this value.

 Parameters:
 - value (4 bits): The following values correspond to the indicated timeouts:
 0x0 = Disabled
 0x1 = 0.5 min
 0x2 = 1 min
 0x3 = 2 min
 0x4 = 3 min
 0x5 = 4 min
 0x6 = 5 min
 0x7 = 10 min
 0x8 = 15 min
 0x9 = 20 min
 0xA = 30 min
 0xB = 40 min
 0xC = 60 min
 0xD = 75 min
 0xE = 90 min
 0xF = 120 min

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::configureDCTO(uint8_t value)
{
    // Isolate only the relevant information
    value = value & 0x0F;

    // Clear previous data
    configRegister[5] = configRegister[5] & ~0xF0;

    // Append new data
    configRegister[5] = configRegister[5] | (value << 4);
}

/************************************************************************
 This method configures the LTC6804 device.  This method transmits the
 contents of the configRegister[] array to the configuration registers on
 the LTC6804 device.  This method should be called after the application
 has made all the necessary "configure" method calls.

 Parameters:
 - None

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::writeConfigRegisters()
{
    // Write Configuration Registers Command
    uint16_t command = WRCFG;
    TxDataArray[0] = command >> 8;
    TxDataArray[1] = command;

    // Calculate PEC and Append to Packet
    uint16_t pec = calculatePEC15(TxDataArray, 2);
    TxDataArray[2] = pec >> 8;
    TxDataArray[3] = pec;

    // Configuration Register Data
    TxDataArray[4] = configRegister[0];
    TxDataArray[5] = configRegister[1];
    TxDataArray[6] = configRegister[2];
    TxDataArray[7] = configRegister[3];
    TxDataArray[8] = configRegister[4];
    TxDataArray[9] = configRegister[5];

    // Calculate PEC and Append to Packet
    pec = calculatePEC15(&TxDataArray[4], 6);
    TxDataArray[10] = pec >> 8;
    TxDataArray[11] = pec;

    // Send SPI Data
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    SPIWrite(12);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

/************************************************************************
 Perform a read of all of the Cell Voltage Registers.

 Parameters:
 - None

 Returns:
 - readStatus: Starting at the right-most bit, each bit indicates
 whether a read was successful (1) or unsuccessful (0).

 ************************************************************************/
uint8_t LTC6811::getCellVoltages()
{
    uint8_t readStatus = 0;

    if (readCellVoltageRegister (GROUP_A))
    {
        readStatus |= (1 << 0);
    }
    if (readCellVoltageRegister (GROUP_B))
    {
        readStatus |= (1 << 1);
    }
    if (readCellVoltageRegister (GROUP_C))
    {
        readStatus |= (1 << 2);
    }
    if (readCellVoltageRegister (GROUP_D))
    {
        readStatus |= (1 << 3);
    }

    return readStatus;
}

/************************************************************************
 Perform a read of all of the Auxiliary Registers.

 Parameters:
 - None

 Returns:
 - readStatus: Starting at the right-most bit, each bit indicates
 whether a read was successful (1) or unsuccessful (0).

 ************************************************************************/
uint8_t LTC6811::getAuxData()
{
    uint8_t readStatus = 0;

    if (readAuxARegister())
    {
        readStatus |= (1 << 0);
    }
//    if (readAuxBRegister())
//    {
//        readStatus |= (1 << 1);
//    }

    return readStatus;
}

/************************************************************************
 Perform a read of all of the Status Registers.

 Parameters:
 - None

 Returns:
 - readStatus: Starting at the right-most bit, each bit indicates
 whether a read was successful (1) or unsuccessful (0).

 ************************************************************************/
uint8_t LTC6811::getStatusData()
{
    uint8_t readStatus = 0;

    if (readStatusARegister())
    {
        readStatus |= (1 << 0);
    }
    if (readStatusBRegister())
    {
        readStatus |= (1 << 1);
    }

    return readStatus;
}

/************************************************************************
 Performs SPI write.  SPI CS must be handled by the caller of this method.

 Parameters:
 - numBytes: Number of bytes to be transmitted.

 Returns:
 - bytesProcessed: Number of bytes processed out of "numBytes."

 ************************************************************************/
bool LTC6811::SPIWrite(uint8_t numBytes)
{
    uint8_t bytesProcessed = 0;

    while (bytesProcessed < numBytes)
    {
        SSIDataPut(SSI0_BASE, TxDataArray[bytesProcessed]);

        bytesProcessed++;
    }

    while (SSIBusy (SSI0_BASE))
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
bool LTC6811::SPITransfer(uint8_t numBytes)
{
    uint8_t bytesProcessed = 0;
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    while (SSIDataGetNonBlocking(SSI0_BASE, (uint32_t *) &RxDataArray[0]))
        ;
    //SSIDataGet(SSI0_BASE, &RxDataArray[0]);
    while (bytesProcessed < numBytes)
    {
        //put data into the Tx FIFO

        SSIDataPut(SSI0_BASE, TxDataArray[bytesProcessed]);
        //chill out for a bit
        while (SSIBusy (SSI0_BASE))
            ; //look into this

        SSIDataGet(SSI0_BASE, (uint32_t *) &RxDataArray[bytesProcessed]);


        bytesProcessed++;
    }

    //TODO: add fail condition
    return true;
}

/************************************************************************
 Start ADC for LTC6811 Cells.

 Parameters:
 - None

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::startCellADC()
{
    // Start Cell Voltage ADC Conversion Command
    unsigned short command = 0x360;
    TxDataArray[0] = command >> 8;
    TxDataArray[1] = command;

    // Calculate PEC and Append to Packet
    uint16_t pec = calculatePEC15(TxDataArray, 2);
    TxDataArray[2] = pec >> 8;
    TxDataArray[3] = pec;

    // Perform SPI Transfer
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    SPIWrite(4);

//    if (waitForADC)
//    {
//        //Wait for ADC to complete before returning.  MISO pin is high when complete.
//        while (!GPIOPin::MISO.get())
//            ;
//    }

    while (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4))
        ;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

/************************************************************************
 Start ADC for LTC6811 GPIO.

 Parameters:
 - None

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::startGPIOADC()
{
    // Start GPIO ADC Conversion Command
    uint16_t command = 0x560;
    TxDataArray[0] = command >> 8;
    TxDataArray[1] = command;

    // Calculate PEC and Append to Packet
    uint16_t pec = calculatePEC15(TxDataArray, 2);
    TxDataArray[2] = pec >> 8;
    TxDataArray[3] = pec;

    // Perform SPI Transfer
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    SPIWrite(4);

//    if (waitForADC)
//    {
//        //Wait for ADC to complete before returning.  MISO pin is high when complete.
//        while (!GPIOPin::MISO.get())
//            ;
//    }
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void LTC6811::sendWakeup()
{
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    delayUs(5);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

/************************************************************************
 This method fetches read data from the LTC6804.  The specific data that
 is fetched is determined by the "registerGroup" argument.  The fetched
 data is saved in an array called "RxDataArray".  The return value of
 this method should be checked to determine whether the fetched data
 matches its PEC; a return value of TRUE indicates that the data received
 matches the PEC sent with the data.

 Parameters:
 - registerGroup: Identifies the register group to retrieve data from.

 Returns:
 - TRUE: If PEC computed for received data matches received PEC
 - FALSE: If PEC computed for received data does NOT match received PEC

 ************************************************************************/
bool LTC6811::readRegisterGroup(RegisterGroup registerGroup)
{
    // Identify the register group to read from
    uint16_t command = (uint16_t)registerGroup;
    TxDataArray[0] = command >> 8;
    TxDataArray[1] = command;

    // Calculate PEC and Append to Packet
    uint16_t pec = calculatePEC15(TxDataArray, 2);
    TxDataArray[2] = pec >> 8;
    TxDataArray[3] = pec;

    // Loop for all 8 Bytes
    for (int i = 0; i < 8; i++)
    {
        TxDataArray[i + 4] = 0x00;
    }

    // Perform SPI Transfer
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    SPITransfer(12);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // Check integrity of received data
    if (checkPEC (&RxDataArray[4]))
    {
        // Computed PEC matches received PEC
        return true;
    }
    else
    {
        return false;
    }
}

/************************************************************************
 Read data from the LTC6804 Configuration Register Group.

 Parameters:
 - None

 Returns:
 - TRUE: If read was successful
 - FALSE: If read failed

 ************************************************************************/
//bool LTC6811::readConfigRegister()
//{
//    if (readRegisterGroup (CONFIG))
//    {
//        // ADC Mode Option
//        adcopt = (RxDataArray[4] & 0x01);
//        // Reference Powered Status
//        refon = ((RxDataArray[4] & 0x04) >> 2);
//        // GPIO Pin Control
//        gpio_config = ((RxDataArray[4] & 0xF8) >> 3);
//        // Undervoltage Comparison Voltage
//        vuv = (((RxDataArray[6] & 0x0F) << 8) | RxDataArray[5]);
//        // Overvoltage Comparison Voltage
//        vov = ((RxDataArray[7] << 4) | ((RxDataArray[6] & 0xF0) >> 4));
//        // Discharge Cell Status
//        dcc = (((RxDataArray[9] & 0x0F) << 8) | RxDataArray[8]);
//        // Discharge Time Out Value
//        dcto = ((RxDataArray[9] & 0xF0) >> 4);
//
//        return true;
//    }
//    else
//    {
//        // Read failed
//        return false;
//    }
//}

/************************************************************************
 Read data from the LTC6804 Cell Voltage Register Group.

 Parameters:
 - registerGroup: Identifies the Cell Voltage Register Group to read:
 GROUP_A = read from Cell Voltage Register Group A
 GROUP_B = read from Cell Voltage Register Group B
 GROUP_C = read from Cell Voltage Register Group C
 GROUP_D = read from Cell Voltage Register Group D

 Returns:
 - TRUE: If read was successful
 - FALSE: If read failed

 ************************************************************************/
bool LTC6811::readCellVoltageRegister(RegisterGroup registerGroup)
{
    bool didReadSucceed = true;
    uint16_t temp = 0;

    if (readRegisterGroup(registerGroup))
    {
        switch (registerGroup)
        {
        case GROUP_A:
            // Cell 1
            temp = RxDataArray[5] << 8 | RxDataArray[4];
            cell_voltage[0] = temp;
            scaled_cell_voltage[0] = temp * (float) (1e-4);
            // Cell 2
            temp = RxDataArray[7] << 8 | RxDataArray[6];
            cell_voltage[1] = temp;
            scaled_cell_voltage[1] = temp * (float) (1e-4);
            // Cell 3
            temp = RxDataArray[9] << 8 | RxDataArray[8];
            cell_voltage[2] = temp;
            scaled_cell_voltage[2] = temp * (float) (1e-4);
            break;
        case GROUP_B:
            // Cell 4
            temp = RxDataArray[5] << 8 | RxDataArray[4];
            cell_voltage[3] = temp;
            scaled_cell_voltage[3] = temp * (float) (1e-4);
            // Cell 5
            temp = RxDataArray[7] << 8 | RxDataArray[6];
            cell_voltage[4] = temp;
            scaled_cell_voltage[4] = temp * (float) (1e-4);
            // Cell 6
            temp = RxDataArray[9] << 8 | RxDataArray[8];
            cell_voltage[5] = temp;
            scaled_cell_voltage[5] = temp * (float) (1e-4);
            break;
        case GROUP_C:
            // Cell 7
            temp = RxDataArray[5] << 8 | RxDataArray[4];
            cell_voltage[6] = temp;
            scaled_cell_voltage[6] = temp * (float) (1e-4);
            // Cell 8
            temp = RxDataArray[7] << 8 | RxDataArray[6];
            cell_voltage[7] = temp;
            scaled_cell_voltage[7] = temp * (float) (1e-4);
            // Cell 9
            temp = RxDataArray[9] << 8 | RxDataArray[8];
            cell_voltage[8] = temp;
            scaled_cell_voltage[8] = temp * (float) (1e-4);
            break;
        case GROUP_D:
            // Cell 10
            temp = RxDataArray[5] << 8 | RxDataArray[4];
            cell_voltage[9] = temp;
            scaled_cell_voltage[9] = temp * (float) (1e-4);
            // Cell 11
            temp = RxDataArray[7] << 8 | RxDataArray[6];
            cell_voltage[10] = temp;
            scaled_cell_voltage[10] = temp * (float) (1e-4);
            // Cell 12
            temp = RxDataArray[9] << 8 | RxDataArray[8];
            cell_voltage[11] = temp;
            scaled_cell_voltage[11] = temp * (float) (1e-4);
            break;
        default:
            didReadSucceed = false;
            break;
        }
    }
    else
    {
        // Read failed
        didReadSucceed = false;
    }

    return didReadSucceed;
}

/************************************************************************
 Read data from the LTC6804 AUX A Register Group.

 Parameters:
 - None

 Returns:
 - TRUE: If read was successful
 - FALSE: If read failed

 ************************************************************************/
bool LTC6811::readAuxARegister()
{
    uint16_t temp = 0;

    if (readRegisterGroup (AUX_A))
    {
        // GPIO 1
        temp = RxDataArray[5] << 8 | RxDataArray[4];
        gpio_voltage[0] = temp;
        scaled_gpio_voltage[GPIO1] = temp * (float) (1e-4);
        // GPIO 2
        temp = RxDataArray[7] << 8 | RxDataArray[6];
        gpio_voltage[1] = temp;
        scaled_gpio_voltage[GPIO2] = temp * (float) (1e-4);
        // GPIO 3
        temp = RxDataArray[9] << 8 | RxDataArray[8];
        gpio_voltage[2] = temp;
        scaled_gpio_voltage[GPIO3] = temp * (float) (1e-4);

        return true;
    }
    else
    {
        // Read failed
        return false;
    }
}

/************************************************************************
 Read data from the LTC6804 AUX B Register Group.

 Parameters:
 - None

 Returns:
 - TRUE: If read was successful
 - FALSE: If read failed

 ************************************************************************/
//bool LTC6811::readAuxBRegister()
//{
//    uint16_t temp = 0;
//
//    if (readRegisterGroup (AUX_B))
//    {
//        // GPIO 4
//        temp = RxDataArray[5] << 8 | RxDataArray[4];
//        gpio_voltage[3] = temp;
//        scaled_gpio_voltage[3] = temp * (float) (1e-4);
//        // GPIO 5
//        temp = RxDataArray[7] << 8 | RxDataArray[6];
//        gpio_voltage[4] = temp;
//        scaled_gpio_voltage[4] = temp * (float) (1e-4);
//        // REF
//        temp = RxDataArray[9] << 8 | RxDataArray[8];
//        second_reference_voltage = temp;
//        scaled_second_reference_voltage = temp * (float) (1e-4);
//        return true;
//    }
//    else
//    {
//        // Read failed
//        return false;
//    }
//}

/************************************************************************
 Read data from the LTC6804 Status A Register Group.

 Parameters:
 - None

 Returns:
 - TRUE: If read was successful
 - FALSE: If read failed

 ************************************************************************/
bool LTC6811::readStatusARegister()
{
    uint16_t temp = 0;

    if (readRegisterGroup (STATUS_A))
    {
        // Sum of Cells Voltage
        temp = RxDataArray[5] << 8 | RxDataArray[4];
        sum_of_cells_voltage = temp;
        scaled_sum_of_cells_voltage = temp * (float) (2e-3);
        // Internal Temperature
        temp = RxDataArray[7] << 8 | RxDataArray[6];
        internal_temperature = temp;
        scaled_internal_temperature = ((int32_t) temp * 133 / 10000 - 273);
        // Analog Power Supply
        temp = RxDataArray[9] << 8 | RxDataArray[8];
        analog_supply_voltage = temp;
        scaled_analog_supply_voltage = temp * (float) (1e-4);
        return true;
    }
    else
    {
        // Read failed
        return false;
    }
}

/************************************************************************
 Read data from the LTC6804 Status B Register Group.

 Parameters:
 - None

 Returns:
 - TRUE: If read was successful
 - FALSE: If read failed

 ************************************************************************/
bool LTC6811::readStatusBRegister()
{
    uint16_t temp = 0;

    if (readRegisterGroup (STATUS_B))
    {
        // Digital Supply Voltage
        temp = RxDataArray[5] << 8 | RxDataArray[4];
        digital_supply_voltage = temp;
        scaled_digital_supply_voltage = temp * (float) (1e-4);
        // Cell Over/Under Voltage Flags
        under_over_voltage_flag[0] = (RxDataArray[6] >> 0) & 0x03;
        under_over_voltage_flag[1] = (RxDataArray[6] >> 2) & 0x03;
        under_over_voltage_flag[2] = (RxDataArray[6] >> 4) & 0x03;
        under_over_voltage_flag[3] = (RxDataArray[6] >> 6) & 0x03;
        under_over_voltage_flag[4] = (RxDataArray[7] >> 0) & 0x03;
        under_over_voltage_flag[5] = (RxDataArray[7] >> 2) & 0x03;
        under_over_voltage_flag[6] = (RxDataArray[7] >> 4) & 0x03;
        under_over_voltage_flag[7] = (RxDataArray[7] >> 6) & 0x03;
        under_over_voltage_flag[8] = (RxDataArray[8] >> 0) & 0x03;
        under_over_voltage_flag[9] = (RxDataArray[8] >> 2) & 0x03;
        under_over_voltage_flag[10] = (RxDataArray[8] >> 4) & 0x03;
        under_over_voltage_flag[11] = (RxDataArray[8] >> 6) & 0x03;
        // Thermal Shutdown Flag
        thermal_shutdown_flag = (RxDataArray[9] >> 0) & 0x01;
        // MUX Self Test Failure Flag
        mux_self_test_failure_flag = (RxDataArray[9] >> 1) & 0x01;

        return true;
    }
    else
    {
        // Read failed
        return false;
    }
}

/************************************************************************
 Generate PEC lookup table.

 Parameters:
 - None

 Returns:
 - Nothing

 ************************************************************************/
void LTC6811::initPEC15Table()
{
    int16_t remainder;
    for (int i = 0; i < 256; i++)
    {
        remainder = i << 7;
        for (int bit = 8; bit > 0; --bit)
        {
            if (remainder & 0x4000)
            {
                remainder = ((remainder << 1));
                remainder = (remainder ^ CRC15_poly);
            }
            else
            {
                remainder = ((remainder << 1));
            }
        }
        pec15Table[i] = remainder & 0xFFFF;
    }
}

/************************************************************************
 Calculate the PEC for a set of data.

 Parameters:
 - data: Pointer to data in which to calculate a PEC for.
 - len: Length of data (in bytes).

 Returns:
 - 16-bit PEC for data

 ************************************************************************/
uint16_t LTC6811::calculatePEC15(uint8_t *data, int len)
{
    int16_t remainder, address;
    remainder = 16; // PEC seed
    for (int i = 0; i < len; i++)
    {
        // Calculate PEC table address
        address = ((remainder >> 7) ^ data[i]) & 0xFF;
        remainder = (remainder << 8) ^ pec15Table[address];
    }

    // The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
    return (remainder * 2);
}

/************************************************************************
 Compute the PEC for a 6-byte block of data, and compare the calculated
 PEC with the received PEC (should follow the 6-byte data block).

 Parameters:
 - data: Pointer to data including received PEC for the data

 Returns:
 - TRUE: If PEC computed for received data matches received PEC
 - FALSE: If PEC computed for received data does NOT match received PEC

 ************************************************************************/
bool LTC6811::checkPEC(uint8_t* data)
{
    uint16_t local_pec;
    uint16_t remote_pec;

    // Check 6 Bytes against PEC
    local_pec = calculatePEC15(data, 6);
    remote_pec = data[6] << 8 | data[7];

    if (remote_pec != local_pec)
    {
        return false;
    }
    else
    {
        // No PEC Errors
        return true;
    }
}

/************************************************************************
 Read data from the LTC6804 Configuration Register Group.

 Parameters:
 - None

 Returns:
 - TRUE: If read was successful
 - FALSE: If read failed

 ************************************************************************/
bool LTC6811::readConfigRegister()
{
    if (readRegisterGroup(CONFIG))
    {
        // ADC Mode Option
        adcopt = (RxDataArray[4] & 0x01);
        // Reference Powered Status
        refon = ((RxDataArray[4] & 0x04) >> 2);
        // GPIO Pin Control
        gpio_config = ((RxDataArray[4] & 0xF8) >> 3);
        // Undervoltage Comparison Voltage
        vuv = (((RxDataArray[6] & 0x0F) << 8) | RxDataArray[5]);
        // Overvoltage Comparison Voltage
        vov = ((RxDataArray[7] << 4) | ((RxDataArray[6] & 0xF0) >> 4));
        // Discharge Cell Status
        dcc = (((RxDataArray[9] & 0x0F) << 8) | RxDataArray[8]);
        // Discharge Time Out Value
        dcto = ((RxDataArray[9] & 0xF0) >> 4);

        return true;
    }
    else
    {
        // Read failed
        return false;
    }
}

bool LTC6811::testWriteRead(uint8_t testData)
{
    // Write test data
    for (int i = 0; i < 6; i++)
    {
        configRegister[i] = testData;
    }
    writeConfigRegisters();

    // Perform readback
    if (readConfigRegister())
    {
        if (testData == 0x00)
        {
            if ((adcopt != 0) || (refon != 0) || (vuv != 0) || (vov != 0)
                    || (dcc != 0) || (dcto != 0))
            {
                // Readback did not match write
                return false;
            }
        }
        else if (testData == 0xFF)
        {
            if ((adcopt != 0x01) || (refon != 0x01) || (vuv != 0x0FFF)
                    || (vov != 0x0FFF) || (dcc != 0x0FFF) || (dcto != 0x0F))
            {
                // Readback did not match write
                return false;
            }
        }
        else
        {
            // Unrecognized test data, fail the test
            return false;
        }
    }
    else
    {
        // Read failed
        return false;
    }

    return true;
}

uint8_t LTC6811::deviceIdTest()
{
    // Initialize to known state
    initialize();
    delayMs(100);
    //readConfigRegister();
    // Test write and readback of Configuration Registers
    if (testWriteRead(0x00) == false)
    {
        return 1;
    }


    return 0;
}
