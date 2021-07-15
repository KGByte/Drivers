/*
 * LTC6811.h
 *
 *  Created on: Jun 29, 2020
 *      Author: kgarland
 */

#ifndef LTC6811_H_
#define LTC6811_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "Utilities/utility.h"

#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"


class LTC6811
{
public:
    // Register Groups for LTC6811
    enum RegisterGroup
    {
        CONFIG = 0x02,
        GROUP_A = 0x04,
        GROUP_B = 0x06,
        GROUP_C = 0x08,
        GROUP_D = 0x0A,
        AUX_A = 0x0C,
        AUX_B = 0x0E,
        STATUS_A = 0x10,
        STATUS_B = 0x12
    };

    // LTC6811 Command Codes
    enum Command
    {
        WRCFG = 0x001,
        RDCFG = 0x002,
        RDCVA = 0x004,
        RDCVB = 0x006,
        RDCVC = 0x008,
        RDCVD = 0x00A,
        RDAUXA = 0x00C,
        RDAUXB = 0x00E,
        RDSTATA = 0x010,
        RDSTATB = 0x012,
        CLRCELL = 0x711,
        CLRAUX = 0x712,
        CLRSTAT = 0x713,
        PLADC = 0x714,
        DIAGN = 0x715,
        WRCOMM = 0x721,
        RDCOMM = 0x722,
        STCOMM = 0x723
    };

    enum Cell
    {
        CELL0 = 0,
        CELL1,
        CELL2,
        CELL3,
        CELL4,
        CELL5,
        CELL6,
        CELL7,
        CELL8,
        CELL9,
        CELL10,
        CELL11,
        NUM_CELLS
    };

    enum GPIOInput
    {
        GPIO1 = 1,
		GPIO2,
		GPIO3,
		GPIO4,
		GPIO5,
		NUM_GPIOS
    };

    LTC6811();
    virtual ~LTC6811();

    // Call to Initialize
    void initialize();

    // Change ADCOPT value.  Must call writeConfigRegisters() to set.
    void configureADCOPT(uint8_t value);

    // Change REFON value.  Must call writeConfigRegisters() to set.
    void configureREFON(uint8_t value);

    // Change GPIO value.  Must call writeConfigRegisters() to set.
    void configureGPIO(uint8_t value);

    // Change VUV value.  Must call writeConfigRegisters() to set.
    void configureVUV(float underVoltage_mV);

    // Change VOV value.  Must call writeConfigRegisters() to set.
    void configureVOV(float overVoltage_mV);

    // Change DCC value.  Must call writeConfigRegisters() to set.
    void configureDCC(uint16_t value);

    // Change DCTO value.  Must call writeConfigRegisters() to set.
    void configureDCTO(uint8_t value);

    // Calculate the PEC of a given set of data
    uint16_t calculatePEC15(uint8_t *data, int len);

    // Perform PEC check on a 6-byte block
    bool checkPEC(uint8_t* data);

    // Initialize PEC Table
    void initPEC15Table();

    // Write Configuration Registers with Defaut Values
    void writeConfigRegisters();

    // Read all of the Cell Voltage Registers
    uint8_t getCellVoltages();

    // Read all of the Auxiliary Registers
    uint8_t getAuxData();

    // Read all of the Status Registers
    uint8_t getStatusData();

    // Command the LTC6811 to start an ADC Conversion
    void startCellADC();

    // Command the LTC6811 to start an ADC Conversion for the GPIO Pins
    void startGPIOADC();

    void sendWakeup();
    // Read data from the AUX A Register Group
    bool readAuxARegister();

    // Read data from the AUX B Register Group
    bool readAuxBRegister();

    // void readConfigRegister();

    // Read data from the Cell Voltage Register Group
    bool readCellVoltageRegister(RegisterGroup registerGroup);

    // Utility method for other member methods to call.  Retrieves data
    // from the LTC6804 device.
    bool readRegisterGroup(RegisterGroup registerGroup);

    // Read data from the Status A Register Group
    bool readStatusARegister();

    // Read data from the Status B Register Group
    bool readStatusBRegister();

    bool readConfigRegister();

    bool SPIWrite(uint8_t numBytes);
    bool SPITransfer(uint8_t numBytes);

    bool testWriteRead(uint8_t testData);
    uint8_t deviceIdTest();

    // Cell Voltages
    uint16_t cell_voltage[NUM_CELLS];       // 100uV / bit
    float scaled_cell_voltage[NUM_CELLS];   // V

    // GPIO Voltages
    uint16_t gpio_voltage[NUM_GPIOS];       // 100uV / bit
    float scaled_gpio_voltage[NUM_GPIOS];   // V

private:
    uint8_t RxDataArray[20];
    uint8_t TxDataArray[20];

    //used for PEC calculation, taken from LTC6811 datasheet
    int16_t pec15Table[256];
    int16_t CRC15_poly = 0x4599;

    // Configuration Register.  This array is used to configure the
    // LTC6804_2.  The data contained in this array is modified using
    // the configure methods, and this data is sent to the LTC6804_2
    // using the writeConfigRegisters() method.
    uint8_t configRegister[6] = { 0 };

    // ADC Mode Option (1 bit)
    uint8_t adcopt;

    // Reference Powered Up (1 bit)
    uint8_t refon;

    // GPIOx Pin Control (5 bits)
    uint8_t gpio_config;

    // Undervoltage Comparison Voltage (12 bits)
    uint16_t vuv;

    // Overvoltage Comparison Voltage (12 bits)
    uint16_t vov;

    // Discharge Cell Control (12 bits)
    uint16_t dcc;

    // Discharge Time Out Value (4 bits)
    uint8_t dcto;

//    // Second Reference Voltage
    uint16_t second_reference_voltage;      // 100uV / bit
    float scaled_second_reference_voltage;  // V

    // INA213 (Low Gain) Output Current
    uint16_t current_sense_voltage;         // 100uV /bit

    // Internal Temperature
    uint16_t internal_temperature;          // 0.1 DegC / bit
    float scaled_internal_temperature;      // DegC

    // Sum of Cells Voltage
    uint16_t sum_of_cells_voltage;          // 2mV / bit
    float scaled_sum_of_cells_voltage;      // V

    // Power Supply Voltages
    uint16_t analog_supply_voltage;         // 100uV / bit
    uint16_t digital_supply_voltage;        // 100uV / bit
    float scaled_analog_supply_voltage;     // V
    float scaled_digital_supply_voltage;    // V

    // Cell Over/Under Voltage Flags
    uint8_t under_over_voltage_flag[NUM_CELLS];

    // Thermal Shutdown Flag.  This bit is set when Thermal Shutdown occurs.
    // Reading the status register will clear this bit.
    uint8_t thermal_shutdown_flag;

    // MUX Self-Test Failure Flag
    uint8_t mux_self_test_failure_flag;
};

#endif /* LTC6811_H_ */
