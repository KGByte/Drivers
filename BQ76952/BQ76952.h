
#ifndef _BQ76952_H_
#define _BQ76952_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "Library/I2C_interface.h"
#include "Library/SPI_Interface.h"
#include "Utilities/Utility.h"


#include "inc/hw_i2c.h"
#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/ssi.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"


class BQ76952
{
public:
    //section 12 in TRM for info on individual bit fields
    enum class DirectCommands:uint8_t
    {
        kControlStatus      = 0x00,
        kSafetyAlertA       = 0x02,
        kSafetyStatusA,
        kSafetyAlertB,
        kSafetyStatusB,
        kSafetyAlertC,
        kSafetyStatusC,
        kPermFaultAlertA    = 0x0A,
        kPermFaultStatusA,
        kPermFaultAlertB,
        kPermFaultStatusB,
        kPermFaultAlertC,
        kPermFaultStatusC,
        kPermFaultAlertD,
        kPermFaultStatusD,
        kBatteryStatus,
        kStackVoltage       = 0x34,
        kPackPinVoltage     = 0x36,
        kLDPinVoltage       = 0x38,
        kCC2Current         = 0x3A,
        kAlarmStatus        = 0x62,
        kAlarmRawStatus     = 0x64,
        kAlarmEnable        = 0x66,
        kInternalTemp       = 0x68,
        kCFETOffTemp        = 0x6A,
        kDFETOffTemp        = 0x6C,
        kAlertTemp          = 0x6E,
        kTS1Temp            = 0x70,
        kTS2Temp            = 0x72,
        kTS3Temp            = 0x74,
        kHDQTemp            = 0x76,
        kDCHGTemp           = 0x78,
        kDDSGTemp           = 0x7A,
        kFETStatus          = 0x7F
    };

    //Subcommands are indirect commands
    //Command only subcommands - basically write only 
    enum class SubCommands:uint16_t
    {
        kExitDeepSleep      = 0x000E,
        kEnterDeepSleep     = 0x000F,
        kShutdown           = 0x0010,
        kReset              = 0x0012,
        kPDSGTest           = 0x001C,
        kFuseToggle         = 0x001D,
        kPCHGTest           = 0x001E,
        kCHGTest            = 0x001F,
        kDSGTest            = 0x0020,
        kFETEnable          = 0x0022,
        kPFEnable           = 0x0024,       //Toggle PF_EN in manufacturing Status (PF_EN enables/disables permanent failure checks)
        kPFReset            = 0x0029,       //clear PF (Permanent Fault) Status   
        kSeal               = 0x0030,       //place device in sealed status
        kEnterConfigUpdate  = 0x0090,
        kExitConfigUpdate   = 0x0092,
        kAllFETsOff         = 0x0095,
        kAllFETsOn          = 0x0096,
        kSleepEnable        = 0x0099,
        kSleepDisable       = 0x009A,
        kSwapToSPI          = 0x7C35,
        kSwapCommMode       = 0x29BC
    };

    //subcommands with data - meaning these subcommands return data to read
    enum class DataSubCommands:uint16_t
    {
        //coming soon April 2021
        kDeviceID        = 0x0001,
        kManuStatus      = 0x0057,
        kDAStatus1       = 0x0071,          //get cell and current 1-4
        kDAStatus2,                   //get cell and current 5-8
        kDAStatus3,                   //get cell and current 9-12
        kDAStatus4,                   //get cell and current 13-16
        kDAStatus5,                   //Vreg, VSS, Min Max cell V, Battery Sum V, Temps, Min Max Temps, CC1 and 3 current, CC2 and 3 counts
        kDAStatus6,                   //other stuff see datasheet
        kDAStatus7,                   // TS3, HDQ, DCHG, DDSG 32 bit counts
        kCBActiveCells   = 0x0083,      //Write: Starts balancing certain cells, 0x0000 to turn off. Read: Which cells are balancing
        kCBSetLevel,                  //Cell balancing levels
        kOTPCheck        = 0x00A0,
        kOTPWrite        = 0x00A1
    };

    //voltage measurements are direct commands
    enum class VoltageMeasurements:uint8_t
    {
        kCellVoltage1   = 0x14,
        kCellVoltage2   = 0x16,
        kCellVoltage3   = 0x18,
        kCellVoltage4   = 0x1A,
        kCellVoltage5   = 0x1C,
        kCellVoltage6   = 0x1E,
        kCellVoltage7   = 0x20,
        kCellVoltage8   = 0x22,
        kCellVoltage9   = 0x24,
        kCellVoltage10  = 0x26,
        kCellVoltage11  = 0x28,
        kCellVoltage12  = 0x2A,
        kCellVoltage13  = 0x2C,
        kCellVoltage14  = 0x2E,
        kCellVoltage15  = 0x30,
        kCellVoltage16  = 0x32,
    };

    //OTP data registers
    enum class DataMemory:uint16_t
    {
        kFETCtrl              = 0x0097,
        kCellGain1            = 0x9180,
        kCellGain2            = 0x9182,
        kCellGain3            = 0x9184,
        kCellGain4            = 0x9186,
        kCellGain5            = 0x9188,
        kCellGain6            = 0x918A,
        kCellGain7            = 0x918C,
        kCellGain8            = 0x918E,
        kCellGain9            = 0x9190,
        kCellGain10           = 0x9192,
        kCellGain11           = 0x9194,
        kCellGain12           = 0x9196,
        kCellGain13           = 0x9198,
        kCellGain14           = 0x919A,
        kCellGain15           = 0x919C,
        kCellGain16           = 0x919E,
        kPackGain             = 0x91A0,
        kTOSGain              = 0x91A2,
        kCCGain               = 0x91A8,
        kCapacityGain         = 0x91AC,
        kCellOffset           = 0x91B0,
        kCUVThreshOverride    = 0x91D4,
        kCOVThreshOverride    = 0x91D6,
        kFuseMinBlowVolts     = 0x9231,
        kFuseBlowTimeout      = 0x9233,
        kPowerConfig          = 0x9234,
        kReg12Config          = 0x9236,
        kReg0Config           = 0x9237,
        kCommType             = 0x9239,
        kSPIConfig            = 0x923C,
        kCommIdleTime         = 0x923D,
        kProtectionConfig     = 0x925F,
        kEnableProtA          = 0x9261,
        kEnableProtB          = 0x9262,
        kEnableProtC          = 0x9263,
        kCHGFetProtA          = 0x9265,
        kCHGFetProtB          = 0x9266,
        kCHGFetProtC          = 0x9267,
        kDSGFetProtA          = 0x9269,
        kDSGFetProtB          = 0x926A,
        kDSGFetProtC          = 0x926B,
        kCUVThreshold         = 0x9275,
        kCUVDelay             = 0x9276,
        kCUVRecovHysteresis   = 0x927B,
        kCOVThreshold         = 0x9278,
        kCOVDelay             = 0x9279,
        kCOVRecovHysteresis   = 0x927C,
        kCOVLatchLimit        = 0x927D,
        kCOVLatchCountDelay   = 0x927E,
        kCOVLatchRecovTime    = 0x927F,
        kSCDThreshold         = 0x9286,
        kSCDDelay             = 0x9287,
        kSCDRecovTime         = 0x9294,
        kSCDLatchLimit        = 0x9295,
        kSCDLatchCountDelay   = 0x9296,
        kSCDLatchRecovTime    = 0x9297,
        kSCDLatchRecovThresh  = 0x9298,
        kCFETConfig           = 0x92FA,
        kDFETConfig           = 0x92FB,
        kAlertConfig          = 0x92FC,
        kTS1                  = 0x92FD,
        kTS2                  = 0x92FE,
        kTS3                  = 0x92FF,
        kDCHGConfig           = 0x9301,
        kDDSGConfig           = 0x9302,
        kDAConfig             = 0x9303,
        kVcellMode            = 0x9304,
        kCC3Samples           = 0x9307,
        kFETOptions           = 0x9308,   //1 byte
        kChargePumpCtrl       = 0x9309,   //1 byte
        kPreDischargeTimeout  = 0x930E,   //1 byte
        kPreDchgStopDelta     = 0x930F,
        kSFAlertMaskA         = 0x926F,
        kSFAlertMaskB         = 0x9270,
        kSFAlertMaskC         = 0x9271,
        kBodyDiodeThresh      = 0x9273,
        kPFAlertMaskA         = 0x92C4,
        kPFAlertMaskB         = 0x92C5,
        kPFAlertMaskC         = 0x92C6,
        kPFAlertMaskD         = 0x92C7,
        kDsgCurrentThresh     = 0x9310,
        kChgCurrentThresh     = 0x9312,
        kCellOpenWire         = 0x9314,
        KManufacturingInit    = 0x9343,
        kBalancingConfig      = 0x9335,
        kMinCellTemp          = 0x9336,
        kMaxCellTemp          = 0x9337,
        kMaxInternalTemp      = 0x9338,
        kCellBalanceInterval  = 0x9339,
        kMaxCellsBalance      = 0x933A,
        kMinCellBalanceVolts  = 0x933B,
        kCellBalanceMinDelta  = 0x933D,
        kCellBalanceStopDelta = 0x933E,
        kRelaxBalanceMin      = 0x933F,
        kRelaxBalanceDelta    = 0x9341,
        kRelaxBalanceStop     = 0x9342,
    };

    /*
        Cell balancing notes: 
        13.3.4.1 Default Alarm Mask Register bit 2 is set when balancing is active
        13.3.11.1 Cell balancing config register bits 0-4 set cell balancing config options
        13.3.11.2-12 More cell balancing options like temp and voltage
        10.1 Host controlled cell balancing subcommands: 
            0x0083 CB_ACTIVE_CELLS() -> when written, starts balancing on specified cells (write 0x0000 to turn off)
                                     -> when read, reports bit mask of cells actively being balanced
            0x0084 CB_SET_LEVEL()    -> cell voltage threshold in mV

    */

    BQ76952(SPIInterface *spi_interface);
    ~BQ76952();

    int16_t raw_voltage_array[16];

    //send a direct command and read back response if there is one
    uint8_t GetSingleByteStatus(DirectCommands direct_command);
    //send a direct command and read back response if there is one - multiple byte edition
    uint16_t GetMultiByteStatus(DirectCommands direct_command);

    int16_t GetSignedStatus(DirectCommands direct_command);
    //get a single cell voltage
    int16_t GetCellVoltage(uint8_t cell_number);

    int16_t GetCellVoltageI2C(uint8_t cell_number);
    //get all cell voltages and pack into array
    void GetAllVoltages(bool i2c_comms = true);

    void GetCellGroupA();
    void GetCellGroupB();
    void GetCellGroupC();
    void GetCellGroupD();
    //overvoltage trigger
    void SetOverVoltageProtection(uint32_t mv_value, uint32_t ms_delay);
    //undervoltage trigger
    void SetUnderVoltageProtection(uint32_t mv_value, uint32_t ms_delay);
    //charging current trigger
    void SetChargingCurrentProtection(uint8_t val, uint8_t delay);
    //discharging current trigger
    void SetDischargingCurrentProtection(uint8_t val, uint8_t delay);
    //enter config update mode to make changes to OTP
    void EnterConfigMode(bool i2c_comms = true);
    //exit config update mode after changes to OTP are complete
    void ExitConfigMode(bool i2c_comms = true);
    //get data from data subcommand
    void ReadSubcommand(DataSubCommands data_subcommand, uint8_t num_bytes);
    // send subcommand
    void SendSubcommand(SubCommands subcommand);

    void SendSubcommandI2C(SubCommands subcommand);

    void ReadSubcommandI2C(DataSubCommands subcommand, uint8_t num_bytes);
    //write to RAM OTP to change device settings - can be done on manufacturing line
    void WriteDataMemory(DataMemory data_memory, uint8_t num_bytes, uint16_t data);

    //special case that sends 4 bytes
    void WriteCCGain(DataMemory data_memory, uint32_t data);

    void ReadDataMemory(DataMemory data_memory, uint8_t num_bytes);

    void WriteDataMemoryI2C(DataMemory data_memory, uint8_t num_bytes, bool read_only = false, uint16_t data = 0);
    //Initialize desired OTP settings
    void DeviceConfigure();
    //If device is in sleep mode, interal HFO may not be running. HFO wakes on falling edge of CS
    //In normal or sleep mode, HFO takes about 135us to stabalize before SPI comms is available
    //In deepsleep mode, it may take about 4.5ms :horror: // 4us for an 8 bit SPI transaction (at 2MHz)
    void WakeupOScillator(uint32_t delay_us);

    void SPIConfigure();

    void SwapToSPI();

    int16_t GetCurrent();

    void GetDataMemoryI2C(DataMemory data_memory);

//    int16_t GetLDVoltage(VoltageMeasurements voltage);
//
//    int16_t GetStackVoltage(VoltageMeasurements voltage);
//
//    int16_t GetPackVoltage(VoltageMeasurements voltage);

    uint16_t GetDeviceId(bool i2c_comms = true);

    uint16_t GetManufactureData();

    int16_t GetCellOffset();

    int16_t GetCellGain();

    int16_t GetDataMemory(DataMemory data_memory);
    //sets up crc8 table
    void CalculateCRC8Table();

    uint8_t CheckOTP();

    uint8_t WriteOTP();

    uint16_t SendDirectCommandI2C(DirectCommands direct_command, uint8_t num_bytes, uint16_t data = 0, bool read_only = true);

    void SendDirectCommand(DirectCommands direct_command, uint8_t num_bytes, uint16_t data = 0, bool read_only = true);


private:
    static constexpr uint8_t kDeviceAddressW    {0x08};  //I2C device address
    static constexpr uint8_t kSubcommandLow     {0x3E};  //Subcommand lower byte
    static constexpr uint8_t kSubcommandHigh    {0x3F};  //Subcommand upper byte
    static constexpr uint8_t kCRCAddress        {0x60};  //checksum address
    static constexpr uint8_t kDataLength        {0x61};  //data length address
    static constexpr uint8_t kSubcommandStart   {0x40};  //Start of subcommand buffer data (32 bytes)
    static constexpr uint8_t kNumCells          {16};    //16s-2p for Vita - change as needed+
    static constexpr uint8_t kWriteBit          {0x80};  //set SPI to Write mode (3E becomes BE when writing)

    //primary means of communication
    SPIInterface *spi;

    I2CInterface *i2c;


    uint32_t tx_data_array[10];
    uint32_t rx_data_array[10];
    uint8_t checksum_array[10];
    uint32_t subcommand_data_array[32];     //32 byte subcommand response
    uint8_t subcommand_data_array_i2c[32];
    uint8_t data_memory_array[4];
    uint8_t status_array[2];
    uint8_t crc_table_array[256];

    uint8_t CalculateChecksum(uint8_t *data, uint8_t data_length);

    //calculates and returns the crc8 value
    uint8_t ComputeCRC8(uint32_t *data, uint8_t data_length);
    //send a direct command



    //returns appropriate command for each cell number - called in GetCellVoltages
    static inline uint8_t GetCellNumber(uint8_t cell_number) { return (static_cast<uint8_t>(VoltageMeasurements::kCellVoltage1) + (cell_number * 2)); }
};


#endif //_BQ76952_H_
