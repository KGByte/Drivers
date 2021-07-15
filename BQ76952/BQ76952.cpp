#include <Drivers/BQ76952.h>

BQ76952::BQ76952(SPIInterface *spi_interface) :
        spi { spi_interface }
{
}

BQ76952::~BQ76952()
{
}

//send a direct command and read back response if there is one
//TODO turn this into a more direct function like GetStatusA or GetBatteryStatus
//same with Multi Byte Status
uint8_t BQ76952::GetSingleByteStatus(DirectCommands direct_command)
{
    uint8_t status;
    SendDirectCommand(direct_command, 1);
    status = status_array[0];
    return status;
}

//send a direct command and read back response if there is one - multiple byte edition
uint16_t BQ76952::GetMultiByteStatus(DirectCommands direct_command)
{
    uint16_t status;
    SendDirectCommand(direct_command, 2);
    status = (status_array[1] << 8) | status_array[0];
    return status;
}

int16_t BQ76952::GetSignedStatus(DirectCommands direct_command)
{
    uint16_t status;
    SendDirectCommand(direct_command, 2);
    status = (status_array[1] << 8) | status_array[0];
    return status;
}

//TODO don't be lazy and re-do this shit
uint16_t BQ76952::SendDirectCommandI2C(DirectCommands direct_command,
                                   uint8_t num_bytes, uint16_t data,
                                   bool read_only)
{
    uint8_t cmd = static_cast<uint8_t>(direct_command);
    uint8_t rx_array[3];
    uint16_t val;

    i2c->Read(rx_array, num_bytes, kDeviceAddressW, cmd);

    delayUs(250);
//    i2c->I2CSingleWrite(cell, kDeviceAddressW);
//
//    i2c->ReadOnly(rx_voltage_array, 2, kDeviceAddressR);

    val = static_cast<int16_t>(rx_array[1] << 8
            | rx_array[0]);
    return val;
}
void BQ76952::SendDirectCommand(DirectCommands direct_command,
                                uint8_t num_bytes, uint16_t data,
                                bool read_only)
{
    uint8_t msb = (data >> 8) & 0xFF;
    uint8_t lsb = (data & 0xFF);
    uint8_t cmd = 0;
    cmd = static_cast<uint8_t>(direct_command);
    bool success = false; //this flag is set when the data read back matches what was sent

    if (read_only) //most direct commands are read only and we write no data
    {
        for (uint8_t index = 0; index < num_bytes; index++)
        {
            tx_data_array[0] = cmd;
            tx_data_array[1] = 0xFF; //Write garbage cus we are reading
            tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

            while (!success)
            {
                GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
                spi->Transfer(tx_data_array, rx_data_array, 3);
                GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
                delayUs(55);
                if (rx_data_array[0] == cmd)
                {
                    success = true;
                    //TODO Add logic for checking CRC
                    status_array[index] = rx_data_array[1]; //index 1 is where data should  be inside rx buffer
                }
            }

            cmd++; //increment command for next transaction
            success = false;
        }
    }
    else //this is not a read only transaction (writing to Alarm status or Alarm enable)w
    {
        for (uint8_t index = 0; index < num_bytes; index++) //writing 2 bytes of data to one of these registers
        {
            if (index == 0) //first pass, we have to send lsb and checksum on address + lsb
            {
                tx_data_array[0] = cmd | kWriteBit;
                tx_data_array[1] = lsb;
            }
            else //second pass, we send msb and crc on address + msb as well as incremented address for the command
            {
                tx_data_array[0] = cmd | kWriteBit;
                tx_data_array[1] = msb;
            }
            tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

            while (!success)
            {
                GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
                spi->Transfer(tx_data_array, rx_data_array, 3);
                GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
                delayUs(55);
                if (rx_data_array[0] == cmd)
                {
                    success = true;
                    //TODO Add logic for checking CRC
                    //not getting any data
                }
            }

            cmd++; //increment command for next transaction
            success = false;
        }
    }
}
//get a single cell voltage
int16_t BQ76952::GetCellVoltage(uint8_t cell_number)
{
    bool success = false;
    uint32_t tx_voltage_array[3];
    uint8_t rx_voltage_array[2];
    uint8_t cell = GetCellNumber(cell_number);
    int16_t voltage;

    for (uint8_t index = 0; index < 2; index++) //writing 2 bytes of data to one of these registers
    {
        tx_voltage_array[0] = cell;
        tx_voltage_array[1] = 0xFF; //not sending anything when reading voltages
        //TODO compute time for this function
        tx_voltage_array[2] = ComputeCRC8(tx_voltage_array, 2);

        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
        spi->Transfer(tx_voltage_array, rx_data_array, 3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        delayUs(55);
        while (success == false)   //! or plain?
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            spi->Transfer(tx_voltage_array, rx_data_array, 3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delayUs(50);
            if (rx_data_array[0] == cell)
            {
                success = true;
                //TODO Add logic for checking CRC
                rx_voltage_array[index] = rx_data_array[1];
            }
        }

        success = false;
        cell++;
    }
    voltage = static_cast<int16_t>(rx_voltage_array[1] << 8
            | rx_voltage_array[0]);
    return voltage;
}

int16_t BQ76952::GetCellVoltageI2C(uint8_t cell_number)
{
    uint8_t rx_voltage_array[3];
    uint8_t cell = GetCellNumber(cell_number);
    int16_t voltage;

    i2c->Read(rx_voltage_array, 2, kDeviceAddressW, cell);
//    i2c->I2CSingleWrite(cell, kDeviceAddressW);
//
//    i2c->ReadOnly(rx_voltage_array, 2, kDeviceAddressR);

    voltage = static_cast<int16_t>(rx_voltage_array[1] << 8
            | rx_voltage_array[0]);
    return voltage;
}

//get all cell voltages and pack into array
void BQ76952::GetAllVoltages(bool i2c_comms)  //default false
{
    if (i2c_comms)
    {
        for (uint8_t index = 0; index < kNumCells; index++)
        {
            raw_voltage_array[index] = GetCellVoltageI2C(index);
        }
    }
    else
    {
        for (uint8_t index = 0; index < kNumCells; index++)
        {
            raw_voltage_array[index] = GetCellVoltage(index);
        }
    }
}

void BQ76952::GetCellGroupA()
{
    for (uint8_t index = 0; index < 4; index++)
    {
        raw_voltage_array[index] = GetCellVoltage(index);
    }
}

void BQ76952::GetCellGroupB()
{
    for (uint8_t index = 0; index < 4; index++)
    {
        raw_voltage_array[index + 4] = GetCellVoltage(index);
    }
}
void BQ76952::GetCellGroupC()
{
    for (uint8_t index = 0; index < 4; index++)
    {
        raw_voltage_array[index + 8] = GetCellVoltage(index);
    }
}
void BQ76952::GetCellGroupD()
{
    for (uint8_t index = 0; index < 4; index++)
    {
        raw_voltage_array[index + 12] = GetCellVoltage(index);
    }
}


int16_t BQ76952::GetCurrent()
{
    int16_t current = 0;
    SendDirectCommand(DirectCommands::kCC2Current, 2);
    current = (status_array[1] << 8) | status_array[0];
    return current;
}
//overvoltage trigger
void BQ76952::SetOverVoltageProtection(uint32_t mv_value, uint32_t ms_delay)
{
}
//undervoltage trigger
void BQ76952::SetUnderVoltageProtection(uint32_t mv_value, uint32_t ms_delay)
{
}
//charging current trigger
void BQ76952::SetChargingCurrentProtection(uint8_t val, uint8_t delay)
{
}
//discharging current trigger
void BQ76952::SetDischargingCurrentProtection(uint8_t val, uint8_t delay)
{
}
//enter config update mode to make changes to OTP
void BQ76952::EnterConfigMode(bool i2c_comms)
{
    if (i2c_comms)
    {
        SendSubcommandI2C(SubCommands::kEnterConfigUpdate);
    }
    else
    {
        SendSubcommand(SubCommands::kEnterConfigUpdate);
    }

}
//exit config update mode after changes to OTP are complete
void BQ76952::ExitConfigMode(bool i2c_comms)
{
    if (i2c_comms)
    {
        SendSubcommandI2C(SubCommands::kExitConfigUpdate);
    }
    else
    {
        SendSubcommand(SubCommands::kExitConfigUpdate);
    }

}
//get data from data subcommand
void BQ76952::ReadSubcommand(DataSubCommands data_subcommand, uint8_t num_bytes)
{
    uint16_t subcmd = static_cast<uint16_t>(data_subcommand);
    uint8_t msb = (subcmd >> 8) & 0xFF;
    uint8_t lsb = (subcmd & 0xFF);
    uint8_t read_address = 0x3E; //used when reading back data on MISO line
    bool success = false;
    uint8_t data_length = 0;
    uint8_t start_address = kSubcommandStart;

    for (uint8_t index = 0; index < 2; index++)
    {
        if (index == 0)
        {
            tx_data_array[0] = kWriteBit | kSubcommandLow;
            tx_data_array[1] = lsb;
        }
        else
        {
            tx_data_array[0] = kWriteBit | kSubcommandHigh;
            tx_data_array[1] = msb;
        }
        tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

//        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
//        spi->Transfer(tx_data_array, rx_data_array, 3);
//        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        //delayUs(600);
        while (!success)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            spi->Transfer(tx_data_array, rx_data_array, 3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delayUs(600);
            if (rx_data_array[0] == read_address | kWriteBit) //checking for
            {
                success = true;
                //TODO Add logic for checking CRC
                //not getting any data
            }
        }
        read_address++;
        success = false;
    }
    //3. Read the length of the response from data length address
    tx_data_array[0] = kDataLength;
    tx_data_array[1] = 0xFF;
    tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

    while (!success)
    {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
        spi->Transfer(tx_data_array, rx_data_array, 3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        delayUs(600);
        if (rx_data_array[0] == kDataLength) //checking for
        {
            success = true;
            //TODO Add logic for checking CRC
            data_length = rx_data_array[1]; //length of data is in the second spot in the receive buffer
        }
    }

    success = false;
    //4. Loop for length of response data and read response into array starting from Subcommand start address
    //   Send dummy bytes when reading (0xFF is fine)

    for (uint8_t length = 0; length < data_length; length++)
    {
        tx_data_array[0] = start_address;
        tx_data_array[1] = 0xFF;
        tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

        //read response starting at 0x40
        while (!success)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            spi->Transfer(tx_data_array, rx_data_array, 3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delayUs(600);
            if (rx_data_array[0] == start_address) //checking for
            {
                success = true;
                //TODO Add logic for checking CRC
                subcommand_data_array[length] = rx_data_array[1]; //subcommand response is at index 1 of receive buffer
            }
        }
        //5. Increment Subcommand start address and keep reading
        //   Responses should always be in index 1 of receieve buffer
        start_address++;
        success = false;
    }
}

//subcommand without a response (enter config mode, etc)
void BQ76952::SendSubcommand(SubCommands subcommand)
{
    uint16_t subcmd = static_cast<uint16_t>(subcommand);
    uint8_t msb = (subcmd >> 8) & 0xFF;
    uint8_t lsb = (subcmd & 0xFF);
    uint8_t read_address = 0x3E; //used when reading back data on MISO line

    bool success = false;

    for (uint8_t index = 0; index < 2; index++)
    {
        if (index == 0)
        {
            tx_data_array[0] = kWriteBit | kSubcommandLow;
            tx_data_array[1] = lsb;
        }
        else
        {
            tx_data_array[0] = kWriteBit | kSubcommandHigh;
            tx_data_array[1] = msb;
        }
        tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
        spi->Transfer(tx_data_array, rx_data_array, 3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

        while (!success)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            spi->Transfer(tx_data_array, rx_data_array, 3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delayUs(600);
            if (rx_data_array[0] == (read_address | kWriteBit)) //checking for
            {
                success = true;
                //TODO Add logic for checking CRC
                //not getting any data
            }
        }

        read_address++;
        success = false;
    }
}


//Used to enter config update mode
void BQ76952::SendSubcommandI2C(SubCommands subcommand)
{
    uint8_t local_array[4];
    uint16_t subcmd = static_cast<uint16_t>(subcommand);
    uint8_t msb = (subcmd >> 8) & 0xFF;
    uint8_t lsb = (subcmd & 0xFF);

    local_array[0] = kSubcommandLow;
    local_array[1] = lsb;
    local_array[2] = msb;

    i2c->Write(local_array, 3, kDeviceAddressW);
}

//num_bytes is how many bytes we want to read - determined by the subcommand
void BQ76952::ReadSubcommandI2C(DataSubCommands subcommand, uint8_t num_bytes)
{
    uint8_t local_array[4];
    uint16_t subcmd = static_cast<uint16_t>(subcommand);
    uint8_t msb = (subcmd >> 8) & 0xFF;
    uint8_t lsb = (subcmd & 0xFF);

    local_array[0] = kSubcommandLow;
    local_array[1] = lsb;
    local_array[2] = msb;

    i2c->Write(local_array, 3, kDeviceAddressW);

    delayUs(600);
    i2c->Read(subcommand_data_array_i2c, num_bytes, kDeviceAddressW, kSubcommandStart);
}

uint16_t BQ76952::GetDeviceId(bool i2c_comms)
{
    uint16_t device_id = 0;

    if (i2c_comms)
    {
        ReadSubcommandI2C(DataSubCommands::kDeviceID, 2);

        device_id = subcommand_data_array_i2c[1] << 8 | subcommand_data_array_i2c[0];
    }
    else
    {
        ReadSubcommand(DataSubCommands::kDeviceID, 2);

        device_id = subcommand_data_array[1] << 8 | subcommand_data_array[0];
    }

    return device_id;
}

uint16_t BQ76952::GetManufactureData()
{
    uint16_t data = 0;

    ReadSubcommand(DataSubCommands::kManuStatus, 2);

    data = subcommand_data_array[1] << 8 | subcommand_data_array[0];

    return data;
}
//write to RAM OTP to change device settings - can be done on manufacturing line
//similar to subcommands except actually writing data
//num_bytes represents the number of bytes we are writing to the data_memory register
//some registers are 1 byte writes and some are 2 bytes
//this might look really weird and bad but trust me it's how it's done
//see software manual for BQ76952
void BQ76952::WriteDataMemory(DataMemory data_memory, uint8_t num_bytes, uint16_t data)
{
    uint16_t datacmd = static_cast<uint16_t>(data_memory);
    uint8_t msb_cmd = (datacmd >> 8) & 0xFF;
    uint8_t lsb_cmd = (datacmd & 0xFF);
    uint8_t msb_data = (data >> 8) & 0xFF;
    uint8_t lsb_data = (data & 0xFF);
    uint8_t read_address = kSubcommandLow; //used when reading back data on MISO line
    //uint8_t data_length = 0;
    uint8_t start_address = kSubcommandStart;
    uint8_t checksum = 0;
    checksum_array[0] = msb_cmd;
    checksum_array[1] = lsb_cmd;
    checksum_array[2] = lsb_data;
    checksum_array[3] = msb_data;

    bool success = false;

    //writing the data memory address to subcommand low and subcommand high
    for (uint8_t index = 0; index < 2; index++)
    {
        tx_data_array[0] = kWriteBit | read_address;

        if (index == 0)
        {
            tx_data_array[1] = lsb_cmd;
        }
        else
        {
            tx_data_array[1] = msb_cmd;
        }
        tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

        while (!success)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            spi->Transfer(tx_data_array, rx_data_array, 3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delayUs(600);

            if (rx_data_array[0] == read_address | kWriteBit) //checking for
            {
                success = true;
                //not getting any data
            }
        }
        read_address++;
        success = false;
    }
    //TODO add case for num_bytes == 2
    //sending data we are writing to the start data buffer (0x40)

    if (num_bytes == 1)
    {
        tx_data_array[0] = start_address | kWriteBit;
        tx_data_array[1] = lsb_data;
        tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

        //read response starting at 0x40
        while (!success)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            spi->Transfer(tx_data_array, rx_data_array, 3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            delayUs(600);
            if (rx_data_array[0] == start_address | kWriteBit)
            {
                success = true;
                //TODO Add logic for checking CRC
            }
        }
        checksum = CalculateChecksum(checksum_array, 3); //3 byte checksum
    }
    else
    {
        for (uint8_t index = 0; index < 2; index++)
        {
            tx_data_array[0] = kWriteBit | start_address;

            if (index == 0)
            {
                tx_data_array[1] = lsb_data;
            }
            else
            {
                tx_data_array[1] = msb_data;
            }
            tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

            while (!success)
            {
                GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
                spi->Transfer(tx_data_array, rx_data_array, 3);
                GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
                delayUs(600);

                if (rx_data_array[0] == start_address | kWriteBit) //checking for
                {
                    success = true;
                    //not getting any data
                }
            }
            start_address++;
            success = false;
        }
        //TODO
        //case for writing 2 bytes
        checksum = CalculateChecksum(checksum_array, 4); //4 byte checksum
    }
    //reset success
    success = false;

    //write checksum to 0x60
    tx_data_array[0] = kCRCAddress | kWriteBit;
    tx_data_array[1] = checksum;
    tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

//read response starting at 0x40
    while (!success)
    {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
        spi->Transfer(tx_data_array, rx_data_array, 3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        delayUs(600);

        if (rx_data_array[0] == kCRCAddress | kWriteBit)
        {
            success = true;
        }
    }
    //reset success
    success = false;

    //write data length to 0x61 -> data length is command address + subcommand low and high + checksum
    tx_data_array[0] = kDataLength | kWriteBit;
    tx_data_array[1] = 0x05;
    tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

    while (!success)
    {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
        spi->Transfer(tx_data_array, rx_data_array, 3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        delayUs(600);
        if (rx_data_array[0] == kDataLength | kWriteBit) //checking for readback
        {
            success = true;
        }
    }
}
//special case that sends 4 bytes
void BQ76952::WriteCCGain(DataMemory data_memory, uint32_t data)
{
//    uint16_t datacmd = static_cast<uint16_t>(data_memory);
//    uint8_t msb_cmd = (datacmd >> 8) & 0xFF;
//    uint8_t lsb_cmd = (datacmd & 0xFF);
//    uint8_t msb_data = (data >> 8) & 0xFF;
//    uint8_t lsb_data = (data & 0xFF);
//    uint8_t read_address = kSubcommandLow; //used when reading back data on MISO line
//    //uint8_t data_length = 0;
//    uint8_t start_address = kSubcommandStart;
//    uint8_t checksum = 0;
}

//num_bytes refers to the number of bytes we are reading.
//It is specific to each data memory command
//could be 1, 2 or 4
//handled in the specific function
void BQ76952::ReadDataMemory(DataMemory data_memory, uint8_t num_bytes)
{
    uint16_t datacmd = static_cast<uint16_t>(data_memory);
    uint8_t msb = (datacmd >> 8) & 0xFF;
    uint8_t lsb = (datacmd & 0xFF);
    uint8_t read_address = kSubcommandLow;
    uint8_t start_address = kSubcommandStart;
    bool success = false;


    for (uint8_t index = 0; index < 2; index++)
    {
        tx_data_array[0] = read_address | kWriteBit; //3E | 80 = BE, second time 3F | 80 = BF
        //first byte sent is lsb to subcommand low
        //second byte sent is msb to subcommand high
        if (index == 0)
        {
            tx_data_array[1] = lsb;
        }
        else
        {
            tx_data_array[1] = msb;
        }

        tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

        while (!success)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            spi->Transfer(tx_data_array, rx_data_array, 3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            //delayUs(500);

            if (rx_data_array[0] == (read_address | kWriteBit)) //check for readback on MISO
            {
                success = true; //break out of loop and onto next transaction
            }
        }

        read_address++;     //increment address to subcommand high
        success = false;        //reset for next transaction
    }


    //next we want to read the data from 0x40 (data start address)
    //depending on what command it is, select the appropriate number of bytes

    for (uint8_t index = 0; index < num_bytes; index++)
    {
        tx_data_array[0] = start_address;                   //no write bit because reading
        tx_data_array[1] = 0xFF;
        tx_data_array[2] = ComputeCRC8(tx_data_array, 2);

        while(!success)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            spi->Transfer(tx_data_array, rx_data_array, 3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            //delayUs(500);

            if (rx_data_array[0] == start_address)
            {
                data_memory_array[index] = rx_data_array[1];  //extract data and store in array
                success = true;
            }
        }

        //reset success for next byte and increment start address
        success = false;
        start_address++;
    }
}

int16_t BQ76952::GetCellGain()
{
    int16_t gain = 0;
    ReadDataMemory(DataMemory::kCellGain1, 2);
    gain = data_memory_array[1] << 8 | data_memory_array[0];
    return gain;
}

int16_t BQ76952::GetCellOffset()
{
    int16_t offset = 0;
    ReadDataMemory(DataMemory::kCellOffset, 2);
    offset = data_memory_array[1] << 8 | data_memory_array[0];
    return offset;
}

int16_t BQ76952::GetDataMemory(DataMemory data_memory)
{
    int16_t data = 0;
    ReadDataMemory(data_memory, 2);
    data = data_memory_array[1] << 8 | data_memory_array[0];
    return data;
}

void BQ76952::WriteDataMemoryI2C(DataMemory data_memory, uint8_t num_bytes, bool read_only, uint16_t data)
{
    uint8_t local_array[10];
    uint8_t checksum = 0;
    uint16_t datacmd = static_cast<uint16_t>(data_memory);
    uint8_t msb_cmd = (datacmd >> 8) & 0xFF;
    uint8_t lsb_cmd = (datacmd & 0xFF);
    uint8_t data_length = 0;
    uint8_t msb_data = (data >> 8) & 0xFF;
    uint8_t lsb_data = (data & 0xFF);

    if (read_only)                              //used to read configurations such as commm type, enabled protections, etc
    {
        local_array[0] = kSubcommandLow;
        local_array[1] = lsb_cmd;
        local_array[2] = msb_cmd;

        i2c->Write(local_array, 3, kDeviceAddressW);  //write to data memory register that we want to read from
        //no checksum for reading
        delayUs(1000);
        i2c->Read(data_memory_array, 1, kDeviceAddressW, kSubcommandStart); //write to 0x40 where data starts and then read from there
    }
    //this is used to determine how many bytes of data we are writing to 0x61 (kDataLength)
    //if our sending data length is 1, then we write 5 bytes, else we write 6.
    //This is for the write only case. When readng it may be different and will be explained
    else                                        //example: write 0x60 to spi config register 0x923C
    {                                           //num_bytes is 1, only writing 0x60
        if (num_bytes == 1)
        {
            data_length = 0x05;
        }
        else
        {
            data_length = 0x06;
            //extra code for splitting data into msb and lsb
            //TODO comding soon
        }

        local_array[0] = kSubcommandLow;        //0x3E
        local_array[1] = lsb_cmd;               //0x3C
        local_array[2] = msb_cmd;               //0x92
        local_array[3] = lsb_data;

        checksum = CalculateChecksum(&local_array[1], 3);
        i2c->Write(local_array, data_length - 1, kDeviceAddressW);

        local_array[0] = kCRCAddress;
        local_array[1] = checksum;
        local_array[2] = data_length;

        i2c->Write(local_array, data_length - 2, kDeviceAddressW);
    }
}

void BQ76952::GetDataMemoryI2C(DataMemory data_memory)
{
    WriteDataMemoryI2C(data_memory, 0, true);
}
//Initialize desired OTP settings
//Enter config update mode first!!
void BQ76952::DeviceConfigure()
{
    //TS1 - thermistor reference configure as ADCIN
//    WriteDataMemory(DataMemory::kTS1, 1, 0xB3);
//    //TS2 - thermistor A grouping configure as ADCIN
//    WriteDataMemory(DataMemory::kTS2, 1, 0xB3);
//    //TS3 - thermistor B grouping configure as ADCIN
//    WriteDataMemory(DataMemory::kTS3, 1, 0xB3);
    //DFETOFF - configured as BOTHOFF, active high

    //DCHG

    //DDSG

    //ALERT

    //If I2C, add extra config for I2C->SPI swapping

    WriteDataMemoryI2C(DataMemory::kTS1, 1, false, 0xB3);
    delayUs(2000);
    WriteDataMemoryI2C(DataMemory::kTS2, 1, false, 0xB3);   //spi with crc
    delayUs(2000);
    WriteDataMemoryI2C(DataMemory::kTS3, 1, false, 0xB3);
    delayUs(2000);
}

uint8_t BQ76952::CheckOTP()
{
    uint8_t otp_check;
    ReadSubcommandI2C(DataSubCommands::kOTPCheck, 1);
    otp_check = subcommand_data_array_i2c[0];
    return otp_check;
}

uint8_t BQ76952::WriteOTP()
{
    uint8_t write_otp;
    ReadSubcommandI2C(DataSubCommands::kOTPWrite, 1);
    write_otp = subcommand_data_array_i2c[0];
    return write_otp;
}


void BQ76952::SwapToSPI()
{
    //send I2C command to switch to SPI comms
    SendSubcommandI2C(SubCommands::kSwapToSPI);
    delayUs(2000);
}

/*
 * 2. If using SPI or HDQ, enable REG0 and REG1 (the MCU on the EVM is operating at 3.3V logic levels). In
 Data Memory set REG0 Config to 0x1 and set REG1 Config to 0xD (REG1 enabled at 3.3V).
 3. If using SPI mode: In Data Memory configure SPI Configuration to 0x60. This sets the MISO output to the
 REG1 voltage level.
 */
//REG0 address = 0x9237    set to 0x0001
//REG12 address = 0x9236   set to 0x0D
//SPI Comms configure address = 0x923C
void BQ76952::SPIConfigure()
{
    WriteDataMemoryI2C(DataMemory::kCommIdleTime, 1, false, 0xFF);
    delayUs(2000);
    GetDataMemoryI2C(DataMemory::kCommIdleTime);
    WriteDataMemoryI2C(DataMemory::kCommType, 1, false, 0x10);   //spi with crc
    delayUs(500);
    GetDataMemoryI2C(DataMemory::kCommType);
    WriteDataMemoryI2C(DataMemory::kReg0Config, 1, false, 0x01);
    delayUs(2000);
    GetDataMemoryI2C(DataMemory::kReg0Config);
    WriteDataMemoryI2C(DataMemory::kReg12Config, 1, false, 0x0D);
    delayUs(2000);
    GetDataMemoryI2C(DataMemory::kReg12Config);
    WriteDataMemoryI2C(DataMemory::kSPIConfig, 1, false, 0x60);
    delayUs(2000);
    GetDataMemoryI2C(DataMemory::kSPIConfig);
    WriteDataMemoryI2C(DataMemory::kCFETConfig, 1, false, 0x00);  //configure for SPI CS
    delayUs(2000);
    GetDataMemoryI2C(DataMemory::kCFETConfig);
}

uint8_t BQ76952::CalculateChecksum(uint8_t *data, uint8_t data_length)
{
    uint16_t sum = 0;
    uint8_t checksum = 0;
    for (uint8_t index = 0; index < data_length; index++)
    {
        sum += data[index];
    }

    checksum = (~sum) & 0xFF;
    return checksum;
}
//trust me this works
//void BQ76952::CalculateCRC8Table()
//{
//    const uint16_t generator = 0x107; //crc polynomial from datasheet x^8 + x^2 + x + 1 = 1 0000 0111
//
//    for (uint8_t divident = 0; divident < 256; divident++)
//    {
//
//        uint8_t current_byte = divident;
//
//        for (uint8_t bit = 0; bit < 8; bit++)
//        {
//            if ((current_byte & 0x80) != 0) //magic
//            {
//                current_byte <<= 1;        //magic
//                current_byte ^= generator; //magic
//            }
//            else
//            {
//                current_byte <<= 1; //very important magic
//            }
//        }
//        crc_table_array[divident] = current_byte;
//    }
//}

uint8_t BQ76952::ComputeCRC8(uint32_t *ptr, uint8_t len)
{
    unsigned char i;
    unsigned char crc=0;
    while(len--!=0)
    {
        for(i=0x80; i!=0; i/=2)
        {
            if((crc & 0x80) != 0)
            {
                crc *= 2;
                crc ^= 0x107;
            }
            else
                crc *= 2;

            if((*ptr & i)!=0)
                crc ^= 0x107;
        }
        ptr++;
    }
    return(crc);
}


