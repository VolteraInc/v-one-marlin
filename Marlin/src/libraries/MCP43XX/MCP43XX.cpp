#include "MCP43XX.h"
#include <SPI.h>

// Instantiation of object
MCP43XX::MCP43XX(uint8_t CSPin, uint8_t potResistance)
{
    this->_CSPin = CSPin;
    this->_potResistance = potResistance;

    //configure HW
    ::pinMode(_CSPin, OUTPUT);
    ::digitalWrite(_CSPin, HIGH);

    //load current HW state into objecst
    this->_wiper0Position = ReadWiperPosition(0);
    this->_wiper1Position = ReadWiperPosition(1);
    this->_wiper2Position = ReadWiperPosition(2);
    this->_wiper3Position = ReadWiperPosition(3);
}

uint8_t MCP43XX::GetWiperPosition(uint8_t potNum)
{
  return ReadWiperPosition(potNum); //should not use comms in longrun and just expose member
}

void MCP43XX::WiperIncrement(uint8_t potNum)
{
    uint8_t cmdByte = B00000000;
    
    switch (potNum)
    {
    case 0:
        cmdByte = ADDRESS_WIPER_0 | COMMAND_INCREMENT;
        this->_wiper0Position++;
        break;
    
    case 1:
        cmdByte = ADDRESS_WIPER_1 | COMMAND_INCREMENT;
        this->_wiper1Position++;
        break;

    case 2:
        cmdByte = ADDRESS_WIPER_2 | COMMAND_INCREMENT;
        this->_wiper2Position++;
        break;

    case 3:
        cmdByte = ADDRESS_WIPER_3 | COMMAND_INCREMENT;
        this->_wiper3Position++;
        break;
    }

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    SPI.transfer(cmdByte);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();
}

void MCP43XX::WiperDecrement(uint8_t potNum)
{
    uint8_t cmdByte = B00000000;

    switch (potNum)
    {
    case 0:
        cmdByte = ADDRESS_WIPER_0 | COMMAND_DECREMENT;
        this->_wiper0Position--;
        break;
    
    case 1:
        cmdByte = ADDRESS_WIPER_1 | COMMAND_DECREMENT;
        this->_wiper1Position--;
        break;
    
    case 2:
        cmdByte = ADDRESS_WIPER_2 | COMMAND_DECREMENT;
        this->_wiper2Position--;
        break;
    
    case 3:
        cmdByte = ADDRESS_WIPER_3 | COMMAND_DECREMENT;
        this->_wiper3Position--;
        break;
    }

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    SPI.transfer(cmdByte);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();
}

void MCP43XX::SetWiperPosition(uint8_t potNum, uint8_t value)
{
    uint8_t cmdByte = B00000000;
    uint8_t dataByte = value;
 
    switch (potNum)
    {
    case 0:
        cmdByte = ADDRESS_WIPER_0 | COMMAND_WRITE; 
        _wiper0Position = value;
        break;
    
    case 1:
        cmdByte = ADDRESS_WIPER_1 | COMMAND_WRITE;
        _wiper1Position = value;
        break;
    
    case 2:
        cmdByte = ADDRESS_WIPER_2 | COMMAND_WRITE; 
        _wiper2Position = value;
        break;
    
    case 3:
        cmdByte = ADDRESS_WIPER_3 | COMMAND_WRITE;
        _wiper3Position = value;
        break;
    }

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    SPI.transfer(cmdByte);
    SPI.transfer(dataByte);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();
}

void MCP43XX::SetWiperMin(uint8_t potNum)
{
    SetWiperPosition(potNum, 0);
}

void MCP43XX::SetWiperMin()
{
    SetWiperMin(0);
    SetWiperMin(1);
    SetWiperMin(2);
    SetWiperMin(3);
}

void MCP43XX::SetWiperMax(uint8_t potNum)
{
    SetWiperPosition(potNum, 255);
}

void MCP43XX::SetWiperMax()
{
    SetWiperMax(0);
    SetWiperMax(1);
    SetWiperMax(2);
    SetWiperMax(3);
}

void MCP43XX::SetWiperMid(uint8_t potNum)
{
    SetWiperPosition(potNum, 128);
}

void MCP43XX::SetWiperMid()
{
    SetWiperMid(0);
    SetWiperMid(1);
    SetWiperMid(2);
    SetWiperMid(3);
}

void MCP43XX::SaveNVWiperPosition()
{
    SaveNVWiperPosition(0);
    SaveNVWiperPosition(1);
    SaveNVWiperPosition(2);
    SaveNVWiperPosition(3);
}

void MCP43XX::SaveNVWiperPosition(uint8_t potNum)
{
    uint8_t cmdByte = B00000000;
    uint8_t dataByte;
    
    switch (potNum)
    {
    case 0:
        cmdByte = ADDRESS_NV_WIPER_0 | COMMAND_WRITE;
        dataByte = _wiper0Position;
        break;

    case 1:
        cmdByte = ADDRESS_NV_WIPER_1 | COMMAND_WRITE;
        dataByte = _wiper1Position;
        break;

    case 2:
        cmdByte = ADDRESS_NV_WIPER_2 | COMMAND_WRITE;
        dataByte = _wiper2Position;
        break;

    case 3:
        cmdByte = ADDRESS_NV_WIPER_3 | COMMAND_WRITE;
        dataByte = _wiper3Position;
        break;
        
    }

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    SPI.transfer(cmdByte);
    SPI.transfer(dataByte);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();
}

uint16_t MCP43XX::ReadWiperPosition(uint8_t potNum)
{
    uint8_t cmdByte = B00000000;
    uint8_t hByte = B00000000;
    uint8_t lByte = B00000000;

    switch (potNum)
    {
    case 0:
        cmdByte = ADDRESS_WIPER_0 | COMMAND_READ;
        break;
    
    case 1:
        cmdByte = ADDRESS_WIPER_1 | COMMAND_READ;
        break;
    
    case 2:
        cmdByte = ADDRESS_WIPER_2 | COMMAND_READ;
        break;
    
    case 3:
        cmdByte = ADDRESS_WIPER_3 | COMMAND_READ;
        break;
    }

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    hByte = SPI.transfer(cmdByte);
    lByte = SPI.transfer(DUMMY_DATA);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();

    return ((uint16_t)hByte << 8 | (uint16_t)lByte) & BITMASK_READ_DATA_REGISTER;
}

uint16_t MCP43XX::ReadStatusRegister()
{
    uint8_t cmdByte = B00000000;
    uint8_t hByte = B00000000;
    uint8_t lByte = B00000000;
    
    cmdByte = ADDRESS_STATUS | COMMAND_READ;

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    hByte = SPI.transfer(cmdByte);
    lByte = SPI.transfer(DUMMY_DATA);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();

    return ((uint16_t)hByte << 8 | (uint16_t)lByte) & BITMASK_READ_DATA_REGISTER;
}

uint16_t MCP43XX::ReadTconRegister()
{
    uint8_t cmdByte = B00000000;
    uint8_t hByte = B00000000;
    uint8_t lByte = B00000000;

    cmdByte = ADDRESS_TCON0 | COMMAND_READ;

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    hByte = SPI.transfer(cmdByte);
    lByte = SPI.transfer(DUMMY_DATA);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();

    return ((uint16_t)hByte << 8 | (uint16_t)lByte) & BITMASK_READ_DATA_REGISTER;
}

void MCP43XX::WriteTcon0Register(uint16_t value)
{
    uint8_t cmdByte = B00000000;
    uint8_t dataByte = B00000000;
    if (value > 255)
    {
        cmdByte |= B00000001;
    }
    else
    {
        dataByte = (byte)(value & 0X00FF);
    }

    cmdByte = cmdByte | ADDRESS_TCON0 | COMMAND_WRITE;

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    SPI.transfer(cmdByte);
    SPI.transfer(dataByte);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();

}

void MCP43XX::WriteTcon1Register(uint16_t value)
{
    uint8_t cmdByte = B00000000;
    uint8_t dataByte = B00000000;
    if (value > 255)
    {
        cmdByte |= B00000001;
    }
    else
    {
        dataByte = (byte)(value & 0X00FF);
    }
    
    cmdByte = cmdByte | ADDRESS_TCON1 | COMMAND_WRITE;

    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_CSPin, LOW);

    SPI.transfer(cmdByte);
    SPI.transfer(dataByte);
    
    ::digitalWrite(_CSPin, HIGH);
    SPI.endTransaction();

}

void MCP43XX::Startup(uint8_t potNum)
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;


    switch (potNum)
    {
    case 0:
        lByte = lByte | BITMASK_POT0_STARTUP;
        break;
    
    case 1:
        lByte = lByte | BITMASK_POT1_STARTUP;
        break;
    
    case 2:
        lByte = lByte | BITMASK_POT2_STARTUP;
        break;
    
    case 3:
        lByte = lByte | BITMASK_POT3_STARTUP;
        break;
    }

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    if(potNum < 2)
    {
        this->WriteTcon0Register(tconData);
    }
    else
    {
        this->WriteTcon1Register(tconData);
    }
}

void MCP43XX::Shutdown(uint8_t potNum)
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;

    switch (potNum)
    {
    case 0:
        lByte = lByte & ~BITMASK_POT0_STARTUP;
        break;
    
    case 1:
        lByte = lByte & ~BITMASK_POT1_STARTUP;
        break;
    
    case 2:
        lByte = lByte & ~BITMASK_POT2_STARTUP;
        break;
    
    case 3:
        lByte = lByte & ~BITMASK_POT3_STARTUP;
        break;
    }

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    if(potNum < 2)
    {
        this->WriteTcon0Register(tconData);
    }
    else
    {
        this->WriteTcon1Register(tconData);
    }
}

void MCP43XX::TerminalBConnect(uint8_t potNum)
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;

    switch (potNum)
    {
    case 0:
        lByte = lByte | BITMASK_POT0_B_TERMINAL_CONNECT;
        break;
    
    case 1:
        lByte = lByte | BITMASK_POT1_B_TERMINAL_CONNECT;
        break;
    
    case 2:
        lByte = lByte | BITMASK_POT2_B_TERMINAL_CONNECT;
        break;
    
    case 3:
        lByte = lByte | BITMASK_POT3_B_TERMINAL_CONNECT;
        break;
    }

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    if(potNum < 2)
    {
        this->WriteTcon0Register(tconData);
    }
    else
    {
        this->WriteTcon1Register(tconData);
    }
}

void MCP43XX::TerminalBDisconnect(uint8_t potNum)
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;

    switch (potNum)
    {
    case 0:
        lByte = lByte & ~BITMASK_POT0_B_TERMINAL_CONNECT;
        break;
    
    case 1:
        lByte = lByte & ~BITMASK_POT1_B_TERMINAL_CONNECT;
        break;
    
    case 2:
        lByte = lByte & ~BITMASK_POT2_B_TERMINAL_CONNECT;
        break;
    
    case 3:
        lByte = lByte & ~BITMASK_POT3_B_TERMINAL_CONNECT;
        break;
    }

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    if(potNum < 2)
    {
        this->WriteTcon0Register(tconData);
    }
    else
    {
        this->WriteTcon1Register(tconData);
    }
}

void MCP43XX::TerminalAConnect(uint8_t potNum)
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;
    
    switch (potNum)
    {
    case 0:
        lByte = lByte | BITMASK_POT0_A_TERMINAL_CONNECT;
        break;
    
    case 1:
        lByte = lByte | BITMASK_POT1_A_TERMINAL_CONNECT;
        break;
    
    case 2:
        lByte = lByte | BITMASK_POT2_A_TERMINAL_CONNECT;
        break;
    
    case 3:
        lByte = lByte | BITMASK_POT3_A_TERMINAL_CONNECT;
        break;
    }

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    if(potNum < 2)
    {
        this->WriteTcon0Register(tconData);
    }
    else
    {
        this->WriteTcon1Register(tconData);
    }
}

void MCP43XX::TerminalADisconnect(uint8_t potNum)
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;

    switch (potNum)
    {
    case 0:
        lByte = lByte & ~BITMASK_POT0_A_TERMINAL_CONNECT;
        break;
    
    case 1:
        lByte = lByte & ~BITMASK_POT1_A_TERMINAL_CONNECT;
        break;
    
    case 2:
        lByte = lByte & ~BITMASK_POT2_A_TERMINAL_CONNECT;
        break;
    
    case 3:
        lByte = lByte & ~BITMASK_POT3_A_TERMINAL_CONNECT;
        break;
    }

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    if(potNum < 2)
    {
        this->WriteTcon0Register(tconData);
    }
    else
    {
        this->WriteTcon1Register(tconData);
    }
}

void MCP43XX::WiperConnect(uint8_t potNum)
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;

    switch (potNum)
    {
    case 0:
        lByte = lByte | BITMASK_POT0_WIPER_TERMINAL_CONNECT;
        break;
    
    case 1:
        lByte = lByte | BITMASK_POT1_WIPER_TERMINAL_CONNECT;
        break;
    
    case 2:
        lByte = lByte | BITMASK_POT2_WIPER_TERMINAL_CONNECT;
        break;
    
    case 3:
        lByte = lByte | BITMASK_POT3_WIPER_TERMINAL_CONNECT;
        break;
    }

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    if(potNum < 2)
    {
        this->WriteTcon0Register(tconData);
    }
    else
    {
        this->WriteTcon1Register(tconData);
    }
}

void MCP43XX::WiperDisconnect(uint8_t potNum)
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;

    switch (potNum)
    {
    case 0:
        lByte = lByte & ~BITMASK_POT0_WIPER_TERMINAL_CONNECT;
        break;
    
    case 1:
        lByte = lByte & ~BITMASK_POT1_WIPER_TERMINAL_CONNECT;
        break;
    
    case 2:
        lByte = lByte & ~BITMASK_POT2_WIPER_TERMINAL_CONNECT;
        break;
    
    case 3:
        lByte = lByte & ~BITMASK_POT3_WIPER_TERMINAL_CONNECT;
        break;
    }

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    if(potNum < 2)
    {
        this->WriteTcon0Register(tconData);
    }
    else
    {
        this->WriteTcon1Register(tconData);
    }
}

void MCP43XX::InitTcon()
{
    uint16_t tconData = this->ReadTconRegister();
    uint8_t hByte = (uint8_t)tconData >> 8;
    uint8_t lByte = (uint8_t)tconData & 0xff;

    lByte = lByte | DUMMY_DATA;

    tconData = (uint16_t)hByte << 8 | (uint16_t)lByte;
    this->WriteTcon0Register(tconData);
    this->WriteTcon1Register(tconData);
    
}

uint8_t MCP43XX::ResistanceToPosition(float resistance) //ignores wiper resistance
{
    //calculate pot position closest to desired resistance

    if(resistance < 0.0)
    {
        return 0;
    }
    else if(resistance > this->_potResistance)
    {
        return 255;
    }
    else
    {
        return (float)resistance/((float)(this->_potResistance))*256;
    }
}

float MCP43XX::PositionToResistance(uint8_t position) //returns current resistance of pot ignoring wiper resistance
{
    if(position < 0)
    {
        return 0.0;
    }
    else if(position > 255)
    {
        return this->_potResistance;
    }
    else
    {
        return float(this->_potResistance)/256 * position;
    }
}