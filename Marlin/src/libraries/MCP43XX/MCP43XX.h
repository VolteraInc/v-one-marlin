#ifndef MCP43XX_h
#define MCP43XX_h

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "Wprogram.h"
#endif

#define SPI_FREQ 1000000

#define ADDRESS_WIPER_0     B00000000
#define ADDRESS_WIPER_1     B00010000
#define ADDRESS_NV_WIPER_0  B00100000
#define ADDRESS_NV_WIPER_1  B00110000
#define ADDRESS_TCON0       B01000000

#define ADDRESS_WIPER_2     B01100000
#define ADDRESS_WIPER_3     B01110000
#define ADDRESS_NV_WIPER_2  B10000000
#define ADDRESS_NV_WIPER_3  B10010000
#define ADDRESS_TCON1       B10100000

#define ADDRESS_STATUS      B01010000

#define COMMAND_WRITE       B00000000
#define COMMAND_INCREMENT   B00000100
#define COMMAND_DECREMENT   B00001000
#define COMMAND_READ        B00001100

#define DUMMY_DATA B11111111

#define BITMASK_READ_DATA_REGISTER 0X01FF //B0000000111111111

#define BITMASK_POT0_STARTUP B00001000
#define BITMASK_POT1_STARTUP B10000000
#define BITMASK_POT2_STARTUP B00001000
#define BITMASK_POT3_STARTUP B10000000

#define BITMASK_POT0_B_TERMINAL_CONNECT B00000001
#define BITMASK_POT1_B_TERMINAL_CONNECT B00010000
#define BITMASK_POT2_B_TERMINAL_CONNECT B00000001
#define BITMASK_POT3_B_TERMINAL_CONNECT B00010000

#define BITMASK_POT0_WIPER_TERMINAL_CONNECT B00000010
#define BITMASK_POT1_WIPER_TERMINAL_CONNECT B00100000
#define BITMASK_POT2_WIPER_TERMINAL_CONNECT B00000010
#define BITMASK_POT3_WIPER_TERMINAL_CONNECT B00100000

#define BITMASK_POT0_A_TERMINAL_CONNECT B00000100
#define BITMASK_POT1_A_TERMINAL_CONNECT B01000000
#define BITMASK_POT2_A_TERMINAL_CONNECT B00000100
#define BITMASK_POT3_A_TERMINAL_CONNECT B01000000

class MCP43XX
{
public:
  // Constructor
  MCP43XX(uint8_t CSPin, uint8_t potResistance);

  void WiperIncrement(uint8_t potNum);
  void WiperDecrement(uint8_t potNum);
  
  void SetWiperPosition(uint8_t potNum, uint8_t value); // Need confirmation if it is working 
  void SetWiperMin();
  void SetWiperMin(uint8_t potNum);
  void SetWiperMax();
  void SetWiperMax(uint8_t potNum);
  void SetWiperMid();
  void SetWiperMid(uint8_t potNum);

  void SaveNVWiperPosition();
  void SaveNVWiperPosition(uint8_t potNum);

  uint16_t ReadWiperPosition(uint8_t potNum);

  void Startup(uint8_t potNum);
  void Shutdown(uint8_t potNum);

  void TerminalBConnect(uint8_t potNum);
  void TerminalBDisconnect(uint8_t potNum);
  void TerminalAConnect(uint8_t potNum);
  void TerminalADisconnect(uint8_t potNum);
  void WiperConnect(uint8_t potNum);
  void WiperDisconnect(uint8_t potNum);

private:
  uint8_t _wiper0Position, _wiper1Position, _wiper2Position, _wiper3Position;
  uint8_t _CSPin;
  uint8_t _potResistance;
  
  void InitTcon();
  void WriteTcon0Register(uint16_t value);
  void WriteTcon1Register(uint16_t value);
  uint16_t ReadTconRegister();
  uint16_t ReadStatusRegister();

  uint8_t ResistanceToPosition(float resistance);
  float PositionToResistance(uint8_t position);

};

#endif
