#pragma once

#include "macros.h"
#include "MarlinSerial.h"

#define MYSERIAL customizedSerial

#define SERIAL_EOL MYSERIAL.print("\n")
#define SERIAL_PAIR(name,value) do{ serialprintPGM(PSTR(name)); MYSERIAL.print(value); } while(0)
#define SERIAL_PAIR_F(name,value,precision) do{ serialprintPGM(PSTR(name)); MYSERIAL.print(value, precision); } while(0)

#define SERIAL_PROTOCOL(x) (MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) (MYSERIAL.print((x),(y)))
#define SERIAL_PROTOCOLPGM(x) (serialprintPGM(PSTR(x)))
#define SERIAL_PROTOCOLLN(x)    do{ MYSERIAL.print(x); SERIAL_EOL; }while(0)
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(PSTR(x "\n")))

extern const char echomagic[] PROGMEM;
extern const char errormagic[] PROGMEM;

#define SERIAL_ERROR_START (serialprintPGM(errormagic))
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START (serialprintPGM(echomagic))
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)
#define SERIAL_ECHO_F(x,y) SERIAL_PROTOCOL_F((x),(y))

#define SERIAL_ECHOPAIR SERIAL_PAIR
//TODO: replace all SERIAL_ECHOPAIR with SERIAL_PAIR

// For serial printing from PROGMEM. (Saves loads of SRAM.)
FORCE_INLINE void serialprintPGM(const char* str) {
  while (char ch = pgm_read_byte(str++)) MYSERIAL.write(ch);
}

template <typename T>
void serialArray(const T A[], size_t size) {
  for (auto i = 0u; i < size; ++i) {
    if (i != 0) {
      MYSERIAL.print(',');
    }
    MYSERIAL.print(A[i]);
  }
}

extern bool logging_enabled;
