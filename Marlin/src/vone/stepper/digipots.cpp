#include "digipots.h"

#include <SPI.h>
#include "../../../Configuration.h"
#include "../../../pins.h"

// Refer to http://ww1.microchip.com/downloads/en/DeviceDoc/22242A.pdf
// To get understanding of write/read codes and addresses.

static void s_digiPotWrite(uint8_t address, uint8_t value) {
  address = address + 0x00; // 0x00 adds 'write' code. (no action here...)
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
  SPI.transfer(address); //  send in the address and value via SPI:
  SPI.transfer(value);
  digitalWrite(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
  SPI.endTransaction();
}

static uint8_t s_digiPotRead(uint8_t address) {
  address =  address + 0x0C; // 0x0C adds 'read' code.
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
  SPI.transfer(address);
  uint8_t val = SPI.transfer(0x00); // Send dummy to clock in value.:
  digitalWrite(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
  SPI.endTransaction();
  return val;
}

// Initialize Digipot Motor Current
void digiPotInit() {
  const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;
  SPI.begin();
  pinMode(DIGIPOTSS_PIN, OUTPUT);

  // Cycle the SS pin - make sure digiPot initializes correctly.
  digitalWrite(DIGIPOTSS_PIN,LOW);
  digitalWrite(DIGIPOTSS_PIN,HIGH);

  for (int i = 0; i < 4; ++i) {
    digiPotSetCurrent(i, digipot_motor_current[i]);
  }
}

void digiPotSetCurrent(uint8_t axis, uint8_t current) {
  const uint8_t digipot_addrs[] = DIGIPOT_ADDRESS;
  s_digiPotWrite(digipot_addrs[axis], current);
}

uint8_t digiPotGetCurrent(uint8_t axis) {
  const uint8_t digipot_addrs[] = DIGIPOT_ADDRESS;
  return s_digiPotRead(digipot_addrs[axis]);
}