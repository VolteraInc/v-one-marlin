#pragma once

#include <stdint.h>
#include <SPI.h>
#include "../../../serial.h"

#define SPICS_PIN 				67
#define DATARDY_PIN				62

#define DUMMY_BYTE				0x00

//Command Bytes
#define ADS126X_NOP               0x00
#define ADS126X_RESET             0x06
#define ADS126X_START             0x08
#define ADS126X_STOP              0x0A
#define ADS126X_RDATA             0x12
#define ADS126X_SYOCAL            0x16
#define ADS126X_GANCAL            0x17
#define ADS126X_SFOCAL            0x19
#define ADS126X_RREG              0x20
#define ADS126X_WREG              0x40
#define ADS126X_LOCK			  0xF2
#define ADS126X_UNLOCK			  0xF5

//Register Addresses
#define ADS126X_ID                0x00
#define ADS126X_STATUS            0x01
#define ADS126X_MODE0             0x02
#define ADS126X_MODE1             0x03
#define ADS126X_MODE2             0x04
#define ADS126X_MODE3             0x05
#define ADS126X_REF               0x06
#define ADS126X_OFCAL0            0x07
#define ADS126X_OFCAL1            0x08
#define ADS126X_OFCAL2            0x09
#define ADS126X_FSCAL0            0x0A
#define ADS126X_FSCAL1            0x0B
#define ADS126X_FSCAL2            0x0C
#define ADS126X_IMUX           	  0x0D
#define ADS126X_IMAG              0x0E
#define ADS126X_PGA               0x10
#define ADS126X_INPMUX            0x11
#define ADS126X_INPBIAS           0x12

//Masking Values

//Write Values

// MODE0

//  Data rate
#define ADS126X_DR_2_5 			0b00000000
#define ADS126X_DR_5 			0b00001000
#define ADS126X_DR_10			0b00010000
#define ADS126X_DR_16_6 		0b00011000
#define ADS126X_DR_20			0b00100000
#define ADS126X_DR_50			0b00101000
#define ADS126X_DR_60			0b00110000
#define ADS126X_DR_100			0b00111000
#define ADS126X_DR_400			0b01000000
#define ADS126X_DR_1200			0b01001000
#define ADS126X_DR_2400			0b01010000
#define ADS126X_DR_4800			0b01011000
#define ADS126X_DR_7200			0b01100000
#define ADS126X_DR_14400		0b01101000
#define ADS126X_DR_19200		0b01110000
#define ADS126X_DR_25600		0b01111000
#define ADS126X_DR_40000		0b10000000

//  Digital Filter
#define ADS126X_FILT_SINC1		0b00000000
#define ADS126X_FILT_SINC2		0b00000001
#define ADS126X_FILT_SINC3		0b00000010
#define ADS126X_FILT_SINC4		0b00000011
#define ADS126X_FILT_FIR		0b00000100

// MODE1

//  Chop and AC-Excitation Modes
#define ADS126X_CHOP_NORM		0b00000000
#define ADS126X_CHOP			0b00100000
#define ADS126X_2W_EX			0b01000000
#define ADS126X_4W_EX			0b01100000

//  ADC Conversion Mode
#define ADS126X_ADC_CONT		0b00000000
#define ADS126X_ADC_ONESHOT		0b00010000

//  Delay (in microseconds)
#define ADS126X_DELAY_0			0b00000000
#define ADS126X_DELAY_50		0b00000001
#define ADS126X_DELAY_59		0b00000010
#define ADS126X_DELAY_67		0b00000011
#define ADS126X_DELAY_85		0b00000100
#define ADS126X_DELAY_119		0b00000101
#define ADS126X_DELAY_189		0b00000110
#define ADS126X_DELAY_328		0b00000111
#define ADS126X_DELAY_605		0b00001000
#define ADS126X_DELAY_1160		0b00001001
#define ADS126X_DELAY_2270		0b00001010
#define ADS126X_DELAY_4490		0b00001011
#define ADS126X_DELAY_8930		0b00001100
#define ADS126X_DELAY_17800		0b00001101

// MODE2

//  to-do - currently not using this functionality

// MODE3

//  to-do - currently not using this functionality

// REF

//  to-do - default reference config works for our use

// IMUX

//  to-do - currently not using this functionality

// IMAG

//  to-do - currently not using this functionality

// PGA

//  PGA Bypass
#define ADS126X_PGA_NORM		0b00000000
#define ADS126X_PGA_BYPASS		0b10000000

//  PGA Gain
#define ADS126X_PGA_GAIN_1		0b00000000
#define ADS126X_PGA_GAIN_2		0b00000001
#define ADS126X_PGA_GAIN_4		0b00000010
#define ADS126X_PGA_GAIN_8		0b00000011
#define ADS126X_PGA_GAIN_16		0b00000100
#define ADS126X_PGA_GAIN_32		0b00000101
#define ADS126X_PGA_GAIN_64		0b00000110
#define ADS126X_PGA_GAIN_128	0b00000111

// INPMUX

#define ADS126X_AINCOM			0b00000000
#define ADS126X_AIN0			0b00000001
#define ADS126X_AIN1			0b00000010
#define ADS126X_AIN2			0b00000011
#define ADS126X_AIN3			0b00000100
#define ADS126X_AIN4			0b00000101
#define ADS126X_AIN5			0b00000110
#define ADS126X_AIN6			0b00000111
#define ADS126X_AIN7			0b00001000
#define ADS126X_AIN8			0b00001001
#define ADS126X_AIN9			0b00001010
#define ADS126X_INT_TEMP		0b00001011
#define ADS126X_INT_DIV4		0b00001100
#define ADS126X_INT_DIV4_POS	0b00001101
#define ADS126X_IN_OPEN			0b00001110
#define ADS126X_IN_CONN			0b00001111

// INPBIAS

//  to-do - currently not using this functionality


//SPI config

#define SPI_FREQ 16000000/8//(16000000/8)

class ADS126X {
  public:
  	//init
	ADS126X(uint8_t CSPin, uint8_t STARTPin, uint8_t PWDNPin, uint8_t RESETPin, uint8_t DRDYPin);

	//config
	bool setDataRate(uint16_t dataRate);
	bool setFilter(uint8_t filter);
	bool setPGAGain (uint8_t gain);
	bool setMux(uint8_t muxP, uint8_t muxN);
	void enableChopper(bool chop);
	void setOneShot(bool OS);
	void setPGABypass(bool bypass);
	
	//standalone setup for SPI bus
	void beginSPI();
	void pauseSPI();
	
	//quick commands
	void noOp();
	void lock();
	void unlock();
    void reset();
    void start();
    void stop();
	//uint32_t read();
	uint8_t getChipID();
	
	//calibration
	void sysOffCal();
	void gainCal();
	void selfOffCal();
	
	//status - to-do - currently not using this functionality
	//bool PGAAlarmLow();
	//bool PGAAlarmHigh();
	//bool refAlarmLow();
	bool dataReady();
	
	//stream data
	uint32_t getADCData();
	
    //General Commands
	uint8_t readRegister(uint8_t addr);
    
  private:
    uint8_t _CSPin, _STARTPin, _PWDNPin, _RESETPin, _DRDYPin;
	uint8_t _filter = 4, _gain = 1, _muxP = 16, _muxN = 16;
    uint16_t _dataRate = 20;
	bool _bypass = false, _chop = false, _OS = false, _conversionActive = false;

    bool sendCommand(uint8_t cmdByte);
    
    bool writeRegister(uint8_t addr, uint8_t payload);

};
