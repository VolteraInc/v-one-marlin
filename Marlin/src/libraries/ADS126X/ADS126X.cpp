#include "ADS126X.h"

ADS126X::ADS126X(uint8_t CSPin, uint8_t STARTPin, uint8_t PWDNPin, uint8_t RESETPin, uint8_t DRDYPin)
{
	this->_CSPin = CSPin;
	this->_STARTPin = STARTPin;
	this->_PWDNPin = PWDNPin;
	this->_RESETPin = RESETPin;
	this->_DRDYPin = DRDYPin;
	
	//config HW
    ::pinMode(_CSPin, OUTPUT);
    ::digitalWrite(_CSPin, HIGH);
	
    ::pinMode(_STARTPin, OUTPUT);
    ::digitalWrite(_STARTPin, LOW);
	
    ::pinMode(_PWDNPin, OUTPUT);
    ::digitalWrite(_PWDNPin, HIGH);
	
	::pinMode(_RESETPin, OUTPUT);
    ::digitalWrite(_RESETPin, HIGH);
	
	::pinMode(_DRDYPin, INPUT);
	
}

uint32_t ADS126X::getADCData()
{
	uint8_t cmdByte = ADS126X_RDATA, msbByte = 0x00, midByte = 0x00, lsbByte = 0x00;
	uint32_t result = 0x00000000;
	
	//pauseSPI();

	//check if conversions are underway
	if(!_conversionActive)
	{
		start();
	}
	
	beginSPI();
	
	while(!dataReady()); //wait for data ready signal, to-do must add timeout
	
	SPI.transfer(cmdByte);
	if(SPI.transfer(DUMMY_BYTE) == cmdByte) //verify we're talking to the chip
	{
		msbByte = SPI.transfer(DUMMY_BYTE);
		midByte = SPI.transfer(DUMMY_BYTE);
		lsbByte = SPI.transfer(DUMMY_BYTE);
		
		
		log << msbByte << endl;
		//log << F("new") << endl;
		log << midByte << endl; //these aren't executing?
		log << lsbByte << endl;
		//log << F("new2") << endl;
		
		//combine bytes
		result |= msbByte;
		result <<= 8;
		
		result |= midByte;
		result <<= 8;
		
		result |= lsbByte;
		//log << result << endl;

		pauseSPI();
		//stop();
		return result;
	}
	
	pauseSPI();
	//stop();
	return 0xFFFFFFFF; //this is an impossible result
}

bool ADS126X::setDataRate(uint16_t dataRate) //TO IMPLEMENT THIS WILL CURRENTLY GET RID OF THE FILTER SETTING
{
	uint8_t _addr = ADS126X_MODE0;
	uint8_t _payload;
	
	//check if this is the current setting
	if (dataRate == this->_dataRate)
	{
		return true; //already configured as requested
	}
	
	//get current register contents
	_payload = readRegister(_addr);
	
	//clear data rate bitset
	_payload &= 0b00000111;
	
	switch (dataRate) {
		case 2:
			_payload |= ADS126X_DR_2_5;
		case 5:
			_payload |= ADS126X_DR_5;
		case 10:
			_payload |= ADS126X_DR_10;
		case 17:
			_payload |= ADS126X_DR_16_6;
		case 20:
			_payload |= ADS126X_DR_20;
		case 50:
			_payload |= ADS126X_DR_50;
		case 60:
			_payload |= ADS126X_DR_60;
		case 100:
			_payload |= ADS126X_DR_100;
		case 400:
			_payload |= ADS126X_DR_400;
		case 1200:
			_payload |= ADS126X_DR_1200;
		case 2400:
			_payload |= ADS126X_DR_2400;
		case 4800:
			_payload |= ADS126X_DR_4800;
		case 7200:
			_payload |= ADS126X_DR_7200;
		case 14400:
			_payload |= ADS126X_DR_14400;
		case 19200:
			_payload |= ADS126X_DR_19200;
		case 25600:
			_payload |= ADS126X_DR_25600;
		case 40000:
			_payload |= ADS126X_DR_40000;
		default:
			//invalid dataRate
			return false;
	}
	this->_dataRate = dataRate;
	writeRegister(_addr, _payload);
	return true;
}

bool ADS126X::setFilter(uint8_t filter)
{
	uint8_t _addr = ADS126X_MODE0;
	uint8_t _payload;
	
	//check if this is the current setting
	if (filter == this->_filter)
	{
		return true; //already configured as requested
	}
	
	//get current register contents
	_payload = readRegister(_addr);
	
	//clear data rate bitset
	_payload &= 0b11111000;
	
	switch (filter) {
		case 0:
			_payload |= ADS126X_FILT_SINC1;
		case 1:
			_payload |= ADS126X_FILT_SINC2;
		case 2:
			_payload |= ADS126X_FILT_SINC3;
		case 3:
			_payload |= ADS126X_FILT_SINC4;
		case 4:
			_payload |= ADS126X_FILT_FIR;
		default:
			//invalid dataRate
			return false;
	}
	this->_filter = filter;
	writeRegister(_addr, _payload);
	return true;
}

bool ADS126X::setPGAGain (uint8_t gain)
{
	uint8_t _addr = ADS126X_PGA;
	uint8_t _payload;
	
	//check if this is the current setting
	if (gain == this->_gain)
	{
		return true; //already configured as requested
	}
	
	//get current register contents
	_payload = readRegister(_addr);
	
	//clear data rate bitset
	_payload &= 0b11111000;
	
	switch (gain) {
		case 1:
			_payload |= ADS126X_PGA_GAIN_1;
		case 2:
			_payload |= ADS126X_PGA_GAIN_2;
		case 4:
			_payload |= ADS126X_PGA_GAIN_4;
		case 8:
			_payload |= ADS126X_PGA_GAIN_8;
		case 16:
			_payload |= ADS126X_PGA_GAIN_16;
		case 32:
			_payload |= ADS126X_PGA_GAIN_32;
		case 64:
			_payload |= ADS126X_PGA_GAIN_64;
		case 128:
			_payload |= ADS126X_PGA_GAIN_128;
		default:
			//invalid dataRate
			return false;
	}
	this->_gain = gain;
	writeRegister(_addr, _payload);
	return true;
}
void ADS126X::enableChopper(bool chop)
{
	uint8_t _addr = ADS126X_MODE1;
	uint8_t _payload;
	
	//check if this is the current setting
	if (chop == this->_chop)
	{
		return; //already configured as requested
	}
	
	//get current register contents
	_payload = readRegister(_addr);
	
	//flip relevant bit
	_payload ^= 0b00100000;
	
	this->_chop = chop;
	writeRegister(_addr, _payload);
	return;
}
void ADS126X::setOneShot(bool OS)
{
	uint8_t _addr = ADS126X_MODE1;
	uint8_t _payload;
	
	//check if this is the current setting
	if (OS == this->_OS)
	{
		return; //already configured as requested
	}
	
	//get current register contents
	_payload = readRegister(_addr);
	
	//flip relevant bit
	_payload ^= 0b00010000;
	
	this->_OS = OS;
	writeRegister(_addr, _payload);
	return;
}

void ADS126X::setPGABypass(bool bypass)
{
	uint8_t _addr = ADS126X_PGA;
	uint8_t _payload;
	
	//check if this is the current setting
	if (bypass == this->_bypass)
	{
		return; //already configured as requested
	}
	
	//get current register contents
	_payload = readRegister(_addr);
	
	//flip relevant bit
	_payload ^= 0b10000000;
	
	this->_bypass = bypass;
	writeRegister(_addr, _payload);
	return;
}

bool ADS126X::setMux(uint8_t muxP, uint8_t muxN)
{
	uint8_t _addr = ADS126X_INPMUX;
	uint8_t _payload = 0x00;
	
	uint8_t muxOptions[] = {ADS126X_AIN0, ADS126X_AIN1, ADS126X_AIN2, ADS126X_AIN3, ADS126X_AIN4, ADS126X_AIN5, ADS126X_AIN6, ADS126X_AIN7, ADS126X_AIN8, ADS126X_AIN9, ADS126X_AINCOM};
	
	//check if this is the current setting
	if (muxP == this->_muxP && muxP == this->_muxN)
	{
		return true; //already configured as requested
	}
	
	if(muxP < 11 && muxN < 11) //check for legal choice, e.g. inputs 0-10
	{
		//low 4 bits are for MUXN, top 4 bits are for MUXP
		_payload |= muxOptions[muxN];
		_payload |= (muxOptions[muxP] << 4);
		
		writeRegister(_addr, _payload);
		this->_muxP = muxP;
		this->_muxN = muxN;
		
		return true;
	}

	return false;
}

void ADS126X::noOp()
{
	sendCommand(ADS126X_NOP);
}

void ADS126X::lock()
{
	sendCommand(ADS126X_LOCK);
}

void ADS126X::unlock()
{
	sendCommand(ADS126X_UNLOCK);
}

void ADS126X::reset()
{
	sendCommand(ADS126X_RESET);
}

void ADS126X::start() //using HW pin here
{
	digitalWrite(_STARTPin, HIGH);
	_conversionActive = true;
}

void ADS126X::stop() //using HW pin here
{
	digitalWrite(_STARTPin, LOW);
	_conversionActive = false;
}

bool ADS126X::dataReady()
{
	return !digitalRead(_DRDYPin);
}

void ADS126X::sysOffCal()
{
	sendCommand(ADS126X_SYOCAL);
}
void ADS126X::gainCal()
{
	sendCommand(ADS126X_GANCAL);
}
void ADS126X::selfOffCal()
{
	sendCommand(ADS126X_SFOCAL);
}

void ADS126X::beginSPI() //to save time during datastream
{
	SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE1));
	::digitalWrite(this->_CSPin, LOW);
}

void ADS126X::pauseSPI()
{
	::digitalWrite(this->_CSPin, HIGH);
	SPI.endTransaction();
}

bool ADS126X::sendCommand(uint8_t cmdByte) 
{
	//begin SPI (time this)
	beginSPI();

    SPI.transfer(cmdByte);
    if(SPI.transfer(DUMMY_BYTE) == cmdByte) //check echo back for chip functionality
	{
		pauseSPI();
		return true;
	}
    
	//if we get here we didn't get the echo back
    pauseSPI();
	return false;
}

bool ADS126X::writeRegister(uint8_t addr, uint8_t payload) 
{
	uint8_t cmdByte = ADS126X_WREG | addr; //this creates the command to write to resiter at addr
	
	//begin SPI (time this)
	beginSPI();
	
	SPI.transfer(cmdByte);
	if(SPI.transfer(payload) == cmdByte) //check echo back for chip functionality
	{ 
		pauseSPI();
		return true; //successful write
	}
    
	//if we get here we didn't get the echo back
    pauseSPI();
	return false; //failed write
	
}

uint8_t ADS126X::readRegister(uint8_t addr) 
{
	uint8_t regData = 0;
	uint8_t cmdByte = ADS126X_RREG | addr; //this creates the command to read to resiter at addr
	
	//begin SPI (time this)
	beginSPI();
	
	SPI.transfer(cmdByte);
	if(SPI.transfer(DUMMY_BYTE) == cmdByte) //check echo back for chip functionality
	{ 
		regData = SPI.transfer(DUMMY_BYTE);
		pauseSPI();
		return regData; //successful read
	}
    
	//if we get here we didn't get the echo back
    pauseSPI();
	return regData; //failed read
}
