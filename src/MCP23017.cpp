#include "MCP23017.h"
#define _BV(bit) 	(1 << (bit))
 
MCP23017::MCP23017(uint8_t address, TwoWire& bus) {
	_deviceAddr = address;
	_bus = &bus;
}

MCP23017::~MCP23017() {}

size_t MCP23017::init()
{
	//BANK = 	0 : sequential register addresses
	//MIRROR = 	0 : use configureInterrupt 
	//SEQOP = 	1 : sequential operation disabled, address pointer does not increment
	//DISSLW = 	0 : slew rate enabled
	//HAEN = 	0 : hardware address pin is always enabled on 23017
	//ODR = 	0 : open drain output
	//INTPOL = 	0 : interrupt active low
	return writeRegister(MCP23017_REGISTER::IOCON, 0b00100000);

	//enable all pull up resistors (will be effective for input pins only)
	//writeRegister(MCP23017_REGISTER::GPPUA, 0xFF, 0xFF);
}

size_t MCP23017::portMode(MCP23017_PORT port, uint8_t value)
{
	return writeRegister(MCP23017_REGISTER::IODIRA + port, value);
}

size_t MCP23017::pinMode(uint8_t pin, uint8_t mode)
{
	MCP23017_REGISTER iodirreg = MCP23017_REGISTER::IODIRA;
	uint8_t iodir;
	if(pin > 7)
	{
		iodirreg = MCP23017_REGISTER::IODIRB;
		pin -= 8;
	}

	iodir = readRegister(iodirreg);
	if(mode == OUTPUT) iodir &= ~_BV(pin);
	else iodir |= _BV(pin);

	return writeRegister(iodirreg, iodir);
}

size_t MCP23017::digitalWrite(uint8_t pin, uint8_t state)
{
	MCP23017_REGISTER gpioreg = MCP23017_REGISTER::GPIOA;
	uint8_t gpio;
	if(pin > 7)
	{
		gpioreg = MCP23017_REGISTER::GPIOB;
		pin -= 8;
	}

	gpio = readRegister(gpioreg);
	if(state == HIGH) gpio |= _BV(pin);
	else gpio &= ~_BV(pin);

	return writeRegister(gpioreg, gpio);
}

uint8_t MCP23017::digitalRead(uint8_t pin)
{
	MCP23017_REGISTER gpioreg = MCP23017_REGISTER::GPIOA;
	uint8_t gpio;
	if(pin > 7)
	{
		gpioreg = MCP23017_REGISTER::GPIOB;
		pin -=8;
	}

	gpio = readRegister(gpioreg);
	if((gpio & _BV(pin)) == _BV(pin)) return HIGH;
	return LOW;
}

size_t MCP23017::writePort(MCP23017_PORT port, uint8_t value)
{
	return writeRegister(MCP23017_REGISTER::GPIOA + port, value);
}

size_t MCP23017::write(uint16_t value)
{
	return writeRegister(MCP23017_REGISTER::GPIOA, lowByte(value), highByte(value));
}

uint8_t MCP23017::readPort(MCP23017_PORT port)
{
	return readRegister(MCP23017_REGISTER::GPIOA + port);
}

uint16_t MCP23017::read()
{
	uint8_t a = readPort(MCP23017_PORT::A);
	uint8_t b = readPort(MCP23017_PORT::B);

	return a | b << 8;
}

size_t MCP23017::writeRegister(MCP23017_REGISTER reg, uint8_t value)
{
	size_t retVal; //Pass = 0, Fail !0 with error code
	_bus->beginTransmission(_deviceAddr);
	retVal =  (0==_bus->write(static_cast<uint8_t>(reg))) ? mcpErr_SeqErrBit : 0;
	retVal |= (0==_bus->write(value)) ? mcpErr_SeqErrBit  : 0;
	retVal |= _bus->endTransmission();
	return retVal;
}

size_t MCP23017::writeRegister(MCP23017_REGISTER reg, uint8_t portA, uint8_t portB)
{
	size_t retVal; //Pass = 0, Fail !0 with error code
	_bus->beginTransmission(_deviceAddr);
	retVal =  (0==_bus->write(static_cast<uint8_t>(reg))) ? mcpErr_SeqErrBit : 0;
	retVal |= (0==_bus->write(portA)) ? mcpErr_SeqErrBit : 0;
	retVal |= (0==_bus->write(portB)) ? mcpErr_SeqErrBit : 0;
	retVal |= _bus->endTransmission();
	return retVal;
}


uint8_t MCP23017::readRegister(MCP23017_REGISTER reg)
{
	_bus->beginTransmission(_deviceAddr);
	_bus->write(static_cast<uint8_t>(reg));
	_bus->endTransmission();
	_bus->requestFrom(_deviceAddr, (uint8_t)1);
	return _bus->read();
}

void MCP23017::readRegister(MCP23017_REGISTER reg, uint8_t& portA, uint8_t& portB)
{
	_bus->beginTransmission(_deviceAddr);
	_bus->write(static_cast<uint8_t>(reg));
	_bus->endTransmission();
	_bus->requestFrom(_deviceAddr, (uint8_t)2);
	portA = _bus->read();
	portB = _bus->read();
}

#ifdef _MCP23017_INTERRUPT_SUPPORT_

size_t MCP23017::interruptMode(MCP23017_INTMODE intMode)
{
	uint8_t iocon = readRegister(MCP23017_REGISTER::IOCON);
	if(intMode == MCP23017_INTMODE::OR) iocon |= static_cast<uint8_t>(MCP23017_INTMODE::OR);
	else iocon &= ~(static_cast<uint8_t>(MCP23017_INTMODE::OR));

	return writeRegister(MCP23017_REGISTER::IOCON, iocon);
}

size_t MCP23017::interrupt(MCP23017_PORT port, uint8_t mode)
{
	uint8_t retVal;
	MCP23017_REGISTER defvalreg = MCP23017_REGISTER::DEFVALA + port;
	MCP23017_REGISTER intconreg = MCP23017_REGISTER::INTCONA + port;

	//enable interrupt for port
	retVal = writeRegister(MCP23017_REGISTER::GPINTENA + port, 0xFF);
	switch(mode)
	{
	case CHANGE:
		//interrupt on change
		retVal |= writeRegister(intconreg, 0);
		break;
	case FALLING:
		//interrupt falling : compared against defval, 0xff
		retVal |= writeRegister(intconreg, 0xFF);
		retVal |= writeRegister(defvalreg, 0xFF);
		break;
	case RISING:
		//interrupt falling : compared against defval, 0x00
		retVal |= writeRegister(intconreg, 0xFF);
		retVal |= writeRegister(defvalreg, 0x00);
		break;
	}
	return retVal;
}

void MCP23017::interruptedBy(uint8_t& portA, uint8_t& portB)
{
	readRegister(MCP23017_REGISTER::INTFA, portA, portB);
}

size_t MCP23017::disableInterrupt(MCP23017_PORT port)
{
	return writeRegister(MCP23017_REGISTER::GPINTENA + port, 0x00);
}

void MCP23017::clearInterrupts()
{
	uint8_t a, b;
	clearInterrupts(a, b);
}

void MCP23017::clearInterrupts(uint8_t& portA, uint8_t& portB)
{
	readRegister(MCP23017_REGISTER::INTCAPA, portA, portB);
}

#endif
