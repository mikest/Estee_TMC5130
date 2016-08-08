/*
MIT License

Copyright (c) 2016 Mike Estee

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "Estee_TMC5130.h"

Estee_TMC5130::Estee_TMC5130( uint8_t chipSelectPin, uint32_t fclk, const SPISettings &spiSettings, SPIClass &spi )
: _CS(chipSelectPin), _fclk(fclk), _spiSettings(spiSettings), _spi(spi)
{

}

Estee_TMC5130::~Estee_TMC5130()
{
	;
}


bool Estee_TMC5130::begin( uint8_t ihold, uint8_t irun, uint8_t stepper_direction )
{
	// set initial currents and delay
	uint32_t iholdrun = SET_IHOLD(ihold) | SET_IRUN(irun) | SET_IHOLDDELAY(7);
	writeRegister(IHOLD_IRUN, iholdrun );

	// use position mode
	writeRegister(RAMPMODE, 0x0 );

	// set ramp curves
	//writeRegister(A_1, 0xffff );
	writeRegister(V_1, 0x0 ); // disable A1 & D1 in position mode, amax, vmax only
	writeRegister(AMAX, 0xffff );
	writeRegister(VMAX, 0xffff );
	//writeRegister(DMAX, 0xffff );

	// set chopper config
	writeRegister(CHOPCONF, 0x00010085);//0x140101D5);
	writeRegister(GCONF, 0x1084 | stepper_direction);

	return false;
}

void Estee_TMC5130::end()
{
	// no-op, just stop talking....
	; // FIXME: try and shutdown motor/chips?
}


// From "28.1 Using the Internal Clock", how to figure out the step/sec scaling value
float Estee_TMC5130::updateFrequencyScaling()
{
	int32_t vmax = 10000;
	int32_t dt_ms = 100;

	writeRegister(VMAX, 0);
	writeRegister(RAMPMODE, 1);	// velocity mode
	writeRegister(AMAX, 60000);
	writeRegister(VMAX, vmax);
	int32_t xactual1 = readRegister(XACTUAL);
	delay(dt_ms);
	int32_t xactual2 = readRegister(XACTUAL);
	writeRegister(VMAX,0);	// halt

	// scaling factor
	return (vmax * (dt_ms/1000.0)) / (xactual2 - xactual1);
}

#pragma mark -

// calls to read/write registers must be bracketed by the begin/endTransaction calls

void _chipSelect( uint8_t pin, bool select )
{
	digitalWrite(pin, select?LOW:HIGH);
	if( select )
		delayMicroseconds(100);   // per spec, settling time is 100us
}

void Estee_TMC5130::_beginTransaction()
{
	_spi.beginTransaction(_spiSettings);
	_chipSelect(_CS, true);
}

void Estee_TMC5130::_endTransaction()
{
	_chipSelect(_CS, false);
	_spi.endTransaction();
}

uint32_t Estee_TMC5130::readRegister(uint8_t address)
{
	// request the read for the address
	_beginTransaction();
	_spi.transfer(address);
	_spi.transfer(0x00);
	_spi.transfer(0x00);
	_spi.transfer(0x00);
	_spi.transfer(0x00);
	_endTransaction();

	// skip a beat
	#define nop() asm volatile("nop")
	nop();
	nop();
	nop();

	// read it in the second cycle
	_beginTransaction();
	_spi.transfer(address);
	uint32_t value = 0;
	value |= _spi.transfer(0x00) << 24;
	value |= _spi.transfer(0x00) << 16;
	value |= _spi.transfer(0x00) << 8;
	value |= _spi.transfer(0x00);
	_endTransaction();

	return value;
}

uint8_t Estee_TMC5130::writeRegister(uint8_t address, uint32_t data)
{
	// address register
	_beginTransaction();
	uint8_t status = _spi.transfer(address | WRITE_ACCESS);

	// send new register value
	_spi.transfer((data & 0xFF000000) >> 24);
	_spi.transfer((data & 0xFF0000) >> 16);
	_spi.transfer((data & 0xFF00) >> 8);
	_spi.transfer(data & 0xFF);
	_endTransaction();

	return status;
}


uint8_t Estee_TMC5130::readStatus()
{
 	// read general config
 	_beginTransaction();
 	uint8_t status = _spi.transfer(GCONF);
 	// send dummy data
 	_spi.transfer(0x00);
 	_spi.transfer(0x00);
 	_spi.transfer(0x00);
 	_spi.transfer(0x00);
 	_endTransaction();

	return status;
}


#pragma mark -

