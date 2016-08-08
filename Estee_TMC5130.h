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

#ifndef ESTEE_TMC5130_H
#define ESTEE_TMC5130_H

#include <Arduino.h>
#include <SPI.h>


class Estee_TMC5130 {
	uint8_t _CS;
	uint32_t _fclk;
	SPISettings _spiSettings;
	SPIClass &_spi;
public:
	Estee_TMC5130( uint8_t chipSelectPin,	// pin to use for the SPI bus SS line
		uint32_t fclk=F_CPU,
		const SPISettings &spiSettings=SPISettings(1000000, MSBFIRST, SPI_MODE0), // spi bus settings to use
		SPIClass& spi=SPI ); // spi class to use
	~Estee_TMC5130();

	// start/stop this module
	bool begin(uint8_t ihold, uint8_t irun, uint8_t stepper_direction/*=NORMAL_MOTOR_DIRECTION*/);
	void end();

	uint32_t readRegister(uint8_t address);	// addresses are from TMC5130.h
	uint8_t  writeRegister(uint8_t address, uint32_t data);
	uint8_t  readStatus();

	// internal clock measuring
	// NOTE: Driver MUST BE DISABLED DURING THIS CALL
	float updateFrequencyScaling();

	// high level interface
	void setRampCurves() {}

private:
	void _beginTransaction();
	void _endTransaction();
};


// TRINAMIC TMC5130 Register Address Defines
#define GCONF				0x00 	//Global configuration flags
#define X_COMPARE 			0x05	//Position  comparison  register
#define IHOLD_IRUN			0x10	//Driver current control
#define TCOOLTHRS			0x14	//This is the lower threshold velocity for switching on smart energy coolStep and stallGuard feature.
#define RAMPMODE			0x20	//Driving mode (Velocity, Positioning, Hold)

// q & qdot (position, velocity)
#define XACTUAL				0x21	//Actual motor position
#define VACTUAL 			0x22	//Actual  motor  velocity  from  ramp  generator

// Ramp curves
#define VSTART				0x23	//Motor start velocity
#define A_1					0x24	//First  acceleration  between  VSTART  and  V1
#define V_1					0x25	//First  acceleration  /  deceleration  phase  target velocity
#define AMAX				0x26	//Second  acceleration  between  V1  and  VMAX
#define VMAX 				0x27	//This is the target velocity in velocity mode. It can be changed any time during a motion.
#define DMAX				0x28	//Deceleration between VMAX and V1
#define D_1					0x2A 	//Deceleration  between  V1  and  VSTOP
									//Attention:  Do  not  set  0  in  positioning  mode, even if V1=0!
#define VSTOP				0x2B	//Motor stop velocity (unsigned)
									//Attention: Set VSTOP > VSTART!
									//Attention:  Do  not  set  0  in  positioning  mode, minimum 10 recommend!
#define TZEROWAIT			0x2C	//Defines  the  waiting  time  after  ramping  down
									//to  zero  velocity  before  next  movement  or
									//direction  inversion  can  start.  Time  range  is about 0 to 2 seconds.

#define XTARGET				0x2D	//Target position for ramp mode
#define SW_MODE 			0x34	//Switch mode configuration
#define RAMP_STAT			0x35	//Ramp status and switch event status
#define XLATCH				0x36	//Latches  XACTUAL  upon  a programmable switch event


#define CHOPCONF			0x6C	//Chopper and driver configuration
#define 	DISS2G(n)		(((n)&0x1)<<30)	// Short to GND protection is on=1,disabled=0
#define 	DEDGE(n)		(((n)&0x1)<<29) // 1: Enable step impulse at each step edge to reduce step frequency requirement
#define 	INTPOL(n)		(((n)&0x1)<<28) // 1: The actual microstep resolution (MRES) becomes extrapolated to
											// 256 microsteps for smoothest motor operation (useful for Step/Dir operation, only)
#define 	MRES(n)			(((n)&0xF)<<24) // %0000: Native 256 microstep setting. Normally use this setting
											// with the internal motion controller.
											// %0001 … %1000:
											// 128, 64, 32, 16, 8, 4, 2, FULLSTEP
											// Reduced microstep resolution esp. for Step/Dir operation.
											// The resolution gives the number of microstep entries per
											// sine quarter wave.
											// The driver automatically uses microstep positions which
											// result in a symmetrical wave, when choosing a lower
											// microstep resolution.
											// step width=2^MRES [microsteps]
#define 	SYNC(n)			(((n)&0xF)<<20) // This register allows synchronization of the chopper for
											// both phases of a two phase motor in order to avoid the
											// occurrence of a beat, especially at low motor velocities. It
											// is automatically switched off above VHIGH.
											// %0000: Chopper sync function chopSync off
											// %0001 … %1111:
											// Synchronization with fSYNC = fCLK/(sync*64)
											// Hint: Set TOFF to a low value, so that the chopper cycle is
											// ended, before the next sync clock pulse occurs. Set for the
											// double desired chopper frequency for chm=0, for the
											// desired base chopper frequency for chm=1
#define 	VHIGHCHM(n)		(((n)&0x1)<<19) // This bit enables switching to chm=1 and fd=0, when VHIGH
											// is exceeded. This way, a higher velocity can be achieved.
											// Can be combined with vhighfs=1. If set, the TOFF setting
											// automatically becomes doubled during high velocity
											// operation in order to avoid doubling of the chopper
											// frequency.
#define 	VHIGHFS(n)		(((n)&0x1)<<18) // This bit enables switching to fullstep, when VHIGH is
											// exceeded. Switching takes place only at 45° position.
											// The fullstep target current uses the current value from
											// the microstep table at the 45° position.
#define 	VSENSE(n)		(((n)&0x1)<<17) //0: Low sensitivity, high sense resistor voltage
											//1: High sensitivity, low sense resistor voltage
#define 	TBL(n)			(((n)&0x3)<<15) // %00 … %11:
											// Set comparator blank time to 16, 24, 36 or 54 clocks
											// Hint: %01 or %10 is recommended for most applications
#define 	CHM(n)			(((n)&0x1)<<14) // 0 Standard mode (spreadCycle)
											// 1 Constant off time with fast decay time.
											// Fast decay time is also terminated when the
											// negative nominal current is reached. Fast decay is
											// after on time.
#define 	RNDTF(n)		(((n)&0x1)<<13) // 0 Chopper off time is fixed as set by TOFF
											// 1 Random mode, TOFF is random modulated by
											// dNCLK= -12 … +3 clocks.
#define 	DISFDCC(n)		(((n)&0x1)<<12) // chm=1:
											// disfdcc=1 disables current comparator usage for termination
											// of the fast decay cycle
#define 	TFD3(n)			(((n)&0x1)<<11) // chm=1: MSB of fast decay time setting TFD[3]
#define 	HEND(n)			(((n)&0xF)<<7)  // chm=0 %0000 … %1111:
											// Hysteresis is -3, -2, -1, 0, 1, …, 12
											// (1/512 of this setting adds to current setting)
											// This is the hysteresis value which becomes
											// used for the hysteresis chopper.
											// chm=1 %0000 … %1111:
											// Offset is -3, -2, -1, 0, 1, …, 12
											// This is the sine wave offset and 1/512 of the
											// value becomes added to the absolute value
											// of each sine wave entry.
#define 	HSTRT_TFD(n)	(((n)&0x7)<<4)  // chm=0 %000 … %111:
											// Add 1, 2, …, 8 to hysteresis low value HEND
											// (1/512 of this setting adds to current setting)
											// Attention: Effective HEND+HSTRT ≤ 16.
											// Hint: Hysteresis decrement is done each 16
											// clocks

											// TFD [2..0]
											// fast decay time setting
											// chm=1 Fast decay time setting (MSB: tfd3):
											// %0000 … %1111:
											// Fast decay time setting TFD with
											// NCLK= 32*HSTRT (%0000: slow decay only)
#define 	TOFF(n)			(((n)&0xF)<<0)  // Off time setting controls duration of slow decay phase
											// NCLK= 12 + 32*TOFF
											// %0000: Driver disable, all bridges off
											// %0001: 1 – use only with TBL ≥ 2
											// %0010 … %1111: 2 … 15

#define COOLCONF			0x6D	//coolStep smart current control register and stallGuard2 configuration
#define 	SFILT(n)		(((n)&0x1)<<24) // 0 Standard mode, high time resolution for
											// stallGuard2
											// 1 Filtered mode, stallGuard2 signal updated for each
											// four fullsteps (resp. six fullsteps for 3 phase motor)
											// only to compensate for motor pole tolerances
#define 	SGT(n)			(((n)&0x7F)<<16)// This signed value controls stallGuard2 level for stall
											// output and sets the optimum measurement range for
											// readout. A lower value gives a higher sensitivity. Zero is
											// the starting value working with most motors.
											// -64 to +63: A higher value makes stallGuard2 less
											// sensitive and requires more torque to
											// indicate a stall.
#define 	SEIMIN(n)		(((n)&0x1)<<15) // 0: 1/2 of current setting (IRUN)
											// 1: 1/4 of current setting (IRUN)	
#define 	SEDN(n)			(((n)&0x3)<<13) // %00: For each 32 stallGuard2 values decrease by one
											// %01: For each 8 stallGuard2 values decrease by one
											// %10: For each 2 stallGuard2 values decrease by one
											// %11: For each stallGuard2 value decrease by one
#define 	SEMAX(n)		(((n)&0xF)<<8)  // If the stallGuard2 result is equal to or above
											// (SEMIN+SEMAX+1)*32, the motor current becomes
											// decreased to save energy.
											// %0000 … %1111: 0 … 15
#define 	SEUP(n)			(((n)&0x3)<<5)  // Current increment steps per measured stallGuard2 value %00 … %11: 1, 2, 4, 8
#define 	SEMIN(n)		(((n)&0xF)<<0)  // If the stallGuard2 result falls below SEMIN*32, the motor
											// current becomes increased to reduce motor load angle.
											// %0000: smart current control coolStep off
											// %0001 … %1111: 1 … 15

#define DRV_STATUS 			0x6F	// stallGuard2 value and driver error flags

#define SET_IHOLDDELAY(n)	(((n)&0xF)<<16)
#define SET_IRUN(n)			(((n)&0x1F)<<8)
#define SET_IHOLD(n)		(((n)&0x1F)<<0)


#define WRITE_ACCESS			0x80	// Write access for spi communication

// Polarity for reference switch
#define REF_SW_HIGH_ACTIV	0x00 	// non-inverted, high active: a high level on REFL stops the motor
#define REF_SW_LOW_ACTIV	0x0C	// inverted, low active: a low level on REFL stops the motor

// Motor direction
#define NORMAL_MOTOR_DIRECTION	0x00	// Normal motor direction
#define INVERSE_MOTOR_DIRECTION	0x10	// Inverse motor direction

// Modes for RAMPMODE register
#define POSITIONING_MODE	0x00		// using all A, D and V parameters)
#define VELOCITY_MODE_POS	0x01		// positiv VMAX, using AMAX acceleration
#define VELOCITY_MODE_NEG	0x02		// negativ VMAX, using AMAX acceleration
#define HOLD_MODE			0x03		// velocity remains unchanged, unless stop event occurs

#define VZERO				0x400		// flag in RAMP_STAT, 1: signals that the actual velocity is 0.

#endif // ESTEE_TMC5130_H