/*
 * config.h
 *
 * Created: 1/14/2017 2:33:39 PM
 *  Author: Tim
 */ 


//_______INCLUDES_________  If left out there will be multiple "not defined" errors in can_lib files!
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "compiler.h"
#include "at90can_drv.h"

#ifndef CONFIG_H_
#define CONFIG_H_

#define FOSC  16000        // 16 MHz External crystal

// -------------- CAN LIB CONFIGURATION
#define CAN_BAUDRATE   1000        // in kBit/s
#define MY_ID 0x1eb

#include "can_lib.h" //The macros above must be defined before including this header file

//_____Various_Definitions________
#define inTestMode 0 //hard code whether the board should go into test mode
#define TRAN_RESET_ENABLED 0
#define SYSTEM_LOOP_TIME 0x20
#define SLEEPTIME 1800000 //30 minutes
#define SAMPLES 100
#define CURRENT_SURGE_TIME 500
#define TRICKLE_LEVEL 20
#define OC_THLD_MOTEC 1024 //Over-current thresholds defined in mA
#define OC_THLD_FUEL 1024  //***CHANGE THESE***
#define OC_THLD_EDL 1024
#define OC_THLD_FAN 1024
#define OC_THLD_12V 1024
#define CS_FACTOR_MOTEC 7050 //4.7V/10kOhm * 15000   Factors are the number of mA corresponding to 4.7 V on an ADC input
#define CS_FACTOR_FUEL 8065 //4.7V/8.45kOhm * 14500
#define CS_FACTOR_EDL 5521 //4.7V/6.98kOhm * 8200
#define CS_FACTOR_FAN 11672 //4.7V/6.04kOhm * 15000
#define CS_FACTOR_12V 9133 //4.7V/4.22kOhm * 8200
#define MOTEC_index 0
#define FUEL_index 1
#define EDL_index 2
#define FAN_index 3
#define _12V_index 4
#define DISABLED_E 0b00
#define DISABLED_OC 0b01
#define DISABLED_OK 0b10
#define ENABLED_OK 0b11

//____Status_Modes
#define ERROR 0x00
#define OKAY 0xff
#define OverCurrentERROR 0x01
#define STANDBY 0x04
#define SLEEP 0x05
#define TESTMODE 0xee
#define LEDTEST 0xeb
#define TESTSUCCESS 0xed
#define TESTERROR 0xec

//_______Custom_Structures_________

typedef struct
{
	unsigned char bit0:1;
	unsigned char bit1:1;
	unsigned char bit2:1;
	unsigned char bit3:1;
	unsigned char bit4:1;
	unsigned char bit5:1;
	unsigned char bit6:1;
	unsigned char bit7:1;
}bitField;

typedef struct
{
	unsigned char bit0:   1;
	unsigned char bit1:   1;
	unsigned char bit23:  2;
	unsigned char bit456: 3;
	unsigned char bit7:   1;
}mulBitField;

//______I/O_Definitions/Naming________
#define FUEL_SW ((volatile bitField*)_SFR_MEM_ADDR(PINC))->bit7
#define FAN_SW ((volatile bitField*)_SFR_MEM_ADDR(PIND))->bit0

#define LEDs ((volatile mulBitField*)_SFR_MEM_ADDR(PORTC))->bit456
#define LEDR ((volatile bitField*)_SFR_MEM_ADDR(PORTC))->bit4
#define LEDG ((volatile bitField*)_SFR_MEM_ADDR(PORTC))->bit5
#define LEDB ((volatile bitField*)_SFR_MEM_ADDR(PORTC))->bit1

#define MOTEC ((volatile bitField*)_SFR_MEM_ADDR(PORTC))->bit0
#define FUEL ((volatile bitField*)_SFR_MEM_ADDR(PORTB))->bit1
#define EDL ((volatile bitField*)_SFR_MEM_ADDR(PORTB))->bit7
#define FAN ((volatile bitField*)_SFR_MEM_ADDR(PORTD))->bit1
#define _12V ((volatile bitField*)_SFR_MEM_ADDR(PORTB))->bit0


//Current sense config patterns
#define CS_MOTEC 0x02 //ADC2
#define CS_FUEL 0x03 //ADC3
#define CS_EDL 0x05 //ADC5
#define CS_FAN 0x06 //ADC6
#define CS_12V 0x07 //ADC7


#endif /* CONFIG_H_ */