/*
 * Power_Distribution_active.c This project file is intended to have the capability to fully operate the car. 
 *                             Further changes may need to be made after in vehicle testing.
 *
 * Created: 3/9/2017 12:47:09 PM
 * Author : Timothy Mathison
 */ 

#include "lib/config.h"

//____Prototypes____
void wait_ms(int ms);
unsigned int avgTC();
Bool checkTC(unsigned int value, unsigned int threshold, Bool state, long lastEvent);
U8 sendCANData(U8 * data, st_cmd_t * message);
U8 * createDATA(void);
U8 test(void);

//____Global_Variables
volatile unsigned long TIME = 0;//# of ms since system startup
volatile Bool fuelSwState = OFF;
volatile Bool fanSwState = OFF;
volatile Bool transistorStates[5] = {DISABLED_OK, DISABLED_OK, DISABLED_OK, DISABLED_OK, DISABLED_OK};
volatile int transistorCurrent[SAMPLES]; //holds the last "SAMPLES" # of current sense reads
volatile int avgTransistorCurrent[5]; //holds the last computed average current for each transistor
volatile long lastEventTime[5] = {0, 0, 0, 0, 0};
U8 ADC_select[5] = {CS_MOTEC, CS_FUEL, CS_EDL, CS_FAN, CS_12V};
unsigned int CS_thresholds[5] = {OC_THLD_MOTEC, OC_THLD_FUEL, OC_THLD_EDL, OC_THLD_FAN, OC_THLD_12V};
unsigned int CS_factors[5] = {CS_FACTOR_MOTEC, CS_FACTOR_FUEL, CS_FACTOR_EDL, CS_FACTOR_FAN, CS_FACTOR_12V};
volatile Bool adcFlag = 0;
U8 CT_index = 0; //current transistor current sense being read
st_cmd_t CAN_message;
volatile int currentBoardStatus = ERROR; //Will change quickly in main loop


						 //format ->  bgr
U8 LEDPatterns[16][6] = {   {0b110, 0b111, 0b110, 0b111, 0b110, 0b111}, //error
							{0b110, 0b001, 0b110, 0b001, 0b110, 0b001}, //over current error
							{0b000, 0b000, 0b000, 0b000, 0b000, 0b000}, 
							{0b000, 0b000, 0b000, 0b000, 0b000, 0b000}, 
							{0b011, 0b111, 0b011, 0b111, 0b011, 0b111}, //standby
							{0b011, 0b111, 0b111, 0b111, 0b111, 0b111}, //Sleep
							{0b000, 0b000, 0b000, 0b000, 0b000, 0b000},
							{0b000, 0b000, 0b000, 0b000, 0b000, 0b000},
							{0b000, 0b000, 0b000, 0b000, 0b000, 0b000},
							{0b000, 0b000, 0b000, 0b000, 0b000, 0b000},
							{0b000, 0b000, 0b000, 0b000, 0b000, 0b000},
							{0b110, 0b100, 0b000, 0b001, 0b011, 0b111}, //LED test
							{0b010, 0b011, 0b010, 0b011, 0b010, 0b011}, //test fail
							{0b001, 0b011, 0b001, 0b011, 0b001, 0b011}, //test success
							{0b001, 0b010, 0b001, 0b010, 0b001, 0b010}, //test
							{0b001, 0b101, 0b001, 0b101, 0b001, 0b101}   }; //okay(running)

//timing interrupt to increment a number of timer variables 1 ms at a time.
ISR(TIMER0_COMPA_vect)
{
	static unsigned int fuelDebounce = 0xaaaa;
	static unsigned int fanDebounce = 0xaaaa;
	TIME++;
	if((TIME & 7) == 4) //if TIME is a multiple of 4
	{
		fuelDebounce = (fuelDebounce << 1) | FUEL_SW;
		if(fuelDebounce == 0x0000) fuelSwState = ON;
		else if(fuelDebounce == 0xffff) fuelSwState = OFF;
		
		fanDebounce = (fanDebounce << 1) | FAN_SW;
		if(fanDebounce == 0x0000) fanSwState = ON;
		else if(fanDebounce == 0xffff) fanSwState = OFF;
	}
}

//Occurs every half second to blink LEDs
ISR(TIMER1_COMPA_vect)
{
	static U8 i = 0;
	U8 pattern = currentBoardStatus & 0x0f;
	LEDs = LEDPatterns[pattern][i];
	LEDB = LEDPatterns[pattern][i++] >> 2;
	i %= 6;
}

ISR(ADC_vect)//Every time an ADC finishes, add data to transistor current array
{
	static U8 index = 0;
	volatile int read = ADCH; //read high byte (top 2 bits of the ADC result)
	transistorCurrent[index] = (read << 8) + ADCL; //read low byte (lower 8 bits) and place entire 10 bit value in array
	index = (index + 1) % SAMPLES; //mod by the length of the array
	if(adcFlag)
	{
		ADMUX = 0x40 | ADC_select[CT_index]; //change source
		adcFlag = 0;
	}
}

void setup(void)
{
	DDRC = 0x73; // PC0, PC1, PC4, PC5, PC6 outputs, rest inputs
	PORTC = 0x80; //pull up resistor on for PC7
	DDRD = 0x02; // PD1 output
	PORTD = 0x01; //PD0 pull up resistor enabled
	DDRB = 0x83; //PB0, PB1, PB7 outputs
	
	//initialize timer 0
	TCNT0 = 0; //timer 0 initialized to 0
	OCR0A = 249; // resets when it reaches 250 (1 ms)
	TCCR0A = 2;  //timer resets rather than overflowing
	TCCR0B = 0b00000011; //use system clock divided by 64, one tick every 4 microseconds assuming 62.5ns system clock intervals
	
	//initialize timer 1
	TCNT1 = 0; //timer 1 initialized to 0
	OCR1A = 31249; //62499 --> 1s
	TCCR1A = 0x00;
	TCCR1B = 0b00001100; //resets instead of overflowing, and system clock divided by 256
	
	//configure interrupts
	TIMSK0 = 1 << OCIE0A; //timer 0 interrupt enabled on compA
	TIMSK1 = 1 << OCIE1A; //set bit 2 to enable interrupt on compA OCIE1A
	asm("sei"); //Global enable interrupts
	
	//Set up ADC
	ADCSRB = 0xa0; //high speed mode, connect AVREF to AVcc
	ADMUX = 0x42; //AVcc reference and ADC2 selected
	ADCSRA = 0xec; //Enable ADC, start conversion, auto conversion, ADC-clock = system/16 (1 MHz)
	DIDR0 = 0xec;
	
	//Initialize CAN...
	can_init(1);
	
}

//_____MAIN______

int main(void)
{
	setup();
	volatile long nextLoop;
	unsigned int current;
	Bool action;
	volatile Bool fuelState;
	volatile Bool fanState;
	
	if(!inTestMode)
	{
		//Initialize all systems
		_12V = ON;
		transistorStates[_12V_index] = ENABLED_OK;
		MOTEC = ON;
		transistorStates[MOTEC_index] = ENABLED_OK;
		EDL = ON;
		transistorStates[EDL_index] = ENABLED_OK;
		FUEL = OFF;
		FAN = OFF;
		
		currentBoardStatus = STANDBY;
		
		//volatile U8 tran; //threshold values pre-processing
		//volatile unsigned long temp;
		//for(tran = 0; tran < 5; tran++)
		//{
			//temp = CS_thresholds[tran];
			//temp = temp * 1024;
			//temp = temp / CS_factors[tran]; // * conversion factor
			//CS_thresholds[tran] = temp;
		//}
		
		while (1)
		{
			nextLoop = TIME + SYSTEM_LOOP_TIME;

			current = avgTC();
			avgTransistorCurrent[CT_index] = current;
			fuelState = transistorStates[FUEL_index]; //remember previous states in case they change
			fanState = transistorStates[FAN_index];
			action = checkTC(current, CS_thresholds[CT_index], transistorStates[CT_index], lastEventTime[CT_index]);
			if(action != transistorStates[CT_index])
			{
				transistorStates[CT_index] = action;
				currentBoardStatus = Min(currentBoardStatus, action);
			}
			MOTEC = transistorStates[MOTEC_index] & (transistorStates[MOTEC_index] >> 1);
			EDL = transistorStates[EDL_index] & (transistorStates[EDL_index] >> 1);
			_12V = transistorStates[_12V_index] & (transistorStates[_12V_index] >> 1);
			
			//If fuel transistor is not in an error state, let the fuel switch determine the state
			if(transistorStates[FUEL_index] & 0b10) //2nd least sig bit = 0 indicates error state
			{
				FUEL = fuelSwState;
				transistorStates[FUEL_index] = 0b10 | fuelSwState;
			}
			else
			{
				FUEL = OFF;
			}
			//If fan transistor is not in an error state, let the fan switch determine the state
			if(transistorStates[FAN_index] & 0b10) //2nd least sig bit = 0 indicates error state
			{
				FAN = fanSwState;
				transistorStates[FAN_index] = 0b10 | fanSwState;
			}
			else
			{
				FAN = OFF;
			}
			
			if(currentBoardStatus == STANDBY && fuelSwState)
			{
				currentBoardStatus = OKAY;
			}
			else if(currentBoardStatus == OKAY && !fuelSwState)
			{
				currentBoardStatus = STANDBY;
			}
			
			if(CT_index == 4) //everytime the current through every transistor has been read
			{
				if(TIME > 100)
				{
					can_get_status(&CAN_message);
				}
				//can_init(1);
				U8 * data = createDATA();
				U8 canStatus = sendCANData(data, &CAN_message);
				if(canStatus != CAN_CMD_ACCEPTED)
				{
					can_init(1);
					sendCANData(data, &CAN_message);
				}
			}
			
			//Change the transistor from which current values should be read
			CT_index = (CT_index + 1) % 5;
			
			if(!fuelSwState && (TIME - lastEventTime[FUEL_index] > SLEEPTIME))
			{
				volatile U8 previousStatus = currentBoardStatus;
				currentBoardStatus = SLEEP;
				_12V = OFF;
				MOTEC = OFF;
				EDL = OFF;
				FUEL = OFF;
				FAN = OFF;
				while(!fuelSwState){asm("nop");} //wait for the fuel switch to turn on
				lastEventTime[FUEL_index] = TIME;
				if(previousStatus & 0x04)
				{
					currentBoardStatus = OKAY;
				}
				else
				{
					currentBoardStatus = previousStatus;
				}
				_12V = ON;
				MOTEC = ON;
				EDL = ON;
				FUEL = ON;
				CT_index = 0;
			}
			adcFlag = 1; //raise flag so the interrupt knows to change ADC source
			
			//If the states of the Fuel or Fan systems have change, update event time
			if(fuelState != transistorStates[FUEL_index])
			{
				lastEventTime[FUEL_index] = TIME;
			}
			if(fanState != transistorStates[FAN_index])
			{
				lastEventTime[FAN_index] = TIME;
			}
			
			while(TIME < nextLoop){asm("nop");} //wait before executing next system loop
		}
	}
	
	currentBoardStatus = TESTMODE;
	U8 errors = test();
	if(!errors)
	{
		currentBoardStatus = TESTSUCCESS;
	}
	else
	{
		currentBoardStatus = TESTERROR;
	}
	while(1)
	{
		FUEL = fuelSwState;
		FAN = fanSwState;
	}
	
	return 0;
}

//____Helper_Methods____
void wait_ms(int ms)
{
	unsigned long until = TIME + ms;
	while(until >= TIME){asm("nop");}
}

unsigned int avgTC()
{
	long avg = 0;
	U8 i;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)//lock interrupts while reading content of array
	{
		for(i = 0; i < SAMPLES; i++)
		{
			avg += transistorCurrent[i];
		}
	}
	avg = avg / SAMPLES;
	return (unsigned int)avg;
}

Bool checkTC(unsigned int value, unsigned int threshold, Bool state, long lastEvent)
{
	if((TIME - lastEvent) > CURRENT_SURGE_TIME)
	{
		//if((state != ENABLED_OK) && (value > TRICKLE_LEVEL)) //10 to allow for slight current levels when the state is off
		//{
			//return DISABLED_E; //error - value is too high for a disabled transistor
		//}
		if(value > threshold)
		{
			return DISABLED_OC; //OC - over current error
		}
	}
	
	return state;
}

U8 sendCANData(U8 * data, st_cmd_t * message)
{
	//static int data = 0b1100101010100011;
	(*message).cmd = CMD_TX_DATA; //Type of CAN message
	(*message).ctrl.ide = 0;      //CAN 2.0A
	(*message).id.std = MY_ID;
	(*message).dlc = 8;           //# of Bytes of data
	(*message).pt_data = data;
	return can_cmd(message);
}

U8 * createDATA(void)
{
	static U8 buffer[8];
	static unsigned long long data;
	U8 * pointer = (U8 *)&data + 7;
	
	U8 i;
	for(i = 0; i < 5; i++)
	{
		data = data << 12;
		data |= (avgTransistorCurrent[i] << 2) | transistorStates[i];
	}
	data = data << 4;
	data |= currentBoardStatus & 0x0f;
	
	for(i = 0; i < 8; i++)
	{
		buffer[i] = *(pointer - i);
	}
	
	return &buffer[0];
}

//____TEST_METHOD____
U8 test(void)
{
	volatile U8 errorCount = 0;
	
	//test something here
	
	return errorCount;
}
