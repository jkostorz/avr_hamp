/*
*		BSD 3-Clause License
*
* 	©2020 Janusz Kostorz
* 	All rights reserved.
*
* 	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
*		1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
*		2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
*		3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
*
* 	THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* 	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* 	End of BSD license
*
*/

//#define WM8816_INVERT

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include "hamp.h"
#include "remote.h"
#include "wm8816.h"

// REMOTE CONTROL GLOBAL VARIABLE
uint16_t remotecontrol = 0;

// MAIN
int16_t main(void)
{

	// --- STANDBY MODE ---

	// WATCHDOG SETUP TO 15MS
	wdt_reset();
	wdt_enable(WDTO_15MS);

	// PORTS SETUP - PULLUP
	PORTB = ENCODER_I_A | ENCODER_I_B | SUPPLY_WATCHDOG_I_POS | SUPPLY_WATCHDOG_I_NEG | SWITCH_MUTE_I;
	PORTD = SWITCH_POWER_I;

	// PORTS SETUP - OUTPUTS
	DDRB = LED_POWER_O;
	DDRC = RELAY_SUPPLY_O | RELAY_HEADPHONES_O | LED_MUTE_O | IC_MUTE_O | LED_YELLOW_O;
	DDRD = IC_CLOCK_O | IC_DATA_O | IC_CS_O_LEFT | IC_CS_O_RIGHT;

	// PORTS SETUP - LOGIC HI FOR ACTIVE LOW
	PORTB |= LED_POWER_O;
	PORTC |= RELAY_HEADPHONES_O | LED_MUTE_O | LED_YELLOW_O;

	// DEFINE UNUSED PORTS - ATMEGA328P DATASHEET RECOMMENDATION
	PORTC |= (1 << 4);
	PORTD |= (1 << 0) | (1 << 1);

	// DISABLE UNUSED PERIPHERALS - ATMEGA328P DATASHEET RECOMMENDATION
	ACSR |= (1 << ACD);
	PRR |= (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRUSART0) | (1 << PRADC);

	// WAIT IF POWER BUTTON IS PRESSED
	while (!(SWITCH_POWER_P & SWITCH_POWER_I))
		wdt_reset();

	// SETUP INTERRUPTS FOR REMOTE CONTROL
	EICRA |= 1 << ISC11;
	EIMSK |= 1 << INT1;
	sei();

	// STANDBY LOOP
	uint8_t pwm = 0;
	while (1)
	{

		// POWER LED PWM
		pwm = pwm == 19 ? 0 : pwm + 1;
		if (pwm)
			LED_POWER_P |= LED_POWER_O;
		else
			LED_POWER_P &= ~LED_POWER_O;

		// CHECK FOR PRESS POWER BUTTON
		if (!(SWITCH_POWER_P & SWITCH_POWER_I))
			break;

		// WATCHDOG RESET
		wdt_reset();

// DEBUG MODE - CHECK WORKING INFRARED RECEIVER
#ifdef _DEBUGMODE_IR_

		// CHECK FOR NEW REMOTE CONTROL CODE
		if (remotecontrol)
		{

			// DISPLAY BITS
			uint16_t i = 0;
			for (i = 0; i < 16; i++)
			{

				// TURN OFF LEDS
				LED_MUTE_P |= LED_MUTE_O;
				LED_YELLOW_P |= LED_YELLOW_O;
				wait_ms(440);

				// PAUSE BETWEEN ADDRESS AND COMMAND
				if (i == 8)
					wait_ms(1000);

				// Lo ON YELLOW LED Hi ON MUTE LED
				if (remotecontrol & 0b1000000000000000)
					LED_MUTE_P &= ~LED_MUTE_O;
				else
					LED_YELLOW_P &= ~LED_YELLOW_O;

				remotecontrol = remotecontrol << 1;

				wait_ms(60);
			}

			// TURN OFF LEDS
			LED_MUTE_P |= LED_MUTE_O;
			LED_YELLOW_P |= LED_YELLOW_O;

			// PAUSE AFTER END DISPLAY
			wait_ms(1000);

			// NEXT CODE ACCEPTED
			remotecontrol = 0;
		}

#endif

		// CHECK FOR PRESS POWER BUTTON IN REMOTE CONTROL
		if (remotecontrol == RC_POWER_TOGGLE || remotecontrol == RC_POWER_ON)
		{
			wait_ms(300);
			remotecontrol = 0;
			break;
		}

		// CLEAR REMOTE CONTROL VARIABLE
		if (remotecontrol)
			remotecontrol = 0;
	}

	// --- STARTUP MODE ---

	// TURN ON POWER LED
	LED_POWER_P &= ~LED_POWER_O;

	// TURN ON MUTE LED
	LED_MUTE_P &= ~LED_MUTE_O;

	// WAIT FOR BUTTONS RELEASE
	while (!(SWITCH_POWER_P & SWITCH_POWER_I) || !(SWITCH_MUTE_P & SWITCH_MUTE_I))
		wdt_reset();

	// SET LOGIC LOW ON WM8816 DIGITAL PINS BEFORE POWER ON ANALOG PART - DATASHEET RECOMMENDATION
	IC_MUTE_Lo;
	IC_CLOCK_Lo;
	IC_DATA_Lo;
	IC_CSL_Lo;
	IC_CSR_Lo;

	// POWER ON TO ANALOG PART
	RELAY_SUPPLY_P |= RELAY_SUPPLY_O;

	// WAIT TO STABILIZE VOLTAGE
	wait_ms(1000);

	// CHECK FOR POSITIVE AND NEGATIVE VOLTAGE
	if ((SUPPLY_WATCHDOG_P & SUPPLY_WATCHDOG_I_POS) | (SUPPLY_WATCHDOG_P & SUPPLY_WATCHDOG_I_NEG))
	{

		// ERROR - HIGH RISK OF EQUIPMENT DAMAGE

		// POWER OFF FOR ANALOG PART
		RELAY_SUPPLY_P &= ~RELAY_SUPPLY_O;

		// TURN OFF POWER LED
		LED_POWER_P |= LED_POWER_O;

		// TURN OFF MUTE LED
		LED_MUTE_P |= LED_MUTE_O;

		// TURN ON YELLOW LED
		LED_YELLOW_P &= ~LED_YELLOW_O;

		// WAIT FOR POWER PLUG REMOVE - NEVER ENDING LOOP
		while (1)
			wdt_reset();
	}

	// CHECK HEADPHONE CONNECT OTHERWISE GO TO STANDBY
	if (!(SWITCH_HEADPHONES_P & SWITCH_HEADPHONES_I))
	{

		// POWER OFF FOR ANALOG PART
		RELAY_SUPPLY_P &= ~RELAY_SUPPLY_O;

		// WAIT FOR DISCHARGE CAPACITORS
		wait_ms(2000);

		// GO TO STANDBY
		while (1)
			;
	}

	// SET LOGIC HI ON DIGITAL PINS OF WM8816 AFTER POWER UP
	IC_CLOCK_Hi;
	IC_DATA_Hi;
	IC_CSL_Hi;
	IC_CSR_Hi;

	// LOAD SAVED VOLUME LEVEL
	uint8_t u8_volume_ee = ee_read_byte(EEPROM_VOLUME);

	// CHECK IF VOLUME LOADED FROM EEPROM IS SAFE FOR USER
	if (u8_volume_ee > VOLUME_SAFE)
		u8_volume_ee = VOLUME_SAFE;
	if (u8_volume_ee < VOLUME_MIN)
		u8_volume_ee = VOLUME_MIN;

	// TURN OFF MUTE LED
	LED_MUTE_P |= LED_MUTE_O;

	// UNMUTE WM8816
	IC_MUTE_Hi;

	// WRITE COMMAND TO WM8816 - SET VOLUME TO VOLUME_MIN
	func_ic_send((IC_WM8816_BOTH_CHANNEL_GAINS_WRITE << 8) | VOLUME_MIN);

	// CONNECT HEADPHONES
	RELAY_HEADPHONES_P &= ~RELAY_HEADPHONES_O;

	// SMOOTH SETUP VOLUME
	uint8_t u8_volume = VOLUME_MIN;
	while (u8_volume < u8_volume_ee)
	{

		// SPEED OF  VOLUME CHANGE
		if (u8_volume < 128 && u8_volume < u8_volume_ee)
			u8_volume++;
		if (u8_volume < 64 && u8_volume < u8_volume_ee)
			u8_volume++;
		if (u8_volume < 32 && u8_volume < u8_volume_ee)
			u8_volume++;
		if (u8_volume < 16 && u8_volume < u8_volume_ee)
			u8_volume++;
		if (u8_volume < 8 && u8_volume < u8_volume_ee)
			u8_volume++;
		if (u8_volume < 4 && u8_volume < u8_volume_ee)
			u8_volume++;
		u8_volume++;

		// WRITE COMMAND TO WM8816
		func_ic_send((IC_WM8816_BOTH_CHANNEL_GAINS_WRITE << 8) | u8_volume);

		// SLOWDOWN VOLUME RAMP
		wait_ms(2);
	}

	// INIT ENCODER STATE
	uint8_t u8_encoder = (((ENCODER_P & ENCODER_I_B) | (ENCODER_P & ENCODER_I_A)) >> 1);

	// VALUE TO RESTORE WHEN UNMUTE
	uint8_t u8_volume_mute = 0;

	// --- LISTENING MODE ---

	// MAIN LOOP
	while (1)
	{

		// WATCHDOG RESET
		wdt_reset();

		// CHECK FOR POSITIVE AND NEGATIVE VOLTAGE
		if ((SUPPLY_WATCHDOG_P & SUPPLY_WATCHDOG_I_POS) | (SUPPLY_WATCHDOG_P & SUPPLY_WATCHDOG_I_NEG))
		{

			// ERROR - HIGH RISK OF EQUIPMENT DAMAGE

			// DISCONNECT HEADPHONES
			RELAY_HEADPHONES_P |= RELAY_HEADPHONES_O;

			// POWER OFF FOR ANALOG PART
			RELAY_SUPPLY_P &= ~RELAY_SUPPLY_O;

			// TURN OFF POWER LED
			LED_POWER_P |= LED_POWER_O;

			// TURN OFF MUTE LED
			LED_MUTE_P |= LED_MUTE_O;

			// RURN ON YELLOW LED
			LED_YELLOW_P &= ~LED_YELLOW_O;

			// WAIT FOR REMOVE POWER PLUG - NEVER ENDING LOOP
			while (1)
				wdt_reset();
		}

		// CHECK MUTE BUTTON PUSH
		if (!(SWITCH_MUTE_P & SWITCH_MUTE_I) && !u8_volume_mute)
		{

			// SAVE VOLUME AND SET MUTE
			u8_volume_mute = u8_volume;
			u8_volume = VOLUME_MIN;
			LED_MUTE_P &= ~LED_MUTE_O;
			LED_YELLOW_P |= LED_YELLOW_O;

			// UPDATE WM8816 GAIN REGISTER
			func_ic_send((IC_WM8816_BOTH_CHANNEL_GAINS_WRITE << 8) | VOLUME_MIN);
		}

		// CHANGE VOLUME - CHECK ENCODER MOVE
		if ((u8_encoder & 0b11) != ((ENCODER_P & ENCODER_I_B) | (ENCODER_P & ENCODER_I_A)) >> 1)
		{

			// MOVE PREVIOUS STATE BITS LEFT AND ADD NEW STATE
			u8_encoder = (u8_encoder << 2) | (((ENCODER_P & ENCODER_I_B) | (ENCODER_P & ENCODER_I_A)) >> 1);

			// CHECK ENCODER MOVE RIGHT - HH HL LL LH HH... ( PIN B | PIN A )
			if (u8_volume < VOLUME_HIGH && u8_encoder == 0b10000111)
			{
				if (u8_volume > VOLUME_HIGH - VOLUME_STEP)
					u8_volume = VOLUME_HIGH;
				u8_volume += VOLUME_STEP;
			}

			// CHECK ENCODER MOVE LEFT - HH LH LL HL HH...
			if (u8_volume > VOLUME_MIN && u8_encoder == 0b01001011)
			{
				if (u8_volume < VOLUME_MIN + VOLUME_STEP)
					u8_volume = VOLUME_MIN;
				u8_volume -= VOLUME_STEP;
			}

			// RESTORE VOLUME IF UNMUTED
			if (u8_volume_mute)
			{
				u8_volume = u8_volume_mute > VOLUME_SAFE ? VOLUME_SAFE : u8_volume_mute;
				u8_volume_mute = 0;
				LED_MUTE_P |= LED_MUTE_O;
			}

			// UPDATE WM8816 GAIN REGISTER
			func_ic_send((IC_WM8816_BOTH_CHANNEL_GAINS_WRITE << 8) | u8_volume);

			// TURN ON YELLOW LED IF VOLUME IS UPPER VOLUME_SAFE
			if (u8_volume > VOLUME_SAFE && !u8_volume_mute)
				LED_YELLOW_P &= ~LED_YELLOW_O;
			else
				LED_YELLOW_P |= LED_YELLOW_O;
		}

		// CHECK REMOTE CONTROL CODES - MUTE BUTTON
		if (u8_volume_mute == 0 && remotecontrol == RC_MUTE)
		{

			// SAVE VOLUME AND SET MUTE
			u8_volume_mute = u8_volume;
			u8_volume = VOLUME_MIN;
			LED_MUTE_P &= ~LED_MUTE_O;
			LED_YELLOW_P |= LED_YELLOW_O;

			// UPDATE WM8816 GAIN REGISTER
			func_ic_send((IC_WM8816_BOTH_CHANNEL_GAINS_WRITE << 8) | VOLUME_MIN);
		}

		// CHECK REMOTE CONTROL CODES - VOLUME UP
		if (remotecontrol == RC_VOLUME_UP)
		{
			if (u8_volume < VOLUME_HIGH)
				u8_volume += 1;

			// RESTORE VOLUME IF UNMUTED
			if (u8_volume_mute)
			{
				u8_volume = u8_volume_mute > VOLUME_SAFE ? VOLUME_SAFE : u8_volume_mute;
				u8_volume_mute = 0;
				LED_MUTE_P |= LED_MUTE_O;
			}

			// UPDATE WM8816 GAIN REGISTER
			func_ic_send((IC_WM8816_BOTH_CHANNEL_GAINS_WRITE << 8) | u8_volume);

			// TURN ON YELLOW LED IF VOLUME IS UPPER VOLUME_SAFE
			if (u8_volume > VOLUME_SAFE && !u8_volume_mute)
				LED_YELLOW_P &= ~LED_YELLOW_O;
			else
				LED_YELLOW_P |= LED_YELLOW_O;
		}

		// CHECK REMOTE CONTROL CODES - VOLUME DOWN
		if (remotecontrol == RC_VOLUME_DOWN)
		{
			if (u8_volume > VOLUME_MIN)
				u8_volume -= 1;

			// RESTORE VOLUME IF UNMUTED
			if (u8_volume_mute)
			{
				u8_volume = u8_volume_mute > VOLUME_SAFE ? VOLUME_SAFE : u8_volume_mute;
				u8_volume_mute = 0;
				LED_MUTE_P |= LED_MUTE_O;
			}

			// UPDATE WM8816 GAIN REGISTER
			func_ic_send((IC_WM8816_BOTH_CHANNEL_GAINS_WRITE << 8) | u8_volume);

			// TURN ON YELLOW LED IF VOLUME IS UPPER VOLUME_SAFE
			if (u8_volume > VOLUME_SAFE && !u8_volume_mute)
				LED_YELLOW_P &= ~LED_YELLOW_O;
			else
				LED_YELLOW_P |= LED_YELLOW_O;
		}

// DEBUG MODE - CHECK ENCODER WORKING
#ifdef _DEBUGMODE_ENCODER_

		// INPUT A ON LED YELLOW
		if (ENCODER_P & ENCODER_I_A)
			LED_YELLOW_P &= ~LED_YELLOW_O;
		else
			LED_YELLOW_P |= LED_YELLOW_O;

		// INPUT B ON LED MUTE
		if (ENCODER_P & ENCODER_I_B)
			LED_MUTE_P &= ~LED_MUTE_O;
		else
			LED_MUTE_P |= LED_MUTE_O;

#endif

		// CHECK FOR PRESS POWER BUTTON
		if (!(SWITCH_POWER_P & SWITCH_POWER_I) || remotecontrol == RC_POWER_TOGGLE || remotecontrol == RC_POWER_OFF)
		{

			// --- GO TO STANDBY MODE ---

			// TURN OFF POWER LED
			LED_POWER_P |= LED_POWER_O;

			// TURN OFF YELLOW LED
			LED_YELLOW_P |= LED_YELLOW_O;

			// TURN ON MUTE LED
			LED_MUTE_P &= ~LED_MUTE_O;

			// WM8816 MUTE
			IC_MUTE_Lo;

			// DISCONNECT HEADPHONES
			RELAY_HEADPHONES_P |= RELAY_HEADPHONES_O;

			// WAIT FOR RELAY
			wait_ms(20);

			// POWER OFF ANALOG PART
			RELAY_SUPPLY_P &= ~RELAY_SUPPLY_O;

			// SAVE VOLUME TO EEPROM
			if (u8_volume_mute)
				u8_volume = u8_volume_mute;
			ee_write_byte(EEPROM_VOLUME, u8_volume);

			// WAIT FOR CAPACITORS DISCHARGE
			wait_ms(2000);

			// GO TO STANDBY
			while (1)
				;
		}

		// CLEAR REMOTE CONTROL VARIABLE
		if (remotecontrol)
			remotecontrol = 0;
	}
}

// SEND DATA TO WM8816
void func_ic_send(uint16_t data)
{

	// FOR READ BYTE FROM CHIP ALL DATA BITS MUST BE SET TO LOGIC HI
	if (data & 0b0000010000000000)
		data |= 0b11111111;

	// PULL DOWN CLOCK FOR LONG TIME BEFORE TRANSMISSION
	IC_CLOCK_Lo;
	wait_us(IC_WM8816_CLOCK_TIME * 2);
	IC_CLOCK_Hi;

	// CHIP SELECT IN THE MIDDLE OF CLOCK HI
	wait_us(IC_WM8816_CLOCK_TIME / 2);
	IC_CSL_Lo;
	IC_CSR_Lo;
	wait_us(IC_WM8816_CLOCK_TIME / 2);

	// CLOCK CYCLE LO + SET BIT A7 + CLOCK CYCLE HI
	if (data & 0b1000000000000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT A6 + CLOCK CYCLE HI
	if (data & 0b0100000000000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT A5 + CLOCK CYCLE HI
	if (data & 0b0010000000000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT A4 + CLOCK CYCLE HI
	if (data & 0b0001000000000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT A3 + CLOCK CYCLE HI
	if (data & 0b0000100000000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT A2 + CLOCK CYCLE HI
	if (data & 0b0000010000000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT A1 + CLOCK CYCLE HI
	if (data & 0b0000001000000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT A0 + CLOCK CYCLE HI
	if (data & 0b0000000100000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT D7 + CLOCK CYCLE HI
	if (data & 0b0000000010000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT D6 + CLOCK CYCLE HI
	if (data & 0b0000000001000000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT D5 + CLOCK CYCLE HI
	if (data & 0b0000000000100000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT D4 + CLOCK CYCLE HI
	if (data & 0b0000000000010000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT D3 + CLOCK CYCLE HI
	if (data & 0b0000000000001000)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT D2 + CLOCK CYCLE HI
	if (data & 0b0000000000000100)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT D1 + CLOCK CYCLE HI
	if (data & 0b0000000000000010)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + SET BIT D0 + CLOCK CYCLE HI
	if (data & 0b0000000000000001)
	{
		IC_CLOCK_Lo;
		IC_DATA_Hi;
	}
	else
	{
		IC_CLOCK_Lo;
		IC_DATA_Lo;
	}
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);

	// CLOCK CYCLE LO + CHIP UNSELECT + CLOCK CYCLE HI
	IC_CLOCK_Lo;
	IC_CSL_Hi;
	IC_CSR_Hi;
	wait_us(IC_WM8816_CLOCK_TIME);
	IC_CLOCK_Hi;

	// TRNASMISION END
	wait_us(IC_WM8816_CLOCK_TIME * 2);

	// SET DATA PIN LOGIC HI
	IC_DATA_Hi;
}

// EEPROM READ BYTE
uint8_t ee_read_byte(uint8_t addr)
{

	while (EECR & (1 << EEPE))
		wdt_reset();
	EEAR = addr;
	EECR |= (1 << EERE);
	return EEDR;
}

// EEPROM WRITE BYTE
void ee_write_byte(uint8_t addr, uint8_t data)
{

	while (EECR & (1 << EEPE))
		wdt_reset();
	EEAR = addr;
	EEDR = data;
	EECR |= (1 << EEMPE);
	EECR |= (1 << EEPE);
}

// DELAY FUNCTIONS

static inline void asm_wait_us(uint16_t count)
{
	asm volatile("cp %A0, __zero_reg__ \n\t"
							 "cpc %B0, __zero_reg__ \n\t"
							 "breq loop_out_%= \n\t"
							 "loop%=: \n\t"
							 "sbiw %0, 1 \n\t"
							 "brne loop%= \n\t"
							 "loop_out_%=: \n\t"
							 : "=w"(count)
							 : "0"(count));
}

void wait_ms(uint16_t time)
{
	while (time--)
	{
		wait_us(1000);
#if defined(_AVR_WDT_H_)
		wdt_reset();
#endif
	}
}

// EXTERNAL INTERRUPTION INT1
ISR(INT1_vect)
{

	// TURN OFF INTERRUPTS
	cli();

	// CHECK IF PREVIOUS COMMAND IS DONE
	if (!(remotecontrol))
	{

		// TURN OFF POWER LED
		LED_POWER_P |= LED_POWER_O;

		// WAIT FOR RC6 START BIT + MODE BIT + MODE BIT + MODE BIT + TOGGLE BIT + THREE FOURTH OF FIRST ADRESS BIT ___--_-_-_--__
		wait_us((RC6_PULSE_TIME * 5) + RC6_PULSE_TIME + RC6_PULSE_TIME + RC6_PULSE_TIME + (RC6_PULSE_TIME * 2) + (RC6_PULSE_TIME * 3 / 4));

		//READ ADDRESS AND COMMAND
		uint8_t i;
		for (i = 0; i < 16; i++)
		{
			remotecontrol <<= 1;

			// CHECK STATE ON THREE FOURTH OF BIT TIME
			if (REMOTE_P & REMOTE_I)
				remotecontrol |= 1;

			// WAIT FOR NEXT BIT
			wait_us(RC6_PULSE_TIME);

			// WATCHDOG RESET
			wdt_reset();
		}
	}

	// WATCHDOG RESET
	wdt_reset();

	//TURN ON POWER LED
	LED_POWER_P &= ~LED_POWER_O;

	sei();
}
