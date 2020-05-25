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

/*
*
* HARDWARE
*
* PB0 - OUT Lo - POWER LED ON
* PB1 - IN PULL UP - ENCODER INPUT A
* PB2 - IN PULL UP - ENCODER INPUT B
* PB3 - IN PULL UP Lo - POSITIVE POWER SUPPLY OK
* PB4 - IN PULL UP Lo - NEGATIVE POWER SUPPLY OK
* PB5 - IN PULL UP Lo - MUTE BUTTON PUSH
*
* PC0 - OUT Hi - ANALOG POWER SUPPLY RELEY
* PC1 - OUT Lo - HEADPHONES RELEY
* PC2 - OUT Lo - MUTE LED ON
* PC3 - OUT Lo - WM8816 MUTE PIN
* PC5 - OUT Lo - YELLOW LED ON
*
* PD2 - IN PULL UP Lo - POWER BUTTON PUSH
* PD3 - IN PULL UP - REMOTE CONTROL
* PD4 - OUT Lo - WM8816 CLOCK (DEFAULT NON INVERTED)
* PD5 - OUT Lo - WM8816 DATA WRITE (DEFAULT NON INVERTED)
* PD6 - OUT Lo - WM8816 CSL (DEFAULT NON INVERTED)
* PD7 - OUT Lo - WM8816 CSL (DEFAULT NON INVERTED)
*
*/

#ifndef _HAMP_
#define _HAMP_

// VOLUME DEFINITIONS
#define VOLUME_STEP 4   //   ±2.0dB - VOLUME CHANGE STEPS
#define VOLUME_MUTE 1   // -115.5dB - VOLUME IN MUTE MODE
#define VOLUME_MIN 32   //  -96.0dB - VOLUME LOWEST SET
#define VOLUME_SAFE 212 //   -6.0dB - VOLUME SAVE LEVEL
#define VOLUME_HIGH 236 //   +6.0dB - VOLUME HIGHEST SET

// EEPROM STORE
#define EEPROM_VOLUME 0
uint8_t EEMEM e8_volume = VOLUME_MIN;

// SWITCH - POWER
#define SWITCH_POWER_P PIND
#define SWITCH_POWER_I (1 << 2)

// SWITCH - MUTE
#define SWITCH_MUTE_P PINB
#define SWITCH_MUTE_I (1 << 5)

// SWITCH - HEADPHONES
#define SWITCH_HEADPHONES_P PIND
#define SWITCH_HEADPHONES_I (1 << 3)

// ANALOG SUPPLY WATCHDOG
#define SUPPLY_WATCHDOG_P PINB
#define SUPPLY_WATCHDOG_I_POS (1 << 3)
#define SUPPLY_WATCHDOG_I_NEG (1 << 4)

// ENCODER
#define ENCODER_P PINB
#define ENCODER_I_A (1 << 1)
#define ENCODER_I_B (1 << 2)

// LED POWER
#define LED_POWER_P PORTB
#define LED_POWER_O (1 << 0)

// LED YELLOW
#define LED_YELLOW_P PORTC
#define LED_YELLOW_O (1 << 5)

//LED MUTE
#define LED_MUTE_P PORTC
#define LED_MUTE_O (1 << 2)

// RELEY ANALOG SUPPLY
#define RELEY_SUPPLY_P PORTC
#define RELEY_SUPPLY_O (1 << 0)

// RELEY HEADPHONES
#define RELEY_HEADPHONES_P PORTC
#define RELEY_HEADPHONES_O (1 << 1)

// WM8816
#define IC_MUTE_P PORTC
#define IC_MUTE_O (1 << 3)

#define IC_CLOCK_P PORTD
#define IC_CLOCK_O (1 << 4)

#define IC_DATA_P PORTD
#define IC_DATA_O (1 << 5)

#define IC_CS_P PORTD
#define IC_CS_O_LEFT (1 << 6)
#define IC_CS_O_RIGHT (1 << 7)

// DECLARATIONS

void func_ic_send(uint16_t data);
uint8_t ee_read_byte(uint8_t addr);
void ee_write_byte(uint8_t addr, uint8_t data);
static inline void asm_wait_us(uint16_t count);
void wait_ms(uint16_t time);

#define wait_us(us) asm_wait_us((uint16_t)(((((us)*1000L) / (1000000000 / F_CPU)) - 1) / 4))

// WM8816 MACROS
#ifndef WM8816_INVERT

// Lo ON ATMEL = Lo ON WM8816
#define IC_MUTE_Lo IC_MUTE_P &= ~IC_MUTE_O;
#define IC_MUTE_Hi IC_MUTE_P |= IC_MUTE_O;
#define IC_CLOCK_Lo IC_CLOCK_P &= ~IC_CLOCK_O;
#define IC_CLOCK_Hi IC_CLOCK_P |= IC_CLOCK_O;
#define IC_DATA_Lo IC_DATA_P &= ~IC_DATA_O;
#define IC_DATA_Hi IC_DATA_P |= IC_DATA_O;
#define IC_CSL_Lo IC_CS_P &= ~IC_CS_O_LEFT;
#define IC_CSL_Hi IC_CS_P |= IC_CS_O_LEFT;
#define IC_CSR_Lo IC_CS_P &= ~IC_CS_O_RIGHT;
#define IC_CSR_Hi IC_CS_P |= IC_CS_O_RIGHT;

#else

// Lo ON ATMEL = Hi ON WM8816
#define IC_MUTE_Lo IC_MUTE_P |= IC_MUTE_O;
#define IC_MUTE_Hi IC_MUTE_P &= ~IC_MUTE_O;
#define IC_CLOCK_Lo IC_CLOCK_P |= IC_CLOCK_O;
#define IC_CLOCK_Hi IC_CLOCK_P &= ~IC_CLOCK_O;
#define IC_DATA_Lo IC_DATA_P |= IC_DATA_O;
#define IC_DATA_Hi IC_DATA_P &= ~IC_DATA_O;
#define IC_CSL_Lo IC_CS_P |= IC_CS_O_LEFT;
#define IC_CSL_Hi IC_CS_P &= ~IC_CS_O_LEFT;
#define IC_CSR_Lo IC_CS_P |= IC_CS_O_RIGHT;
#define IC_CSR_Hi IC_CS_P &= ~IC_CS_O_RIGHT;

#endif

#endif
