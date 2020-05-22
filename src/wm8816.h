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

#ifndef _MODULE_IC_WM8816_H_
	#define _MODULE_IC_WM8816_H_

	#define IC_WM8816_CLOCK_TIME											20

	#define IC_WM8816_RIGHT_CHANNEL_GAIN_READ					0b11110111
	#define IC_WM8816_RIGHT_CHANNEL_GAIN_WRITE				0b11110011

	#define IC_WM8816_LEFT_CHANNEL_GAIN_READ					0b11101111
	#define IC_WM8816_LEFT_CHANNEL_GAIN_WRITE					0b11101011


	#define IC_WM8816_PEAK_DETECTOR_REFERENCE_READ		0b11100111
	#define IC_WM8816_PEAK_DETECTOR_REFERENCE_WRITE		0b11100011

	#define IC_WM8816_PEAK_DETECTOR_STATUS_READ				0b11011111
	#define IC_WM8816_PEAK_DETECTOR_STATUS_WRITE			0b11011011

	#define IC_WM8816_BOTH_CHANNEL_GAINS_WRITE				0b11001011

	#define IC_WM8816_TEST_READ												0b11111111
	#define IC_WM8816_TEST_WRITE											0b11111011
	
#endif
