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

#ifndef _REMOTE_
#define _REMOTE_

#ifndef RC
#define RC_POWER_TOGGLE 0b0000000000001100 // RC6 REMOTE CONTROL - POWER
#define RC_POWER_ON 0b0000000001101110     // RC6 REMOTE CONTROL - GREEN
#define RC_POWER_OFF 0b0000000001101101    // RC6 REMOTE CONTROL - RED
#define RC_VOLUME_UP 0b0000000000010000    // RC6 REMOTE CONTROL - VOLUME UP
#define RC_VOLUME_DOWN 0b0000000000010001  // RC6 REMOTE CONTROL - VOLUME DOWN
#define RC_MUTE 0b0000000000001101         // RC6 REMOTE CONTROL - MUTE
#endif

#endif