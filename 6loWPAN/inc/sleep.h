/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/
/*
  $Id: sleep.h,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

#ifndef SLEEP_H
#define SLEEP_H

void nodeSleep(u16 seconds);

/**
   @brief Set this parameter to use the watchdog timer rather than the
   RTC oscillator for sleep timing.  Setting WDOG_SLEEP to zero
   selects the RTC oscillator running at 32KHz.  Setting WDOG_SLEEP to
   one selects the AVR's watchdog timer for timing sleep mode.

   @ingroup sleep
*/
#ifndef WDOG_SLEEP
#define WDOG_SLEEP 0
#endif

// Define some fake registers so that the compiler has a valid symbol to use
// (and then optimize away)
#if PLATFORM == SPITFIRE || PLATFORM == DSK001
#define PRR0  (*(volatile uint8_t *)(0x64))
#define PRR1  (*(volatile uint8_t *)(0x64))
#define DDRA  (*(volatile uint8_t *)(0x64))
#define PORTA (*(volatile uint8_t *)(0x64))
#else
#define PRR   (*(volatile uint8_t *)(0x64))
#endif


#endif // SLEEP_H
