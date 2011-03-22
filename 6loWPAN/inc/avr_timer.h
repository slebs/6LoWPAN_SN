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
  $Id: avr_timer.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
*/
#ifndef AVRTIMERH
#define AVRTIMERH

#include "../inc/system.h"

void timerInit(void);

/**
   @return the time, in mS. Note this time is in the range
   0 - 65535 milliseconds, and the time overflows every
   65 seconds!
*/
u16 macGetTime(void);

/**
   @brief Sets a timeout value, used for sixlowpan sleeping.
*/
void macSetTimeout(u16 timeout);

u8 macSetAlarm(u16 microseconds, void(*callback)(void));

void macTimerEnd(u8 timerID);
/**
   @addtogroup timer_module
   @{
*/

u8 macSetLongAlarm(u16 seconds, void(*callback)(void));


/**
   Microseconds per tick of the timer. Adjust if the timer settings
   change.  Note that this number must divide cleanly into 1000 for
   macSetAlarm to work.
*/
#define MS_PER_TICK (1)


/**
   @name Tick Timer Macros

   For the macAlarm timer.
   @{
*/
/// Sets timer Prescaler to 8, sets timer output compare register init
/// the timer to run all the time.
#define TIMER_INIT() TCCR(TICKTIMER,B) |= (1 << CS(TICKTIMER,1)) |  \
        (1 << WGM(TICKTIMER,2)), OCR(TICKTIMER,A) = (MS_PER_TICK * 1000 /\
                                                     (8000000UL/F_CPU))
/// Stops the timer for sleep
//#define TIMER_STOP() (TCCR(TICKTIMER,B) = 0)
/// Clear the timer count to zero
#define TIMER_CLEAR() (TCNT(TICKTIMER) = 0)
/// Enable the timer to run
#define TIMER_ENABLE() (TIMSK(TICKTIMER) = (1 << OCIE(TICKTIMER,A)))
/// Stop the timer
#define TIMER_STOP() (TIMSK(TICKTIMER) &= ~(1 << OCIE(TICKTIMER,A)))
/// Tick timer ISR vector function
#define TICKVECT COMPVECT(TICKTIMER)   // IAR doesn't accept this macro inside the ISR() macro
/** @} */

/** @} */

#endif
