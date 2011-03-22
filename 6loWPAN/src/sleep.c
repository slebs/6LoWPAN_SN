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
  $Id: sleep.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
*/
#include <stdio.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "../inc/mac.h"
#include "../inc/system.h"
#include "../inc/sleep.h"
#include "../inc/radio.h"
#include "../inc/serial.h"


#if defined(DOXYGEN)
#redefine RUMSLEEP 1
#endif

#if ((NODETYPE == ENDDEVICE) && (RUMSLEEP == 1) || DOXYGEN)

/**
   @defgroup sleep Sleep functions

   Function to support the sleeping of the node.  Currently, this code
   only runs on the end nodes, as part of the sensor application.  It
   can be called at any time to put the node to sleep, however.

   There is one paramater to note - the @ref WDOG_SLEEP flag, which,
   when set, causes the sleep timing to be driven from the watchdog
   timer rather than the low-power AVR oscillator.

   If the low-power oscillator is used (WDOG_SLEEP = 0), then the
   oscillator must be driven from an on-board 32.768 KHz crystal.

   @{
*/

void twiInit(void);

/**
   @brief Put the node to sleep.  This function must put all the parts of the
   hardware to sleep, and wake them up.

   @param tenthSeconds The time to sleep in tenths of a second.

   @ingroup sleep
*/
void nodeSleep(u16 tenthSeconds)
{
    // Check to see if we really should sleep
    if (!macConfig.sleeping)
        // Just return, rather than sleeping
        return;

    // ************** Power down the other board peripherals
    LED_OFF(1);

    // ************** Power down the radio
    // wait for radio to be done
    u8 state = BUSY_TX_ARET;
    while (state == BUSY_TX_ARET ||
           state == BUSY_RX_AACK)
        state = radioGetTrxState();

    // Now put radio to sleep
    radioEnterSleepMode();

    // ************** Power down the on-chip modules
    if (PLATFORM == SPITFIRE)
        PRR = 0xbf;
    else if (PLATFORM != DSK001)
    {
        // Disable ADC
        ADCSRA &= ~(1 << ADEN);
        // Turn off comparator
        ACSR |= (1 << ACD);

        PRR0 = 0xaf;  // shutdown all but timer2
        PRR1 = 0xff;  // shutdown all of these peripherals
    }

    // For Raven, turn off more stuff
    if (PLATFORM == RAVEN)
    {
        // Turn off GPIO's
        DDRA  = 0;
        PORTA = 0xff;
        // PB0 = TST, leave
        // PB1 = RST, leave
        // PB2 = unused, in high
        // PB3 = slptr, leave
        // PB4 = sel, leave
        // PB5 - PB7, leave
        PORTB |= 1 << 2;

        // Put 24C02 in standby:
        DDRC = 0;
        PORTC = 0xff;
        //        DDRD &= ~0xbf;
        PORTD |= 0xbf; // All but PD6, which is IRQ from radio

        // Turn off BOD
        AVR_ENTER_CRITICAL_REGION();
#define BODS  6
#define BODSE 5
        MCUCR  |= (1 << BODSE) | (1<< BODS);
        MCUCR  &= ~(1 << BODSE);
        AVR_LEAVE_CRITICAL_REGION();
    }

    // Turn off all extra ports on SPITFIRE
    if (PLATFORM == SPITFIRE)
    {
        // Pull open pins high
        PORTC |= 0x0f;
        PORTD |= 0x47;
    }

    if (PLATFORM == DSK001)
    {
        // Turn off accelerometer
        DDRC  |=  0x0f;
        PORTC &= ~0x0f;
        // Make all pins zero or one
        DDRB |= 1;
        PORTB &= ~1;
        DDRD = 0xff;
        PORTD = 0x80;
        // Turn off BOD
        AVR_ENTER_CRITICAL_REGION();
        MCUCR  |=  (1 << BODSE) | (1 << BODS);
        MCUCR  |=  (1 << BODS);
        MCUCR  &= ~(1 << BODSE);
        AVR_LEAVE_CRITICAL_REGION();

        // Turn off ADC
        if (ADC_ENABLED)
        {
            //            HAL_WAIT_ADC();
            HAL_STOP_ADC();
        }

        // Turn off all modules, except timer2
        PRR = 0xbf;
    }


    // ************** Set the timer to wake up
    if (!WDOG_SLEEP)
    {
        // Set TIMER2 Asyncronous Mode.
        ASSR |= (1 << AS2);
        // Set TIMER2 Prescaler to 1024.
        TCCR2B |= ((1 << CS22)|(1 << CS21)|(1 << CS20));
        // Wait for TCNT2 write to finish.
        while(ASSR & (1 << TCR2BUB))
            ;
    }

    // Sleep as many times as needed to sleep for the full time
    while (tenthSeconds)
    {
        if (WDOG_SLEEP)
        {
            u8 time;
            // Set Watchdog timer for max time left
            if (tenthSeconds >= 80)
            {
                // 8 seconds
                time = WDTO_8S;
                tenthSeconds -= 80;
            }
            else if (tenthSeconds >= 40)
            {
                // 4 seconds
                time = WDTO_4S;
                tenthSeconds -= 40;
            }
            else if (tenthSeconds >= 20)
            {
                // 2 seconds
                time = WDTO_2S;
                tenthSeconds -= 20;
            }
            else if (tenthSeconds >= 10)
            {
                // 1 seconds
                time = WDTO_1S;
                tenthSeconds -= 10;
            }
            else if (tenthSeconds >= 5)
            {
                // 0.5 seconds
                time = WDTO_500MS;
                tenthSeconds -= 5;
            }
            else if (tenthSeconds >= 2)
            {
                // 0.25 seconds
                time = WDTO_250MS;
                tenthSeconds -= 2;
            }
            else
            {
                // 0.1 seconds (or so) left
                time = WDTO_120MS;
                tenthSeconds = 0;
            }

            wdt_reset();
            wdt_enable(time);

            MCUSR = 0;// &= ~(1 << WDRF);
            AVR_ENTER_CRITICAL_REGION();
            WDTCSR |= (1 << WDCE) | (1 << WDE);
            WDTCSR |= 1 << WDIF;
            WDTCSR |= (1 << WDCE) | (1 << WDE);
            WDTCSR |= 1 << WDIE;
            WDTCSR |= (1 << WDCE) | (1 << WDE);
            WDTCSR &= ~(1 << WDE);
            wdt_reset();
            AVR_LEAVE_CRITICAL_REGION();
            MCUSR = 0;
        }
        else  // No WDOG, use 32KHz timer
        {
            // Set TIMER2 output compare register from user.
            if (tenthSeconds > 75)
            {
                // Just decrement by the max timeout
                OCR2A = 240; // 7.5 seconds, max timeout
                tenthSeconds -= 75;
            }
            else
            {
                // Can measure the remaining time in one timer cycle

                tenthSeconds = tenthSeconds * 16 / 5;
                if (!tenthSeconds)
                    tenthSeconds++;
                OCR2A = tenthSeconds;
                tenthSeconds = 0;
            }
            // Wait for OCR2 write to finish.
            while(ASSR & (1 << OCR2AUB))
                ;
            // Reset TIMER2 timer counter value.
            TCNT2 = 0;
            // Wait for TCNT2 write to finish before entering sleep.
            while(ASSR & (1 << TCN2UB))
                ;

            // Clear interrupt flag
            TIFR2 |= (1 << OCF2A);
            // Enable TIMER2 output compare interrupt.
            TIMSK2 |= (1 << OCIE2A);
        }

        // ************** Go to sleep
        AVR_ENTER_CRITICAL_REGION();
        set_sleep_mode(WDOG_SLEEP ? SLEEP_MODE_PWR_DOWN : SLEEP_MODE_PWR_SAVE);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
        AVR_LEAVE_CRITICAL_REGION();

        wdt_disable();
    }

    // ************** Awake now, wake up everything
    if (PLATFORM == SPITFIRE)
        PRR = 0x03;
    else if (PLATFORM == DSK001)
        PRR = 0x02;
    else
    {
        PRR0 = 0;
        PRR1 = 0;
    }

    if (SERIAL)
        serial_init(NULL);

/*     if (PLATFORM == SPITFIRE) */
/*         // Re-init TWI */
/*         twiInit(); */

    if (ADC_ENABLED && macConfig.associated)
        HAL_INIT_ADC();

    // Bring SPI port back up (must re-init after sleep)
    hal_spi_init();

    // Wake up radio.
    radioLeaveSleepMode();

    // Set RF212 to 250KB mode.
    //radioSetup900();

    radioSetTrxState(PLL_ON);
}


/**
    @brief TIMER2 Real Time Clock Sleep ISR function.
*/
ISR(TIMER2_COMPA_vect)
{
    // Disable TIMER2 output compare interrupt.
    TIMSK2 &= ~(1 << OCIE2A);
}

/**
   @brief Watchdog timer vector handler.
*/
ISR(WDT_vect)
{
    wdt_disable();
}


#else  // if (NODETYPE != COORD)
// For coord, provide dummy function so that code compiles
void nodeSleep(u16 seconds) {}

#endif // if (NODETYPE != COORD)

/** @} */

