/* Copyright (c) 2010  Dresden Elektronik Ingenieurtechnik GmbH
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
/**
 * @file io_access.h
 *
 * @brief Header file for IO access Module
 *
 * $Id: io_access.h,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-04-14
 */

#ifndef IO_ACCESS_H_
#define IO_ACCESS_H_

/* === Macros =============================================================== */

/****************************************
 * GLOBAL IO definitions
 ****************************************/

/**
 * enable external memory interface (available only on ATMega1281, not on deRFMega128)
 */
#if defined(SINGLE_CHIP) || defined(DOXYGEN)
#define XRAM_ENABLE()
#else
#define XRAM_ENABLE()           do {        \
    XMCRA |= _BV(SRE); XMCRB = _BV(XMBK);   \
} while (0)
#endif

/**
 * Memory address mapped to SensTermBoard peripherals -> only ATmega1281
 */
#ifndef SINGLE_CHIP
#define PERIPHERAL_XRAM_ADDRESS   (0x4000)
#endif


/**
 ***********************************
 * @name GLOBAL PLATFORM definitions
 * @{
 */
/**
 * PLATFORM Definition -> all available platforms
 */
#define PLATFORM_RCB          (1)   ///< specify Radio Controller Board Platform
#define PLATFORM_STB          (2)   ///< specify Sensor Terminal Board Platform
#define PLATFORM_BB           (3)   ///< specify Breakout Board Platform

/**
 * global LED platform definitions
 */
#define PLATFORM_LED_ON       (1)   ///< set LED on
#define PLATFORM_LED_OFF      (2)   ///< set LED off
#define PLATFORM_LED_TOGGLE   (3)   ///< toggle LED

/** @} */


/****************************************
 * SPECIFIC PLATFORM definitions
 ****************************************/

/**
 *********************************************************
 * @name Radio Controller Board (RCB) platform definitions
 * @{
 */
/*
 * RCB LED pin and Port mapping
 */
#define RCB_LED_0             (2)      ///< pin mapping of RCB LED 0
#define RCB_LED_1             (3)      ///< pin mapping of RCB LED 1
#define RCB_LED_2             (4)      ///< pin mapping of RCB LED 0

#define RCB_LED_PORT          (PORTE)  ///< port mapping of RCB LED's
#define RCB_LED_DDR           (DDRE)   ///< data direction register mapping of RCB LED's

/*
 * RCB Button Pin and Port mapping
 */
#define RCB_BUTTON_0          (PE5)    ///< pin mapping of RCB Button

#define RCB_BUTTON_PORT       (PORTE)  ///< port mapping of RCB Button
#define RCB_BUTTON_DDR        (DDRE)   ///< data direction register mapping of RCB Button
#define RCB_BUTTON_PINS       (PINE)   ///< input register mapping of RCB Button

/** @} */

/**
 ********************************************************
 * @name Sensor Terminal Board (STB) platform definitions
 * @{
 */
/*
 * STB LED pin mapping
 */
#define STB_LED_0             (0)         ///< pin mapping of STB LED 0
#define STB_LED_1             (1)         ///< pin mapping of STB LED 1

/*
 * STB Temperature pin mapping and switches
 */
#define STB_TEMPERATURE       (2)         ///< pin mapping of STB temperature sensor
#define STB_TEMPERATURE_ON    (1)         ///< enable temperature sensor reading
#define STB_TEMPERATURE_OFF   (2)         ///< disable temperature sensor reading

/*
 * STB Button pin mapping
 */
#define STB_BUTTON_0          (0)         ///< pin mapping of STB Button

/*
 * STB specific control and select pin and port mapping
 */
#define STB_WR_BIT            (PE4)       ///< pin mapping of STB WR bit
#define STB_RD_BIT            (PE5)       ///< pin mapping of STB RD bit
#define STB_CONTROL_PORT      (PORTE)     ///< port mapping of STB control port
#define STB_CONTROL_DDR       (DDRE)      ///< data direction register mapping of STB control port

#define STB_CS0_BIT           (PD6)       ///< pin mapping of STB Chip Select 0 bit
#define STB_CS1_BIT           (PD7)       ///< pin mapping of STB Chip Select 1 bit
#define STB_SELECT_PORT       (PORTD)     ///< port mapping of STB select port
#define STB_SELECT_DDR        (DDRD)      ///< data direction register mapping of STB select port

#define STB_DATA_PORT         (PORTB)     ///< port mapping of STB data port
#define STB_DATA_DDR          (DDRB)      ///< data direction register mapping of STB data port
#define STB_DATA_PINS         (PINB)      ///< input register mapping of STB data port

/** @} */

/* === Globals ============================================================== */

/* === Prototypes =========================================================== */

uint8_t button_pressed(uint8_t platform);

void led_set (uint8_t platform, uint8_t led, uint8_t mode);

void stb_status_temperature(uint8_t mode);


#endif /* IO_ACCESS_H_ */
