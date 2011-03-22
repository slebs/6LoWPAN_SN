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
 $Id: hal_avr.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
 */
#ifndef HAL_AVR_H
#define HAL_AVR_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include "../inc/stdbool.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "../inc/rum_types.h"
#include "../inc/deRFaddon/io_access.h"

/**
 @addtogroup radio
 @{
 */

/**
 @name Platform types.

 Note that the order of these definitions is not arbitrary, and
 there are comparisons of the @ref PLATFORM variable based on
 position within the list.  In other words, re-arranging the list
 can break the code.  Feel free to add new platforms on the end of
 the list.  Note that an RCB230 board can be loaded with an 'RF231
 chip, and vice-versa.  The firmware identifies which radio chip is
 loaded and acts accordingly.  The RCB230/231 distinction is in
 which board is being used, not which chip is loaded.

 @{
 */
#define RCB230     1  ///< Radio control board, designed for use with RF230
#define RCB231     2  ///< Radio control board, designed for use with RF231
#define RCB212     3  ///< Radio control board, designed for use with RF212
#define RCBSIP     4
#define RAVEN      5  ///< Raven demo board (with LCD display)
#define RAVENUSB   6  ///< Raven demo board (USB version)
#define SPITFIRE   7  ///< Small sensor demo board
#define ZIGBIT9    8  ///< ZigBit module, operating in the 900MHz band
#define ZIGBIT24   9  ///< ZigBit module, operating in the 2.4GHz band
#define DSK001    10  ///< TRT Button, model DSK001
#define RCBSINGLE 11  ///< Radio control board, designed for use with ATMEGA128RFA1
/** @} */

/**
 @def BAND
 @brief The BAND macro is set to one particular band based
 on which platform selected for compilation.  Do not manually set
 this macro.

 @name RF Bands supported

 The @ref BAND macro is set to one of the following two bands.
 @{
 */
#define BAND900            1
#define BAND2400           2
#define BAND_SINGLE_CHIP   3
/** @} */

#if defined DOXYGEN
/**
 @brief Defines the platform for which we are building the firmware.
 See @ref RCB230 under "Platform types" above for possible values.
 */

#define PLATFORM RCB231
#define F_CPU 8000000UL
#endif

/* RCB with 231 Device */
#if PLATFORM==RCB231
#ifndef __AVR_ATmega1281__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

// 1281 RF231
/**
 @name Pin configurations for the RCB231 platform.

 Change these values to port to other platforms.
 @{
 */
#   define SSPORT     B          ///< Radio (SPI Slave) Select port
#   define SSPIN      (0x00)     ///< Radio (SPI Slave) Select pin
#   define SPIPORT    B          ///< Radio SPI port
#   define MOSIPIN    (0x02)     ///< Radio SPI MOSI pin
#   define MISOPIN    (0x03)     ///< Radio SPI MISO pin
#   define SCKPIN     (0x01)     ///< Radio SPI SCK pin
#   define RSTPORT    B          ///< Radio reset port
#   define RSTPIN     (0x05)     ///< Radio reset pin
#   define SLPTRPORT  B          ///< Radio SLP_TR port
#   define SLPTRPIN   (0x04)     ///< Radio SLP_TR pin
#   define USART      1          ///< AVR UART used for debug serial port
#   define ADPORT     F          ///< ADC port
#   define ADPIN      (0x00)     ///< ADC pin for sensor input
#   define DIDR       DIDR0      ///< AVR DIDR register (see @ref HAL_INIT_ADC)
#   define TICKTIMER  3          ///< AVR timer used for tick timing
#   define RADIO_VECT INT0_vect  ///< Radio interrupt vector
/// Macro to enable the radio interrupt
#   define HAL_ENABLE_RADIO_INTERRUPT( ) EICRA |= 3, EIMSK |= 1
/// Macro to disable the radio interrupt
#   define HAL_DISABLE_RADIO_INTERRUPT( ) EICRA &= ~3, EIMSK &= ~1
/** Macro to initialize the ADC converter.  note that on
 some platforms, there is no ADC so this macro does nothing. */
#   define HAL_INIT_ADC() DIDR0 |= (1 << ADPIN), ADMUX = 0xC0 | ADPIN, ADCSRA = 0x84
/// Macro to stop the ADC
#   define HAL_STOP_ADC() ADCSRA &= ~0x80
/// Macro to sample the ADC
#   define HAL_SAMPLE_ADC() ADCSRA |= (1 << ADSC) | (1<< ADIF)
/// Macro to wait for the ADC to finish sampling
#   define HAL_WAIT_ADC() while (!(ADCSRA & (1<<ADIF))) {;}; ADCSRA |= (1<<ADIF);
/// Macro to READ the ADC value
#   define HAL_READ_ADC() ADC
#   define BAND BAND2400

// LED Macros
//        #define LED_INIT() (DDRE |= ((1<<2) | (1<<3) | (1<<4)), PORTE |= ((1<<2) | (1<<3) | (1<<4)))
// LED_ON(led), where led = 1-3
//        #define LED_ON(led) (PORTE &= ~(1 << (led+1)))
//        #define LED_OFF(led) (PORTE |= 1 << (led+1))
//        #define LED_TOGGLE(led) (PORTE ^= 1 << (led+1))

#define LED_INIT()
#define LED_ON(led)
#define LED_OFF(led)
#define LED_TOGGLE(led)

// Button macros
#define BUTTON_SETUP()                  DDRE &= ~(1 << PE5), PORTE |= (1 << PE5)
#define BUTTON_PRESSED() (DDRE &= ~0x20, PORTE |= 0x20, !(PINE & 0x20))

/* Constant defines for the LQI calculation */
#ifndef RSSI_BASE_VAL
#define RSSI_BASE_VAL                   (-90)
#endif
#ifndef ED_THRESHOLD
#define ED_THRESHOLD                    (60)
#endif
#ifndef ED_MAX
#define ED_MAX                          (-RSSI_BASE_VAL - ED_THRESHOLD)
#endif
#ifndef LQI_MAX
#define LQI_MAX                         (3)
#endif
/** @} */

/* RCB with 231/1284 Device on SIP chip */
#elif PLATFORM==RCBSIP
#ifndef __AVR_ATmega1284P__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

// 1284 RF231
#   define SSPORT     B
#   define SSPIN      (0x04)
#   define SPIPORT    B
#   define MOSIPORT   B
#   define MOSIPIN    (0x05)
#   define MISOPIN    (0x06)
#   define SCKPORT    B
#   define SCKPIN     (0x07)
#   define RSTPORT    B
#   define RSTPIN     (0x02)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x03)
#   define USART      1
#   define RADIO_VECT TIMER1_CAPT_vect
#   define TICKTIMER  3
#   define HAL_ENABLE_RADIO_INTERRUPT( ) ( TIMSK1 |= ( 1 << ICIE1 ) ),  \
                        TCCR1B = HAL_TCCR1B_CONFIG,   /* Set clock prescaler */ \
                        TIFR1 |= (1 << ICF1)        /* Clear Input Capture Flag. */
#   define HAL_DISABLE_RADIO_INTERRUPT( ) ( TIMSK1 &= ~( 1 << ICIE1 ), TCCR1B = 0)
#   define PB0 0
#   define PD4 4
#   define PC0 0
#   define PC4 4
#   define PC6 6
#   define PC7 7
#   define HAL_INIT_ADC()
#   define HAL_STOP_ADC()
#   define HAL_SAMPLE_ADC()
#   define HAL_WAIT_ADC()
#   define HAL_READ_ADC() 0
#   define BAND BAND2400

// LED Macros
#define LED_INIT()              DDRD |= (1 << PD4), PORTD |= ~(1 << PD4)
// LED_ON(led), where led = 1-3
#define LED_ON(led)             DDRD |= (1 << PD4), PORTD &= ~(1 << PD4)
#define LED_OFF(led)            DDRD &= ~(1 << PD4), PORTD |= (1 << PD4)

// Button macros
#define BUTTON_SETUP()          DDRB &= ~(1 << PB0), PORTB |= (1 << PB0)
#define BUTTON_PRESSED()        (!(PINB & (1 << PB0)))

/* Constant defines for the LQI calculation */
#ifndef RSSI_BASE_VAL
#define RSSI_BASE_VAL                   (-90)
#endif
#ifndef ED_THRESHOLD
#define ED_THRESHOLD                    (60)
#endif
#ifndef ED_MAX
#define ED_MAX                          (-RSSI_BASE_VAL - ED_THRESHOLD)
#endif
#ifndef LQI_MAX
#define LQI_MAX                         (3)
#endif

/* RCB with 230 Device */
#elif PLATFORM==RCB230

#ifndef __AVR_ATmega1281__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

// 1281 RF230 rev B
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    B
#   define RSTPIN     (0x05)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define USART      1
#   define ADPORT     F
#   define ADPIN      (0x00)
#   define DIDR       DIDR0
#   define TICKTIMER  3
#   define RADIO_VECT TIMER1_CAPT_vect
#   define HAL_ENABLE_RADIO_INTERRUPT( ) ( TIMSK1 |= ( 1 << ICIE1 ) ),  \
                        TCCR1B = HAL_TCCR1B_CONFIG,   /* Set clock prescaler */ \
                        TIFR1 |= (1 << ICF1)        /* Clear Input Capture Flag. */
#   define HAL_DISABLE_RADIO_INTERRUPT( ) ( TIMSK1 &= ~( 1 << ICIE1 ), TCCR1B = 0 )
#   define HAL_INIT_ADC() DIDR0 |= (1 << ADPIN), ADMUX = 0xC0 | ADPIN, ADCSRA = 0x80
#   define HAL_STOP_ADC()  ADCSRA &= ~0x80
#   define HAL_SAMPLE_ADC() ADCSRA |= (1 << ADSC)
#   define HAL_WAIT_ADC() while (!(ADCSRA & (1<<ADIF))) {;}; ADCSRA |= (1<<ADIF)
#   define HAL_READ_ADC() ADC
#   define BAND BAND2400

// LED Macros
//        #define LED_INIT() (DDRE |= ((1<<2) | (1<<3) | (1<<4)), PORTE |= ((1<<2) | (1<<3) | (1<<4)))
// LED_ON(led), where led = 1-3
//        #define LED_ON(led) (PORTE &= ~(1 << (led+1)))
//        #define LED_OFF(led) (PORTE |= 1 << (led+1))
//        #define LED_TOGGLE(led) (PORTE ^= 1 << (led+1))

#define LED_INIT()
#define LED_ON(led)
#define LED_OFF(led)
#define LED_TOGGLE(led)

// Button macros
#define BUTTON_SETUP()                  DDRE &= ~(1 << PE5), PORTE |= (1 << PE5)

//TODO make clean
#define BUTTON_PRESSED() (DDRE &= ~0x20, PORTE |= 0x20, !(PINE & 0x20))

//#define BUTTON_PRESSED() (button_pressed(PLATFORM_STB))

/* Constant defines for the LQI calculation */
#ifndef RSSI_BASE_VAL
#define RSSI_BASE_VAL                   (-91)
#endif
#ifndef ED_THRESHOLD
#define ED_THRESHOLD                    (60)
#endif
#ifndef ED_MAX
#define ED_MAX                          (-RSSI_BASE_VAL - ED_THRESHOLD)
#endif
#ifndef LQI_MAX
#define LQI_MAX                         (3)
#endif

/* RCB with 212 Device */
#elif PLATFORM==RCB212

#ifndef __AVR_ATmega1281__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    B
#   define RSTPIN     (0x05)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define USART      1
#   define ADPORT     F
#   define ADPIN      (0x00)
#   define DIDR       DIDR0
#   define TICKTIMER  3
#   define RADIO_VECT INT0_vect
#   define HAL_ENABLE_RADIO_INTERRUPT( ) EICRA |= 3, EIMSK |= 1
#   define HAL_DISABLE_RADIO_INTERRUPT( ) EICRA &= ~3, EIMSK &= ~1
#   define HAL_INIT_ADC() DIDR0 |= (1 << ADPIN), ADMUX = 0xC0 | ADPIN, ADCSRA = 0x84
#   define HAL_STOP_ADC() ADCSRA &= ~0x80
#   define HAL_SAMPLE_ADC() ADCSRA |= (1 << ADSC) | (1<< ADIF)
#   define HAL_WAIT_ADC() while (!(ADCSRA & (1<<ADIF))) {;}; ADCSRA |= (1<<ADIF)
#   define HAL_READ_ADC() ADC
#   define BAND BAND900    // RF212 RCB
// LED Macros
//        #define LED_INIT() (DDRE |= ((1<<2) | (1<<3) | (1<<4)), PORTE |= ((1<<2) | (1<<3) | (1<<4)))
// LED_ON(led), where led = 1-3
//        #define LED_ON(led) (PORTE &= ~(1 << (led+1)))
//        #define LED_OFF(led) (PORTE |= 1 << (led+1))

#define LED_INIT()
#define LED_ON(led)
#define LED_OFF(led)
#define LED_TOGGLE(led)

// Button macros
#define BUTTON_SETUP()                  DDRE &= ~(1 << PE5), PORTE |= (1 << PE5)
#define BUTTON_PRESSED() (DDRE &= ~0x20, PORTE |= 0x20, !(PINE & 0x20))

/* Constant defines for the LQI calculation */
#ifndef RSSI_BASE_VAL
#define RSSI_BASE_VAL                   (-97)
#endif
#ifndef ED_THRESHOLD
#define ED_THRESHOLD                    (60)
#endif
#ifndef ED_MAX
#define ED_MAX                          (-RSSI_BASE_VAL - ED_THRESHOLD)
#endif
#ifndef LQI_MAX
#define LQI_MAX                         (3)
#endif

/* Atmel Raven */
#elif PLATFORM==RAVEN

#ifndef __AVR_ATmega1284P__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

/* 1284 raven */
#   define SSPORT     B
#   define SSPIN      (0x04)
#   define SPIPORT    B
#   define MOSIPIN    (0x05)
#   define MISOPIN    (0x06)
#   define SCKPIN     (0x07)
#   define RSTPORT    B
#   define RSTPIN     (0x01)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x03)
#   define USART      0
#   define TICKTIMER  3
#   define HAS_SPARE_TIMER
#   define RADIO_VECT TIMER1_CAPT_vect
#   define HAL_ENABLE_RADIO_INTERRUPT( ) ( TIMSK1 |= ( 1 << ICIE1 ) ),  \
                        TCCR1B = HAL_TCCR1B_CONFIG,   /* Set clock prescaler */ \
                        TIFR1 |= (1 << ICF1)        /* Clear Input Capture Flag. */
#   define HAL_DISABLE_RADIO_INTERRUPT( ) ( TIMSK1 &= ~( 1 << ICIE1 ) )
// Raven's 1284 has no analog connections
#   define HAL_INIT_ADC()
#   define HAL_STOP_ADC()
#   define HAL_SAMPLE_ADC()
#   define HAL_WAIT_ADC()
#   define HAL_READ_ADC() ADC
#   define BAND BAND2400
#   define PC4 4
#   define PC6 6
#   define PC7 7

// LED Macros
#define LED_INIT()
// LED_ON(led), where led = 1-3
// Needs code programmed into Mega3290 to work!!
// RAVEN only has one LED - so we never check which one... should do that probably
#define LED_ON(led)
#define LED_OFF(led)

// Button macros - needs code programmed into Mega3290 to work!!
#define BUTTON_SETUP()
#define BUTTON_PRESSED() 0

/* Spitfire (small) PCB */
#elif PLATFORM==SPITFIRE

#if (!defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168P__))
#error "Incorrect MCU for Platform! Check Makefile"
#endif

#   define SSPORT     B
#   define SSPIN      (0x02)
#   define SPIPORT    B
#   define MOSIPIN    (0x03)
#   define MISOPIN    (0x04)
#   define SCKPIN     (0x05)
#   define RSTPORT    B
#   define RSTPIN     (0x01)
#   define SLPTRPORT  D
#   define SLPTRPIN   (0x04)
#   define USART
#   define TICKTIMER  1
#   define RADIO_VECT PCINT0_vect
#   define HAL_ENABLE_RADIO_INTERRUPT( ) PCICR |= (1<<0), PCMSK0 |= (1<<0)
#   define HAL_DISABLE_RADIO_INTERRUPT( ) PCMSK0 &= ~(1<<0)
#   define HAL_INIT_ADC()  ADMUX = 0x47, ADCSRA = 0x80
#   define HAL_STOP_ADC()  ADCSRA &= ~0x80
#   define HAL_SAMPLE_ADC() ADCSRA |= (1 << ADSC)
#   define HAL_WAIT_ADC() while (!(ADCSRA & (1<<ADIF))) {;}; ADCSRA |= (1<<ADIF)
#   define HAL_READ_ADC() ADC
#   define BAND BAND900

#define PD7 7

// LED Macros
#define LED_INIT()             DDRD |= (1 << PD7), PORTD |= (1 << PD7)
// LED_ON(led), where led doesn't matter - there is only one LED on the board.
#define LED_ON(led)            DDRD |= (1 << PD7), PORTD &= !(1 << PD7)
#define LED_OFF(led)           DDRD &= ~(1 << PD7), PORTD |= (1 << PD7)

// Button macros (this platform has no button
#define BUTTON_SETUP()
#define BUTTON_PRESSED()        0

/* Raven USB Stick */
#elif PLATFORM==RAVENUSB

#ifndef __AVR_AT90USB1287__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    B
#   define RSTPIN     (0x05)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define USART        1
#   define TICKTIMER  3
#   define RADIO_VECT TIMER1_CAPT_vect
#   define HAL_ENABLE_RADIO_INTERRUPT( ) ( TIMSK1 |= ( 1 << ICIE1 ) ),  \
                        TCCR1B = HAL_TCCR1B_CONFIG,   /* Set clock prescaler */ \
                        TIFR1 |= (1 << ICF1)        /* Clear Input Capture Flag. */
#   define HAL_DISABLE_RADIO_INTERRUPT( ) ( TIMSK1 &= ~( 1 << ICIE1 ), TCCR1B = 0 )
#   define HAL_INIT_ADC()
#   define HAL_STOP_ADC()
#   define HAL_SAMPLE_ADC()
#   define HAL_WAIT_ADC()
#   define HAL_READ_ADC() 0
#   define BAND BAND2400

// Button macros (this platform has no button
#define BUTTON_SETUP()
#define BUTTON_PRESSED()        0

// LED's for Raven USB
#define Leds_init()                 (DDRD  |=  0xA0, DDRE  |=  0xC0)
#define Leds_on()                   (PORTD |=  0x80, PORTD &= ~0x40, PORTE &=  ~0xC0)
#define Leds_off()                  (PORTD &= ~0x80, PORTD |=  0x40, PORTE |=   0xC0)
#define Led0_on()                   (PORTD |=  0x80)
#define Led1_on()                   (PORTD &= ~0x20)
#define Led2_on()                   (PORTE &= ~0x80)
#define Led3_on()                   (PORTE &= ~0x40)
#define Led0_off()                  (PORTD &= ~0x80)
#define Led1_off()                  (PORTD |=  0x20)
#define Led2_off()                  (PORTE |=  0x80)
#define Led3_off()                  (PORTE |=  0x40)
#define Led0_toggle()               (PIND |= 0x80)
#define Led1_toggle()               (PIND |= 0x20)
#define Led2_toggle()               (PINE |= 0x80)
#define Led3_toggle()               (PINE |= 0x40)

// LED Macros
#define LED_INIT()                  Leds_init()
// LED_ON(led), where led doesn't matter - there is only one LED on the board.
#define LED_ON(led)                 Led0_on()
#define LED_OFF(led)                Led0_off()

/* ZIGBIT modules */
#elif PLATFORM==ZIGBIT9 || PLATFORM==ZIGBIT24

#ifndef __AVR_ATmega1281__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    A
#   define RSTPIN     (0x07)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define USART      1
#   define TICKTIMER  3
#   define RADIO_VECT INT5_vect
#   define HAL_ENABLE_RADIO_INTERRUPT( ) EICRB |= 0x0C, EIMSK |= 0x20
#   define HAL_DISABLE_RADIO_INTERRUPT( ) EICRB &= ~0x0C, EIMSK &= ~0x20

#   define HAL_INIT_ADC()
#   define HAL_STOP_ADC()
#   define HAL_SAMPLE_ADC()
#   define HAL_WAIT_ADC()
#   define HAL_READ_ADC() 0

// Button macros (PE6, needs pullup, short to gnd when pressed)
#define BUTTON_SETUP()     (DDRE &= ~(1 << 6), PINE |= (1 << 6))
#define BUTTON_PRESSED()   (!(PINE & (1 << 6)))

// LED's for MESHBEAN USB
// LED1 = PB5
// LED2 = PB6
// LED3 = PB7
#define Leds_init()      (DDRB  |=  0xE0)
#define Leds_on()        (PORTB |=  0x80)
#define Leds_off()       (PORTB &= ~0xE0)
#define Led0_on()
#define Led1_on()        (PORTB |=  0x80)
#define Led2_on()        (PORTB |=  0x40)
#define Led3_on()        (PORTB |=  0x20)
#define Led0_off()
#define Led1_off()       (PORTB &= ~0x80)
#define Led2_off()       (PORTB &= ~0x40)
#define Led3_off()       (PORTB &= ~0x20)
#define Led0_toggle()
#define Led1_toggle()    (PINB  |= 0x80)
#define Led2_toggle()    (PINB  |= 0x40)
#define Led3_toggle()    (PINB  |= 0x20)

// LED Macros
#define LED_INIT()                  Leds_init()
// LED_ON(led), where led doesn't matter - there is only one LED on the board.
#define LED_ON(led)                 Led1_on()
#define LED_OFF(led)                Led1_off()

// Define which band we're operating in
#if PLATFORM==ZIGBIT24
#define BAND BAND2400
#else
#define BAND BAND900
#endif

/* TRT "button" board DSK001 */
#elif PLATFORM==DSK001

#ifndef __AVR_ATmega328P__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

#   define SSPORT     B
#   define SSPIN      (0x02)
#   define SPIPORT    B
#   define MOSIPIN    (0x03)
#   define MISOPIN    (0x04)
#   define SCKPIN     (0x05)
#   define RSTPORT    C
#   define RSTPIN     (0x05)
#   define SLPTRPORT  C
#   define SLPTRPIN   (0x04)
#   define USART      0
#   define TICKTIMER  1
#   define RADIO_VECT PCINT0_vect
#   define HAL_ENABLE_RADIO_INTERRUPT( ) PCICR |= (1 << PCIE0), PCMSK0 |= (1 << PCINT1)
#   define HAL_DISABLE_RADIO_INTERRUPT( ) PCMSK0 &= ~(1 << PCINT1)

#   define HAL_INIT_ADC()    ADMUX = 0xc8, ADCSRA = (1 << ADEN) | (1<<ADPS1) | (1<<ADPS2)
#   define HAL_STOP_ADC()    ADMUX = 0x0F, ADCSRA = 1<<ADIF
#   define HAL_SAMPLE_ADC()  ADCSRA |= (1 << ADSC) | (1 << ADIF)
#   define HAL_WAIT_ADC()    while (!(ADCSRA & (1<<ADIF))) {;}; ADCSRA |= (1<<ADIF)
#   define HAL_READ_ADC()    ADC

#   define HAL_SELECT_ACCELZ()   ADMUX = (ADMUX & 0xF0) | 0x02
#   define HAL_SELECT_ACCELY()   ADMUX = (ADMUX & 0xF0) | 0x01
#   define HAL_SELECT_ACCELX()   ADMUX = (ADMUX & 0xF0) | 0x00

#   define HAL_ACCEL_INIT()      DDRC = (DDRC & 0xf0) | (1<<3), PORTC &= 0xf0;
#   define HAL_ACCEL_ON()        PORTC |= 1<<3
#   define HAL_ACCEL_OFF()       PORTC &= ~(1<<3)

// Button macros (no button on this platform)
#define BUTTON_SETUP()
#define BUTTON_PRESSED()  0

// LED is on Port D Pin 7
#define Leds_init()   //   (DDRD  |=  0x80, PORTD &= ~0x80)
#define Leds_on()   (DDRD  |=  0x80, PORTD &= ~0x80)
#define Leds_off()  (DDRD  &=  ~0x80)
#define Led0_on()   (DDRD  |=  0x80, PORTD &= ~0x80)
#define Led1_on()   (DDRD  |=  0x80, PORTD &= ~0x80)
#define Led2_on()   (DDRD  |=  0x80, PORTD &= ~0x80)
#define Led3_on()   (DDRD  |=  0x80, PORTD &= ~0x80)
#define Led0_off()  (DDRD  &=  ~0x80)
#define Led1_off()  (DDRD  &=  ~0x80)
#define Led2_off()  (DDRD  &=  ~0x80)
#define Led3_off()  (DDRD  &=  ~0x80)
#define Led0_toggle() DDRD = DDRD ^ 0x80)
#define Led1_toggle()  DDRD = DDRD ^ 0x80
#define Led2_toggle()  DDRD = DDRD ^ 0x80
#define Led3_toggle()  DDRD = DDRD ^ 0x80

// LED Macros
#define LED_INIT()                  Leds_init()
// LED_ON(led), where led doesn't matter - there is only one LED on the board.
#define LED_ON(led)                 Led1_on()
#define LED_OFF(led)                Led1_off()

// Define which band we're operating in
#define BAND BAND2400

* RCB with ATmega128RFA1 Device */
#elif PLATFORM==RCBSINGLE
#ifndef __AVR_ATmega128RFA1__
#error "Incorrect MCU for Platform! Check Makefile"
#endif

#ifndef SINGLE_CHIP
#define SINGLE_CHIP
#endif

// 128RFA1
/**
 @name Pin configurations for 128RFA1 platform.

 Change these values to port to other platforms.
 @{

 */

#define CLKM_1MHz    (1)
#   define SSPORT     B          ///< Radio (SPI Slave) Select port
#   define SSPIN      (0x00)     ///< Radio (SPI Slave) Select pin
#   define SPIPORT    B          ///< Radio SPI port
#   define MOSIPIN    (0x02)     ///< Radio SPI MOSI pin
#   define MISOPIN    (0x03)     ///< Radio SPI MISO pin
#   define SCKPIN     (0x01)     ///< Radio SPI SCK pin
#   define RSTPORT    B          ///< Radio reset port
#   define RSTPIN     (0x05)     ///< Radio reset pin
#   define SLPTRPORT  B          ///< Radio SLP_TR port
#   define SLPTRPIN   (0x04)     ///< Radio SLP_TR pin
#   define USART      1          ///< AVR UART used for debug serial port
#   define ADPORT     F          ///< ADC port
#   define ADPIN      (0x00)     ///< ADC pin for sensor input
#   define DIDR       DIDR0      ///< AVR DIDR register (see @ref HAL_INIT_ADC)
#   define TICKTIMER  3          ///< AVR timer used for tick timing
#   define RADIO_VECT   ///< Radio interrupt vector
#   define RADIO_VECT1 TRX24_RX_START_vect  ///< Radio interrupt vector
#   define RADIO_VECT2 TRX24_RX_END_vect  ///< Radio interrupt vector
#   define RADIO_VECT3 TRX24_TX_END_vect  ///< Radio interrupt vector
#   define RADIO_VECT4 TRX24_CCA_ED_DONE_vect  ///< Radio interrupt vector
#   define RADIO_VECT5 BAT_LOW_vect  ///< Radio interrupt vector
#   define RADIO_VECT6 TRX24_PLL_UNLOCK_vect  ///< Radio interrupt vector
#   define RADIO_VECT7 TRX24_PLL_LOCK_vect  ///< Radio interrupt vector
#   define RADIO_VECT8 TRX24_AWAKE_vect  ///< Radio interrupt vector
/// Macro to enable the radio interrupt
#   define HAL_ENABLE_RADIO_INTERRUPT( ) \
                     IRQ_MASK |= _BV(RX_START_EN), \
                     IRQ_MASK |= _BV(RX_END_EN), \
                     IRQ_MASK |= _BV(TX_END_EN), \
                     IRQ_MASK |= _BV(CCA_ED_DONE_EN), \
                     BATMON   |= _BV(BAT_LOW_EN), \
                     IRQ_MASK |= _BV(PLL_UNLOCK_EN), \
                     IRQ_MASK |= _BV(PLL_LOCK_EN), \
                     IRQ_MASK |= _BV(AWAKE_EN)

/// Macro to disable the radio interrupt
#   define HAL_DISABLE_RADIO_INTERRUPT( ) \
                     IRQ_MASK &= ~_BV(RX_START_EN), \
                     IRQ_MASK &= ~_BV(RX_END_EN), \
                     IRQ_MASK &= ~_BV(TX_END_EN), \
                     IRQ_MASK &= ~_BV(CCA_ED_DONE_EN), \
                     BATMON   &= ~_BV(BAT_LOW_EN), \
                     IRQ_MASK &= ~_BV(PLL_UNLOCK_EN), \
                     IRQ_MASK &= ~_BV(PLL_LOCK_EN), \
                     IRQ_MASK &= ~_BV(AWAKE_EN)
/** Macro to initialize the ADC converter.  note that on
 some platforms, there is no ADC so this macro does nothing. */
#   define HAL_INIT_ADC()
/// Macro to stop the ADC
#   define HAL_STOP_ADC()
/// Macro to sample the ADC
#   define HAL_SAMPLE_ADC()
/// Macro to wait for the ADC to finish sampling
#   define HAL_WAIT_ADC()
/// Macro to READ the ADC value
#   define HAL_READ_ADC()

#   define BAND BAND_SINGLE_CHIP

// LED Macros
//        #define LED_INIT() (DDRE |= ((1<<PE2) | (1<<PE3) | (1<<PE4)), PORTE |= ((1<<2) | (1<<3) | (1<<4)))
// LED_ON(led), where led = 1-3
//        #define LED_ON(led) (PORTE &= ~(1 << (led+1)))
//        #define LED_OFF(led) (PORTE |= 1 << (led+1))
//        #define LED_TOGGLE(led) (PORTE ^= 1 << (led+1))

#define LED_INIT()
#define LED_ON(led)              RCB_LED_PORT &= ~(1 << (led)); RCB_LED_DDR  |=  (1 << (led));
#define LED_OFF(led)		 	 RCB_LED_PORT |= (1 << (led));  RCB_LED_DDR  |= (1 << (led));
#define LED_TOGGLE(led)			 RCB_LED_PORT ^= (1 << (led));  RCB_LED_DDR  |= (1 << (led));

// Button macros
#define BUTTON_SETUP()   (DDRE &= ~(1 << PE5), PORTE |= (1 << PE5))
#define BUTTON_PRESSED() (DDRE &= ~0x20, PORTE |= 0x20, !(PINE & 0x20))

/* Constant defines for the LQI calculation */
#ifndef RSSI_BASE_VAL
#define RSSI_BASE_VAL                   (-91)
#endif
#ifndef ED_THRESHOLD
#define ED_THRESHOLD                    (30)
#endif
#ifndef ED_MAX
#define ED_MAX                          (-RSSI_BASE_VAL - ED_THRESHOLD)
#endif
#ifndef LQI_MAX
#define LQI_MAX                         (3)
#endif

/** @} */

#else
#error "PLATFORM undefined or incorrect value"
#endif

#if BAND == BAND2400
#include "at86rf23x_registermap.h"
#elif BAND == BAND900
#include "at86rf212_registermap.h"
#elif BAND == BAND_SINGLE_CHIP
#include <avr/iom128rfa1.h>
#include "atmega128rfa1_registermap.h"
#else
#error "BAND Undefined!"
#endif

/**
 @name Macros used to generate register names

 The various CAT macros (DDR, PORT, and PIN) are used to assign
 port/pin/DDR names to various macro variables.  The variables are
 assigned based on the specific connections made in the hardware.
 For example TCCR(TICKTIMER,A) can be used in place of TCCR0A if
 TICKTIMER is defined as 0. This setup allows changing which
 resources are used on a PC board with minimal changes.

 @{
 */
#define CAT(x, y)      x##y                ///< Concatenate two strings
#define CAT2(x, y, z)  x##y##z             ///< Concatenate three strings
#define DDR(x)         CAT(DDR,  x)        ///< Data direction register macro
#define PORT(x)        CAT(PORT, x)
#define PIN(x)         CAT(PIN,  x)
#define UCSR(num, let) CAT2(UCSR,num,let)
#define RXEN(x)        CAT(RXEN,x)
#define TXEN(x)        CAT(TXEN,x)
#define TXC(x)         CAT(TXC,x)
#define RXC(x)         CAT(RXC,x)
#define RXCIE(x)       CAT(RXCIE,x)
#define UCSZ(x,y)      CAT2(UCSZ,x,y)
#define UBRR(x,y)      CAT2(UBRR,x,y)
#define UDRE(x)        CAT(UDRE,x)
#define UDRIE(x)       CAT(UDRIE,x)
#define UDR(x)         CAT(UDR,x)
#define TCNT(x)        CAT(TCNT,x)
#define TIMSK(x)       CAT(TIMSK,x)
#define TCCR(x,y)      CAT2(TCCR,x,y)
#define COM(x,y)       CAT2(COM,x,y)
#define OCR(x,y)       CAT2(OCR,x,y)
#define CS(x,y)        CAT2(CS,x,y)
#define WGM(x,y)       CAT2(WGM,x,y)
#define OCIE(x,y)      CAT2(OCIE,x,y)
#define COMPVECT(x)    CAT2(TIMER,x,_COMPA_vect)
#define UDREVECT(x)    CAT2(USART,x,_UDRE_vect)
#define RXVECT(x)      CAT2(USART,x,_RX_vect)
/**  @} */

/**
 @name Macros to use in source code for platform-specific names
 @{
 */
#ifdef SINGLE_CHIP

/* SLPTR pin */
#define TRX_INIT_SLPTR_PIN()
#define hal_set_slptr_high()     (TRXPR |= _BV(SLPTR))
#define hal_set_slptr_low()      (TRXPR &= ~_BV(SLPTR))
#define hal_get_slptr()          (TRXPR & _BV(SLPTR))
/* RESET pin */
#define TRX_INIT_RESET_PIN()
#define hal_set_rst_high()       (TRXPR |= _BV(TRXRST))
#define hal_set_rst_low()        (TRXPR &= ~_BV(TRXRST))
#define hal_get_rst()            (TRXPR & _BV(TRXRST))
#else
#define SLP_TR                SLPTRPIN            ///< Pin number that corresponds to the SLP_TR pin.
#define DDR_SLP_TR            DDR( SLPTRPORT )    ///< Data Direction Register that corresponds to the port where SLP_TR is connected.
#define PORT_SLP_TR           PORT( SLPTRPORT )   ///< Port (Write Access) where SLP_TR is connected.
#define PIN_SLP_TR            PIN( SLPTRPORT )    ///< Pin (Read Access) where SLP_TR is connected.
#define hal_set_slptr_high( ) ( PORT_SLP_TR |= ( 1 << SLP_TR ) )      /// < This macro pulls the SLP_TR pin high.
#define hal_set_slptr_low( )  ( PORT_SLP_TR &= ~( 1 << SLP_TR ) )     ///< This macro pulls the SLP_TR pin low.
#define hal_get_slptr( ) (    ( PIN_SLP_TR & ( 1 << SLP_TR ) ) >> SLP_TR )  ///< Read current state of the SLP_TR pin (High/Low).
#define RST                   RSTPIN              ///< Pin number that corresponds to the RST pin.
#define DDR_RST               DDR( RSTPORT )      ///< Data Direction Register that corresponds to the port where RST is
#define PORT_RST              PORT( RSTPORT )     ///< Port (Write Access) where RST is connected.
#define PIN_RST               PIN( RSTPORT )      ///< Pin (Read Access) where RST is connected.
#define hal_set_rst_high( )   ( PORT_RST |= ( 1 << RST ) )  ///< This macro pulls the RST pin high.
#define hal_set_rst_low( )    ( PORT_RST &= ~( 1 << RST ) ) ///< This macro pulls the RST pin low.
#define hal_get_rst( )        ( ( PIN_RST & ( 1 << RST )  ) >> RST )  ///< Read current state of the RST pin (High/Low).
#endif
/* have to define following parameters to keep compiler happy - for both: single and dual chip*/
//#define HAL_SS_PIN            SSPIN               ///< The slave select pin.
#define HAL_PORT_SPI          PORT( SPIPORT )     ///< The SPI module PORT.
#define HAL_DDR_SPI           DDR( SPIPORT )      ///< Data Direction Register for the SPI port.
#define HAL_DD_SS             SSPIN               ///< Data Direction bit for SS.
#define HAL_DD_SCK            SCKPIN              ///< Data Direction bit for SCK.
#define HAL_DD_MOSI           MOSIPIN             ///< Data Direction bit for MOSI.
#define HAL_DD_MISO           MISOPIN             ///< Data Direction bit for MISO.
#define HAL_PORT_SS           PORT(SSPORT)
#define HAL_DDR_SS            DDR(SSPORT)
#define HAL_SS_HIGH( ) (PORT(SSPORT) |=  ( 1 << SSPIN )) //!< MACRO for pulling SS high.
#define HAL_SS_LOW( )  (PORT(SSPORT) &= ~( 1 << SSPIN )) //!< MACRO for pulling SS low.
/// @}


/**
 Macro to enable the RCB breakout board (RCB_BB). To enable the
 RCB_BB serial port, pins PC4-PC7 must be managed.

 - PC4 is the ENABLE pin to the serial driver.  If this pin is low,
 the driver chip is enabled, otherwise the chip is disabled.

 - PC5 is an input for the INVALID signal from the serial driver
 chip.  This signal is currently unused.

 - PC6 is the FORCEON output to the serial driver chip.  If this pin
 is low, the driver will only drive the serial lines if the driver
 chip senses valid serial input signal levels.  If this pin is
 high, then the serial driver is always on (assuming the ENABLE
 pin is low).

 - PC7 is the FORCEOFF pin, which is used to put the chip into sleep
 mode.  See the MAX3221E datasheet for details.
 */
#define INIT_RCB_BB()  DDRC  |= (1 << PC4)|(1 << PC6)|(1 << PC7), \
        PORTC |= (1 << PC7), PORTC &= ~((1 << PC4)|(1 << PC6))

/**
 @name Macros defined for the radio received packet timer

 These macros define the setup of the AVR timer used to time the
 radio interrupt, and to ensure that the hal_get_system_time
 function returns the system time in symbols (16 us ticks).

 @{
 */
#if ( F_CPU == 16000000UL )
#define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS12 ) )
#define HAL_US_PER_SYMBOL ( 1 )
#define HAL_SYMBOL_MASK   ( 0xFFFFffff )
#elif ( F_CPU == 8000000UL )
#define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) ) //clk/64
#define HAL_US_PER_SYMBOL ( 2 )
#define HAL_SYMBOL_MASK   ( 0x7FFFffff )
#elif ( F_CPU == 4000000UL )
#define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) )
#define HAL_US_PER_SYMBOL ( 1 )
#define HAL_SYMBOL_MASK   ( 0xFFFFffff )
#elif ( F_CPU == 2000000UL )
#define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) )
#define HAL_US_PER_SYMBOL ( 2 )
#define HAL_SYMBOL_MASK   ( 0x7FFFffff )
#elif ( F_CPU == 1000000UL )
#define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) )
#define HAL_US_PER_SYMBOL ( 2 )
#define HAL_SYMBOL_MASK   ( 0x7FFFffff )
#else
#error "Clock speed not supported."
#endif
/** @} */

/** This macro will protect any subsequent code from interrupts. */
#define AVR_ENTER_CRITICAL_REGION( ) {u8 volatile saved_sreg = SREG; cli( )

/** This macro ends a protected block of code and must always be used
 in conjunction with @ref AVR_ENTER_CRITICAL_REGION.  */
#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

// Debugging macros
#if DEBUG && SERIAL
#include "serial.h"
#define debugMsgChr(...) (fprintf(stderr, __VA_ARGS__))						 //serial_putchar(c) UART_PRINT
#define debugMsgStr(...) (fprintf(stderr, __VA_ARGS__))						 // serial_puts((char *)s) //debugStrUSB(s)
#define debugMsgFlt(n) sprintf(debugStr,"%f",n), (fprintf(stderr, debugStr)) //serial_puts(debugStr)
#define debugMsgInt(i) sprintf(debugStr,"%d",i), (fprintf(stderr, debugStr)) //serial_puts(debugStr)
#define debugMsgHex(x) sprintf(debugStr,"%x",x), (fprintf(stderr, debugStr)) //serial_puts(debugStr)
#define debugMsgCrLf() serial_puts("\r\n")
#elif DEBUG && OTA_DEBUG
#define otaDebugMsg(s,a) sprintf(debugStr,s,a);   \
    macOtaDebugRequest((u8*)debugStr);
#define otaDebugMsg0(s) sprintf(debugStr,s);   \
    macOtaDebugRequest((u8*)debugStr);
#define debugMsgChr(c) otaDebugMsg("%c", c)
#define debugMsgStr(s) otaDebugMsg("%s", s)
#define debugMsgFlt(n) otaDebugMsg("%f", n)
#define debugMsgInt(i) otaDebugMsg("%d",i)
#define debugMsgHex(x) otaDebugMsg("%x",x)
#define debugMsgCrLf() otaDebugMsg0("\r\n")
#else
#define debugMsgChr(c)
#define debugMsgStr(...)  (fprintf(stderr, __VA_ARGS__))
#define debugMsgFlt(n)
#define debugMsgInt(i)
#define debugMsgHex(x)
#define debugMsgCrLf()
#endif

// Macros used to ensure compatibility with ARM code
// ARM is a 32-bit machine, so it needs to declare a
// temporary variable, which is guaranted to be on a
// 32-bit boundary.  The AVR should not declare the
// temporary variable because it eats up a lot of flash
// and isn't required for AVR.
#define  DECLARE64(x)
#define  USE64(x) ((u64*)&x)

/// Enable the interrupt from the radio transceiver.
#define hal_enable_trx_interrupt( ) HAL_ENABLE_RADIO_INTERRUPT( )

/// Disable the interrupt from the radio transceiver.
#define hal_disable_trx_interrupt( ) HAL_DISABLE_RADIO_INTERRUPT( )

/// @name Macros for radio operation.
/// @{
#ifdef SINGLE_CHIP
#define HAL_PLL_LOCK_MASK      ( 0x01 ) //!< Mask for the PLL_LOCK interrupt. -> not used
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) //!< Mask for the PLL_UNLOCK interrupt. -> not used
#define HAL_RX_START_MASK      ( 0x04 ) //!< Mask for the RX_START interrupt.
#define HAL_RX_END_MASK        ( 0x08 ) //!< Mask for the RX_END interrupt.
#define HAL_ED_READY_MASK      ( 0x10 ) //!< Mask for the ED_READY interrupt.
#define HAL_TRX_END_MASK       ( 0x40 ) //!< Mask for the TRX_END interrupt.
#define HAL_TRX_UR_MASK        ( )      //!< Mask for the TRX_UR interrupt. -> not used
#define HAL_BAT_LOW_MASK       ( )      //!< Mask for the BAT_LOW interrupt.
#else
#define HAL_PLL_LOCK_MASK      ( 0x01 ) //!< Mask for the PLL_LOCK interrupt.
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) //!< Mask for the PLL_UNLOCK interrupt.
#define HAL_RX_START_MASK      ( 0x04 ) //!< Mask for the RX_START interrupt.
#define HAL_TRX_END_MASK       ( 0x08 ) //!< Mask for the TRX_END interrupt.
#define HAL_ED_READY_MASK      ( 0x10 ) //!< Mask for the ED_READY interrupt.
#define HAL_TRX_UR_MASK        ( 0x40 ) //!< Mask for the TRX_UR interrupt.
#define HAL_BAT_LOW_MASK       ( 0x80 ) //!< Mask for the BAT_LOW interrupt.
#endif
/// @}

//#define        delay_us( us )   ( _delay_loop_2( ( F_CPU / 4000000UL ) * ( us ) ) )
#define        delay_us( us )   ( _delay_loop_2(( ( us * 2 ) / ( 8000000UL / F_CPU )) +1))

/*============================ TYPDEFS =======================================*/
//! RX_START event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_rx_start_event_handler().
typedef void (*hal_rx_start_isr_event_handler_t)(u32 const isr_timestamp,
		u8 const frame_length);

//! RRX_END event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_trx_end_event_handler().
typedef void (*hal_trx_end_isr_event_handler_t)(u32 const isr_timestamp);

typedef void (*rx_callback_t)(u16 data);

/*============================ PROTOTYPES ====================================*/
void hal_init(void);
void hal_spi_init(void);

u8 hal_register_read(u16 address);
void hal_register_write(u16 address, u8 value);
u8 hal_subregister_read(u16 address, u8 mask, u8 position);
void hal_subregister_write(u16 address, u8 mask, u8 position, u8 value);
//void hal_frame_read(void);
uint8_t* hal_frame_read(void);
void hal_frame_write(u8 *write_buffer, u8 length);
void hal_sram_read(u8 address, u8 length, u8 *data);
void hal_sram_write(u8 address, u8 length, u8 *data);

void halGetEeprom(u8 *addr, u8 length, u8 *dest);
void halPutEeprom(u8 *addr, u8 length, u8 *src);

bool calibrate_rc_osc(void);
void halSetupClock(void);

/**
 Macro to retrieve MAC address stored in EEPROM.
 */
#define halGetMacAddr(p)  halGetEeprom(offsetof(tEepromContents, eepromMacAddress),          \
                                       sizeof(typeof(((tEepromContents*)0)->eepromMacAddress)), \
                                       p)
#define halPutMacAddr(p)  halPutEeprom(offsetof(tEepromContents, eepromMacAddress),          \
                                       sizeof(typeof(((tEepromContents*)0)->eepromMacAddress)), \
                                       p)

// Macro needed for 1287USB platform (RAVENUSB)
#define Clear_prescaler()                       (CLKPR = (1<<CLKPCE),CLKPR = 0)

#endif

/** @} */
/* /\*EOF*\/ */
