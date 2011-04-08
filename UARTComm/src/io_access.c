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
 * @file io_access.c
 *
 * @brief I/O related functions on dresden elektronik platforms.
 *
 * This file implements I/O specific functions to get access to resources
 * of dresden elektronik platforms (e.g. Radio Controller Board, Sensor Terminal Board).
 *
 * $Id: io_access.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2009-04-14
 */

/* === Includes ============================================================= */

#include <stdint.h>
#include <avr/io.h>

#include "../inc/io_access.h"


/* === Macros =============================================================== */

/* === Globals ============================================================== */

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
 * This is the LED/BUTTON fifo address -> for STB access with MEGA1281
 */
#ifndef SINGLE_CHIP
static volatile unsigned char* pPERIPHERAL = (unsigned char *) PERIPHERAL_XRAM_ADDRESS;
#endif

/**
 * RCB LED and RCB button state are stored
 */
static uint8_t PERIPHERAL = 0x00;

/**
 * @brief Check if button on specified platform is pressed.
 *
 * @param platform   specify platform where to check button state
 *
 * @return           button state pressed (1) or unpressed (0)
 */
uint8_t button_pressed(uint8_t platform)
{
   if(platform == PLATFORM_RCB)
   {
      /* Switch port to input. */
      RCB_BUTTON_PORT |= (1 << RCB_BUTTON_0);
      RCB_BUTTON_DDR  &= ~(1 << RCB_BUTTON_0);

      return (!(RCB_BUTTON_PINS & (1 << RCB_BUTTON_0)) ? 1 : 0);
   }
   else if(platform == PLATFORM_STB)
   {
#ifdef SINGLE_CHIP
      uint8_t data = 0;

      // save registers to restore later
      uint8_t sControlPORT = STB_CONTROL_PORT;
      uint8_t sControlDDR  = STB_CONTROL_DDR;

      // set #WR bit inactive
      STB_CONTROL_PORT |= _BV(STB_WR_BIT);
      STB_CONTROL_DDR  |= _BV(STB_WR_BIT);

      // prepare chip select for io
      STB_SELECT_PORT |=  _BV(STB_CS0_BIT); //CS0 bit
      STB_SELECT_DDR  |=  _BV(STB_CS0_BIT);
      STB_SELECT_PORT &= ~_BV(STB_CS1_BIT); //CS1 bit
      STB_SELECT_DDR  |=  _BV(STB_CS1_BIT);

      // generate a single low / hi edge RD
      STB_CONTROL_PORT |= _BV(STB_RD_BIT); // #RD bit
      STB_CONTROL_DDR  |= _BV(STB_RD_BIT);
      // and now back ...
      STB_CONTROL_PORT &= ~_BV(STB_RD_BIT);

      // Switch port to input
      STB_DATA_DDR  &= ~_BV(STB_BUTTON_0);
      STB_DATA_PORT |=  _BV(STB_BUTTON_0);

      // read input pin
      data = STB_DATA_PINS;

      // disable input read
      STB_CONTROL_PORT |= _BV(STB_RD_BIT);

      // Switch port back to output
      STB_DATA_DDR |= _BV(STB_BUTTON_0);

      // disable chip select
      STB_SELECT_DDR  |=  _BV(STB_CS0_BIT);
      STB_SELECT_PORT &= ~_BV(STB_CS0_BIT);
      STB_SELECT_DDR  |=  _BV(STB_CS1_BIT);
      STB_SELECT_PORT &= ~_BV(STB_CS1_BIT);

      // restore registers
      STB_CONTROL_PORT  = sControlPORT;
      STB_CONTROL_DDR   = sControlDDR;

      return ((data & _BV(STB_BUTTON_0)) ? 1 : 0);
#else
      return ((*pPERIPHERAL & _BV(STB_BUTTON_0)) ? 1 : 0);
#endif
      return 0;
   }

   return 0;
}

/**
 * @brief Set LED on specified platform.
 *
 * @param platform specify platform where LED should be set (PLATFORM_RCB and PLATFORM_STB)
 * @param led LED which should set
 * @param mode switch mode from LED (PLATFORM_LED_ON, PLATFORM_LED_OFF and PLATFORM_LED_TOGGLE)
 */
void led_set (uint8_t platform, uint8_t led, uint8_t mode)
{
   if(platform == PLATFORM_RCB)
   {
      switch(mode)
      {
      case PLATFORM_LED_ON:
         RCB_LED_PORT &= ~(1 << (led));
         RCB_LED_DDR  |=  (1 << (led));
         break;
      case PLATFORM_LED_OFF:
         RCB_LED_PORT |= (1 << (led));
         RCB_LED_DDR  |= (1 << (led));
         break;
      case PLATFORM_LED_TOGGLE:
         RCB_LED_PORT ^= (1 << (led));
         RCB_LED_DDR  |= (1 << (led));
         break;
      }
   }
   else if(platform == PLATFORM_STB)
   {
#ifdef SINGLE_CHIP
      switch(mode)
      {
      case PLATFORM_LED_ON:
         PERIPHERAL &= ~_BV(led);
         break;
      case PLATFORM_LED_OFF:
         PERIPHERAL |= _BV(led);
         break;
      }
      // save registers to restore later
      uint8_t sControlPORT = STB_CONTROL_PORT;
      uint8_t sControlDDR  = STB_CONTROL_DDR;
      uint8_t sDataPORT    = STB_DATA_PORT;
      uint8_t sDataDDR     = STB_DATA_DDR;

      // set #RD bit inactive
      STB_CONTROL_PORT |= _BV(STB_RD_BIT);
      STB_CONTROL_DDR  |= _BV(STB_RD_BIT);

      STB_DATA_PORT = PERIPHERAL; // low active
      STB_DATA_DDR |= _BV(STB_LED_0); // set LED0 output active
      STB_DATA_DDR |= _BV(STB_LED_1); // set LED1 output active

      // prepare chip select for io
      STB_SELECT_PORT |=  _BV(STB_CS0_BIT); //CS0 bit
      STB_SELECT_DDR  |=  _BV(STB_CS0_BIT);
      STB_SELECT_PORT &= ~_BV(STB_CS1_BIT); //CS1 bit
      STB_SELECT_DDR  |=  _BV(STB_CS1_BIT);

      // generate a single low / hi edge WR
      STB_CONTROL_PORT |= _BV(STB_WR_BIT); // #WR bit
      STB_CONTROL_DDR  |= _BV(STB_WR_BIT);
      // and now back ...
      STB_CONTROL_PORT &= ~_BV(STB_WR_BIT);
      STB_CONTROL_PORT |= _BV(STB_WR_BIT);

      // disable chip select
      STB_SELECT_PORT &= ~_BV(STB_CS0_BIT);
      STB_SELECT_DDR  |=  _BV(STB_CS0_BIT);
      STB_SELECT_PORT &= ~_BV(STB_CS1_BIT);
      STB_SELECT_DDR  |=  _BV(STB_CS1_BIT);

      // restore registers
      STB_CONTROL_PORT  = sControlPORT;
      STB_CONTROL_DDR   = sControlDDR;
      STB_DATA_PORT     = sDataPORT;
      STB_DATA_DDR      = sDataDDR;
#else
      switch(mode)
      {
      case PLATFORM_LED_ON:
         PERIPHERAL |= _BV(led);
         break;
      case PLATFORM_LED_OFF:
         PERIPHERAL &= ~_BV(led);
         break;
      }
      *pPERIPHERAL = ~PERIPHERAL | ~0x07; // Memory mapped IO
#endif
   }
}


/**
 * @brief Activate/Deactivate temperature sensor on Sensor Terminal Board
 *
 * @param mode activate/deactivate temperature sensor (STB_TEMPERATURE_ON and STB_TEMPERATURE_OFF)
 */
void stb_status_temperature(uint8_t mode)
{
   if(mode == STB_TEMPERATURE_ON)
   {
#ifdef SINGLE_CHIP
      PERIPHERAL &= ~_BV(2);
#else
      PERIPHERAL |= _BV(2);
#endif
   }
   else if(mode == STB_TEMPERATURE_OFF)
   {
#ifdef SINGLE_CHIP
      PERIPHERAL |= _BV(2);
#else
      PERIPHERAL &= ~_BV(2);
#endif
   }
#ifdef SINGLE_CHIP
   // save registers to restore later
   uint8_t sControlPORT = STB_CONTROL_PORT;
   uint8_t sControlDDR  = STB_CONTROL_DDR;
   uint8_t sDataPORT    = STB_DATA_PORT;
   uint8_t sDataDDR     = STB_DATA_DDR;

   // set #RD bit inactive
   STB_CONTROL_PORT |= _BV(STB_RD_BIT);
   STB_CONTROL_DDR  |= _BV(STB_RD_BIT);

   STB_DATA_PORT   = PERIPHERAL; // low active
   STB_DATA_DDR   |= _BV(STB_TEMPERATURE); // set temperature output active

   // prepare chip select for io
   STB_SELECT_PORT |=  _BV(STB_CS0_BIT); //CS0 bit
   STB_SELECT_DDR  |=  _BV(STB_CS0_BIT);
   STB_SELECT_PORT &= ~_BV(STB_CS1_BIT); //CS1 bit
   STB_SELECT_DDR  |=  _BV(STB_CS1_BIT);

   // generate a single low / hi edge WR
   STB_CONTROL_PORT |= _BV(STB_WR_BIT); // #WR bit
   STB_CONTROL_DDR  |= _BV(STB_WR_BIT);
   // and now back ...
   STB_CONTROL_PORT &= ~_BV(STB_WR_BIT);
   STB_CONTROL_PORT |= _BV(STB_WR_BIT);

   // disable chip select
   STB_SELECT_PORT &= ~_BV(STB_CS0_BIT);
   STB_SELECT_DDR  |=  _BV(STB_CS0_BIT);
   STB_SELECT_PORT &= ~_BV(STB_CS1_BIT);
   STB_SELECT_DDR  |=  _BV(STB_CS1_BIT);

   // set temperature output inactive
   STB_DATA_DDR    &= ~_BV(STB_TEMPERATURE);

   // restore registers
   STB_CONTROL_PORT  = sControlPORT;
   STB_CONTROL_DDR   = sControlDDR;
   STB_DATA_PORT     = sDataPORT;
   STB_DATA_DDR      = sDataDDR;
#else
   *pPERIPHERAL = ~PERIPHERAL | ~0x07; // Memory mapped IO
#endif
}

/* EOF */

