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
 * @file usb.c
 *
 * @brief USB related functions for AVR 8-Bit MCUs
 *
 * This file implements USB (FTDI) related transmission and reception
 * functions for AVR 8-Bit MCUs.
 *
 * $Id: usb.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2009-11-30
 */


/* === Includes ============================================================= */

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "../../inc/deRFaddon/usb.h"

/* === Macros =============================================================== */

/* === Globals ============================================================== */

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
 * @brief Initializes USB port
 *
 * This function initializes the USB port. The receive and transmit interrupt
 * pins are made as input pins.
 */
void usb_init(void)
{
#ifdef COMMUNICATION_USB
	/* Make sure USB_RXF and USB_TXE are inputs */
	USB_DDR  &= ~(USB_RXF | USB_TXE);           // USB's status signals are inputs
	USB_PORT |=  (USB_RXF | USB_TXE);           // switch internal pull up resitors on
#endif
}

/**
 * @brief Check if data from USB interface is available
 */
uint8_t usb_keypressed (void)
{
#ifdef COMMUNICATION_USB
	/* Make sure USB_RXF and USB_TXE are inputs */
	//USB_DDR  &= ~(USB_RXF | USB_TXE);               // USB's status signals are inputs
	//USB_PORT |=  (USB_RXF | USB_TXE);               // switch internal pull up resitors on
	return (!(USB_PIN & USB_RXF)) ? 0x01 : 0x00;   // FIFO is not full? return true
#endif // COMMUNICATION_USB
	return 0;
}

/**
 * @brief Sends a character over the USB connection
 *
 * Waits while the USB chip anounces a full FIFO buffer. This is the standard I/O
 * implementation to use standard I/O channels (printf, etc.)
 *
 * @param     c          the char to send
 * @param     dummy_file not used
 *
 */
int usb_putc_std (char c, FILE * dummy_file)
{
#ifdef COMMUNICATION_USB
   /* Make sure USB_RXF and USB_TXE are inputs */
   //USB_DDR  &= ~(USB_RXF | USB_TXE);           // USB's status signals are inputs
   //USB_PORT |=  (USB_RXF | USB_TXE);           // switch internal pull up resitors on

   while (USB_PIN & USB_TXE){;}           // Wait for empty transmit buffer
#ifdef SINGLE_CHIP
   USB_STB_DATA_PORT = c;
   USB_STB_DATA_DDR  = 0xff;  // all bits are outputs
   // prepare chip select for usb
   USB_STB_SELECT_PORT &= ~_BV(USB_STB_CS0_BIT);
   USB_STB_SELECT_DDR  |=  _BV(USB_STB_CS0_BIT);
   USB_STB_SELECT_PORT &= ~_BV(USB_STB_CS1_BIT);
   USB_STB_SELECT_DDR  |=  _BV(USB_STB_CS1_BIT);

   // generate a single low / hi edge WR
   USB_STB_CONTROL_PORT |=  _BV(USB_STB_WR_BIT);
   USB_STB_CONTROL_DDR  |=  _BV(USB_STB_WR_BIT);
   /* and now back ... */
   USB_STB_CONTROL_PORT &= ~_BV(USB_STB_WR_BIT);
   USB_STB_CONTROL_PORT |=  _BV(USB_STB_WR_BIT);

   // disable chip select
   USB_STB_SELECT_PORT &= ~_BV(USB_STB_CS0_BIT);
   USB_STB_SELECT_DDR  |=  _BV(USB_STB_CS0_BIT);
   USB_STB_SELECT_PORT &= ~_BV(USB_STB_CS1_BIT);
   USB_STB_SELECT_DDR  |=  _BV(USB_STB_CS1_BIT);
#else
   *pUSB_Fifo = c;                                  // write the byte into the USB FIFO
#endif
   return c;                                        // return the char
#endif // COMMUNICATION_USB
   return 0;
}

/**
 * @brief Sends a character over the USB connection
 *
 * None standard I/O implementation. Standard I/O channels can not use function (printf, etc.)
 *
 * @param   c   the char to send
 *
 */
uint8_t usb_putc (uint8_t c)
{
#ifdef COMMUNICATION_USB
   return usb_putc_std(c, NULL);
#endif // COMMUNICATION_USB
   return 0;

}

/**
 * @brief Receives data from USB interface
 *
 * Standard I/O implementation for standard I/O channels usage (printf, etc.)
 *
 */
int16_t usb_getc_std(void)
{
#ifdef COMMUNICATION_USB
	/* Make sure USB_RXF and USB_TXE are inputs */
	//USB_DDR  &= ~(USB_RXF | USB_TXE);           // USB's status signals are inputs
	//USB_PORT |=  (USB_RXF | USB_TXE);           // switch internal pull up resitors on

	while (USB_PIN & USB_RXF){;}   // wait until new char received
#ifdef SINGLE_CHIP

	int16_t data = 0;
	// prepare data
	USB_STB_DATA_DDR   =   0x00;	// all bits are inputs
	// prepare chip select for usb
   USB_STB_SELECT_PORT &= ~_BV(USB_STB_CS0_BIT);
   USB_STB_SELECT_DDR  |=  _BV(USB_STB_CS0_BIT);
   USB_STB_SELECT_PORT &= ~_BV(USB_STB_CS1_BIT);
   USB_STB_SELECT_DDR  |=  _BV(USB_STB_CS1_BIT);

	// generate a single low / hi edge RD
   USB_STB_CONTROL_PORT |=  _BV(USB_STB_RD_BIT);
   USB_STB_CONTROL_DDR  |=  _BV(USB_STB_RD_BIT);
	USB_STB_CONTROL_PORT &= ~_BV(USB_STB_RD_BIT);

	// give chip a chance to put out the data
	_delay_us(5);
	// read data
	data =  USB_STB_DATA_PINS;

	// stop reading from USB chip
	USB_STB_CONTROL_PORT |=  _BV(USB_STB_RD_BIT);

	// disable chip select
   USB_STB_SELECT_PORT &= ~_BV(USB_STB_CS0_BIT);
   USB_STB_SELECT_DDR  |=  _BV(USB_STB_CS0_BIT);
   USB_STB_SELECT_PORT &= ~_BV(USB_STB_CS1_BIT);
   USB_STB_SELECT_DDR  |=  _BV(USB_STB_CS1_BIT);
	return(data);
#else
	return *pUSB_Fifo;                     // return received char
#endif // SINGLE_CHIP
#endif // COMMUNICATION_USB
	return 0;
}

/* EOF */

