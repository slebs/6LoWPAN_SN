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
 * @file usb.h
 *
 * @brief Header file for USB module
 *
 * $Id: usb.h,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2009-11-02
 */

#ifndef USB_H_
#define USB_H_

/* === Includes ============================================================= */

#include <stdio.h>

/* === Macros =============================================================== */

#if defined(COMMUNICATION_USB) || defined(DOXYGEN)

/**
 * @name USB pin and port mapping to FTDI chip
 * @{
 */
#define USB_PORT                (PORTE)      ///< USB port register (FTDI chip connection)

#define USB_DDR                 (DDRE)       ///< USB port direction register (FTDI chip connection)

#define USB_PIN                 (PINE)       ///< USB port pin register (FTDI chip connection)

#define USB_RXF                 (0x80)       ///< Port pin which gives indication of reception of byte (PE7)

#define USB_TXE                 (0x40)       ///< Port pin which gives indication of transmission of byte (PE6)
/** @} */

/**
 * @name USB specific control and select pin and port mapping on Sensor Terminal Board
 * @{
 */
#define USB_STB_WR_BIT            (PE4)       ///< pin mapping of STB WR bit
#define USB_STB_RD_BIT            (PE5)       ///< pin mapping of STB RD bit
#define USB_STB_CONTROL_PORT      (PORTE)     ///< port mapping of STB control port
#define USB_STB_CONTROL_DDR       (DDRE)      ///< data direction register mapping of STB control port

#define USB_STB_CS0_BIT           (PD6)       ///< pin mapping of STB Chip Select 0 bit
#define USB_STB_CS1_BIT           (PD7)       ///< pin mapping of STB Chip Select 1 bit
#define USB_STB_SELECT_PORT       (PORTD)     ///< port mapping of STB select port
#define USB_STB_SELECT_DDR        (DDRD)      ///< data direction register mapping of STB select port

#define USB_STB_DATA_PORT         (PORTB)     ///< port mapping of STB data port
#define USB_STB_DATA_DDR          (DDRB)      ///< data direction register mapping of STB data port
#define USB_STB_DATA_PINS         (PINB)      ///< input register mapping of STB data port

/** @} */

#ifndef SINGLE_CHIP

/*
 * @name USB address space when using external memory interface on ATMega1281
 * @{
 */
#define USB_FIFO_AD             (0x2200)     ///< Memory address mapped to USB FIFO

#define pUSB_Fifo ((volatile uint8_t *)USB_FIFO_AD)   ///< This is the usb fifo address
/** @} */

#endif // SINGLE_CHIP

#endif //COMMUNICATION_USB

/* === Globals ============================================================== */

/* === Prototypes =========================================================== */

int usb_putc_std (char c, FILE *dummy_file);
void usb_init(void);
uint8_t usb_keypressed (void);
int16_t usb_getc_std(void);
uint8_t usb_putc(uint8_t val);



#endif /* USB_H_ */
