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
  $Id: serial.h,v 1.4.4.1 2009/06/04 22:04:19 mvidales Exp $
*/
#ifndef SERIAL_H
#define SERIAL_H


#define BUFSIZE 128

#if PLATFORM == RAVENUSB
  #include "ravenusb/uart_usb_lib.h"
#endif
#include <stdint.h>

#define SERIAL_ERR 0xFF

/**
   @addtogroup serial
   @{
*/
/**
   RX handler callback function definition.  Define a function like
   the one below to recieve each serial character, and pass this
   function name to the serial_init() function.

   @code
   void rxHandler(u8 ch)
   {
     // Do something with ch, which is the received character.
     // Or, more likely, set a flag to go check for
     // received serial chars in the input buffer.
   }
   @endcode

   Remember that this function is not meant to be used to process the
   serial input stream.  It is called from an interrupt, so you must
   not call application code from the RX handler function.  Use the
   serial_ischar() and serial_getchar() functions to retrieve
   serial data.
*/
typedef void (*t_rx_handler) (u8 ch);


/**
   @name Baud rate definitions

   See the appropriate AVR datasheet for details.  These macros assume
   that the AVR clock speed is 8MHz.

   @{
*/
#define BAUD_RATE_38400  (12)  ///< Register setting for 38.4 KBaud
#define BAUD_RATE_9600   (51)  ///< Register setting for 9600 Baud
/** @} */

#define SERIAL_PAYLOAD_LEN 10
typedef struct {
    u8 len;
    u8 cmd;
    u8 payload[SERIAL_PAYLOAD_LEN];
    u8 state;
} tSerialFrame;


/// @name These are GUI to Radio Binary commands.
/// @{
#define NULL_CMD                      (0)
#define CMD_TEMP                     (0x80)
#define CMD_PING_COORD               (0x81)
#define CMD_PING_GOOGLE              (0x82)
#define CMD_PING_SERVER              (0x83)
#define CMD_PING_NODE                (0x84)
#define CMD_LED                      (0x85)
/// @}

/// @name These are the Radio to GUI binary commands.
/// @{
#define REPORT_PING                   (0xC0)
#define REPORT_PING_BEEP              (0xC1)
#define REPORT_TEXT_MSG               (0xC2)
#define REPORT_ASSOCIATED             (0xC3)
#define REPORT_LED                    (0xC4)
/// @}

void serial_init(t_rx_handler rxh);
void serial_shutdown(void);
void serial_putchar(char ch);
void serial_puts(char *str);
u8 serial_ischar(void);
s8 serial_getchar(void);
u8 serial_gets(char *str, u8 len, u8 echo);
void serial_toggle_pause(void);
u8 rx_buf_count(void);
u8 tx_buf_has_char(void);
u8 get_tx_char(void);
void add_char_to_rx(u8 ch);
u8 serial_rcv_frame(void);
void serial_send_frame(u8 cmd, u8 payload_length, u8 *payload);

// Enable RX,TX and RX interrupt on USART
// 38400 baud @ 8 MHz internal RC oscillator (error = 0.2%)
// 8 bit character size, 1 stop bit and no parity mode
/**
   @name UART setup macros
   @{
*/
/// Configure the UART for use
#if F_CPU == 8000000UL
#define USART_SETUP() ((UCSR(USART,B) = (1 << RXEN(USART))|(1 << TXEN(USART))|(1 << RXCIE(USART))), \
                       (UCSR(USART,C) = (1 << UCSZ(USART,0))|(1 << UCSZ(USART,1))), \
                       (UBRR(USART,H) = BAUD_RATE_38400 >> 8, UBRR(USART,L) = BAUD_RATE_38400 & 0xff))
#endif
#if F_CPU == 4000000UL
#define USART_SETUP() ((UCSR(USART,B) = (1 << RXEN(USART))|(1 << TXEN(USART))|(1 << RXCIE(USART))), \
                       (UCSR(USART,C) = (1 << UCSZ(USART,0))|(1 << UCSZ(USART,1))), \
                       (UCSR(USART,A) = 2),                             \
                       (UBRR(USART,H) = BAUD_RATE_38400 >> 8, UBRR(USART,L) = BAUD_RATE_38400 & 0xff))
#endif
#if F_CPU == 2000000UL
#define USART_SETUP() ((UCSR(USART,B) = (1 << RXEN(USART))|(1 << TXEN(USART))|(1 << RXCIE(USART))), \
                       (UCSR(USART,C) = (1 << UCSZ(USART,0))|(1 << UCSZ(USART,1))), \
                       (UBRR(USART,H) = BAUD_RATE_38400 >> 8, UBRR(USART,L) = BAUD_RATE_38400 & 0xff))
#endif
#if F_CPU == 1000000UL
#define USART_SETUP() ((UCSR(USART,B) = (1 << RXEN(USART))|(1 << TXEN(USART))|(1 << RXCIE(USART))), \
                       (UCSR(USART,C) = (1 << UCSZ(USART,0))|(1 << UCSZ(USART,1))), \
                       (UCSR(USART,A) = 2),                             \
                       (UBRR(USART,H) = BAUD_RATE_38400 >> 8, UBRR(USART,L) = BAUD_RATE_38400 & 0xff))
#endif


/// Shutdown the UART module
#define USART_STOP() UCSR(USART,B) = 0
/// Is there a received character waiting in the UART?
#define USART_CHAR_READY() (!(UCSR(USART,B) & (1<<UDRIE(USART))))
/// Wait for the Data Register to be empty
#define USART_DRE_WAIT() (!(UCSR(USART,A) & (1<<UDRE(USART))))
///  UART data register
#define USART_REG (UDR(USART))
/// Eanble the data register empy interrupt
#define USART_DRE_ENABLE() (UCSR(USART,B) |= (1<<UDRIE(USART)))
/// Disable the data register empy interrupt
#define USART_DRE_DISABLE() ((UCSR(USART,B) &= ~(1<<UDRIE(USART))))
/// UART Rx vector
#define USARTRXVECT RXVECT(USART)
/// UART Tx vector
#define USARTTXVECT UDREVECT(USART)   // IAR doesn't accept this macro inside the ISR() macro
/** @} */
/** @} */

#endif //SERIAL_H
