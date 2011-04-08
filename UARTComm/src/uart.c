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
 * @file uart.c
 *
 * @brief UART related functions for AVR 8-Bit MCUs
 *
 * This file implements UART related transmission and reception
 * functions for AVR 8-Bit MCUs.
 *
 * The debug module is realized through UART module. To enable debugging use UART_DEBUG
 * compiler directive. In uart.h you can enable the UART_QUEUED macro to have a buffered
 * debugging transmission. This make use of the queue module. The debugging messages are
 * not printed directly but with help of a buffered queue. This prevents timing interferences.
 * To print out debugging messages use the UART_PRINT() macro.
 *
 * $Id: uart.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-01-25
 */


/* === Includes ============================================================= */

#include <stdio.h>
#include <avr/io.h>

#include "../inc/uart.h"

/* === Macros =============================================================== */

/* === Globals ============================================================== */

#ifdef UART_DEBUG
#if UART_QUEUED
// Setup a File stream with putchar and getchar functionality over queued UART
FILE debug_stream = FDEV_SETUP_STREAM(queue_put_char_std, NULL, _FDEV_SETUP_WRITE);
#else
// Setup a File stream with putchar and getchar functionality over UART
FILE uart_stream = FDEV_SETUP_STREAM(uart_putc, uart_getc,_FDEV_SETUP_RW);
#endif // UART_QUEUED
#endif

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
 * @brief Initialize UART interface
 *
 */
void uart_init (unsigned long BaudRate)
{
#ifdef RCB_BREAKOUT
   // init MAX3221
#if defined(__AVR_ATmega1281__)
   DDRC &= ~0xF0;
   DDRC |= _BV(7) | _BV(6) | _BV(4);
   PORTC &= ~0xF0;
   PORTC |= _BV(7);
#endif
#if defined(__AVR_ATmega128RFA1__)
   DDRD &= ~0xF0;
   DDRD |= _BV(7) | _BV(6) | _BV(4);
   PORTD &= ~0xF0;
   PORTD |= _BV(7);
#endif
   UCSR1A = (1<<U2X1);                              // enable double speed

   UBRR1L = (UART_BAUD(BaudRate)) & 0xFF;           // set the baudrate register
   UBRR1H = 0;

   UCSR1B = (1<<RXEN1) | (1<<TXEN1);                // enable Receiver and Transmitter
   UCSR1C = (3<<UCSZ10);                            // 8 Data, No Parity, 1 Stop bit
#else
   UCSR0A = (1<<U2X0);                              // enable double speed

   UBRR0L = (UART_BAUD(BaudRate)) & 0xFF;           // set the baudrate register
   UBRR0H = 0;

   UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);  // enable Receiver and Transmitter and RX Interrupt
   UCSR0C = (3<<UCSZ00);                            // 8 Data, No Parity, 1 Stop bit
#endif

#ifdef UART_DEBUG
#if UART_QUEUED
   stderr = &debug_stream;
#else
   stdout = &uart_stream;                           // init standard output over UART
   stdin  = &uart_stream;                           // init standard input over UART
#endif //UART_QUEUED
#endif
}

/**
 * @brief Returns true if the UART has a character available for read
 *
 * @return    0 : No Char available, 1 : a char is available
 *
 */
uint8_t uart_keypressed (void)
{
#ifdef RCB_BREAKOUT
   return ((UCSR1A & _BV(RXC1)) != 0) ? 1 : 0; // data received? return true
#else
   return ((UCSR0A & _BV(RXC0)) != 0) ? 1 : 0; // data received? return true
#endif
}

/**
 * @brief Waits till there is a character available on the UART and gives it back.
 *
 * Waits forever when no char comes in. This is the standard I/O implementation
 * that can be used for standard I/O channels (e.g. printf).
 *
 * @param   dummy_file not used
 *
 * @return  character on the UART
 *
 */
int uart_getc (FILE *dummy_file)
{
   while (!uart_keypressed());  // check for incomming data
#ifdef RCB_BREAKOUT
   return UDR1;  // Return the data
#else
   return UDR0;  // Return the data
#endif
}

/**
 * @brief Waits till there is a character available on the UART and gives it back.
 *
 * Waits forever when no char comes in. This is the standard I/O implementation
 * that can be used for standard I/O channels (e.g. printf).
 *
 * @param   dummy_file not used
 *
 * @return  character on the UART
 *
 */
int uart_getc_wait (FILE *dummy_file)
{
   while (!uart_keypressed());  // check for incomming data
#ifdef RCB_BREAKOUT
   return UDR1;  // Return the data
#else
   return UDR0;  // Return the data
#endif
}

/**
 * @brief Waits till there is a character available on the UART and gives it back.
 *
 * Waits forever when no char comes in.
 *
 * @return  character on the UART
 *
 */
int16_t uart_getc_std (void)
{
   return uart_getc(NULL);
}

/**
 * @brief Sends a character over the UART connection
 *
 * Waits while the UART anounces a full FIFO buffer. This is the standard I/O implementation
 * that can be used for standard I/O channels (e.g. printf).
 *
 * @param     c          the char to send
 * @param     dummy_file not used
 *
 */
int uart_putc (char c, FILE *dummy_file)
{
#ifdef RCB_BREAKOUT
   while ( !(UCSR1A & _BV(UDRE1)) );                // wait for empty tx buffer
   UDR1 = c;                                        // Start transmittion
   return c;                                        // return the char
#else
   while ( !(UCSR0A & _BV(UDRE0)) );                // wait for empty tx buffer
   UDR0 = c;                                        // Start transmittion
   return c;                                        // return the char
#endif
}

/**
 * @brief Sends a character over the UART connection.
 *
 * Waits while the UART anounces a full FIFO buffer
 *
 * @param     val        the char to send
 *
 */
uint8_t uart_putc_std (uint8_t val)
{
   return (uint8_t)uart_putc((uint8_t)val, NULL);
}



/* EOF */

