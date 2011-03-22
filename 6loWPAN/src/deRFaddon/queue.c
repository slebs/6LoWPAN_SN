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
 * @file queue.c
 *
 * @brief Offer queue functions and string convertions.
 *
 * This file implements a Queue structure and conversion functions from integer to string.
 *
 * The queue is organized as FIFO buffer. The first element pushed on queue is pushed out first.
 *
 * The queue is processed within a task, which have to be executed in your function.
 *
 * $Id: queue.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-04-01
 */


/* === Includes ============================================================= */

#include <stdlib.h>

#include "../../inc/hal_avr.h"
#include "../../inc/deRFaddon/queue.h"
#include "../../inc/deRFaddon/uart.h"

/* === Macros =============================================================== */

/*
 * maximum number of characters the buffer can hold
 */
#define MAX_CHARS          (1000)

/* === Globals ============================================================== */

/**
 * @brief Structure that defines the uart queue.
 */
typedef struct {
   volatile uint8_t head;   ///< The index of the last available event
   volatile uint8_t tail;   ///< The index of the first available event
   uint8_t buffer[MAX_CHARS]; ///< List of events in queue
} uart_queue_t;

uart_queue_t uart_queue; ///< The uart queue itself

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
 * @brief Check if any char is available to send
 *
 * @return if any char is available it return 1, else 0
 */
uint8_t queue_char_pending(void)
{
   return (uart_queue.head != uart_queue.tail);
}

/**
 * @brief Put char into buffer
 *
 * @param   c  character for buffer
 */
void queue_put_char(uint8_t c)
{
   AVR_ENTER_CRITICAL_REGION();
   uint8_t newhead;

   if ((uart_queue.head + 1) % MAX_CHARS == uart_queue.tail)
   {
      // queue full, get outta here
      return;
   }

   newhead = uart_queue.head;

   // store in queue
   uart_queue.buffer[newhead] = c;

   // calculate new head index
   newhead++;
   if (newhead >= MAX_CHARS)
   {
      newhead = 0;
   }
   uart_queue.head = newhead;
   AVR_LEAVE_CRITICAL_REGION();
}

/**
 * @brief Put char into buffer.
 *
 * This is the standard I/O implementation, so it can be used for printf facilities.
 *
 * @param   c           character for buffer
 * @param   dummy_file  not used
 */
int queue_put_char_std (char c, FILE *dummy_file)
{
   queue_put_char((uint8_t)c);
   return c;
}

/**
 * @brief Return oldest character
 *
 * @return  oldest character
 */
uint8_t queue_get_char(void)
{
   uint8_t c;
   AVR_ENTER_CRITICAL_REGION();
   volatile uint8_t newtail;

   newtail = uart_queue.tail;

   c = uart_queue.buffer[newtail];

   // calculate new tail
   newtail++;
   if (newtail >= MAX_CHARS)
   {
      newtail = 0;
   }

   uart_queue.tail = newtail;

   AVR_LEAVE_CRITICAL_REGION();
   return(c);
}

/**
 * @brief Return oldest character-
 *
 * Implemented as standard I/O, so it can be used for printf facilities.
 *
 * @param   dummy_file  not used
 *
 * @return  oldest character
 */
int queue_get_char_std(FILE* dummy_file)
{
   return queue_get_char();
}

/**
 * @brief Method to push complete string into buffer
 *
 * A character string of specified length is written into buffer
 *
 * @param str  Pointer to String
 * @param len  length of String
 */
void queue_put_string(char* str, uint8_t len)
{
   while(len > 0)
   {
      queue_put_char(*str);
      str++;
      len--;
   }
   /*
   uint8_t i = 0;

   while(*str != '\0')
   {
      queue_put_char(*str);
      i++;
   }
   */
}

/**
 * @brief Convert byte to string and push on queue
 *
 * @param n unsigned byte
 */
void queue_put_dec8(uint8_t n)
{
   char buf[4]; // 3*max number (256) + 1* zero terminated string [+ 1* minus sign]
   // 32 = space character
   buf[0] = 32; buf[1] = 32; buf[2] = 32; buf[3] = 32;
   itoa(n, buf, 10);
   queue_put_string(buf, sizeof(buf));
}

/**
 * @brief Convert int to string and push on queue
 *
 * @param n unsigned integer
 */
void queue_put_dec16(uint16_t n)
{
   char buf[6]; // 5*max number (65536) + 1* zero terminated string [+ 1* minus sign]
   // 32 = space character
   buf[0] = 32; buf[1] = 32; buf[2] = 32; buf[3] = 32; buf[4] = 32; buf[5] = 32;
   itoa(n, buf, 10);
   queue_put_string(buf, sizeof(buf));
}

/**
 * @brief Convert long int to string and push on queue
 *
 * @param n unsigned long integer
 */
void queue_put_dec32(uint32_t n)
{
   char buf[11]; // 10*max number (2^32) + 1* zero terminated string [+ 1* minus sign]
   uint8_t i = 0;
   for(; i < sizeof(buf); i++)
   {
      buf[i] = 32; // 32 = space character
   }
   itoa(n, buf, 10);
   queue_put_string(buf, sizeof(buf));
}

/*
 * @brief Task which is executed in free time, to send out all saved chars from queue.
 *
 */
void queue_task(void)
{
   while(queue_char_pending())
   {
      uart_putc(queue_get_char(), NULL);
   }
}

/* EOF */

