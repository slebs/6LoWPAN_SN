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
 * @file uart.h
 *
 * @brief Header file for uart module
 *
 * $Id: uart.h,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-01-25
 */

#ifndef UART_H_
#define UART_H_

#include <stdio.h>

#define  UART_QUEUED          (0)   ///< Enable (1) or Disable (0) queued debugging (if enabled, chars not printed directly but over queue task)

#if UART_QUEUED
#include "../../inc/deRFaddon/queue.h"
#endif

#if UART_QUEUED
#define  UART_QUEUED_TASK()    queue_task()
#else
#define  UART_QUEUED_TASK()
#endif

#ifdef UART_DEBUG
#if (UART_QUEUED)
#define UART_PRINT(...) (fprintf(stderr, __VA_ARGS__))
#define UART_PRINT_HEX(x) (fprintf(stderr, "%x", x))
#define UART_PRINT_FLOAT(x) (fprintf(stderr, "%f", x))
#else
#define UART_PRINT(...) (printf(__VA_ARGS__))
#define UART_PRINT_HEX(x) (printf("%x", x))
#define UART_PRINT_FLOAT(x) (printf("%f", x))
#endif //UART_QUEUED
#else
#define UART_PRINT(...)
#define UART_PRINT_HEX(x)
#define UART_PRINT_FLOAT(x)
#endif //UART_DEBUG

/*
 * Calculate the UART baud rate generator prescaler, based on the
 * global F_CPU setting, and the baud rate passed as parameter.  This
 * assumes the U2X bit will always be set.
 */
#define UART_BAUD(rate)    (((F_CPU) + 4UL * (rate)) / (8UL * (rate)) - 1UL)

/* === Macros =============================================================== */

void uart_init (unsigned long BaudRate);
uint8_t uart_keypressed (void);
int uart_getc (struct __file * dummy_file);
int16_t uart_getc_std (void);
int uart_putc (char c, struct __file * dummy_file);
uint8_t uart_putc_std (uint8_t val);


#endif /* UART_H_ */
