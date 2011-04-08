/*
 * uart.h
 *
 *  Created on: 18.02.2011
 *      Author: Kevin
 */

#ifndef UART_H_
#define UART_H_

#include <stdio.h>

#define  UART_QUEUED          (0)   ///< Enable (1) or Disable (0) queued debugging (if enabled, chars not printed directly but over queue task)

#if UART_QUEUED
#include "C:/Eclipse/workspace/projekt/6LoWPAN/src/inc/deRFaddon/queue.h"
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
