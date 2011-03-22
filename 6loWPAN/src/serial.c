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
  $Id: serial.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
*/
#include <stdio.h>
#include "../inc/hal.h"
#include "../inc/serial.h"
#include "../inc/mac.h"
#include "../inc/mac_event.h"
#include "../inc/system.h"

// Raven USB uses extra USB code
#if PLATFORM == RAVENUSB
#include "ravenusb/cdc_task.h"
#else
#define cdc_task()
#define uart_usb_init()
#endif

// Don't include any of this serial code unless we are in debug mode
#if  SERIAL || DOXYGEN

/**
   @addtogroup serial
   @{

   This file implements a stand-alone serial port interface for the
   AVR.  Input and output buffering are implemented.  The port is set
   for 38.4KBPS, no parity, 8 bits, and 1 stop bit.

   To use this serial port, call serial_init() once.  Then you can
   send data over the serial port using serial_putchar() and
   serial_puts().

   To receive serial data, you can call serial_getchar() and
   serial_gets(), but beware that these are blocking functions, and
   will simply wait for an incoming character, preventing any other
   activity on the AVR.  To prevent this, you can either poll the
   serial buffer with the serial_ischar() function, or you can set
   an RX interrupt callback function to be called every time a
   character is received.  Set this callback function as a parameter
   to the serial_init() function.
*/

typedef struct {
    volatile u8 head;
    volatile u8 tail;
    u8 buf[BUFSIZE];
} __attribute__((packed)) tcirc_buf;


static tcirc_buf txbuf;
static tcirc_buf rxbuf;
static t_rx_handler rx_handler;
static u8 rx;
static u8 pause; // Used to pause serial output

enum tSerialStates {sof=0x33, len, cmd, payload, eof};

#if PLATFORM == RAVEN
tSerialFrame SerialFrame[1] = {{ .state=sof}};
#else
tSerialFrame SerialFrame[0];
#endif

#define SOF_CHAR            (0x01)  ///< Start-of-frame character.
#define EOF_CHAR            (0x04)  ///< End-of-frame character.
#define TERM_CHAR           (0x14)  ///< Control-T ASCII value for entering terminal mode.


/*
    @brief Initializes the circular buffer.

    @param cbuf Points to the structure to be initialized.
*/
static void init_circ_buf(tcirc_buf *cbuf)
{
    cbuf->head = cbuf->tail = 0;
}

/*
    @brief This will add a character to the circular buffer.

    @param cbuf Pointer to structure that contains data.
    @param ch Character to add to buffer.
*/
static void add_to_circ_buf(tcirc_buf *cbuf, u8 ch)
{
    // Add char to buffer
    u8 newhead = cbuf->head;
    newhead++;
    if (newhead >= BUFSIZE)
        newhead = 0;
    while (newhead == cbuf->tail)
        if (rx)
            // In Rx mode, waiting won't help, we're inside of interrupt!!
            return;
        else
            cdc_task();


    cbuf->buf[cbuf->head] = ch;
    cbuf->head = newhead;
}

/*
    @brief This will get a character out of the circular buffer.

    @param cbuf Pointer to structure that contains data.

    @return retval Character to be returned from buffer.  If no char
    is available, returns SERIAL_ERR.
*/
static u8 get_from_circ_buf(tcirc_buf *cbuf)
{
    // Get char from buffer
    // Be sure to check first that there is a char in buffer
    u8 newtail = cbuf->tail;
    u8 retval = cbuf->buf[newtail];

    if (newtail == cbuf->head)
        return SERIAL_ERR;

    newtail++;
    if (newtail >= BUFSIZE)
        // Rollover
        newtail = 0;
    cbuf->tail = newtail;

    return retval;
}

/*
    @brief This will determine if there is a character in the circular buffer.

    @param cbuf Pointer to structure that contains data.

    @retval true if buffer contains data.
    @retval false if buffer is empty.
*/
static u8 circ_buf_has_char(tcirc_buf *cbuf)
{
    // Return true if buffer empty
    return (cbuf->head != cbuf->tail);
}

/*
   @brief Returns the number of stored chars in the circular buffer.

   @param cbuf is the circular buffer to query

   @return count Number of chars in buffer.
 */
static u8 circ_buf_count(tcirc_buf *cbuf)
{
    s16 count;

    count = cbuf->head - cbuf->tail;
    if (count < 0)
        count += BUFSIZE;
    return (u8) count;
}


/**
    @brief USART initialization

    This function sets the USART initialization to the following:
            USART Baud rate:    38,400
            USART Data bit:     8
            USART Parity Mode:  Diabled
            USART Stop bit:     1
            USART Receiver:     On
            USART Transmitter:  On
            USART Mode:         Asynchronous

    @param rxh RX interrupt handler, a user-defined function that can
    optionally be called when a serial char is received.  See @ref
    t_rx_handler.
*/
void serial_init(t_rx_handler rxh)
{
    // Initialize the buffers
    init_circ_buf(&rxbuf);
    init_circ_buf(&txbuf);

    if (PLATFORM == RAVENUSB)
        uart_usb_init();
    else
    {
        // Enable RX,TX and RX interrupt on USART
        USART_SETUP();
        
        // save pointer to rx handler function
        rx_handler = rxh;
        
        // Init the RCB_BB if necessary
        if (PLATFORM < RAVEN) // RCB's only
            INIT_RCB_BB();
    }
}

/**
   Turns off the serial port and RCB_BB transmitter chip
*/
void serial_shutdown(void)
{
    if (PLATFORM != RAVENUSB)
        USART_STOP();
}

/**
    @brief This function will place a character into the serial buffer. Will
           simply return if buffer is already full.

    @param ch Character to be placed in the serial output buffer and
    then transmitted over the serial port.
*/
void serial_putchar(char ch)
{
    // If we're paused, don't send anything
    if (pause)
        return;

    // Interrupts really should be on here
    sei();

    // Add byte to transmit buffer
    add_to_circ_buf(&txbuf, ch);

    // Send the first byte if nothing is yet being sent
    // This is determined by seeing if the TX complete interrupt is
    // enabled.
    if (PLATFORM != RAVENUSB && USART_CHAR_READY())
    {
        // Wait for the UDR buffer to get emptied.
        while (USART_DRE_WAIT())
            ;
        // Feed the first byte to get the transmission started.
        USART_REG = get_from_circ_buf(&txbuf);

        // Enable the UART Data Register Empty interrupt.
        USART_DRE_ENABLE();
    }
}

/**
   @brief Sends an ASCIIZ string to the serial port.

   @param str Pointer to ASCIIZ terminated string to send.
*/
void serial_puts(char *str)
{
    while (*str)
    {
        serial_putchar(*str);
        str++;
    }
}

/**
   @brief Check to see if any characters are waiting in the serial
   input buffer.

   @return The number of characters waiting
*/
u8 serial_ischar(void)
{
    return circ_buf_count(&rxbuf);
}

/**
   @brief Check to see if serial transmit buffer can accept more
   chars.

   @return true if we can take for chars, otherwise false.
*/
u8 rx_buf_count(void)
{
    return circ_buf_count(&rxbuf);
}

u8 tx_buf_has_char(void)
{
    return circ_buf_count(&txbuf);
}

u8 get_tx_char(void)
{
    return get_from_circ_buf(&txbuf);
}


/**
   @brief Returns a character from the serial port.  Will wait for input
   if necessary.  To prevent blocking while waiting for serial input,
   check circ_buf_count() first.

   @return Character from serial port.
*/
s8 serial_getchar(void)
{
    // wait for char if necessary
    while (!circ_buf_count(&rxbuf))
        cdc_task();

    // grab one from buffer.
    return get_from_circ_buf(&rxbuf);
}

/**
   @brief Captures a string from serial port.  This function blocks
   until a carriage return character is received.

   @param str Pointer to asciiz string to store characters to.
   @param len Maximum number of characters to get, including terminating zero.
   @param echo Specifies whether to echo each character received back to host.

   @return Number of characters in string.
*/
u8 serial_gets(char *str, u8 len, u8 echo)
{
    u8 count=0;                 // Number of characters we have so far
    u8 ch;

    for(;;)
    {
        ch = serial_getchar();
        if (count > len - 2 &&
            ch != '\r' &&
            ch != 8)
            continue;
        if (echo)
            serial_putchar(ch);
        switch(ch)
        {
        case '\r':
            // end of input
            *str = 0;
            return count;
        case 8:
            // backspace
            if (count)
            {
                count--;
                *str-- = 0;
            }
            break;
        default:
            // just another char for string
            *str++ = ch;
            count++;
            break;
        }
    }
}

/**
   @brief Toggles the pause flag.  When paused, any data sent to the
   serial send function is dropped.
*/
void serial_toggle_pause(void)
{
    pause = !pause;
}

void add_char_to_rx(u8 ch)
{
    if (PLATFORM == RAVENUSB)
    {
        // Put character in Rx Buffer.
        rx = true;
        add_to_circ_buf(&rxbuf, ch);
        rx = false;
    }
}


/**
    @brief This will receive a frame from the ATmega3290p and parse the incoming
    data.

    This function assembles a frame one byte at a time.
    
    @return true if frame has been received.
*/
u8 serial_rcv_frame(void)
{
    if (PLATFORM == RAVEN && SERIAL)
    {
        // Gets a serial frame, if any, and displays data appropriately
        // If wait_for_ack is true, this funtion will wait for serial chars.
        u8 ch;
        static u8 ndx;
        
        if (!serial_ischar())
            return false;
        
        ch = serial_getchar();
        
        switch (SerialFrame->state)
        {
        case sof:
            // looking for SOF
            if (ch == SOF_CHAR)
                SerialFrame->state = len;
            else
                SerialFrame->state = sof;
            break;
        case len:
            // Got SOF, looking for len
            if (ch > SERIAL_PAYLOAD_LEN)
                ch = SERIAL_PAYLOAD_LEN;
            SerialFrame->len = ch;
            SerialFrame->state = cmd;
            ndx = 0;
            break;
        case cmd:
            // Get cmd
            SerialFrame->cmd = ch;
            SerialFrame->state = payload;
            break;
        case payload:
            // Grab  payload
            if (ndx < SerialFrame->len)
            {
                SerialFrame->payload[ndx++] = ch;
                break;
            }
            else
                // All done with payload
                SerialFrame->state = eof;
            // Fall through here, intentionally, since there was no payload.
        case eof:
            // Start over with next char rx'd
            SerialFrame->state = sof;
            // Find EOF or else
            if (ch == EOF_CHAR)
                // All done
                return true;
            break;
        default:
            SerialFrame->state = sof;
            break;
        }
    }
    return false;
}

void serial_send_frame(u8 cmd, u8 payload_length, u8 *payload)
{
    if (PLATFORM == RAVEN && SERIAL)
    {
        // Send a frame to 3290P
        u8 i;
        
        serial_putchar(SOF_CHAR);
        serial_putchar(payload_length);
        serial_putchar(cmd);
        for (i=0;i<=payload_length-1;i++)
   	    serial_putchar(payload[i]);
        serial_putchar(EOF_CHAR);
    }
}



/**
    @brief USART ISR RX function.  If an RX handler was passed to
    serial_init(), then that handler will be called each time this ISR
    is called.
*/
ISR(USARTRXVECT)
{
    // Get byte from serial port
    u8 ch = USART_REG;

    // Put character in Rx Buffer.
    rx = true;
    add_to_circ_buf(&rxbuf, ch);
    rx = false;

    // And call user's RX handler function
    if (rx_handler)
        rx_handler(ch);
}

/**
    @brief USART ISR TX function.
*/
ISR(USARTTXVECT)
{
    if (circ_buf_has_char(&txbuf))
        // Feed a byte
        USART_REG = get_from_circ_buf(&txbuf);
    else
        // No more chars to send, disable this interrupt
        USART_DRE_DISABLE();
}
/** @} */
#endif // SERIAL
