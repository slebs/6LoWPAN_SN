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
 * @file status.c
 *
 * @brief Status messages to observe node configuration/settings (only for DEBUG use).
 *
 * This file implements functions to observe actual configuration of this node.
 * This is only useful for debugging information. To enable status messages the
 * compiler macro STATUS_DEBUG have to be enabled.
 *
 * Status messages are generated every STATUS_ALARM_TIMER (in milliseconds). If timer expired
 * this node create a new status message and send it to coordinator node. Coordinator node
 * evaluate message and send out message via wired connection (USB/RS232).
 *
 * Every node logs the following data:
 *    - MAC address of this node
 *    - short address of this node
 *    - parent address of this node
 *    - number of free buffers (@see bmm.c)
 *    - free running timer value (unsigned long integer)
 *
 * Coordinator node adds the following data when receives status message:
 *    - node type from transmitter node (Router, End node)
 *    - last routed address
 *    - LQI/ED values
 *    - number of free buffers from coordinator (@see bmm.c)
 *    - free running timer value (unsigned long integer)
 *
 *
 * $Id: status.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-03-24
 */


/* === Includes ============================================================= */

#include <stdio.h>

#ifdef STATUS_DEBUG
#include "../../inc/mac_associate.h"
#include "../../inc/avr_timer.h"
#include "../../inc/deRFaddon/deRFapplication.h"
#include "../../inc/deRFaddon/io_access.h"
#include "../../inc/deRFaddon/bmm.h"
#include "../../inc/deRFaddon/commands.h"
#endif

/* === Macros =============================================================== */

/* === Globals ============================================================== */

#ifdef STATUS_DEBUG
#define MS_PER_TICK        (1)

static uint32_t running_timer;
static uint8_t status_active = 0;

#if (NODETYPE == ROUTER) || (NODETYPE == ENDDEVICE)
static uint16_t STATUS_ALARM_TIMER = 10000;
#endif

#endif

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
 * @brief Helper function to switch off LED.
 */
void led_3_off(void)
{
#ifdef STATUS_DEBUG
   led_set(PLATFORM_RCB, RCB_LED_2, PLATFORM_LED_OFF);
#endif
}

/**
 * @brief Helper function to blink LED with automatic switch off.
 */
void blink_led_3(void)
{
#ifdef STATUS_DEBUG
   led_set(PLATFORM_RCB, RCB_LED_2, PLATFORM_LED_ON);
   macSetAlarm(30, led_3_off);
#endif
}

/**
 * @brief Create status message
 *
 * A status message can explicit generated when corresponding request (COMMAND_STATUS_REQUEST)
 * is received. Here the message is generated from time to time by expired timer (STATUS_ALARM_TIMER).
 *
 * A status message is only created from router and end nodes.
 */
void evaluate_status_request(void)
{
#ifdef STATUS_DEBUG
#if (NODETYPE == ROUTER) || (NODETYPE == ENDDEVICE)
   deRFprotocol_t frame;

   payloadStatusFrame_t* dataFrame = (payloadStatusFrame_t*)&frame.payload;

   frame.command = COMMAND_STATUS_RESPONSE;
   frame.option = NO_OPTION;

   dataFrame->mac = macConfig.longAddr;
   dataFrame->short_address = macConfig.shortAddress;
   dataFrame->parentAddress = macConfig.parentShortAddress;
   dataFrame->timer_node = running_timer;
   dataFrame->free_buffers_node = number_of_free_buffers();
   dataFrame->alarm_timer = STATUS_ALARM_TIMER;

   send_data_wireless(DEFAULT_COORD_ADDR, (uint8_t*)&frame, sizeof(deRFprotocol_t), UDP_PORT_END_ROUTER, UDP_PORT_COORD);

   blink_led_3();
   macSetAlarm(STATUS_ALARM_TIMER, evaluate_status_request);
#endif
#endif
}

/**
 * @brief Evaluate status message
 *
 * Received status message is evaluated. This is only done from coordinator. Coordinator adds
 * data that only he knows about. Then message is send out via wired interface (USB/RS232)
 *
 * @param   pFrame   pointer to frame which to evaluate
 *
 */
void evaluate_status_response(uint8_t* pFrame)
{
#ifdef STATUS_DEBUG
#if (NODETYPE == COORD)
   associatedNodes_t* nodes = (associatedNodes_t*)getChildTable();
   associatedNodes_t* node;

   payloadStatusFrame_t* dataFrame = (payloadStatusFrame_t*)&(((deRFprotocol_t*)pFrame)->payload);

   node = &nodes[dataFrame->short_address];

   dataFrame->nodeType = node->nodeType;
   dataFrame->lastRoutedAddress = node->lastRoutedAddress;
   dataFrame->ed = node->ed;
   dataFrame->lqi = node->lqi;
   dataFrame->timer_coord = running_timer;
   dataFrame->free_buffers_coord = number_of_free_buffers();

   // send frame out to serial interface
   send_data_wired(pFrame, sizeof(deRFprotocol_t));
#endif
#endif
}

/**
 * @brief Initialize free running timer
 *
 * Set TIMER4 to free running mode, with output compare mode every 1ms
 *
 */
void init_status_timer(void)
{
#ifdef STATUS_DEBUG
   TCCR4B |= (1 << CS41) | (1 << WGM42);  //prescaler to 8, free running, output compare
   OCR4A   = (MS_PER_TICK * 1000 / (8000000UL/F_CPU)); // compare every 1ms

   running_timer = 0;
#endif
}

/**
 * @brief Start free running timer
 */
void status_timer_enable(void)
{
#ifdef STATUS_DEBUG
   TIMSK4 |= (1 << OCIE4A);
   status_active = 1;
#endif
}

/**
 * @brief Stop free running timer
 */
void status_timer_disable(void)
{
#ifdef STATUS_DEBUG
   TIMSK4 &= ~(1 << OCIE4A);
   status_active = 0;
#endif
}

/**
 * @brief Check if free running timer is active
 */
uint8_t is_status_active(void)
{
#ifdef STATUS_DEBUG
   return status_active;
#endif
   return 0;
}

#ifdef STATUS_DEBUG
ISR(TIMER4_COMPA_vect)
{
   running_timer++;
}
#endif

/* EOF */

