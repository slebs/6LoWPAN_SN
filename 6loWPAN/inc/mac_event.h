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
  $Id: mac_event.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
*/

#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

// Includes
#include "../inc/mac.h"
#include "../inc/mac_route.h"

/**
   @addtogroup mac_associate
   @{
*/

// Macros & Defines

/**
   List of event types processed by the MAC event system.
*/
typedef enum {MAC_EVENT_RX=0x10,               ///< This node has received a packet
              MAC_EVENT_ACK,                   ///< Received an ACK frame
              MAC_EVENT_NACK,                  ///< Sent frame failed
              MAC_EVENT_ACCESS,                ///< Channel access failure
              MAC_EVENT_SCAN,                  ///< This node received a beacon frame
              MAC_EVENT_BEACON_REQ,            ///< This node received a beacon request frame
              MAC_EVENT_TIMER,                 ///< The timer has expired
              MAC_EVENT_ASSOCIATION_REQUEST,   ///< Received an association request frame
              MAC_EVENT_ASSOCIATION_RESPONSE,  ///< Received an association response frame
              MAC_EVENT_ROUTE,                 ///< Received a routing packet.
              MAC_EVENT_SEND,                  ///< Send an packet over radio - added by Dresden Elektronik (18.01.10)
} __attribute__((packed)) event_t;

/**
   Object that defines a single event.  @see event_queue.
*/
typedef struct {
    event_t event;  ///< Event type, see event_t for details.
    u8 *data;       ///< Associated data that goes with the event.  Depends on event type.
    void (*callback) (void);   ///< Function to call when event is execute. Depends on event type. Added by Dresden Elektronik (19.03.10)
} __attribute__((packed)) event_object_t;


// Prototypes
u8 mac_event_pending(void);
void mac_put_event(event_object_t *object);
event_object_t * mac_get_event(void);
void macTask(void);

/** @} */
#endif
