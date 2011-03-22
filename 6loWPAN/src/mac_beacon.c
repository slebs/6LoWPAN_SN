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
  $Id: mac_beacon.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
*/

#include <stdio.h>
#include <string.h>
#include "../inc/mac.h"
#include "../inc/mac_beacon.h"
#include "../inc/mac_route.h"
#include "../inc/radio.h"

#include "../inc/mac_event.h"
#include "../inc/deRFaddon/bmm.h"


/**
   Create and send a beacon frame.  This is called in response to a
   beacon request frame.
*/
void sendBeaconFrame(void)
{
    if (NODETYPE != ENDDEVICE)
    {
       uint8_t* pFrame = bmm_buffer_alloc();

       if(pFrame != NULL)
       {
          ftBeacon *data_frame = (ftBeacon*)(((rx_frame_t*)pFrame)->data);

          data_frame->fcf   = FCF_BEACON;
          data_frame->seq   = macConfig.bsn++;
          data_frame->panid = macConfig.panId;
          data_frame->addr  = macConfig.shortAddress;

          if(NODETYPE == ROUTER)
            data_frame->superFrame = ROUTER_SUPERFRAME;
          else
            data_frame->superFrame = COORD_SUPERFRAME;

          data_frame->netID = 0x06;
          data_frame->hops = macConfig.hopsToCoord;

          ((rx_frame_t*)pFrame)->length = sizeof(ftBeacon);

          event_object_t event;
          event.event = MAC_EVENT_SEND;
          event.data = pFrame;
          event.callback = 0;

          // save Event and send later
          mac_put_event(&event);
       }
    }
}
