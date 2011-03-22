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
  $Id: mac_data.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "../inc/stdbool.h"
#include "../inc/rum_types.h"
#include "../inc/mac.h"
#include "../inc/radio.h"
#include "../inc/mac_data.h"
#include "../inc/mac_route.h"
#include "../inc/mac_associate.h"
#include "../inc/system.h"
#include "../inc/avr_timer.h"
#include "../inc/mac_event.h"

#include "../inc/deRFaddon/bmm.h"

/*
#if (__arm__)
    // Utasker include
    #include "config.h"

    #include "arm_app.h"
    #include "tuip.h"
#endif
 */

/**
   @ingroup mac
   @{
   @defgroup mac_data MAC data functions
   @{

   This module includes code for sending and receiving data using the
   MAC.  Data is sent to another node on the network using
   macDataRequest(), and incoming packets are routed to
   macDataIndication(), which calls appDataIndication().

   If 6LoWPAN is being used, do not call macDataRequest() directly,
   but use the @ref avr6lowpan functions (AVR only).
 */

// Define the length of the header (non-payload part) of a data frame
#define ftDataHeaderSize offsetof(ftData, payload)

//6lowpan interface (temp)
void sixlowpan_DataIndication(ftData *frame, uint8_t payloadlen);


// Globals
static u16 pingAddr;  // temp var
static u16 pingType;  // temp var

// Stored frames (if RUMSLEEP is set)
typedef struct {
   u8  len;         // Length of stored frame (zero if this element is unused)
   u16 addr;        // Short address of node to receive frame
   u8  buf[128];    // Buffer to store frame in
} tStoredFrame;
#define STORED_FRAMES  3     // Number of stored frames
static tStoredFrame storedFrames[STORED_FRAMES * (RUMSLEEP && NODETYPE != ENDDEVICE && VLP)];

// Target function for timer, send a frame prepared by macDataRequest()
/*
 * IT'S NOT NEEDED ANY MORE, SINCE WE HAVE A BUFFER MANAGAGEMENT
 * AND WE DO NOT NEED ANY DELAYED FRAME TRANSMITTING (by Dresden Elektronik)
void mdr_timer(void)
{
   // Create a struct pointer to the global variable...
   ftData *data_frame = (ftData*)(mac_buffer_tx+1);
   radioSendData(*mac_buffer_tx, (u8*)data_frame);
}
*/

/**
    @brief The macsixlowpanDataRequest function is used to send a frame over
    the air to another node.  Any node type can call this function.

    @param addr Short address of the destination node.

    @param len The length of the packet in bytes.

    @param data Pointer to the data to be sent.

    @param type Type of frame to be sent
 */
static void macDataRequestInt(u16 addr, u8 len, u8 * data, u8 type)
{
   u8 rpSent; // Was a routing packet sent?

   // Don't send to self
   if (addr == macConfig.shortAddress || addr == BROADCASTADDR)
   {
      return;
   }

   // This node has no short address
   if (!macConfig.associated)
   {
      return;
   }

   uint8_t* pFrame = bmm_buffer_alloc();

   if(pFrame != NULL)
   {
      // Create a struct pointer to the global variable...
      //ftData *data_frame = (ftData*)(mac_buffer_tx+1);

      ftData *data_frame = (ftData*)(((rx_frame_t*)pFrame)->data);

      // Build the frame.
      data_frame->fcf = FCF_DATA;
      data_frame->seq = macConfig.dsn++;
      data_frame->panid = macConfig.panId;
      data_frame->srcAddr = macConfig.shortAddress;
      data_frame->finalDestAddr = addr;
      data_frame->originAddr = macConfig.shortAddress;

      // send a routing packet if necessary
      rpSent = macSendRoutingPacket(addr);

      if (NODETYPE == COORD)
      {
         // Find the child node that can route this packet
         u16 child = addr;
         u16 parent = macGetParent(child);
         while (parent != DEFAULT_COORD_ADDR)
         {
            child = parent;
            parent = macGetParent(child);
         }
         // send to child node that can route this packet
         data_frame->destAddr = child;
      }
      else
         // All data is send to parent, unless this is a wakeup frame
         if (type == WAKE_NODE)
            data_frame->destAddr = addr;
         else
            data_frame->destAddr = macConfig.parentShortAddress;

      // Frame type is data
      if (macConfig.sleeping && NODETYPE == ENDDEVICE && RUMSLEEP)
      {
         type |= 0x80; // Set high bit of type if we're sleeping
      }

      data_frame->type = type;

      // Copy the payload data to frame. (note: this creates smaller code than using memcpy!!)
      u8 i;
      for(i=0; i<len; i++){
         ((u8*)&data_frame->payload)[i] = *data++;
      }

      // Check addresses again - addr will be different now -> Don't send to self
      if (data_frame->destAddr == macConfig.shortAddress || data_frame->destAddr == BROADCASTADDR)
      {
         bmm_buffer_free(pFrame);
         return;
      }

      ((rx_frame_t*)pFrame)->length = len + ftDataHeaderSize; // save length away

      if (NODETYPE == COORD)
      {
         // See if the child is sleeping (only the coord sends directly to a child node)
         if (RUMSLEEP && macIsChild(addr) && macIsChildSleeping(addr) && VLP) // Send it later, after child is awake
         {
            macHoldFrame(addr, pFrame);
            // buffer is freed inside macHoldFrame()
         }
         else // Node is not sleeping child, send it now.
         {
            event_object_t event;
            event.event = MAC_EVENT_SEND;
            event.data = pFrame;
            event.callback = 0;

            // save Event
            mac_put_event(&event);
         }
      }
      else
      {
         event_object_t event;
         event.event = MAC_EVENT_SEND;
         event.data = pFrame;
         event.callback = 0;

         // save Event
         mac_put_event(&event);
      }
      macConfig.busy = true;
   }
}

/**
   @brief The macsixlowpanDataRequest function is used to send a frame over
   the air to another node.  Any node type can call this function.

   @param addr Short address of the destination node.

   @param len The length of the packet in bytes.

   @param data Pointer to the data to be sent.
 */
void macDataRequest(u16 addr, u8 len, u8 * data)
{
   macDataRequestInt(addr, len, data, DATA_FRAME);
}

/**
    @brief The macWakeRequest function is called by the coordinator to
            send a wakeup packet to a router.

    @param addr Short address of the parent router of the node to wake
    up.
    @param child Short address of the child node to wake up.
 */
void macWakeRequest(u16 addr, u16 child)
{
   if (NODETYPE != ENDDEVICE)
   {
      macDataRequestInt(addr, 2, (u8*)&child, WAKE_NODE);
   }
}

/**
   @brief Send an other-the-air (OTA) debug frame.  This contains a
   string payload that is displayed on the coordintor end.
 */
void macOtaDebugRequest(u8 *str)
{
   if (NODETYPE != COORD)
   {
      macDataRequestInt(DEFAULT_COORD_ADDR, strlen((char *)str)+1, str, DEBUG_FRAME);
   }
}

/**
    @brief The macsixlowpanDataRequest function is used to send a
    frame over the air to another node.  Note that the IPV6LOWPAN flag
    must be set to use any 6LoWPAN functionality.

    @param addr Short address of the destination node.

    @param len The length of the packet in bytes.

    @param data Pointer to the data to be sent.

    @ingroup avr6lowpan
 */
void macsixlowpanDataRequest(u16 addr, u8 len, u8 * data)
{
   if (IPV6LOWPAN == 1)
      macDataRequestInt(addr, len, data, DATA_FRAME_6LOWPAN);
}

/**
   This function is called when the MAC receives a packet that is
   addressed to this node.  The packet is dispatched according to its
   contents.
 */
void macDataIndication(uint8_t* pFrame)
{

   // Sort out the different types of data packets.
   ftData *frame = (ftData*)(((rx_frame_t*)pFrame)->data);

   /*
#if (__arm__)
    volatile s16 queue;
    if(fnGetTCP_state(host_socket) == TCP_STATE_ESTABLISHED)
    {
        if( (frame->fcf != FCF_BEACONREQ) &&
            (frame->fcf != FCF_ASSOC_REQ_DIRECT) &&
            (frame->fcf != FCF_ASSOC_REQ_IND) )
        {
            // ACK handling should occur in the tcpListener function in
            // rumtask.c.
            // fnSendBufTCP returns the number of bytes buffered if successful...
            queue = fnSendBufTCP(host_socket, (u8 *)"WIRELESSD", 9, TCP_BUF_SEND);
            if(telPrintReading && (queue > 0))
                fnDebugMsg("\r\nWIRELESSD buffered for TCP");
            else
                fnDebugMsg("\r\nWIRELESSD buffer problem");
            queue = fnSendBufTCP(host_socket, mac_buffer_rx, mac_buffer_rx[0]+1, TCP_BUF_SEND);
            if(telPrintReading && (queue > 0))
                fnDebugMsg("\r\nData indication buffered for TCP");
            else
                fnDebugMsg("\r\nData indication buffer problem");
        }
    }
#endif
    */
   switch (frame->type & 0x7f)  // Mask high bit just in case it was somehow missed
   {
   case DATA_FRAME:
      // Plain old data, send it up the chain
      appDataIndication();
      break;
   case DEBUG_FRAME:
      // Frame containing debug message, print it on coord
      if (NODETYPE == COORD && OTA_DEBUG && DEBUG)
      {
         debugMsgStr("\r\nNode ");
         debugMsgInt(frame->originAddr);
         debugMsgStr(": ");

         // Remove leading cr/lf's from string
         u8 *p = frame->payload;
         while (*p)
         {
            if (*p > ' ')
               break;
            p++;
         }
         debugMsgStr((char *)p);
      }
      break;
   case WAKE_NODE:
      // Wake up the end node.
      if (NODETYPE == ROUTER)
      {
         u8 addr = ((ftWake*)frame)->addr;
         // See if this is from parent or child
         if ((((ftWake*)frame)->srcAddr) == macConfig.parentShortAddress)
            // Set the flag to wake up the end node when it sends a packet
            macWakeChildNode(addr);
      }
      if (NODETYPE == ENDDEVICE)
      {
         // Wake yourself up now
         macConfig.sleeping = false;
         // Send parent a confirmation that we are awake
         macDataRequestInt(macConfig.parentShortAddress,
               2,
               (u8*)&macConfig.shortAddress,
               WAKE_NODE);
         debugMsgStr("\r\nAwake");
      }
      break;
   case PING_REQ_FRAME:
      // We got a ping request, let the app handle that
      appPingReq(frame->originAddr);
      break;
   case PING_RSP_FRAME:
      // We got a ping response, app will handle it
      appPingRsp(frame->originAddr);
      break;
   case DROP_CHILD_FRAME:
      // Coordinator is telling us to drop a child
      if (NODETYPE == ROUTER)
         macRemoveChild(*(u16*)(&frame->payload));
      break;
   case DATA_FRAME_6LOWPAN:
      //6lowpan data indication
      if (IPV6LOWPAN == 1)
         //sixlowpan_DataIndication(frame, *mac_buffer_rx - 16);
         sixlowpan_DataIndication(frame, (((rx_frame_t*)pFrame)->length) - 16);
      break;
   default:
      break;
   }
}

// Target function to timer, sends ping packet after a delay
void mp(void)
{
   uint8_t* pFrame = bmm_buffer_alloc();

   if(pFrame != NULL)
   {
      ftPing *frame = (ftPing*)(((rx_frame_t*)pFrame)->data);

      frame->fcf = FCF_DATA;
      frame->seq = macConfig.dsn++;
      frame->panid = macConfig.panId;
      frame->srcAddr = macConfig.shortAddress;
      frame->originAddr = macConfig.shortAddress;
      frame->finalDestAddr = pingAddr;
      frame->type = pingType;
      frame->rssi = radioGetSavedRssiValue();
      frame->lqi = radioGetSavedLqiValue();

      ((rx_frame_t*)pFrame)->length = sizeof(ftPing);

      if (NODETYPE == COORD)
      {
         // Find the top parent
         u8 addr = macGetTopParent(pingAddr);
         frame->destAddr = addr;
         // See if the child is sleeping (only the coord sends directly to a child node)
         if (RUMSLEEP && macIsChild(addr) && macIsChildSleeping(addr))
         {
            // Send it later, after child is awake
            macHoldFrame(addr, pFrame);
            // buffer is freed inside macHoldFrame()
            // Don't send frame right now
            return;
         }
      }
      else // End/router nodes
      {
         frame->destAddr = macConfig.parentShortAddress;
      }

      event_object_t event;
      event.event = MAC_EVENT_SEND;
      event.data = pFrame;
      event.callback = 0;

      // save Event
      mac_put_event(&event);
   }
}

/**
   Send a ping packet (either request or response) to another node.

   @param pingTypeArg Which type of ping to send, either @ref
   PING_REQ_FRAME or @ref PING_RSP_FRAME.

   @param addr Short address of node to send ping
 */
void macPing(u8 pingTypeArg, u16 addr)
{
   // Don't send to self
   if (addr == macConfig.shortAddress)
   {
      return;
   }

   // Broadcast addr
   if (!macConfig.associated)
   {
      return;
   }

   pingAddr = addr;
   pingType = pingTypeArg;

   if (NODETYPE == COORD)
   {
      // First send a routing packet
      u8 rpSent;
      rpSent = macSendRoutingPacket(addr);
      //macSetAlarm(rpSent ? MAC_RP_DELAY : 0, mp);
      mp();
      macConfig.busy = true;
   }
   else
   {
      // End/router nodes
      mp();
      macConfig.busy = true;
   }
}

/**
   Save a frame for transmission later. Used to store frames for
   sleeping children.

   @param addr Short address of recipient node

   @param pFrame Pointer to buffer containing the frame and length information
 */
void macHoldFrame(u16 addr, u8 *pFrame)
{
   if (NODETYPE != ENDDEVICE && RUMSLEEP && VLP)
   {
      u8 i,done=0;

      for (i=0;i<STORED_FRAMES;i++)
      {
         if (!storedFrames[i].len)
         {
            if (!done)  // Only store once
            {
               debugMsgStr("\r\nHolding frame");
               // This one's free, use it
               storedFrames[i].len = ((rx_frame_t*)pFrame)->length;
               storedFrames[i].addr = addr;
               //memcpy(storedFrames[i].buf, buf, len);
               memcpy(storedFrames[i].buf, ((rx_frame_t*)pFrame)->data, ((rx_frame_t*)pFrame)->length);
               // Don't store this frame twice
               done = 1;
            }
         }
         else
         {
            // This item is used, make sure it isn't for this address,
            // since we only want to store one pending frame for each
            // sleeping node.
            if (storedFrames[i].addr == addr)
               // Delete this frame
               storedFrames[i].len = 0;
         }
      }
   }

   bmm_buffer_free(pFrame); // free buffer
}

/**
   Send a frame that has been stored for a child node.  This is meant
   to be called immediately upon receiving a packet from a sleeping child node.
 */
void macSendStoredFrame(u16 addr)
{
   if (NODETYPE != ENDDEVICE && RUMSLEEP && VLP)
   {
      u8 i;

      // See if a frame is stored for this node
      for (i=0;i<STORED_FRAMES;i++)
      {
         if (storedFrames[i].len &&
               storedFrames[i].addr == addr)
         {
            // Send this frame, remove checksum
            uint8_t* pFrame = bmm_buffer_alloc();
            if(pFrame != NULL)
            {
               // restore stored frame
               memcpy(((rx_frame_t*)pFrame)->data, storedFrames[i].buf, storedFrames[i].len);

               ((rx_frame_t*)pFrame)->length = storedFrames[i].len;

               event_object_t event;
               event.event = MAC_EVENT_SEND;
               event.data = pFrame;
               event.callback = 0;

               // save Event
               mac_put_event(&event);

               // Clear out this frame
               storedFrames[i].len = 0;
            }

         }
      }
   }

}
/** @} */
/** @} */
