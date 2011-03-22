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

#include <stdio.h>
#include <string.h>
#include "../inc/mac.h"
#include "../inc/mac_associate.h"
#include "../inc/mac_data.h"
#include "../inc/radio.h"
#include "../inc/hal.h"
#include "../inc/mac_route.h"
#include "../inc/system.h"
#include "../inc/avr_timer.h"

#include "../inc/mac_event.h"
#include "../inc/deRFaddon/bmm.h"

/**
 @addtogroup mac
 @{
 */

/**
 Send routing packet down the chain, removing own address from the
 list of addresses in the route.
 */
void macForwardRoutingPacket(uint8_t* pFrame) {
	if (NODETYPE == ROUTER) {
		u8 payloadLength = ((rx_frame_t*) pFrame)->length - sizeof(ftRouting);
		ftRouting *frame = (ftRouting*) (((rx_frame_t*) pFrame)->data);

		u16 *p; // pointer to list of addresses in incoming routing packet
		u8 hopCount = payloadLength / 2; // convert from bytes to words

		// Don't send packets unless we have a valid short address
		if (!macConfig.associated) {
			bmm_buffer_free(pFrame);
			return;
		}

		// set pointer to first address
		p = &frame->shortAddr;

		// set last route address in MAC
		macConfig.lastRoute = p[hopCount - 1];

		debugMsgStr("\r\nSet route=");
		debugMsgHex(macConfig.lastRoute);

		if (hopCount > 1) {
			// must forward to next address in chain

			// remove last address from list
			hopCount--;

			frame->fcf = FCF_ROUTE;
			frame->seq = macConfig.dsn++;
			frame->destAddr = p[hopCount];
			frame->srcAddr = macConfig.shortAddress;
			frame->cmd = ROUTING_PACKET;

			((rx_frame_t*) pFrame)->length = sizeof(ftRouting) + (hopCount - 1)
					* 2;

			event_object_t event;
			event.event = MAC_EVENT_SEND;
			event.data = pFrame;
			event.callback = 0;

			// save Event
			mac_put_event(&event);
		} else {
			bmm_buffer_free(pFrame);
		}
	} else // NODETYPE != ROUTER
	{
		bmm_buffer_free(pFrame);
	}
}

void macRouteAssociateResponse(uint8_t* pFrame) {
	// Send the mac packet down the chain
	// The incoming packet is not for this node, therefore
	// the incoming packet is an indirect packet.

	if (NODETYPE == ROUTER) {
		// find out if we are sending to the endnode.
		ftAssocRespIndirect* inputFrame =
				(ftAssocRespIndirect*) (((rx_frame_t*) pFrame)->data);

		// Don't send packets unless we have a valid short address
		if (!macConfig.associated) {
			bmm_buffer_free(pFrame);
			return;
		}

		if (inputFrame->parentAddr == macConfig.shortAddress) {
			// This frame is being sent from me (a router) to the newly
			// associated node. We have to translate from indirect to
			// direct frame format

			// we are using the same buffer for the input and output frame
			ftAssocRespDirect* outputFrame =
					(ftAssocRespDirect*) (((rx_frame_t*) pFrame)->data);

			u64 macAddr = inputFrame->macAddr;
			u16 shortAddr = inputFrame->shortAddr;

			outputFrame->fcf = FCF_ASSOC_RESP_DIRECT;
			outputFrame->seq = macConfig.dsn++;
			outputFrame->panid = macConfig.panId;
			outputFrame->dstAddr = macAddr;
			outputFrame->srcAddr = macConfig.shortAddress;
			outputFrame->cmd = ASSOCIATION_RESPONSE;
			outputFrame->shortAddr = shortAddr;

			// Add this new node to child table
			macAddChild(outputFrame->shortAddr);

			((rx_frame_t*) pFrame)->length = sizeof(ftAssocRespDirect);

			event_object_t event;
			event.event = MAC_EVENT_SEND;
			event.data = pFrame;
			event.callback = 0;

			// save Event
			mac_put_event(&event);
		} else {
			// This frame is being forwarded to a downstream router, not an
			// end node.  Just re-use the input packet, and change the
			// addresses for src/dest.

			// we are using the same buffer for the input and output frame
			ftAssocRespIndirect* outputFrame =
					(ftAssocRespIndirect*) (((rx_frame_t*) pFrame)->data);

			outputFrame->seq = macConfig.dsn++;
			outputFrame->dstAddr = macConfig.lastRoute;
			outputFrame->srcAddr = macConfig.shortAddress;

			// Also, check to see if the parent of the new node is one of my
			// children.  This is needed because the routing packet mechanism
			// doesn't work properly for a three-hop route.
			if (macIsChild(outputFrame->parentAddr)) {
				outputFrame->dstAddr = outputFrame->parentAddr;
			}

			((rx_frame_t*) pFrame)->length = sizeof(ftAssocRespIndirect);

			event_object_t event;
			event.event = MAC_EVENT_SEND;
			event.data = pFrame;
			event.callback = 0;

			// save Event
			mac_put_event(&event);
		}
	} else // NODETYPE != ROUTER
	{
		bmm_buffer_free(pFrame);
	}
}

void macRouteAssociateRequest(uint8_t* pFrame) {
	// This router node has received an association request packet.  We must
	// Send this packet along, and if the packet came from a MAC address, then
	// we must translate from a direct packet to an indirect packet.
	if (NODETYPE == ROUTER) {
		uint8_t *data_frame = (uint8_t*) (((rx_frame_t*) pFrame)->data);

		if (data_frame[1] == (FCF_ASSOC_REQ_DIRECT >> 8)) // Direct from MAC addr?
		{
			// translate from direct to indirect
			ftAssocReqDirect *input =
					(ftAssocReqDirect*) (((rx_frame_t*) pFrame)->data);
			ftAssocReqIndirect *output =
					(ftAssocReqIndirect*) (((rx_frame_t*) pFrame)->data);

			// save for later use -> so we don't have to allocate a new buffer
			u64 srcAddr = input->srcAddr;
			u16 parentAddr = input->parentAddr;
			u8 type = input->type;

			output->fcf = FCF_ASSOC_RESP_IND;
			output->seq = macConfig.dsn++;
			output->panid = macConfig.panId;
			output->destAddr = macConfig.parentShortAddress; // send to parent
			output->srcAddr = macConfig.shortAddress;
			output->cmd = ASSOCIATION_REQUEST;
			output->parentAddr = parentAddr;
			output->macAddr = srcAddr;
			output->type = type;

			((rx_frame_t*) pFrame)->length = sizeof(ftAssocReqIndirect);

			event_object_t event;
			event.event = MAC_EVENT_SEND;
			event.data = pFrame;
			event.callback = 0;

			// save Event
			mac_put_event(&event);
			debugMsgStr("\r\nReceived assoc req");
		} else {
			// input frame is indirect, output frame is indirect, just copy
			ftAssocReqIndirect *input =
					(ftAssocReqIndirect*) (((rx_frame_t*) pFrame)->data);
			ftAssocReqIndirect *output =
					(ftAssocReqIndirect*) (((rx_frame_t*) pFrame)->data);

			// save for later use
			u64 macAddr = input->macAddr;
			u16 parentAddr = input->parentAddr;
			u8 type = input->type;

			output->fcf = FCF_ASSOC_RESP_IND;
			output->seq = macConfig.dsn++;
			output->panid = macConfig.panId;
			output->destAddr = macConfig.parentShortAddress; // send all packets to parent
			output->srcAddr = macConfig.shortAddress;
			output->cmd = ASSOCIATION_REQUEST;

			output->parentAddr = parentAddr;
			output->macAddr = macAddr;
			output->type = type;

			((rx_frame_t*) pFrame)->length = sizeof(ftAssocReqIndirect);

			event_object_t event;
			event.event = MAC_EVENT_SEND;
			event.data = pFrame;
			event.callback = 0;

			// save Event
			mac_put_event(&event);
		}
	} else // NODETYPE != ROUTER
	{
		bmm_buffer_free(pFrame);
	}
}

/*
 * NOT USED ANY MORE -> THERE IS NO NEED OF DELAYED TIMER EVENTS
 * A TRANSMIT BUFFER IS USED INSTEAD (by Dresden Elektronik)
 void mrd(void)
 {
 // Send packet after routing packet was sent
 if (NODETYPE == COORD)
 {
 // Packet has already been created, just send it.
 ftData *frame = (ftData *)(mac_buffer_tx+1);
 radioSendData(sizeof(ftData), (u8 *)frame);
 }
 }
 */

void ledoff1(void);

/**
 Route a data packet.  There are a few simple rules for routing packets:

 - Routers: if a packet is from my parent, and the destination is my
 child, send the packet to the child.

 - Routers: if a packet is from my parent, and the destination is
 not my child, then send to the last default route.

 - Routers: if a packet is from my child, send the packet to my parent.

 - Coordinator: send the packet down the tree, preceded by a routing
 packet in necessary.

 @param pFrame pointer to buffer (in this case it's the receiving frame)
 */
void macRouteData(uint8_t* pFrame) {
	if (NODETYPE == ROUTER || NODETYPE == COORD) {
		ftData *frame = (ftData*) (((rx_frame_t*) pFrame)->data);

		LED_ON(1);
		macSetAlarm(LED_DELAY, ledoff1);

		// See if this frame is in the child table
		if (macIsChild(frame->finalDestAddr)) {
			// send frame to child
			frame->seq = macConfig.dsn++;
			frame->destAddr = frame->finalDestAddr;
			frame->srcAddr = macConfig.shortAddress;
			debugMsgStr("\r\nRoute data to child");

			// See if the child is sleeping
			if (RUMSLEEP && macIsChildSleeping(frame->finalDestAddr)) { // Send it later, after child is awake
				macHoldFrame(frame->finalDestAddr, (u8*) frame);
				// buffer is freed in macHoldFrame() !!
			} else if (frame->destAddr != BROADCASTADDR) {
				event_object_t event;
				event.event = MAC_EVENT_SEND;
				event.data = pFrame;
				event.callback = 0;

				// subtract 2 bytes from length for checksum length
				((rx_frame_t*) pFrame)->length -= 2; // length info already exist, because this
				// is a pointer to the incoming buffer

				// save Event
				mac_put_event(&event);
			} else {
				bmm_buffer_free(pFrame);
			}
		} else // Not child node, send up or down the chain
		{
			if (NODETYPE == COORD) {
				// Send down the chain
				frame->seq = macConfig.dsn++;
				frame->destAddr = macGetTopParent(frame->finalDestAddr);
				frame->srcAddr = DEFAULT_COORD_ADDR;

				debugMsgStr("\r\nRoute data from Coord");

				// if no routing packet has to be send (e.g. last routing packet went there)
				// the bottom if-clause will be executed
				if (macSendRoutingPacket(frame->finalDestAddr)) {
					macConfig.busy = true;

					((rx_frame_t*) pFrame)->length = sizeof(ftData);

					event_object_t event;
					event.event = MAC_EVENT_SEND;
					event.data = pFrame;
					event.callback = 0;

					// save Event
					mac_put_event(&event);
					return;
				}

			} else if (NODETYPE == ROUTER) {
				// See if we should route up or down
				if (frame->srcAddr == macConfig.parentShortAddress) {
					// this frame is from parent, send it down default route
					frame->seq = macConfig.dsn++;
					frame->destAddr = macConfig.lastRoute;
					frame->srcAddr = macConfig.shortAddress;
					debugMsgStr("Route data down to ");
					debugMsgHex(macConfig.lastRoute);
				} else {
					// this frame is from child, send up the chain
					frame->seq = macConfig.dsn++;
					frame->destAddr = macConfig.parentShortAddress;
					frame->srcAddr = macConfig.shortAddress;
					debugMsgStr("\r\nRoute data up");
				}
			}
			// Make sure we're not broadcasting frames
			if (frame->destAddr != BROADCASTADDR) {
				// subtract 2 bytes from length for checksum length
				((rx_frame_t*) pFrame)->length -= 2;

				event_object_t event;
				event.event = MAC_EVENT_SEND;
				event.data = pFrame;
				event.callback = 0;

				// save Event
				mac_put_event(&event);
			} else {
				bmm_buffer_free(pFrame);
			}
		}
	} else // NODETYPE == ENDDEVICE
	{
		bmm_buffer_free(pFrame);
	}
}

/**
 Create and send a routing packet, returns non-zero if packet was sent.

 @param shortAddr The short address of the end node, or the short
 address of the parent of the final destination node.

 @return Zero if no routing packet was sent (probably because it was not needed).
 @return Non-zero if a routing packet was sent.
 */
u8 macSendRoutingPacket(u16 shortAddr) {
	if (NODETYPE == COORD) {
		uint8_t* pFrame = bmm_buffer_alloc();

		if (pFrame != NULL) {
			volatile u16 topParent; // Return to caller

			ftRouting *frame = (ftRouting*) (((rx_frame_t*) pFrame)->data);
			u8 hops; // Number of router-to-router hops

			// Calculate the routing portion of the routing packet
			hops = macCreateRoute(shortAddr, frame);

			if (hops < 3) {
				bmm_buffer_free(pFrame);
				// Could not create routing packet, or no packet needed
				return 0;
			}

			frame->fcf = FCF_ROUTE;
			frame->seq = macConfig.dsn++;
			frame->panid = macConfig.panId;
			frame->srcAddr = DEFAULT_COORD_ADDR;
			frame->cmd = ROUTING_PACKET;

			topParent = frame->destAddr;

			// See if we can skip routing because the last packet went there.
			if (macGetLastRoute(topParent) == frame->shortAddr) {
				bmm_buffer_free(pFrame);
				return 0;
			}

			// Save route for this child router
			macSetLastRoute(topParent, frame->shortAddr);

			((rx_frame_t*) pFrame)->length = sizeof(ftRouting) + (hops - 3) * 2;

			event_object_t event;
			event.event = MAC_EVENT_SEND;
			event.data = pFrame;
			event.callback = 0;

			// save Event
			mac_put_event(&event);

			// Set the flag to say we sent a routing packet.
			return 1;
		}
	}
	return 0;
}

/** @} */

