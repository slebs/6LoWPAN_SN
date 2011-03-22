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
 * @file link_quality.c
 *
 * @brief Evaluation and generation of link quality messages.
 *
 * This file implements the evaluation and generation of link quality messages. This messages
 * provide a way to update and read back the actual LQI (Link Quality Indication) and
 * ED (Energy Detection) values of every node.
 *
 * Therefore all router nodes save LQI and ED values from their child nodes. The coordinator
 * saves all LQI and ED values from it's direct childs. To get link qualities from nodes that
 * are more than one hop away, the coordinator sends requests to all router nodes (periodically
 * when timer, with preloaded value ITERATION_TIME_QUALITY, expired). When router receives request
 * they send back a response message to coordinator, which contains LQI and ED for each router child.
 * Coordinator evaluate message and set LQI/ED to it's child table (he knows about all nodes).
 *
 * $Id: link_quality.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-02-01
 */


/* === Includes ============================================================= */

#include <stdint.h>
#include <stdio.h>

#include "../../inc/radio.h"
#include "../../inc/deRFaddon/commands.h"
#include "../../inc/avr_timer.h"
#include "../../inc/mac_associate.h"

#include "../../inc/deRFaddon/link_quality.h"
#include "../../inc/deRFaddon/deRFapplication.h"

#include "../../inc/deRFaddon/uart.h"

/* === Macros =============================================================== */

/**
 * every Payload entry consist of 'shortAddress', 'LQI' and 'ED'
 *
 * To get max payload data calculate value through max. protocol payload size
 * divided by elments per node
 */
#define MAX_DATA_QUALITY_PAYLOAD    (MAX_PAYLOAD_PROTOCOL / sizeof(payloadQualityFrame_t))

/**
 * cycle time where coordinator should send link quality request to router nodes
 */
#define ITERATION_TIME_QUALITY      (10000UL)

/* === Globals ============================================================== */

/* === Prototypes =========================================================== */

uint8_t setup_info_for_request(deRFprotocol_t* pFrame, uint8_t indexStart);

/* === Implementation ======================================================= */

/**
 * @brief Is called every time a new message is received and LQI/ED values are stored.
 *
 * When a new message is received this function is called to save actual LQI/ED values.
 * This is executed by coordinator and router nodes, because only this nodes have the
 * ability to save values inside a their table.
 *
 * @param   pFrame   pointer to receiving frame
 */
void check_and_save_quality_values(uint8_t* pFrame)
{
   u8 shortAddress = ((ftData*)(((rx_frame_t*)pFrame)->data))->srcAddr;

#if (NODETYPE == COORD)
   associatedNodes_t* nodes = (associatedNodes_t*)getChildTable();
   associatedNodes_t* node = &nodes[shortAddress];

   // check if node is associated with coordinator (node type is not zero)
   if(node->nodeType)
   {
      // check if node is direct child from coordinator -> parentShortAddress = Coordinator Short Address
      if((node->parentShortAddress) == DEFAULT_COORD_ADDR)
      {
         node->lqi = radioGetSavedLqiValue();
         node->ed  = radioGetSavedEDValue();
      }
   }
#endif

#if (NODETYPE == ROUTER)

   tChildTableItem* nodes = (tChildTableItem*)getChildTable();
   tChildTableItem* node;

   uint8_t i = 0;
   for(; i < MAXCHILDREN; i++)
   {
      node = &nodes[i];
      if (((tChildTableItem*)&nodes[i])->childAddr == shortAddress)
      {
         node->lqi = radioGetSavedLqiValue();
         node->ed  = radioGetSavedEDValue();

         break;
      }
   }
#endif

#if (NODETYPE == ENDDEVICE)
   shortAddress = shortAddress; // make compiler happy
#endif
}

/**
 * @brief Build payload of new frame with LQI and ED values.
 *
 * This is executed by router nodes. A payload entry is made for every child node from router node.
 * If more child nodes than max. available payload space exist, the current child index is returned,
 * if there is enough space zero is returned.
 *
 * @param   pFrame      pointer to actual configuration data
 * @param   indexStart  parameter where to start in child table
 *
 * @return              0 if payload space is greater than cild table size, else index of child table is returned
 */
uint8_t setup_info_for_request(deRFprotocol_t* pFrame, uint8_t indexStart)
{
#if (NODETYPE == ROUTER)
   uint8_t counter = 0;
   if(indexStart > MAXCHILDREN){return 0;}

   tChildTableItem* nodes = (tChildTableItem*)getChildTable();
   tChildTableItem* node;
   uint8_t* pPayload = (uint8_t*)&pFrame->payload;

   for (; indexStart < MAXCHILDREN; indexStart++)
   {
      node = &nodes[indexStart];
      if (node->childAddr != 0)
      {
         if(counter > MAX_PAYLOAD_PROTOCOL)
         {
            return indexStart;
         }
         payloadQualityFrame_t* qualityFrame = (payloadQualityFrame_t*)pPayload;
         qualityFrame->short_address = node->childAddr;
         qualityFrame->lqi = node->lqi;
         qualityFrame->ed  = node->ed;
         pPayload += sizeof(payloadQualityFrame_t);

         counter += sizeof(payloadQualityFrame_t);
      }
   }
#endif

   return 0;
}

/**
 * @brief Called when router node receives a link quality request from coordinator
 *
 * This is only executed by router nodes. A frame is generated when Router has more than on
 * child. Inside frame, LQI/ED values and short address from all child nodes stored.
 * The message is send to coordinator.
 *
 */
void evaluate_quality_request()
{
#if (NODETYPE == ROUTER)

#if LQ_DEBUG_ENABLE
   UART_PRINT("lq request\r\n");
#endif

   deRFprotocol_t frame;
   uint8_t indexStart = 0;
   uint8_t numberOfNodes = 0;

   tChildTableItem* nodes = (tChildTableItem*)getChildTable();
   tChildTableItem* node;

   uint8_t* pDeRFprotocol = (uint8_t*)&frame.payload;

   for (; indexStart < MAXCHILDREN; indexStart++)
   {
      // check if number of child nodes is not greater than max. available payload
      // if greater, than send out current configuration and reset pointers and variables
      // to start a new frame
      if(numberOfNodes > MAX_DATA_QUALITY_PAYLOAD)
      {
         frame.command = COMMAND_INTERN_LINK_QUALITY_RESPONSE;
         frame.option = numberOfNodes;

         send_data_wireless(DEFAULT_COORD_ADDR, (uint8_t*)&frame, sizeof(frame), UDP_PORT_END_ROUTER, UDP_PORT_COORD);

         numberOfNodes = 0; // reset node 'counter'
         pDeRFprotocol = (uint8_t*)&frame.payload; // start again (pointer to start address)
      }

      node = &nodes[indexStart];
      if (node->childAddr > 0)
      {
         payloadQualityFrame_t* qualityFrame = (payloadQualityFrame_t*)pDeRFprotocol;
         qualityFrame->short_address = node->childAddr;
         qualityFrame->lqi = node->lqi;
         qualityFrame->ed  = node->ed;

#if LQ_DEBUG_ENABLE
         UART_PRINT("  -> (%u) LQI: %i  -  ED: %i\r\n", node->childAddr, node->lqi, node->ed);
#endif

         pDeRFprotocol += sizeof(payloadQualityFrame_t);
         numberOfNodes++;
      }
   }

   // send message out (this is last message if more childs than max. available payload exist)
   // (also it's first and only message if less childs than available payload exist)
   if(numberOfNodes > 0)
   {
      frame.command = COMMAND_INTERN_LINK_QUALITY_RESPONSE;
      frame.option = numberOfNodes;

      send_data_wireless(DEFAULT_COORD_ADDR, (uint8_t*)&frame, sizeof(frame), UDP_PORT_END_ROUTER, UDP_PORT_COORD);
   }
#endif
}

/**
 * @brief Executed by Coordinator to evaluate all 'GET_QUALITY' responses from router nodes.
 *
 * The response for every router node is evaluated and LQI/ED value is stored to corresponding
 * child node. The storage place is the child table from coordinator.
 *
 * @param   pFrame   pointer to frame which contains quality response data
 */
void evaluate_quality_response(uint8_t* pFrame)
{
#if (NODETYPE == COORD)

#if LQ_DEBUG_ENABLE
   UART_PRINT("lq response\r\n");
#endif

   // option field contains number of nodes
   uint8_t numberOfNodes = ((deRFprotocol_t*)pFrame)->option;
   if(numberOfNodes > MAX_DATA_QUALITY_PAYLOAD || numberOfNodes == 0)
   {
      return; // do not evaluate this message, it make's no sense
   }

   associatedNodes_t* nodes = (associatedNodes_t*)getChildTable();
   associatedNodes_t* node;

   uint8_t* pPayload = (uint8_t*)&(((deRFprotocol_t*)pFrame)->payload);

   uint8_t i;
   for(i = 0; i < numberOfNodes; i++)
   {
      payloadQualityFrame_t* qualityFrame = (payloadQualityFrame_t*)pPayload;
      // make sure, short address is not coordinator address (>0) and short address is not
      // greater than max. possible address (router can not have more than MAXCHILDREN nodes)
      if(qualityFrame->short_address > 0 && qualityFrame->short_address < MAXCHILDREN)
      {
         node = &nodes[(uint8_t)(qualityFrame->short_address)];

         if (node->nodeType != 0)
         {
            node->lqi = qualityFrame->lqi;
            node->ed  = qualityFrame->ed;

#if LQ_DEBUG_ENABLE
            UART_PRINT("  -> (%u) LQI: %i  -  ED: %i\r\n", qualityFrame->short_address, qualityFrame->lqi, qualityFrame->ed);
#endif
         }
      }
      pPayload += sizeof(payloadQualityFrame_t);
   }
#endif
}

/**
 * @brief Executed by Coordinator to send 'GET_QUALITY' request to all router nodes
 */
void send_quality_request(void)
{
#if (NODETYPE == COORD)

   associatedNodes_t* nodes = (associatedNodes_t*)getChildTable();
   associatedNodes_t* node;

   deRFprotocol_t frame;
   frame.command = COMMAND_INTERN_LINK_QUALITY_REQUEST;
   frame.option = NO_OPTION;

   uint8_t i;
   for(i = 1; i < MAXNODES; i++)
   {
      node = &nodes[i];
      // find out all router nodes
      if ((node->nodeType) == ROUTER)
      {
         send_data_wireless(i, (uint8_t*)&frame, sizeof(frame), UDP_PORT_COORD, UDP_PORT_END_ROUTER);
      }
   }

   // call function again after timer with preloaded value ITERATION_TIME_QUALITY is expired
   macSetAlarm(ITERATION_TIME_QUALITY, send_quality_request);
#endif
}

/* EOF */

