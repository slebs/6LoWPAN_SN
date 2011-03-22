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
 * @file deRFapplication.c
 *
 * @brief Application Module
 *
 * This file implements the application logic. All wireless messages, whether they are for
 * coordinator, router or end nodes, processed here. Also incoming messages over wired interface
 * (USB/RS232) processed here. This could be messages either for coordinator (which assume an
 * application on 'other side� of wired interface) or for end node/router node when User Data is send.
 * Processing messages means to evaluate incoming messages and send a response if necessary.
 *
 *
 * $Id: deRFapplication.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-01-18
 */

/* === Includes ============================================================= */

#include <stdint.h>
#include <string.h>

#include "../../inc/avr_sixlowpan.h"
#include "../../inc/mac_associate.h"

#include "../../inc/deRFaddon/hdlc_light.h"
#include "../../inc/deRFaddon/commands.h"
#include "../../inc/deRFaddon/deRFapplication.h"
#include "../../inc/deRFaddon/link_quality.h"
#include "../../inc/deRFaddon/status.h"
#include "../../inc/deRFaddon/data.h"
#include "../../inc/deRFaddon/uart.h"

#if defined(COMMUNICATION_USB)
#include "../../inc/deRFaddon/usb.h"
#endif
/* === Macros =============================================================== */

/**
 * data buffer size for incoming and outgiong messages via serial interface (RS232/USB)
 */
#define DATA_BUFFER_SIZE               (110)

/**
 * HDLC buffer for outgoing messages over serial interface (USB/RS232)
 */
uint8_t hdlcDataBufferTransmit[DATA_BUFFER_SIZE];

/**
 * HDLC buffer for incoming messages over serial interface (USB/RS232)
 */
uint8_t hdlcDataBufferReceive[DATA_BUFFER_SIZE];

/* === Prototypes ========================================================== */

void process_incoming_user_data(uint8_t* pUserData, uint16_t originAddr);
void send_node_info(uint8_t command);
int16_t getNodeAddressByMAC(uint8_t* pMacAddress);
uint8_t* getMACAddressBySA(uint16_t shortAddress);

/* === Globals ============================================================== */

/* === Implementation ======================================================= */

/**
 * @brief Process incoming coordinator messages over wireless interface.
 *
 * This function is called when a new UDP packet on port 0xF0BC (UDP_PORT_COORD) is received.
 * Depending on 'command' (first entry on udp packet payload) the corresponding service is
 * executed.
 * This function is called at avr_sixlowpan_application.c within sixlowpan_udp_usercall().
 *
 * @param   pUDPpacket  pointer to udp packet (length of packet depends on evaluation of command)
 * @param   originAddr  short address from originating node (this is the node where the message is originally from)
 */
void process_coord_udp_packet(uint8_t* pUDPpacket, uint16_t originAddr)
{
#if NODETYPE == COORD
   if(*pUDPpacket == COMMAND_DATA_FRAME_RESPONSE)
   {
#if RFAPP_UART_ENABLE
      UART_PRINT(" data frame\n");
#endif
      // send received frame from arbitrary node out to serial interface (USB/RS232)
      send_data_wired(pUDPpacket, sizeof(deRFprotocol_t));
   }
   else if(*pUDPpacket == COMMAND_PING_RESPONSE)
   {
#if RFAPP_UART_ENABLE
      UART_PRINT(" ping frame\n");
#endif
      // send received frame from arbitrary node out to serial interface (USB/RS232)
      send_data_wired(pUDPpacket, sizeof(deRFprotocol_t));
   }
   else if(*pUDPpacket == COMMAND_INTERN_LINK_QUALITY_RESPONSE)
   {
#if RFAPP_UART_ENABLE
      UART_PRINT(" lq frame\n");
#endif
      evaluate_quality_response(pUDPpacket);
   }
   else if(*pUDPpacket == COMMAND_USER_DATA_REQUEST)
   {
#if RFAPP_UART_ENABLE
      UART_PRINT(" user data frame\n");
#endif
      process_incoming_user_data(pUDPpacket, originAddr);
   }
#ifdef STATUS_DEBUG
   else if(*pUDPpacket == COMMAND_STATUS_RESPONSE)
   {
#if RFAPP_UART_ENABLE
      UART_PRINT(" status frame\n");
#endif
      evaluate_status_response(pUDPpacket);
   }
   else if(*pUDPpacket == COMMAND_STATUS_REQUEST)
   {
      // there is no request, status message is send periodically
   }
#endif //STATUS_DEBUG

#endif // NODETYPE == COORD
}

/**
 * @brief Process incoming router/end node messages over wireless interface.
 *
 * This function is called when a new UDP packet on port 0xF0BB (UDP_PORT_END_ROUTER) is received.
 * Depending on 'command' (first entry on udp packet payload) the corresponding service is
 * executed.
 * This function is called at avr_sixlowpan_application.c within sixlowpan_udp_usercall().
 *
 * @param   pUDPpacket  pointer to udp packet (length of packet depends on evaluation of command)
 * @param   originAddr  short address from originating node (this is the node where the message is originally from)
 */
void process_endnode_udp_packet(uint8_t* pUDPpacket, uint16_t originAddr)
{
#if NODETYPE != COORD

   if(*pUDPpacket == COMMAND_DATA_FRAME_REQUEST)
   {
#if RFAPP_UART_ENABLE
      UART_PRINT(" data frame\n");
#endif
      evaluate_data_frame(pUDPpacket);
   }
   else if(*pUDPpacket == COMMAND_PING_REQUEST)
   {
#if RFAPP_UART_ENABLE
      UART_PRINT(" ping frame\n");
#endif
      deRFprotocol_t* dataFrame = (deRFprotocol_t*)pUDPpacket;
      dataFrame->command = COMMAND_PING_RESPONSE;
      dataFrame->option = NO_OPTION;
      payloadPingFrame_t* pingFrame = (payloadPingFrame_t*)&dataFrame->payload;
      pingFrame->mac = macConfig.longAddr;

      send_data_wireless(DEFAULT_COORD_ADDR, pUDPpacket, sizeof(deRFprotocol_t), UDP_PORT_END_ROUTER, UDP_PORT_COORD);
   }
   else if(*pUDPpacket == COMMAND_INTERN_LINK_QUALITY_REQUEST)
   {
      evaluate_quality_request();
   }
   // an User Data frame should be send out wired (USB, RS232)
   // the incoming User Data contains MAC address from origin node
   // origin User Data contains MAC address from destination node, but all messages
   // delivered over Coord node. Coord node than replaces destination MAC by originating MAC
   else if(*pUDPpacket == COMMAND_USER_DATA_REQUEST)
   {
#if RFAPP_UART_ENABLE
      UART_PRINT(" user data frame\n");
#endif
      userData_t* userDataFrame = (userData_t*)pUDPpacket;
      userDataFrame->command = COMMAND_USER_DATA_RESPONSE;
      send_data_wired(pUDPpacket, (userDataFrame->length + USER_DATA_HLEN));
   }

#endif // NODETYPE != COORD
}

/**
 * @brief Callback Function from HDLC layer.
 *
 * Is called by hdlc_light if any data packet via serial interface (USB/RS232) is received.
 *
 * @param   pData pointer to data
 * @param   len   length of data
 */
uint8_t evaluate_wired_data(uint8_t* pData, uint16_t len)
{
   if(*pData == COMMAND_USER_DATA_REQUEST)
   {
      userData_t* userDataFrame = (userData_t*)pData;
      // check that length not exceed, if so, shorten length
      if(userDataFrame->length > MAX_USER_DATA_PAYLOAD)
      {
         userDataFrame->length = MAX_USER_DATA_PAYLOAD;
      }
#if NODETYPE == COORD
      // get short address from destination MAC address
      int16_t shortAddress = getNodeAddressByMAC((uint8_t*)&userDataFrame->mac);

      // if shortAddress == -1 , than MAC does not exist and shortAddress == 0 is coord address
      if(shortAddress > 0)
      {
         // replace destination MAC address by MAC address from Coord node
         userDataFrame->mac = macConfig.longAddr;
         // send message wireless to target node
         send_data_wireless(shortAddress, pData, (userDataFrame->length + USER_DATA_HLEN), UDP_PORT_COORD, UDP_PORT_END_ROUTER);
      }
      // if shortAddress == 0, than coord receives an wired message and should send
      // message back to itself, why should we do that?
#else
      // this node is an endnode/router node, so send message to coord node
      // an end node/router node does not replace MAC address, because Coord never knows where to send message
      send_data_wireless(DEFAULT_COORD_ADDR, pData, (userDataFrame->length + USER_DATA_HLEN), UDP_PORT_COORD, UDP_PORT_END_ROUTER);
#endif
   }
   else if(*pData == COMMAND_NODE_INFO_REQUEST)
   {
      nodeInfo_t* nodeInfoFrame = (nodeInfo_t*) pData;
      nodeInfoFrame->command = COMMAND_NODE_INFO_RESPONSE;
      nodeInfoFrame->length = sizeof(macConfig_t) + 1; // 25 + 1 Bytes
      nodeInfoFrame->macConfig = macConfig;
#if   (NODETYPE == COORD)
      nodeInfoFrame->nodeType = 1;
#elif (NODETYPE == ROUTER)
      nodeInfoFrame->nodeType = 2;
#elif (NODETYPE == ENDDEVICE)
      nodeInfoFrame->nodeType = 3;
#else
      nodeInfoFrame->nodeType = 0;
#endif
      send_data_wired((uint8_t*)nodeInfoFrame, sizeof(nodeInfo_t));
   }
#if NODETYPE == COORD
   else if(*pData ==  COMMAND_CHILD_TABLE_REQUEST)
   {
      send_node_info(COMMAND_CHILD_TABLE_RESPONSE);
   }
   else if(*pData == COMMAND_LINK_QUALITY_REQUEST)
   {
      send_node_info(COMMAND_LINK_QUALITY_RESPONSE);
   }
   else if(*pData == COMMAND_PING_REQUEST)
   {
      payloadPingFrame_t* pingFrame = (payloadPingFrame_t*)&(((deRFprotocol_t*)pData)->payload);
      uint8_t shortAddress = getNodeAddressByMAC((uint8_t*)&pingFrame->mac); // send pointer

      if(shortAddress != 0)
      {
         send_data_wireless(shortAddress, pData, sizeof(deRFprotocol_t), UDP_PORT_COORD, UDP_PORT_END_ROUTER);
      }
   }
   else if(*pData == COMMAND_DATA_FRAME_REQUEST)
   {
      payloadDataFrame_t* dataFrame = (payloadDataFrame_t*)&(((deRFprotocol_t*)pData)->payload);
      int16_t shortAddress = getNodeAddressByMAC((uint8_t*)&dataFrame->mac); // send pointer

      // if shortAddress == -1 , than MAC does not exist and shortAddress == 0 is coord address
      if(shortAddress > 0)
      {
         send_data_wireless(shortAddress, pData, sizeof(deRFprotocol_t), UDP_PORT_COORD, UDP_PORT_END_ROUTER);
      }
   }
#endif // NODETYPE == COORD

   return 0;
}

/*
 * @brief Initialize HDLC layer (frame tagging unit).
 */
void hdlc_init(void)
{
   protocol_init();

#ifdef COMMUNICATION_USB
   // TRANSMITTER PART
   protocol_add(  PROTO_TX,
         usb_getc_std,
         usb_keypressed,
         usb_putc,
         NULL,
         evaluate_wired_data
   );

   //protocol_set_buffer(0, (uint8_t*)dataBuffer, DATA_BUFFER_SIZE);

   // RECEIVER PART
   protocol_add(  PROTO_RX,
         usb_getc_std,
         usb_keypressed,
         usb_putc,
         NULL,
         evaluate_wired_data
   );

   protocol_set_buffer(1, (uint8_t*)hdlcDataBufferReceive, DATA_BUFFER_SIZE);
#endif

#ifdef COMMUNICATION_UART
   // TRANSMITTER PART
   protocol_add(  PROTO_TX,
         uart_getc_std,
         uart_keypressed,
         uart_putc_std,
         NULL,
         evaluate_wired_data
   );

   protocol_set_buffer(0, (uint8_t*)hdlcDataBufferTransmit, DATA_BUFFER_SIZE);

   // RECEIVER PART
   protocol_add(  PROTO_RX,
         uart_getc_std,
         uart_keypressed,
         uart_putc_std,
         NULL,
         evaluate_wired_data
   );

   protocol_set_buffer(1, (uint8_t*)hdlcDataBufferReceive, DATA_BUFFER_SIZE);
#endif

}

/**
 * @brief Process an incoming User Data Frame.
 *
 * This function is only used by coordinator. User data is always send over coordinator
 * (like all other packets - see RUM specification). If coordinator receives an User Data
 * package it replace destination MAC address by origination node MAC address. So the
 * destination (receiving) node knows about the node who originally sends the message.
 *
 * @param   pUserData   pointer to incoming user data frame
 * @param   originAddr  short address of originating node
 *
 */
void process_incoming_user_data(uint8_t* pUserData, uint16_t originAddr)
{
#if NODETYPE == COORD
   // on Coord node side an User Data frame could be a message for this node or
   // should be forwarded to an End or router node
   userData_t* userDataFrame = (userData_t*)pUserData;
   int16_t shortAddress = getNodeAddressByMAC((uint8_t*)&userDataFrame->mac);

   // get MAC address from originating node
   u64 originMAC = 0;
   uint8_t* pMAC = getMACAddressBySA(originAddr);
   if(pMAC != NULL)
   {
      memcpy(&originMAC, pMAC, sizeof(u64));
   }

   // coordinator does not know MAC address -> destination is not an child node
   if(shortAddress == -1)
   {
      // check if destination node is coord node
      if(userDataFrame->mac == macConfig.longAddr)
      {
         // message is for this node, send out wired (USB, RS232)
         userDataFrame->command = COMMAND_USER_DATA_RESPONSE;
         userDataFrame->mac = originMAC; //replace destMAC by MAC from originating node
         send_data_wired(pUserData, (userDataFrame->length + USER_DATA_HLEN));
      }
   }
   else
   {
      //replace destMAC by MAC from originating node
      userDataFrame->mac = originMAC;
      // send message wireless to target node

      send_data_wireless(shortAddress, pUserData, (userDataFrame->length + USER_DATA_HLEN), UDP_PORT_COORD, UDP_PORT_END_ROUTER);
   }
#endif // NODETYPE == COORD
}

/*
 * @brief Return short address of node by node's MAC address.
 *
 * Only coordinator node has capability to execute function, because it's the only node
 * who knows everthing about a node in it's network (PAN).
 *
 * @param   macAddress  pointer to MAC address
 */
int16_t getNodeAddressByMAC(uint8_t* pMacAddress)
{
#if (NODETYPE == COORD)
   // save MAC address as 64 Bit value
   u64 thisMacAddress;
   // take memcpy to be compatible with 8/16/32bit architectures
   memcpy(&thisMacAddress, pMacAddress, sizeof(u64));

   associatedNodes_t *nodes;
   nodes = (associatedNodes_t*)getChildTable();

   associatedNodes_t *node;
   u8 i;
   for(i=1;i<MAXNODES;i++)
   {
      node = &nodes[i];

      //skip if empty
      if (!node->nodeType)
      {
         continue;
      }

      //compare MAC addresses
      if(thisMacAddress == node->nodeLongAddress)
         //if(*((u64*)macAddress) == node->nodeLongAddress)
      {
         return i; //return short address -> short address = index
      }
   }
#endif // NODETYPE == COORD
   return -1;
}

/*
 * @brief Return node's MAC address by it's short address.
 *
 * Only coordinator node has capability to execute function, because it's the only node
 * who knows everthing about a node in it's network (PAN).
 *
 * @param   shortAddress  shortAddress of node
 */
uint8_t* getMACAddressBySA(uint16_t shortAddress)
                  {
#if (NODETYPE == COORD)
   associatedNodes_t* nodes = (associatedNodes_t*)getChildTable();
   associatedNodes_t* node = &nodes[shortAddress];

   if (!node->nodeType)
   {
      return NULL;
   }
   else
   {
      return (uint8_t*)&node->nodeLongAddress;
   }
#endif // NODETYPE == COORD
   return NULL;
                  }

/**
 * @brief Checks for incoming messages over wired interface (USB/RS232).
 *
 * This function have to be called periodically. It checks serial interface (USB/RS232) for
 * new incoming messages. If not called periodically, messages can be lost.
 */
void wired_packet_task(void)
{
   protocol_receive(1);
}

/**
 * @brief Send out a UDP Packet over wireless interface.
 *
 * @param   destAddr    short address of destination node
 * @param   pData       pointer to data packet
 * @param   len         length of data packet
 * @param   srcUDPPort  UDP port of source node
 * @param   destUDPPort UDP port of destination node
 */
void send_data_wireless(uint16_t destAddr, uint8_t* pData, uint8_t len, uint16_t srcUDPPort, uint16_t destUDPPort)
{
   sixlowpan_hc01_udp_setup_iplocal(destAddr);
   sixlowpan_hc01_udp_setup_ports(srcUDPPort, destUDPPort);

   memcpy( sixlowpan_hc01_udp_get_payloadptr(), pData, len);
   sixlowpan_hc01_udp_set_payloadsize(len);
   sixlowpan_hc01_udp_send();
}

/**
 * @brief Send data out over wired interface (USB/RS232).
 *
 * This function sends out a data packet over serial interface (USB/RS232). This is done
 * via HDLC layer. So outgoing messages are 'frame tagged'.
 *
 * @param   pData    pointer da data packet
 * @param   length   length of data packet
 */
void send_data_wired(uint8_t* pData, uint8_t length)
{
   protocol_send(0, pData, length);
}

/**
 * @brief Send out message with info's about child node's from coordinator.
 *
 * Is only executed on coordinator. Loops over all child nodes which connected to coordinator.
 * For every associated child node a message is generated which contains the following info's:
 *    - child short address
 *    - child MAC address
 *    - child node type (router, end node)
 *    - last routed address of this child node
 *    - parent address of this child node (direct parent)
 *    - LQI/ED value of this child node
 *
 */
void send_node_info(uint8_t command)
{
#if NODETYPE == COORD
   associatedNodes_t* nodes = (associatedNodes_t*)getChildTable();
   associatedNodes_t* node;
   u8 i;

   deRFprotocol_t frame;

   // i = 0 -> coordinator node
   for(i=1;i<MAXNODES;i++)
   {
      node = &nodes[i];

      if (!(node->nodeType)){
         continue;
      }

      frame.command = command;
      frame.option = NO_OPTION;

      payloadInitUpdateFrame_t* dataFrame = (payloadInitUpdateFrame_t*)&frame.payload;

      dataFrame->mac                 = node->nodeLongAddress;
      dataFrame->nodeType            = node->nodeType;
      dataFrame->shortAddress        = i;
      dataFrame->lastRoutedAddress   = node->lastRoutedAddress;
      dataFrame->ed                  = node->ed;
      dataFrame->lqi                 = node->lqi;
      dataFrame->parentAddress       = node->parentShortAddress;

      // send frame out to usb
      send_data_wired((uint8_t*)&frame, sizeof(deRFprotocol_t));
   }
#endif // NODETYPE == COORD
}

/* EOF */

