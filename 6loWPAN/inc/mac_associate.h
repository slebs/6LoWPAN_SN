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

  $Id: mac_associate.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
*/
#ifndef ASSOCIATION_REQUEST_H
#define ASSOCIATION_REQUEST_H

#include "rum_types.h"
#include "mac.h"

/**
   This is the structure for each element of the table of the
   associated nodes. The node short address is the index to the
   array. NOTE: THE COORD SHORT ADDRESS IS ALWAYS 0.

   @ingroup mac_associate
*/
typedef struct
{
    u8  nodeType;           ///< Type of node, see @ref COORD for types
    u64 nodeLongAddress;    ///< The long (MAC) address of the node
    u16 parentShortAddress; ///< The short address of the parent of this node
    u16 lastRoutedAddress;  ///< The short address of the last route from this node
    u8  lqi;                ///< The LQI value of this node (saved every time a LQI/ED packet is received) -> added by Dresden Elektronik 01.02.10
    u8  ed;                 ///< The ED value of this node (saved every time a LQI/ED packet is received) -> added by Dresden Elektronik 01.02.10
    u8  wakeup:1;             ///< If set, the child node is to be woken up
    u8 sleeping:1;            ///< If set, the child node is a sleeping node
} __attribute__((packed)) associatedNodes_t;

// Define the maximum number of nodes a coordinator can associate
#define MAXNODES 100

// ROUTER NODE DEFINITIONS
#define MAXCHILDREN 50

typedef struct
{
    u16 childAddr;
    u8  lqi;
    u8  ed;
    u8  wakeup:1;
    u8  sleeping:1;
} __attribute__((packed)) tChildTableItem;


// Prototypes
//void macAssociationResponse(void);
void macAssociationResponse(uint8_t* pFrame);
void macAssociationConfirm(void);
void macAddChild(u16 shortAddr);
void macRemoveChild(u16 shortAddr);
u8 macIsChild(u16 shortAddr);
u8 macIsChildSleeping(u16 shortAddr);
u16 macGetTopParent(u16 addr);
u8 macCreateRoute(u16 shortAddr, ftRouting *frame);
u16 macGetParent(u16 shortAddr);
u16 macGetLastRoute(u16 parent);
void macSetLastRoute(u16 parent, u16 addr);
associatedNodes_t *macGetNode(u16 index);
void macInitNodes(void);
u16 findFirstChild(u16 addr);
u16 macFirstChild(void);
u16 macNextChild(void);
u16 findNextSibling(u16 addr);
void macChildIsAwake(ftData *frame);
void macWakeChildNode(u16 addr);
void macClearChildWakeFlag(u16 addr);
u8 macGetHopCount(u16 node_short);

//TODO added by Dresden Elektronik to get acces to child table from coordinator node
uint8_t* getChildTable(void);
//TODO added by Dresden Elektronik to be able to call the Association Timeout method after send event occurs
void setAssociationTimeout(void);

// debug functions
void macPrintTree(void);
void arm_macPrintTree(void);

#endif
