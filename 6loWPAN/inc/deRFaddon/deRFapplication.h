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
 * @file deRFapplication.h
 *
 * @brief Header file for Application Module
 *
 * $Id: deRFapplication.h,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-01-13
 */

#ifndef DERFAPPLICATION_H_
#define DERFAPPLICATION_H_

#include "../../inc/mac.h"

#define RFAPP_UART_ENABLE     (1) ///< Enable (1) or Disable (0) the deRFapplication UART debug print mode

/***********************************************
 * defines UDP ports where application should work
 ***********************************************/

/** UDP Port to talk with coordinator */
#define UDP_PORT_COORD        (0xF0BC)
/** UDP Port to talk with router/end node */
#define UDP_PORT_END_ROUTER   (0xF0BB)


/**********************************************
 * Protocol Port definitions
 *********************************************/
#define RCB_BUTTON_PORT_POSITION (0)
#define RCB_BUTTON_BIT_POSITION  (0)

#define STB_BUTTON_PORT_PROTOCOL (1)
#define STB_BUTTON_BIT_POSITION  (7)

/**********************************************
 * GPIO Breakout Board
 *********************************************/

#if PLATFORM == RCBSINGLE
#define PORT_PA0                 (SELECT_PORT_B)
#define PORT_PA1                 (SELECT_PORT_B)
#define PORT_PA2                 (SELECT_PORT_B)
#define PORT_PA3                 (SELECT_PORT_B)
#define PORT_PA4                 (SELECT_PORT_B)
#define PORT_PA5                 (SELECT_PORT_B)
#define PORT_PA6                 (SELECT_PORT_B)
#define PORT_PE7                 (SELECT_PORT_E)

#define BIT_PA0                  (PB0)
#define BIT_PA1                  (PB1)
#define BIT_PA2                  (PB2)
#define BIT_PA3                  (PB3)
#define BIT_PA4                  (PB4)
#define BIT_PA5                  (PB5)
#define BIT_PA6                  (PB6)
#define BIT_PE7                  (PE7)
#else
#define PORT_PA0                 (SELECT_PORT_A)
#define PORT_PA1                 (SELECT_PORT_A)
#define PORT_PA2                 (SELECT_PORT_A)
#define PORT_PA3                 (SELECT_PORT_A)
#define PORT_PA4                 (SELECT_PORT_A)
#define PORT_PA5                 (SELECT_PORT_A)
#define PORT_PA6                 (SELECT_PORT_A)
#define PORT_PE7                 (SELECT_PORT_E)

#define BIT_PA0                  (PA0)
#define BIT_PA1                  (PA1)
#define BIT_PA2                  (PA2)
#define BIT_PA3                  (PA3)
#define BIT_PA4                  (PA4)
#define BIT_PA5                  (PA5)
#define BIT_PA6                  (PA6)
#define BIT_PE7                  (PA7)

#endif

#define PROTOCOL_PORT_PA0        (4)
#define PROTOCOL_PORT_PA1        (4)
#define PROTOCOL_PORT_PA2        (4)
#define PROTOCOL_PORT_PA3        (4)
#define PROTOCOL_PORT_PA4        (4)
#define PROTOCOL_PORT_PA5        (4)
#define PROTOCOL_PORT_PA6        (4)
#define PROTOCOL_PORT_PE7        (4)

#define PROTOCOL_BIT_PA0         (0)
#define PROTOCOL_BIT_PA1         (1)
#define PROTOCOL_BIT_PA2         (2)
#define PROTOCOL_BIT_PA3         (3)
#define PROTOCOL_BIT_PA4         (4)
#define PROTOCOL_BIT_PA5         (5)
#define PROTOCOL_BIT_PA6         (6)
#define PROTOCOL_BIT_PE7         (7)

/**********************************************
 * GPIO SensTermBoard
 *********************************************/

#if PLATFORM == RCBSINGLE
#define PORT_PE4                 (SELECT_PORT_E)
#define PORT_PE5                 (SELECT_PORT_E)
#define PORT_PD5                 (SELECT_PORT_D)
#define PORT_PD7                 (SELECT_PORT_D)
#define PORT_PB6                 (SELECT_PORT_B)
#define PORT_PB7                 (SELECT_PORT_G)
#define PORT_I2C_SCL             (SELECT_PORT_D)
#define PORT_I2C_SDA             (SELECT_PORT_D)
#define PORT_UART_RXD            (SELECT_PORT_D)
#define PORT_UART_TXD            (SELECT_PORT_D)

#define BIT_PE4                  (PE4) // [deRFmega128] RSTON and #WR on STB; LED on RCB
#define BIT_PE5                  (PE5) // [deRFmega128] #RD and PE5 on STB; Button on RCB;
#define BIT_PD5                  (PD5) // [deRFmega128] PC5 and PD5 on STB
#define BIT_PD7                  (PD7) // [deRFmega128] PC7 and PD7 on STB
#define BIT_PB6                  (PB6) // [deRFmega128] GND on Adapter - PB6 on STB
#define BIT_PB7                  (PG1) // [deRFmega128] PG1 on Adapter - PB7 on STB
#define BIT_I2C_SCL              (PD0)
#define BIT_I2C_SDA              (PD1)
#define BIT_UART_RXD             (PD2)
#define BIT_UART_TXD             (PD3)
#else
#define PORT_PE4                 (SELECT_PORT_E)
#define PORT_PE5                 (SELECT_PORT_E)
#define PORT_PD5                 (SELECT_PORT_D)
#define PORT_PD7                 (SELECT_PORT_D)
#define PORT_PB6                 (SELECT_PORT_B)
#define PORT_PB7                 (SELECT_PORT_B)
#define PORT_I2C_SCL             (SELECT_PORT_B)
#define PORT_I2C_SDA             (SELECT_PORT_D)
#define PORT_UART_RXD            (SELECT_PORT_D)
#define PORT_UART_TXD            (SELECT_PORT_D)

#define BIT_PE4                  (PE4)
#define BIT_PE5                  (PE5)
#define BIT_PD5                  (PD5)
#define BIT_PD7                  (PD7)
#define BIT_PB6                  (PB6)
#define BIT_PB7                  (PB7)
#define BIT_I2C_SCL              (PD0)
#define BIT_I2C_SDA              (PD1)
#define BIT_UART_RXD             (PD2)
#define BIT_UART_TXD             (PD3)
#endif

#define PROTOCOL_PORT_PE4        (2)
#define PROTOCOL_PORT_PE5        (2)
#define PROTOCOL_PORT_PD5        (2)
#define PROTOCOL_PORT_PD7        (2)
#define PROTOCOL_PORT_PB6        (2)
#define PROTOCOL_PORT_PB7        (2)
#define PROTOCOL_PORT_I2C_SCL    (2)
#define PROTOCOL_PORT_I2C_SDA    (2)
#define PROTOCOL_PORT_UART_RXD   (3)
#define PROTOCOL_PORT_UART_TXD   (3)

#define PROTOCOL_BIT_PE4         (0)
#define PROTOCOL_BIT_PE5         (1)
#define PROTOCOL_BIT_PD5         (2)
#define PROTOCOL_BIT_PD7         (3)
#define PROTOCOL_BIT_PB6         (4)
#define PROTOCOL_BIT_PB7         (5)
#define PROTOCOL_BIT_I2C_SCL     (6)
#define PROTOCOL_BIT_I2C_SDA     (7)
#define PROTOCOL_BIT_UART_RXD    (0)
#define PROTOCOL_BIT_UART_TXD    (1)

//OUTPUT
#define PORT_X4_REL1             (SELECT_PORT_E)
#define PORT_X4_REL2             (SELECT_PORT_E)

#define BIT_X4_REL1              (PE2)
#define BIT_X4_REL2              (PE3)

#define PROTOCOL_PORT_X4_REL1    (1)
#define PROTOCOL_PORT_X4_REL2    (1)

#define PROTOCOL_BIT_X4_REL1     (2)
#define PROTOCOL_BIT_X4_REL2     (3)


/*
 * show allocation of protocol port definitions
 *
   PORT_0_0 -  RCB Button -> use a single routine
   PORT_0_1 -  RCB LED -> use a single routine
   PORT_0_2 -  RCB LED -> use a single routine
   PORT_0_3 -  RCB LED -> use a single routine
   PORT_0_4 -  empty
   PORT_0_5 -  empty
   PORT_0_6 -  empty
   PORT_0_7 -  empty

   PORT_1_0 -  STB LED -> use a single routine
   PORT_1_1 -  STB LED -> use a single routine
   PORT_1_2 -  STB X4_REL_1
   PORT_1_3 -  STB X4_REL_2
   PORT_1_4 -  empty
   PORT_1_5 -  empty
   PORT_1_6 -  empty
   PORT_1_7 -  STB Button -> use a single routine

   PORT_2_0 -  STB X3_GPIO
   PORT_2_1 -  STB X3_GPIO
   PORT_2_2 -  STB X3_GPIO
   PORT_2_3 -  STB X3_GPIO
   PORT_2_4 -  STB X3_GPIO
   PORT_2_5 -  STB X3_GPIO
   PORT_2_6 -  STB X4_I2C_SCL
   PORT_2_7 -  STB X4_I2C_SDA

   PORT_3_0 -  STB X5_UART_RXD
   PORT_3_1 -  STB X5_UART_TXD
   PORT_3_2 -  empty
   PORT_3_3 -  empty
   PORT_3_4 -  empty
   PORT_3_5 -  empty
   PORT_3_6 -  empty
   PORT_3_7 -  empty

   PORT_4_0 -  Breakout Board - PA0
   PORT_4_1 -  Breakout Board - PA1
   PORT_4_2 -  Breakout Board - PA2
   PORT_4_3 -  Breakout Board - PA3
   PORT_4_4 -  Breakout Board - PA4
   PORT_4_5 -  Breakout Board - PA5
   PORT_4_6 -  Breakout Board - PA6
   PORT_4_7 -  Breakout Board - PE7

   PORT_5_0 -  empty
   PORT_5_1 -  empty
   PORT_5_2 -  empty
   PORT_5_3 -  empty
   PORT_5_4 -  empty
   PORT_5_5 -  empty
   PORT_5_6 -  empty
   PORT_5_7 -  empty

   PORT_6_0 -  empty
   PORT_6_1 -  empty
   PORT_6_2 -  empty
   PORT_6_3 -  empty
   PORT_6_4 -  empty
   PORT_6_5 -  empty
   PORT_6_6 -  empty
   PORT_6_7 -  empty

   PORT_7_0 -  empty
   PORT_7_1 -  empty
   PORT_7_2 -  empty
   PORT_7_3 -  empty
   PORT_7_4 -  empty
   PORT_7_5 -  empty
   PORT_7_6 -  empty
   PORT_7_7 -  empty
*/

/***********************************************
 * defines protocol specific assignments
 ***********************************************/
#define SELECT_PORT_A            (0)
#define SELECT_PORT_B            (1)
#define SELECT_PORT_C            (2)
#define SELECT_PORT_D            (3)
#define SELECT_PORT_E            (4)
#define SELECT_PORT_F            (5)
#define SELECT_PORT_G            (6)
#define SELECT_NO_PORT           (255)

#define SELECT_NO_BIT            (255)

/** Maximum frame length*/
#define MAX_FRAME_LENGTH         (79)

/** Maximum protocol length @see deRFprotocol_t*/
#define PROTOCOL_LENGTH          (59)

/** Maximum available protocol payload @see deRFprotocol_t*/
#define MAX_PAYLOAD_PROTOCOL     (PROTOCOL_LENGTH - 2) //(Length - (command + option))

/** Maximum available user data payload @see userData_t*/
#define MAX_USER_DATA_PAYLOAD    (MAX_FRAME_LENGTH - 8 - 2) //(Length - MAC - (command + length))

/**
 * @brief Holds the data frame.
 *
 * It's the payload inside the deRFprotocol_t.
 */
typedef struct
{
   uint64_t mac;                       ///< MAC address of this node
   uint8_t  digitalIODirection[8];     ///< represents the direction of the digital data (input = 1, output = 0)
   uint8_t  analogIODirection[1];      ///< represents the direction of the analog data (input = 1, output = 0)
   uint8_t  digitalData[8];            ///< represents the value of the digital data (set = 1, unset = 0)
   int32_t  analogData[8];             ///< represents the value of the analog data (set = 1, unset = 0)
}__attribute__((packed)) payloadDataFrame_t;

/**
 * @brief Hold initialization/update frame values
 *
 * It's the payload inside the deRFprotocol_t.
 */
typedef struct
{
   uint64_t mac;                 ///< MAC address of this node
   uint16_t shortAddress;        ///< short address of this node
   uint16_t parentAddress;       ///< parent address of this node
   uint8_t  nodeType;            ///< node type of this node (coordinator, router, end node)
   uint16_t lastRoutedAddress;   ///< short address of node where last message for this node went to (if hops == 0, then lastRoutedAddress == short address of this node)
   uint8_t  lqi;                 ///< actual LQI value of this node
   uint8_t  ed;                  ///< actual ED value of this node
}__attribute__((packed)) payloadInitUpdateFrame_t;

/**
 * @brief Hold ping frame values
 *
 * It's the payload inside the deRFprotocol_t.
 */
typedef struct
{
   uint64_t mac;                                ///< MAC address of this node
}__attribute__((packed)) payloadPingFrame_t;

/**
 * @brief Hold quality frame values
 *
 * Only for internal use (node <-> node communication).
 * It's the payload inside the deRFprotocol_t.
 */
typedef struct
{
   uint16_t short_address;        ///< short address of this node
   uint8_t  lqi;                  ///< actual LQI value of this node
   uint8_t  ed;                   ///< actual ED value of this node
}__attribute__((packed)) payloadQualityFrame_t;

/**
 * @brief Hold status frame values
 *
 * This is used for test and debugging issues. To enable use STATUS_DEBUG compiler macro.
 */
#if defined(STATUS_DEBUG) || defined(DOXYGEN)
typedef struct
{
   uint64_t mac;                    ///< MAC address of this node
   uint16_t short_address;          ///< short address of this node
   uint16_t parentAddress;          ///< short address of direct parent of this node
   uint8_t  nodeType;               ///< node type of this node (coordinator, router, end node)
   uint16_t lastRoutedAddress;      ///< short address of node where last message for this node went to (if hops == 0, then lastRoutedAddress == short address of this node)
   uint32_t timer_node;             ///< free running timer value of this node
   uint32_t timer_coord;            ///< free running timer value of coordinator
   uint8_t  free_buffers_node;      ///< number of free buffers of this node (have to enable DEBUG_BUFFER in bmm.h)
   uint8_t  free_buffers_coord;     ///< number of free buffers of coordinator (have to enable DEBUG_BUFFER in bmm.h)
   uint8_t  lqi;                    ///< actual LQI value of this node
   uint8_t  ed;                     ///< actual ED value of this node
   uint16_t alarm_timer;            ///< timer value (in ms) when timer expire and new status mesage is created
}__attribute__((packed)) payloadStatusFrame_t;
#endif

/**
 * @brief Structure that hold the main protocol
 *
 * All other (except userData_t and nodeInfo_t) structs are derived from this one.
 */
typedef struct
{
    uint8_t command;                         ///< defines the command, which select the service
    uint8_t option;                          ///< option field, currently not used
    uint8_t payload[MAX_PAYLOAD_PROTOCOL];   ///< payload, depends on command
} __attribute__((packed)) deRFprotocol_t;

/**
 * @brief Structure that hold user data
 */
typedef struct
{
    uint8_t  command;                        ///< defines the command
    uint8_t  length;                         ///< length field, defines the payload length
    uint64_t mac;                            ///< MAC address of node
    uint8_t  payload[MAX_USER_DATA_PAYLOAD]; ///< payload (arbitrary user data)
} __attribute__((packed)) userData_t;

/**
 * @brief Structure hold node info data
 */
typedef struct
{
    uint8_t       command;                   ///< defines the command
    uint8_t       length;                    ///< length field, defines the payload length
    macConfig_t   macConfig;                 ///< structure that hold different info's about this node
    uint8_t       nodeType;                  ///< type of this node (coordinator, router, end node)
} __attribute__((packed)) nodeInfo_t;


/** Header length of User Data frame. */
#define USER_DATA_HLEN        (1 + 1 + 8)    // command + option/length + MAC


/***********************************************
 * Prototypes
 ***********************************************/
void process_coord_udp_packet(uint8_t* pUDPpacket, uint16_t originAddr);
void process_endnode_udp_packet(uint8_t* pUDPpacket, uint16_t originAddr);
uint8_t evaluate_wired_data(uint8_t* pData, uint16_t len);
void hdlc_init(void);
void wired_packet_task(void);
void send_data_wireless(uint16_t destAddr, uint8_t* pData, uint8_t len, uint16_t srcUDPPort, uint16_t destUDPPort);
void send_data_wired(uint8_t* pData, uint8_t length);

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif /* DERFAPPLICATION_H_ */
