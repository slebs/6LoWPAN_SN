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
  $Id: mac.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
*/

#ifndef MAC_H
#define MAC_H

/**
   @addtogroup mac

   The RUM MAC is a quasi-compliant 802.15.4 MAC layer.  It provides
   access to basic radio setup functions for channel, power, etc.
   Also, the MAC provides functions to scan all channels for a
   coordinator, associate with a coordinator, send packets, and
   receive packets via callback functions.

   @section frame_format Frame formats

   For the diagram below, see @ref FCF_DATA, @ref ftData, and @ref
   sftSensorReading.

   @subsection dataframe Data Frame 

   This is the format for a data frame, but the format is similar to
   most other frames.

   @verbatim
fcf  seq panID dest src  finalDest originAddr type appType
6188 34  babe  0000 0001   0000      0001      xx    yy
  |   |   |      |    |      |         |        |     |
  |   |   |      |    |      |         |        |      -- Sensor frame type
  |   |   |      |    |      |         |        |          (See below)
  |   |   |      |    |      |         |         ---- Frame type
  |   |   |      |    |      |         |              DATA_FRAME         1
  |   |   |      |    |      |         |              PING_REQ_FRAME     2
  |   |   |      |    |      |         |              PING_RSP_FRAME     3
  |   |   |      |    |      |         |              DROP_CHILD_FRAME   4
  |   |   |      |    |      |         |              DATA_FRAME_6LOWPAN 5
  |   |   |      |    |      |         |              WAKE_NODE          6
  |   |   |      |    |      |         |              DEBUG_FRAME        7
  |   |   |      |    |      |         |
  |   |   |      |    |      |          -----  Address originating this packet
  |   |   |      |    |      |                 (short address - 2 bytes)
  |   |   |      |    |       ---- Final destination short address         
  |   |   |      |    |
  |   |   |      |     --- Source address (node sending this frame this hop)
  |   |   |      | 
  |   |   |       ---- Destination (node receiving this frame this hop)
  |   |   |
  |   |    --- PAN ID for network
  |   |
  |    --- Sequence number for sending node - incremented each frame
  |
   --- Frame Control Field (see format below)
   @endverbatim

   @subsection sfframe Sensor frame

   This describes the payload of a data frame, used by the sensor application.

   @verbatim

  Sensor Frame Type
 (from ftData above)  Value   Payload contents
 ===================  =====   ================
 SET_NODE_NAME          2     8 bytes of node name (ASCIIZ string)
 REQ_READING_FRAME      3     Time interval between data frames
 READING_FRAME          4     Address, reading, units, name
 CAL_REQ_INFO_FRAME     5     None
 CAL_INFO_FRAME         6     Calibration type, address, units
 REQ_RAW_DATA_FRAME     7     None
 RAW_DATA_FRAME         8     Raw A/D reading
 CAL_CMD_FRAME          9     Index and calibrated reading

 For READING_FRAME:

type addr  reading (6 bytes)   units (5 bytes)     name (8 bytes)
 04   12  31 2E 32 33 34 35 36 64 65 46 00 00  48 61 6C 6C 77 61 79 00
 --   --  -------------------- --------------  -----------------------
  |    |           |                  |                  |
  |    |            --- Reading        --- Units          --- Node name
  |    |                (ASCIIZ)          (ASCIIZ)             (ASCIIZ)
  |    |
  |     --- Short address of node sending reading
  |
   --- Sensor frame type (see table above)
   @endverbatim

   @section mac_calls MAC function calls

   Here is a summary of the function calls and packets sent for
   typical operation.

   @verbatim
Router/End node             Frame type over the air                 Coordinator


                                  Scan channels
                                (For each channel)

appStartScan()
macScan()                 --->   ftBeaconReq      --->        macTask()

mac_logPanDescriptors()   <---     ftBeacon       <---        sendBeaconFrame()



                                   Associate

macAssociate()            --->  ftAssocReqDirect  --->        macTask()

macTask()                 <---  ftAssocRespDirect <---        macAssociationResponse()


                                   Send Data

macDataRequest()          --->       ftData       --->        macDataIndication()


   @endverbatim
*/

// Includes
#include <stdint.h>
#include "../inc/stdbool.h"
#include "../inc/rum_types.h"

#define RUM_VERSION                    "V0.9"

// Define a debug mode (0 = no debug, minimal size, 1 = serial code and messages)
// This can be defined by make file.  For Makefile.linux, use "make DEBUG=0" to
// override this setting.
#ifndef DEBUG
/**
   The global debugging flag.  Set this macro to zero for no debugging
  info (serial output), or set to one for serial output.

   @ingroup app
*/
#define DEBUG 1 // default value of debug is 1
#endif

/**
   Flag to enable the serial port.  Set this to one to enable the port.

   @ingroup serial
*/
#ifndef SERIAL
#define SERIAL 1
#endif

/**
   Flag to enable the OTA (over-the-air) debug port.  Set this to one
   to enable the port.  If enabled, this causes debug messages to be
   sent to the coordinator for display.

   @ingroup mac
*/
#ifndef OTA_DEBUG
#define OTA_DEBUG 0
#endif


// If DEBUG is on, but neither SERIAL or OTA_DEBUG is set, then DEBUG can't work
#if (DEBUG && (SERIAL == 0) && (OTA_DEBUG == 0))
#error "If DEBUG is set, then either SERIAL or OTA_DEBUG must be set"
#endif

/**
   Flag to signify a "very low power" (VLP) node.  This type of node
   uses a weak battery that cannot power the node for very long, or
   some energy-harvesting power source.

   This option is off by default

   @ingroup mac
*/
#ifndef VLP
#define VLP 0
#endif

// Demo mode -
#ifndef DEMO
/**
   The global demo flag.  Set this macro to zero for normal operation,
   or set to one for demo mode, in which nodes are set to low power
   and will choose a parent based on lowest RSSI, rather than the
   normal LQI/hops/RSSI criteria.

   @ingroup app
*/
  #define DEMO 0
#endif


#define CHANNEL255 0xff
/**
   Channel used for the PAN.  This definition is used by all node
   types.  If the PAN_CHANNEL macro is set to CHANNEL255, then the
   node will scan channels before either choosing an operating channel
   (coordinator), or picking a parent node (router and end nodes).

   @ingroup mac_associate
*/
#ifndef PAN_CHANNEL
  #define PAN_CHANNEL (CHANNEL255)
#endif

/**
   When CHINA_MODE is set, there are only 4 channels (1-4).  This mode
   only works on platforms that have BAND_900 set.

   @ingroup mac
*/
#ifndef CHINA_MODE
  #define CHINA_MODE 0
#endif


/**
   PANID used for the PAN ID.  This is only used by the coordinator.
   If this macro is set to BROADCASTPANID, then the coordinator
   chooses a random PAN ID.

   @ingroup mac_associate
*/
#ifndef PAN_ID
  #define PAN_ID BROADCASTPANID
#endif

/**
   Define which data rate we are using (900MHz band only).

   @ingroup radio
*/
#define DATA_RATE_212    BPSK_40


#include "hal.h"
#if __AVR__
#include <stdio.h>
#endif
// Some platforms have no serial port
#if PLATFORM == SPITFIRE
#undef  DEBUG
#define DEBUG 0
#endif
extern char debugStr[DEBUG * 40];


/** @addtogroup mac
    @{
    @name Pre-defined node types

    See @ref NODETYPE for usage.
    @{
*/
#define COORD     1  ///< Coordinator node
#define ROUTER    2  ///< Router node
#define ENDDEVICE 3  ///< End node
/** @}
    @}*/

/**
   The coordinator's short address, always zero
   @ingroup mac
*/
#define DEFAULT_COORD_ADDR      0x0000
#define DEFAULT_FIRST_NODE_ADDR      0x0001
#define DEFAULT_ARM_COORD_LONG  0xAABBCCDDEEFF1122ll

// Define whether we are a coordinator or not.
// This affects compilation
/**
    @brief NODETYPE is defined to be one of the three basic node
    types: coordinator, router, or end node.

    This definition is used widely throughout the code to define the
    different behavior for the different types of nodes. For example,
    there are many places in the code that look like this:

    @code
    if (NODETYPE == COORD)
    {
      // Do something only the coordinator would do.
    }
    @endcode

    This comparison causes the compiler to optimize away the code
    inside the brackets for a router or end node, which saves on flash
    space.

    To define the NODETYPE variable, define one of the following
    variables (as one) in the Makefile or AVR Studio project setting.

    - COORDNODE
    - ROUTER
    - ENDNODE
    @ingroup mac
*/
#ifdef DOXYGEN
#define NODETYPE COORD
#endif
#ifdef COORDNODE
  #define NODETYPE COORD
#endif
#ifdef ENDNODE
  #define NODETYPE ENDDEVICE
#endif
#ifdef ROUTERNODE
  #define NODETYPE ROUTER
#endif


#ifndef NODETYPE
// Warning - NODETYPE must be defined
#error "You must define one of COORDNODE, ROUTERNODE, or ENDNODE to specify which"
#error "target you are compiling for.  With AVR Studio, make sure you"
#error "have -DCOORDNODE listed under '[All Files]', on custom options"
#error "tab of the project options dialog."
#error "With the Makefile for Linux, specify a node type like this:"
#error "make TYPE=COORD"
#error "make TYPE=END"
#endif

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) //!< A frame should be at least 3 bytes.
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) //!< A frame should no more than 127 bytes.

/**
   Locations in EEPROM of the stored data.  Note that this struct only
   exists in EEPROM, and is never created in RAM.  There are various
   macros that use this structure definition to locate data in EEPROM.

   @ingroup app
*/
#if (__AVR__)
#include "sensors.h"
typedef struct {
    u8 eepromMacAddress[8];      ///< The node's unique IEEE MAC address
    tCalFactors calFactors;      ///< The node's calibration data (see sensor.h)
    u16 dataSeconds;             ///< The number of tenth seconds between data readings (see sensor.h)
} tEepromContents;
#endif // __AVR__
// Macros & Defines
#define SUCCESS                         (0)
#define CHANNEL_PAGE_0                  (0)     // This represents the 2.4GHz RF230/231.

/**
   @addtogroup mac
   @{
*/
#define BROADCASTADDR   (0xFFFF) ///< The broadcast address, which all nodes receive.  See IEEE802.15.4
#define BROADCASTPANID (0xFFFF)  ///< The broadcast PAN ID, which all nodes receive.  See IEEE802.15.4


/**
 *  @brief  This struct defines the rx data container.
 *
 *  @see hal_frame_read
 */
typedef struct{
    u8 length;                       ///< Length of frame.
    u8 data[ HAL_MAX_FRAME_LENGTH ]; ///< Actual frame data.
    u8 lqi;                          ///< LQI value for received frame.
    bool crc;                        ///< Flag - did CRC pass for received frame?
} __attribute__((packed)) rx_frame_t;




/**
    @name MAC command frame codes.
    @{
*/
#define ASSOCIATION_REQUEST              (0x01)  ///< Association request
#define ASSOCIATION_RESPONSE             (0x02)  ///< Association response
#define DISSASOCIATION_NOTIFICATION      (0x03)
#define DATA_REQUEST                     (0x04)
#define PAN_ID_CONFLICT_NOTIFICATION     (0x05)
#define ORPHAN_NOTIFICATION              (0x06)
#define BEACON_REQUEST                   (0x07)  ///< Beacon request
#define COORDINATOR_REALIGNMENT          (0x08)
#define GTS_REQUEST                      (0x09)
#define ROUTING_PACKET                   (0xbb)  ///< Routing packet MAC type
/** @} */


#define ACK_USED 1

/**
   @name FCF values

   @brief These are pre-defined values for the Frame Control Field.
   The FCF is contained in the first two bytes of an 802.15.4 frame.
   See the IEEE specification for details.

   @{
*/
#if ACK_USED
#define FCF_BEACONREQ         0x0803  ///< Beacon request, no src, short dest, no ack, MAC frame
#define FCF_ASSOC_REQ_DIRECT  0xC863  ///< Direct association request, long src, short dest, ack, MAC frame
#define FCF_ASSOC_REQ_IND     0x8863  ///< Indirect association request, short src and dest, ack, MAC frame
#define FCF_ASSOC_RESP_IND    0x8863  ///< Indirect association response, short src and dest, ack, MAC frame
#define FCF_ASSOC_RESP_DIRECT 0x8C63  ///< Direct association response, short src, long dest, ack, MAC frame
#define FCF_ROUTE             0x8863  ///< Routing frame, short src and dest, ack, MAC frame
#define FCF_BEACON            0x8000  ///< Beacon frame, short src, no dest, beacon frame
#define FCF_DATA              0x8861  ///< Data frame, short src and dest, ack, data frame
#define FCF_MAC_CMD           0x8863  ///< Mac Command frame, short src and dest, ack, MAC frame
#else
#define FCF_BEACONREQ         0x0803  ///< Beacon request, no src, short dest, no ack, MAC frame
#define FCF_ASSOC_REQ_DIRECT  0xC843  ///< Direct association request, long src, short dest, ack, MAC frame
#define FCF_ASSOC_REQ_IND     0x8843  ///< Indirect association request, short src and dest, ack, MAC frame
#define FCF_ASSOC_RESP_IND    0x8843  ///< Indirect association response, short src and dest, ack, MAC frame
#define FCF_ASSOC_RESP_DIRECT 0x8C43  ///< Direct association response, short src, long dest, ack, MAC frame
#define FCF_ROUTE             0x8843  ///< Routing frame, short src and dest, ack, MAC frame
#define FCF_BEACON            0x8000  ///< Beacon frame, short src, no dest, beacon frame
#define FCF_DATA              0x8841  ///< Data frame, short src and dest, ack, data frame
#define FCF_MAC_CMD           0x8843  ///< Mac Command frame, short src and dest, ack, MAC frame
#endif
/** @} */

/**
   @name Data frame types

   @brief There are several types of data frames defined at the MAC
   level.  Only one type is used by the application, and the others
   are used only by the MAC.

   Any data frame as defined by the IEEE 802.15.4 specification
   (having type = 1 in the FCF) has a type field that contains this
   value.

   Note that the upper bit of the type field is used to denote a
   sleeping node.  If the bit is set, then the node is sleeping.

   @{
*/
#define DATA_FRAME             1   ///< Application data frame
#define PING_REQ_FRAME         2   ///< Ping request frame
#define PING_RSP_FRAME         3   ///< Ping response frame
#define DROP_CHILD_FRAME       4   ///< Frame sent to router to command it to drop a child from its table
#define DATA_FRAME_6LOWPAN     5   ///< 6lowpan Data Frame
#define WAKE_NODE              6   ///< Wake up an end node
#define DEBUG_FRAME            7   ///< Over-the air debug frame
/** @} */

/**
   @name Timeouts

   @brief Timeout constants used by various parts of the program.

   @{
*/
#define ASSOCIATION_TIMEOUT  1000  ///< mSec to wait before giving up on association process
#define MAC_RP_DELAY         5     ///< mSec to wait after sending a routing packet, before sending data
#define MAC_DATA_DELAY       7     ///< mSec to wait between sending data packets
#define MAC_NOTIFY_DELAY     160   ///< mSec to wait before sending a child drop notification
#define LED_DELAY            10    ///< mSec to wait before turning off LED
#define LED_PING_DELAY       400   ///< mSec to wait before turning off LED
/** @} */

/**
   The FCF type, which is just a 16-bit integer
*/
typedef u16 fcf_t;


/**
   Superframe specification frame type.  This bitfield is part of a
   beacon frame.
*/
typedef struct
{
    u8   beacon_order : 4;
    u8   superframe_order : 4;
    u8   final_cap_slot : 4;
    bool batt_life_extension : 1;
    bool reserved : 1;
    bool pan_coord : 1;
    bool association_permit : 1;
} __attribute__((packed)) superframe_spec_t;

/** This union allows access to the superframe specification field
    as a structure or as an integer */
typedef union
{
    superframe_spec_t superframe_struct;
    u16 superframe_data;
} __attribute__((packed)) superframe_t;

/** Defines the PAN descriptor for a received beacon frame.  The
    beacon parameters are stored in this structure  */
typedef struct
{
    u8 coorAddrMode;             ///< Addressing modes from the beacon FCF
    u16 coorPANId;               ///< PAN ID for the coordinator
    u16  coordAddr;              ///< Short address of the node that sent the beacon
    u8 logicalChannel;           ///< Channel that the PAN operates on
    u8 channelPage;              ///< Channel page used by the PAN
    superframe_t superFrameSpec; ///< Superframe spec of the beacon frame
    u8 hopsToCoord;              ///< Number of hops from the beaconing node to the coordinator
    u8 lqi;                      ///< Link Quality Indicator for the received beacon frame
    u8 rssi;                     ///< Received Signal Strength Indication for the received beacon frame
    u32 timeStamp;
    u8 securityFailure;
    u8 securityLevel;
     u8 keyIdMode;
    u8 *keySource;
    u8 keyIndex;
} __attribute__((packed)) panDescriptor_t;


/** Beacon request frame, sent from new node into the ether */
// Beacon Request, FCF=0x0803, CMD=7
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_BEACONREQ
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network, which is the broadcast PAN ID for this frame
    u16   broadcastAddr; ///< The destination address for the frame, which is the broadcast address for this frame
    u8    cmd;           ///< MAC command byte, see @ref BEACON_REQUEST
} __attribute__((packed)) ftBeaconReq;
/** Beacon frame, sent by coordinator/router in response to a beacon
    request */
// Beacon, FCF=0x8000
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_BEACON
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   addr;          ///< Source address, which is short address of the node sending the beacon
    u16   superFrame;    ///< Superframe specification, see @ref superframe_spec_t
    u8    netID;         ///< 6 for 6LoWPAN, or some other code
    u8    hops;
}  __attribute__((packed)) ftBeacon;
/** Direct association request frame, sent by a new node trying to
    join the nework, to a coordinator/router that sent a beacon
    frame */
// Association request, direct
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_ASSOC_REQ_DIRECT
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   destAddr;      ///< Short address of new parent, who receives this packet
    u64   srcAddr;       ///< Long MAC address of new node, sending this packet
    u8    cmd;           ///< MAC command byte, see @ref ASSOCIATION_REQUEST
    u16   parentAddr;    ///< Short address of new parent (should also be srcAddr)
    u8    type;          ///< Node type of new node, see @ref NODETYPE
} __attribute__((packed)) ftAssocReqDirect;
/** Indirect association request frame, forwarded from a router on up
    to the coordinator */
// Association request, indirect
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_ASSOC_REQ_IND
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   destAddr;      ///< Short address of receiving node
    u16   srcAddr;       ///< Short address of sending node
    u8    cmd;           ///< MAC command byte, see @ref ASSOCIATION_REQUEST
    u16   parentAddr;    ///< Short address of new parent
    u64   macAddr;       ///< Long MAC address of new node
    u8    type;          ///< Node type of new node, see @ref NODETYPE
} __attribute__((packed)) ftAssocReqIndirect;
/** Direct association response frame, sent from the new parent
    (coordinator or router) to the new child node */
// Association response, direct
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_ASSOC_RESP_DIRECT
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u64   dstAddr;       ///< Long MAC address of new node
    u16   srcAddr;       ///< Sender (new parent)'s short address
    u8    cmd;           ///< MAC command byte, see @ref ASSOCIATION_RESPONSE
    u16   shortAddr;     ///< Newly-assigned short address for newly-associated node.
} __attribute__((packed)) ftAssocRespDirect;
/** Indirect association response frame, sent from the coordinator and
    (possibly) forwarded by routers to the new node's parent */
// Association response, indirect
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_ASSOC_RESP_IND
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   dstAddr;       ///< Short address of node receiving this packet
    u16   srcAddr;       ///< Short address of node sending this packet
    u8    cmd;           ///< MAC command byte, see @ref ASSOCIATION_RESPONSE
    u16   parentAddr;    ///< Short address of new node's parent
    u64   macAddr;       ///< Long MAC address of new node
    u16   shortAddr;     ///< Newly-assigned short address for newly-associated node.
} __attribute__((packed)) ftAssocRespIndirect;
/** Routing packet, sent by the coordinator and forwarded down a
    single path of routers */
// Routing packet
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_ROUTE
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   destAddr;      ///< Short address of receiving node
    u16   srcAddr;       ///< Short address of sending node
    u8    cmd;           ///< MAC command byte, see @ref ROUTING_PACKET
    u16   shortAddr;     ///< First of a list of short addresses that describe the route
} __attribute__((packed))ftRouting;
/** Data packet, used for data and some MAC functions (see type
    element) */
// Data packet
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_DATA
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   destAddr;      ///< Short address of receiving node
    u16   srcAddr;       ///< Short address of sending node (next hop)
    u16   finalDestAddr; ///< Short address of the final destination node
    u16   originAddr;    ///< Short address of the originating node
    u8    type;          ///< Type of data packet, see @ref DATA_FRAME
    u8    payload[113];  ///< First byte of an array of application payload
} __attribute__((packed)) ftData;
/** Frame used to notify a router that one of its children has
    re-associated with a new parent, and so the router should drop the
    child from its child table */
// Drop child notification packet
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_DATA
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   destAddr;      ///< Short address of receiving node
    u16   srcAddr;       ///< Short address of sending node (next hop)
    u16   finalDestAddr; ///< Short address of the final destination node
    u16   originAddr;    ///< Short address of the originating node
    u8    type;          ///< Frame type, see @ref DROP_CHILD_FRAME
    u16   childAddr;     ///< Short address of child to drop
} __attribute__((packed)) ftDropChild;
/** Frame used to ping one node from another node.  This frame is used
    for both ping requests and responses.  This frame is routed by
    routers virtually unchanged. */
// Ping packet
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_DATA
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   destAddr;      ///< Short address of receiving node
    u16   srcAddr;       ///< Short address of sending node (next hop)
    u16   finalDestAddr; ///< Short address of the final destination node
    u16   originAddr;    ///< Short address of the originating node
    u8    type;          ///< Frame type, see @ref PING_REQ_FRAME and @ref PING_RSP_FRAME
    u8    rssi;
    u8    lqi;
} __attribute__((packed)) ftPing;
/**   Wake a child node.  This frame is sent from the coordinator to the
   target node's parent.
*/
typedef struct{
    fcf_t fcf;           ///< Frame Control Field, see @ref FCF_DATA
    u8    seq;           ///< Frame sequence number
    u16   panid;         ///< The PAN ID for the network
    u16   destAddr;      ///< Short address of receiving node
    u16   srcAddr;       ///< Short address of sending node (next hop)
    u16   finalDestAddr; ///< Short address of the final destination node
    u16   originAddr;    ///< Short address of the originating node
    u8    type;          ///< Type of data packet, see @ref WAKE_NODE
    u16   addr;          ///< Short address of node to wake up.
} __attribute__((packed)) ftWake;


// MAC configuration stuff

/**
   @brief Structure to hold config information for the MAC and for
   this node.  This includes both config info as well as state info.
*/
typedef struct{
    u64 longAddr;           ///< Long (MAC) address of this node
    bool associated;        ///< Is this node associated with a network?
    uint8_t bsn;            ///< Beacon sequence number, incremented each time a beacon is sent
    u8 dsn;                 ///< Data sequence number, incremented each time a data frame is sent
    u16 panId;              ///< The PAN ID for the network this node is associated with
    u16 shortAddress;       ///< The short address for this node
    u16 parentShortAddress; ///< The short address of the parent's node
    u16 lastRoute;          ///< Last address we sent a routed packet to
    u8 hopsToCoord;         ///< Number of hops from this node to the coordinator
    u8 currentChannel;      ///< The channel number this node is operating on
    u16 lastDestAddr;       ///< Used to process send failures
    u8 busy;                ///< Is the mac busy?  Used to protect @ref mac_buffer_tx
    u8 sleeping;            ///< Is the MAC in sleep mode (1), or awake all the time (0)
} __attribute__((packed)) macConfig_t;

/** @} */ // ingroup mac

// Globals
extern macConfig_t macConfig;
extern u8 mac_buffer_tx[sizeof(rx_frame_t)];  // Main global MAC buffer (transmit)
extern u8 mac_buffer_rx[sizeof(rx_frame_t)];  // Main global MAC buffer (receive)

#define macCopyRxToTx() memcpy(mac_buffer_tx, mac_buffer_rx, sizeof(rx_frame_t))


// Protoypes
void sendBeaconFrame(void);
void macSetOperatingChannel(u8 channel);
void macOtaDebugRequest(u8 *str);
#endif

