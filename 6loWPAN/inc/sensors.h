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
  $Id: sensors.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
*/

#ifndef SENSORS_H
#define SENSORS_H

#include "../inc/mac.h"

/**
   @addtogroup sensors
   @{
   @name Sensor types
   @{
*/
#define SENSOR_RANDOM_T  1  ///< Random temperature for demo.
#define SENSOR_RANDOM_H  2  ///< Random humidity for demo.
#define SENSOR_THERMIST  3  ///< Thermistor sensor.
#define SENSOR_TMP275    4  ///< Digital temperature sensor Burr-Brown TMP275
#define SENSOR_NET       5  ///< Network stats
#define SENSOR_DSK_ACEL  6  ///< DSK001 accelerometer
#define SENSOR_DSK_TEMP  7  ///< DSK001 temp sensor (on-board AVR)
/** @} */
/** @} */

#ifndef SENSOR_TYPE
/**
   Definition of sensor type that this module will be using to collect data.

   @ingroup sensors
*/
#define SENSOR_TYPE       SENSOR_RANDOM_H  // Default is random data for demo.
#endif

// Make sure ADC is off unless we need it
#if SENSOR_TYPE == SENSOR_THERMIST || \
    SENSOR_TYPE == SENSOR_DSK_TEMP || \
    SENSOR_TYPE == SENSOR_DSK_ACEL
#define ADC_ENABLED 1
#else
#define ADC_ENABLED 0
#endif

/**
   @addtogroup sensors
   @{

   @section data_structures Data Structures

   The structures listed above describe the frames sent between a
   sensor node and the coordinator.  The 'sft' prefix stands for
   "sensor frame type".
*/

/**
   @name Sensor frame types
   @{
*/
#define SET_NODE_NAME          2   ///< Frame directing the node to change its name string
#define REQ_READING_FRAME      3   ///< Frame requesting that a node begin sending sensor readings
#define READING_FRAME          4   ///< Frame containing a node's sensor reading
#define CAL_REQ_INFO_FRAME     5   ///< Frame requesting a node's calibration information
#define CAL_INFO_FRAME         6   ///< Frame containing a node's calibration information
#define REQ_RAW_DATA_FRAME     7   ///< Frame requesting a node's raw data (A/D) readings
#define RAW_DATA_FRAME         8   ///< Frame containing a node's raw data (A/D) readings
#define CAL_CMD_FRAME          9   ///< Frame sent from coordinator commanding a node to perform a calibration
/** @} */

/// Length of node name string
#define NAME_LENGTH 8

/// Support for changing all node's interval response time.
#define REPORT_ALL    2

// Support for pining all nodes.
#define PING_ALL      1

/** Sensor reading packet, sent by sensor node to report its data to
    the coordinator.
*/
typedef struct{
    u8    type;          ///< Frame type, see @ref READING_FRAME
    u16   addr;          ///< Short address of node sending reading
    u8    reading[6];    ///< Calculated sensor reading, as an ASCII string
    u8    units[5];      ///< Units of sensor reading, as an ASCII string
    char  name[NAME_LENGTH];  ///< Name of sensor
} __attribute__((packed)) sftSensorReading;

/** Sensor calibration request packet, sent by coordinator to ask
    sensor node to reply with its cal info
*/
typedef struct{
    u8    type;          ///< Frame type, see @ref CAL_REQ_INFO_FRAME and @ref REQ_RAW_DATA_FRAME
} __attribute__((packed)) sftRequest;

/** Sensor data request packet, sent by coordinator to ask
    sensor node to reply with its computed data
*/
typedef struct{
    u8    type;          ///< Frame type, see @ref CAL_REQ_INFO_FRAME and @ref REQ_RAW_DATA_FRAME
    u16   time;          ///< Interval between data frames, in 100mSec intervals.
} __attribute__((packed)) sftRequestData;


/** Sensor calibration info packet, sent by sensor node to report its data to the
    coordinator
*/
typedef struct{
    u8    type;          ///< Frame type, see @ref CAL_INFO_FRAME
    u8    calType;       ///< Calibration type, 1=1point, 2=2point
    u16   addr;          ///< Node address
    u8    units[5];      ///< Units of sensor reading, as an ASCII string
} __attribute__((packed)) sftCalInfo;

extern sftCalInfo coordCalInfo[(NODETYPE == COORD)];

/** Change name of node packet, sent to sensor node */
typedef struct{
    u8    type;          ///< Frame type, see @ref SET_NODE_NAME
    char  name[NAME_LENGTH]; ///< Name of sensor
} __attribute__((packed)) sftSetName;

/** Raw data (A/D reading) from a sensor node.
*/
typedef struct{
    u8    type;          ///< Frame type, see @ref RAW_DATA_FRAME
    u16   reading;       ///< Current A/D reading
} __attribute__((packed)) sftRawData;

/** Calibration command frame, sent from the coordinator to a
    end/router node, commanding the node to perform a calibration.
    This packet contains the data required for the calibration.
*/
typedef struct{
    u8    type;           ///< Frame type, see @ref CAL_CMD_FRAME
    u8    index;          ///< Which reading is this - first or second (0 or 1)?
    u8    reading[8];     ///< The user-entered cal reading, as asciiz string
} __attribute__((packed)) sftCalCommand;

/**
   Calibration data, stored in EEPROM.  To calculate measured data,
   the following formula is used:

   @code
   reading = m * ADC + b
   @endcode
*/
typedef struct{
    double m;     ///< The slope of the reading versus ADC input
    double b;     ///< The offset of the reading
} __attribute__((packed)) tCalFactors;


void allNodes(u8 func, u16 val);
extern u16 node_addr;
extern char node_name[NAME_LENGTH*(NODETYPE==COORD)];

void sensorInit(void);
void sensorAutoSendStart(void);
void sensorRcvPacket(u8 *frame);
void sensorRequestReading(u16 addr, u16 time);
void sensorRequestCalInfo(u16 addr);
void sensorRequestRawData(u16 addr);
void sensorSendSetNodeName(u16 addr, char *name);
void sensorSendCalPoint(u8 index, char *str);
u8 sensorCalBusy(void);
char *sensorGetName(void);
void sensorSendSetNodeName(u16 addr, char *name);
void sensorPacketSendSucceed(void);
void sensorPacketSendFailed(void);
void sensorLostNetwork(void);

void sixlowpan_sensorPerSend(u8 len, u8 * data);
void sixlowpan_sensorSend(u16 addr, u8 len, u8 * data);
void sixlowpan_sensorReturn(u8 len, u8 * data);
void sendReading(void);

/** @} */

#endif
