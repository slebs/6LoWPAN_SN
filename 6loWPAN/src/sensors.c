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
  $Id: sensors.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "../inc/radio.h"
#include "../inc/mac.h"
#include "../inc/mac_event.h"
#include "../inc/mac_associate.h"
#include "../inc/system.h"
#include "../inc/sensors.h"
#include "../inc/mac_data.h"
#include "../inc/sleep.h"
#include "../inc/avr_timer.h"


#if (__AVR__)
#include "../inc/serial.h"
#define INTERACTIVE 1
#include <util/twi.h>

#if IPV6LOWPAN
#include "../inc/avr_sixlowpan.h"
#endif

/*
#else // __arm__
#define INTERACTIVE 0
#include "../inc/arm_app.h"
#include "../inc/hal_arm.h"
*/
#endif

/**
   @addtogroup sensors
   @{

   This code demonstrates a simple wireless sensor network, where data
   collected by end nodes and routers is sent to the coordinator.

   This application cooperates with the default @ref app, and can be
   left out of the compilation if desired by setting the @ref APP flag
   to zero.

   For demonstration, it is possible to have sensor nodes send random
   data, which saves the effort of connecting real sensors to each
   node.  To use this "random data" mode, set @ref SENSOR_TYPE to @ref
   SENSOR_RANDOM_T or @ref SENSOR_RANDOM_H.

   @section sleeping Sleeping and waking

   The sensor application causes nodes to sleep and wake.  The idea is
   that an end node can sleep for some interval while it is waiting to
   send a reading.  When it is time to send a reading, the unit wakes
   up, captures a reading, transmits the reading to its parent, and
   then goes back to sleep.

   It is necessary to provide a way to wake up the node, so that we
   can change the data interval or to calibrate the node.

   The mechanism for doing this is to use the @ref ftWake frame as a
   signal to wake up.  When an end node is done tranmitting its data
   frame, it remains awake for a short period (about 50mSec) to be
   able to receive a wakeup frame from its parent.  If no frame is
   received in that time, a timer expires and the node goes back to
   sleep.  If a ftWake frame is received, then the node stays awake
   and stop sending frames.  The node can now be calibrated or a new
   data interval can be set by sending the node an @ref sftRequestData
   frame.

   @section sensorcalls Sensor application function calls

   Here is a listing of how the various sensor calls are made to
   implement the various sensor functions.

   @verbatim
 Coordinator                 Frame type over the air             Router/End node

                                   Calibration

 sensorRequestCalInfo()    ---> CAL_REQ_INFO_FRAME --->        sensorReplyCalInfo()
 sensorRcvCalInfo()        <---   CAL_INFO_FRAME   <---
 (Save cal info)
 sensorSendCalPoint()      --->    CAL_CMD_FRAME   --->        sensorRcvCalCommand()
                                                                (calibrate sensor)
 sensorSendCalPoint()      --->    CAL_CMD_FRAME   --->        sensorRcvCalCommand()
  (if 2-point cal)                                              (calibrate sensor)


                                    Read Data

 sensorRequestReading()    --->  REQ_READING_FRAME --->        sensorSendReading()
 sensorRcvReading()        <---    READING_FRAME   <---        sendReading()
  (do something with reading)

                                    Node Name

 sensorSendSetNodeName()   --->    SET_NODE_NAME   --->        sensorSetNodeNameRcv()

                                                                (set node name)
 (to retrieve the node name,
 just get a data packet, which
 contains the node name.)
   @endverbatim

   @section contdata Sending data continuously

   When the sensorRequestReading() function is called on the
   coordinator, the sensor node is directed to report data
   periodically back to the coordinator.  The @e time argument to this
   function sets the interval between readings, in tenths of a
   second.  This value is stored in EEPROM, and the unit will resume sending data after a power cycle.

*/
#if defined(DOXYGEN)
#define APP SENSOR
#endif


/// Data interval setting.  This is the time between frames in 1/10's of a second.
u16 frameInterval;
u16 appInterval;

// Enable this file only if we are compiling in the sensor app.
#if (APP == SENSOR)

/// Calibration flag, only compiles calibration code in if the flag is set.
#define CAL 1

// What kind of node are we going to be?
#define CAL_POINTS 1  // 1-point cal

#define RETRY_WAIT_PERIOD 50   // Time (mS) to wait after a packet failed to try again.

static tCalFactors calFactors[(NODETYPE != COORD)];
#define  saveCalFactors() halPutEeprom((u8*)offsetof(tEepromContents, calFactors), \
                                       sizeof(tCalFactors), (u8*)calFactors)
#define  getCalFactors()  halGetEeprom((u8*)offsetof(tEepromContents, calFactors), \
                                       sizeof(tCalFactors), (u8*)calFactors)
#define  getInterval()    halGetEeprom((u8*)offsetof(tEepromContents, dataSeconds), \
                                       sizeof(typeof(((tEepromContents*)0)->dataSeconds)), \
                                       (u8*)&frameInterval)
#define  saveInterval()   halPutEeprom((u8*)offsetof(tEepromContents, dataSeconds), \
                                       sizeof(typeof(((tEepromContents*)0)->dataSeconds)), \
                                       (u8*)&frameInterval)

void twiInit(void);
void twiWrite(u8 addr, u8 data);
u16 twiRead(void);

// Stuff for the RANDOM fake data
double fakeData[SENSOR_TYPE == SENSOR_RANDOM_T || SENSOR_TYPE == SENSOR_RANDOM_H];



#if (NODETYPE == COORD)
u16 node_addr;
#endif
char node_name[NAME_LENGTH*(NODETYPE == COORD)];

struct{
    char name[8];  // Node name
} __attribute__((packed)) sensorInfo[(NODETYPE != COORD)];

// Save cal data (sensor node), used to interactively build the info
// over time by getting data back from coordinator
typedef struct{
    double reading[2];  // Entered readings (from coord)
    u16 ad[2];          // A/D readings from sensor (matching
} __attribute__((packed)) tCalData;

// Cal info that coord has to save to do a cal
sftCalInfo coordCalInfo[(NODETYPE == COORD)];

static tCalData calData[(NODETYPE != COORD)];

/// Is the cal process running?  This flad is used to suppress user menus during calibration.
static u8 busyCal;

static u8 sensorSentReading; // flag - have we sent a reading and are waiting for a response?
static u8 sendReadingTimer;

const double tempTable[16] = {
    -15.93,
    6.8,
    22.34,
    34.97,
    46.16,
    56.6,
    66.77,
    77,
    87.64,
    99.07,
    111.85,
    126.84,
    145.65,
    172.08,
    219.2,
    692.72,
};


/**
   Calculates a sensor reading, or returns a fake sensor reading if
   @ref SENSOR_TYPE is set to SENSOR_RANDOM.
*/
static double sensorGetReading(void)
{
    if (NODETYPE != COORD)
    {
        s8 incr;
        u8 x;
        u16 adVal;
        double retval,val;

        if (ADC_ENABLED)
            HAL_SAMPLE_ADC();

        switch (SENSOR_TYPE)
        {
        case SENSOR_NET:
            // RSSI
            return (double) radioGetSavedRssiValue();
        case SENSOR_RANDOM_T:
            incr = (s8)radioRandom(8);
            *fakeData += (double)incr / 1000.0;
            return *fakeData;
        case SENSOR_RANDOM_H:
            incr = (s8)radioRandom(8);
            *fakeData += (double)incr / 5000.0;
            return *fakeData;
        case SENSOR_THERMIST:
            // Wait for ADC reading
            HAL_WAIT_ADC();

            adVal = 50;

            // look up table value
            x = adVal >> 6;
            x &= 0x0f;
            if (x)
            {
                retval = tempTable[x-1];
                // interpolate (val = span)
                val = tempTable[x] - retval;
                val *= adVal & 0x3f;
                val /= 0x40;
                retval += val;
            }
            else
                retval = 0;

            // Add cal offset
            retval += calFactors->b;
            return retval;
        case SENSOR_TMP275:
            retval = (double)(((s16)twiRead()));
            retval /= 256;
            retval *= 9;
            retval /= 5;
            retval += 32;
            return retval;
        case SENSOR_DSK_TEMP:
            // On-board AVR sensor
            // Read the ADC
/*             HAL_SAMPLE_ADC(); */
/*             HAL_WAIT_ADC(); */
            retval = (double)50 * (9/5)/0.931;
            HAL_STOP_ADC();

            // Add cal offset
            retval += calFactors->b;
            return retval;
        default:
            return 0;
        }
    }

    // Turn off the ADC so we can go to sleep.
    if (ADC_ENABLED)
        HAL_STOP_ADC();
}


/**
   Returns a pointer to the name of this node.

   @return a pointer to the name of this node.
*/
char *sensorGetName(void)
{
    return sensorInfo->name;
}

/**
   Initialize the sensor system by configuring the A/D converter.
*/
void sensorInit(void)
{
    if (NODETYPE != COORD)
    {
        // Get frame interval from  EEPROM
        getInterval();
        if (frameInterval == 0xffff)
            frameInterval = 0;

        switch (SENSOR_TYPE)
        {
        case SENSOR_RANDOM_T:
            // Just send faked random data for demo
            radioSetTrxState(RX_AACK_ON); // Needed for good random number
            *fakeData = 70+ (((s8)radioRandom(8))/100.0);
        case SENSOR_RANDOM_H:
            // Just send faked random data for demo
            radioSetTrxState(RX_AACK_ON); // Needed for good random number
            *fakeData = 32 + (((s8)radioRandom(8))/100.0);
            break;
        case SENSOR_THERMIST:
            // Read real ADC data
            HAL_INIT_ADC();
            // First sample takes longer
            HAL_SAMPLE_ADC();

            // Retrieve cal factors from EEPROM
            getCalFactors();

            // See if the cal factors from EEPROM are good
            if (isnan(calFactors->b))
            {
                calFactors->b = 0;
            }
            break;
        case SENSOR_TMP275:
            // TMP275, init the TWI interface
            twiInit();
            break;
        case SENSOR_DSK_TEMP:
            // On-board AVR temp sensor
            HAL_INIT_ADC();
            // First sample takes longer
            HAL_SAMPLE_ADC();
            // Retrieve cal factors from EEPROM
            getCalFactors();

            // See if the cal factors from EEPROM are good
            if (isnan(calFactors->b))
            {
                calFactors->b = 0;
            }
            break;
        default:
            break;
        }
    }
}

/**
   Begin auto-sending data after powerup.  The data is only sent if
   the node previously received a frame of type @ref REQ_READING_FRAME.
*/
void sensorAutoSendStart(void)
{
    if (NODETYPE != COORD)
    {
        if (frameInterval)
            macTimerEnd(sendReadingTimer);
        sendReadingTimer = macSetAlarm(100, sendReading);
    }
}

/**
   Returns true if the node is engaged in a calibration process.  This
   value is used to suppress the main menu operation during the calibration process.

   @return true  if the node is engaged (busy) with a calibration
   @return false if the node is not busy
*/
u8 sensorCalBusy(void)
{
    return busyCal;
}

/**
   @name Coordinator sensor functions
   @{
*/

/**
   Request a reading from a sensor node

   @param addr Short address of node being requested to send data

   @param time Time to wait between readings, in 100mSec intervals. If
   time is zero, then only one reading will be sent.
*/
void sensorRequestReading(u16 addr, u16 time)
{
    if (NODETYPE == COORD)
    {
        sftRequestData req;
        req.type = REQ_READING_FRAME;
        req.time = time;

        if (IPV6LOWPAN)
            sixlowpan_sensorSend(addr, sizeof(sftRequestData), (u8*)&req);
        else
            macDataRequest(addr, sizeof(sftRequestData), (u8*)&req);
    }
}

/**
   Request an end node to send its calibration information.  When the
   node responds with the info, @ref sensorRcvCalInfo() is called.

   @param addr Short address of node
*/
void sensorRequestCalInfo(u16 addr)
{
    if (NODETYPE == COORD)
    {
        sftRequest req;

        req.type = CAL_REQ_INFO_FRAME;

        // Clear the cal info struct
        coordCalInfo->type = 0;

#if IPV6LOWPAN
        sixlowpan_sensorSend(addr, sizeof(sftRequestData), (u8*)&req);
#else
        macDataRequest(addr, sizeof(sftRequestData), (u8*)&req);
#endif
    }
}

void sensorRequestRawData(u16 addr)
{
    if (NODETYPE == COORD)
    {
        sftRequest req;
        req.type = REQ_RAW_DATA_FRAME;

#if IPV6LOWPAN
        sixlowpan_sensorSend(addr, sizeof(sftRequestData), (u8*)&req);
#else
        macDataRequest(addr, sizeof(sftRequestData), (u8*)&req);
#endif
    }
}

/**
   Set the name of the sensor name.

   @param addr Short address of the node to be named.
   @param name String that contains the new name.  The max length is
   @ref NAME_LENGTH bytes.
*/
void sensorSendSetNodeName(u16 addr, char *name)
{
    if (NODETYPE == COORD)
    {
        sftSetName frame;

        frame.type = SET_NODE_NAME;
        strncpy(frame.name, name, NAME_LENGTH);

        // Send frame
#if IPV6LOWPAN
        sixlowpan_sensorSend(addr, sizeof(sftSetName), (u8*)&frame);
#else
        macDataRequest(addr, sizeof(sftSetName), (u8*)&frame);
#endif
    }
}

static void sensorRcvReading(sftSensorReading *reading)
{
    if (NODETYPE == COORD)
    {
        if (DEBUG)
        {
            u8 str[10];
#if (__AVR__)
            debugMsgStr("\r\nReading from node 0x");
            debugMsgHex(reading->addr);
            debugMsgStr(" = ");
            strncpy((char*)str, (char*)reading->reading, 6);
            str[6] = 0;
            debugMsgStr((char *)str);
            debugMsgChr(' ');
            debugMsgStr((char *)reading->units);
            debugMsgStr(", RSSI=");
            debugMsgInt(radioGetSavedRssiValue());
            debugMsgStr(", LQI=");
            debugMsgInt(radioGetSavedLqiValue());
#else // __arm__
            if(telPrintReading)
            {
//                UNCOMMENT THIS TO SEE NAME IN OUTPUT SCREEN!!!
//                if(reading->name[0] != 0x00)
//                {
//                    fnDebugMsg("Reading from ");
//                    debugMsgStr((char *)reading->name);
//                }
//                else
                {
                    fnDebugMsg("Reading from node ");
                    debugMsgInt(reading->addr);
                }
                debugMsgStr(" = ");
                strncpy((char*)str, (char*)reading->reading, 6);
                str[6] = 0;
                debugMsgStr((char *)str);
                debugMsgChr(' ');
                debugMsgStr((char *)reading->units);
                fnDebugMsg("\r\n");
            }
#endif // __AVR__

        }
/*
#if(__arm__)
        // Store this nodes params for application use.
        node_addr = reading->addr;
        strcpy(node_name, reading->name);

        armAppRcvData(reading);
#endif // __arm__
*/
    }
}

// Index is 0 for first point, 1 for second point
void sensorSendCalPoint(u8 index, char *str)
{
    if (NODETYPE == COORD)
    {
        sftCalCommand calCmd = {
            .type = CAL_CMD_FRAME,
            .index = index};
        strncpy((char*)calCmd.reading, str, 6);
        // Send packet to sensor node
#if IPV6LOWPAN
        sixlowpan_sensorSend(coordCalInfo->addr, sizeof(sftCalCommand), (u8*)&calCmd);
#else
        macDataRequest(coordCalInfo->addr, sizeof(sftCalCommand), (u8*)&calCmd);
#endif
       debugMsgStr("\r\nSent cal data to sensor");
    }
}

#if __AVR__
/**
   Coord node calibration routine. This must be called repetitively
   because the sensor node must be queried constantly to get current
   A/D readings.
*/
static void sensorCalProcess(void)
{
    if (NODETYPE == COORD && DEBUG && SERIAL)
    {
        // Save state of where we're at
        static enum {start, read1, read2} progress;
        char str[12];

        switch (progress)
        {
        case start:
            // Print out intro, start getting A/D readings
            debugMsgStr("\r\nCal node, enter applied reading A [");
            debugMsgStr((char *)coordCalInfo->units);
            debugMsgStr("]:");
            progress++;
            busyCal = true;
            break;
        case read1:
            // Don't block unless we have at least one serial char
            if (serial_ischar())
            {
                // Get reading, send to sensor node
                serial_gets(str,10,true);
                // Send frame to sensor node with cal data
                sensorSendCalPoint(0, str);

                progress++;

                // Output stuff for next reading
                if (coordCalInfo->calType == 2)
                {
                    debugMsgStr("\r\nCal node, enter applied reading B [");
                    debugMsgStr((char *)coordCalInfo->units);
                    debugMsgStr("]:");
                }
            }
            break;
        case read2:
            // Get reading two, if there is one
            if (coordCalInfo->calType == 2)
            {
                // Need a second reading
                if (serial_ischar())
                {
                    // Get reading, send to sensor node
                    serial_gets(str,10,true);
                    progress++;  // Default will Start over

                    // Send frame to sensor node with cal data
                    sensorSendCalPoint(1, str);
                }
            }
            else
                // Don't bother with second reading
                progress++;
            break;
        default:
            // Quit, don't call this function again
            // End the cal process, reset progress for next time.
            busyCal = false;
            progress = start;
            // Return so that we don't re-call this function
            return;
        }
        // Thank you, come again!
        macSetAlarm(50, sensorCalProcess);
    }
}
#endif // __AVR__

/**
   Callback function to receive calibration information from a sensor
   node.

   @param calInfo Pointer to tCalInfo structure that contains the node's
   calibration information.
*/
static void sensorRcvCalInfo(sftCalInfo *calInfo)
{
    if (NODETYPE == COORD)
    {
        // Cal info received, perform a calibration.
        *coordCalInfo = *calInfo;
        if (DEBUG)
            // Start the callback function going
            if (INTERACTIVE)
            {
//#if __AVR__
                sensorCalProcess();
//#else // __arm__
//                fnHandleTelnetInput(0, 0, 0);
//#endif
            }
    }
}

static void sensorRcvRawData(sftRawData *rawData)
{
    if (NODETYPE == COORD)
    {
        // Do what you want with the raw data
    }
}

/** @} */
/**
   @name Router/End node sensor functions
   @{
*/

/**
   Sensor node sends raw data back to coordinator
*/
static void sensorReplyRawData(sftRequest *req)
{
    if (NODETYPE != COORD && CAL)
    {
        // Send data back
        sftRawData rawData;
        rawData.type = RAW_DATA_FRAME;
        rawData.reading = 0x8000 + macConfig.dsn * 300;
        // send it
#if IPV6LOWPAN
        sixlowpan_sensorReturn(sizeof(sftRawData), (u8*)&rawData);
#else
        macDataRequest(DEFAULT_COORD_ADDR, sizeof(sftRawData), (u8*)&rawData);
#endif

    }
}

static void sensorReplyCalInfo(sftRequest *req)
{
    if (NODETYPE != COORD && CAL)
    {
        // Return tCalInfo packet
        sftCalInfo calInfo = {
            .type = CAL_INFO_FRAME,
            .calType = CAL_POINTS,
            .addr = macConfig.shortAddress,
            .units = "degF"};

        if (SENSOR_TYPE == SENSOR_RANDOM_H)
            strcpy((char *)calInfo.units, "%RH");

#if IPV6LOWPAN
        sixlowpan_sensorReturn(sizeof(sftCalInfo), (u8*)&calInfo);
#else
        macDataRequest(DEFAULT_COORD_ADDR, sizeof(sftCalInfo), (u8*)&calInfo);
#endif
    }
}

static void sensorRcvCalCommand(sftCalCommand *calCommand)
{
    if (NODETYPE != COORD && CAL)
    {
        // Received a cal command, save away readings, calc when done
        u8 index = calCommand->index;

        // Begin A/D reading
        if (ADC_ENABLED)
            HAL_SAMPLE_ADC();

        // Save user-entered reading
        calData->reading[index] = atof((char*)calCommand->reading);

        // Finish A/D reading
        if (ADC_ENABLED)
        {
            HAL_WAIT_ADC();
            calData->ad[index] = 50;
        }

        // If we have one (single-point) or both readings, do the cal
        if (CAL_POINTS == 1)
        {
            // Wipe out cal before we get a current reading, so that
            // old cal doesn't get used by sensorGetReading()
            calFactors->b = 0;
            // Single-point temperature cal, save the offset
            calFactors->b = calData->reading[index] - sensorGetReading();
            // Save new cal factors to EEPROM
            saveCalFactors();
            if (SENSOR_TYPE == SENSOR_RANDOM_T || SENSOR_TYPE == SENSOR_RANDOM_H)
                // just reset the state variable
                *fakeData = calData->reading[index];
        }
        if (index)
        {
            // Second of two points, calculate calibration parameters
            double m, b;
            m = calData->reading[0] - calData->reading[1];
            b = (double)calData->ad[0];
            b -= (double)calData->ad[1];
            m /= b;
            b = calData->reading[0];
            b -= (m * (double)calData->ad[0]);

            debugMsgStr("\r\nA1=");
            debugMsgHex(calData->ad[0]);
            debugMsgCrLf();
            debugMsgStr("A2=");
            debugMsgHex(calData->ad[1]);
            debugMsgCrLf();
            debugMsgStr("r1=");
            debugMsgFlt(calData->reading[0]);
            debugMsgCrLf();
            debugMsgStr("r2=");
            debugMsgFlt(calData->reading[1]);
            debugMsgCrLf();

            // Store data in cal factors struct
            calFactors->m = m;
            calFactors->b = b;
            // Save new cal factors to EEPROM
            saveCalFactors();
        }
    }
}

static void sensorSleep(void)
{
    /* IPv6LOWPAN Has it's own sleep routines we use... */
#if IPV6LOWPAN
    if (NODETYPE == ENDDEVICE)
    {
        getInterval();
    }

#else
    // Only end nodes sleep
    if(NODETYPE == ENDDEVICE)
    {
        // go to sleep, then wakeup and send again
        getInterval();  // Just to be sure
        u8 sleepTime = frameInterval;
        if (!sleepTime)
            sleepTime = 1;

        // Don't sleep if we've been woken up.
        if (!macConfig.sleeping)
            return;

        nodeSleep(sleepTime);
        // Ah, that was a good nap, now send a packet and go back to sleep
        sendReading();
    }
#endif
}

/**
   Send a single reading to the coordinator and go to sleep if
   required.
*/
void sendReading(void)
{
    if ((NODETYPE != COORD) && (macConfig.associated))
    {
        // Create a data packet
        s8 str[20];

        LED_ON(1);

        // First, start the ADC reading.
        if (ADC_ENABLED)
            HAL_SAMPLE_ADC();

        sftSensorReading reading = {
            .type = READING_FRAME,
            .addr = macConfig.shortAddress,
            .units = "degF" };

        if (SENSOR_TYPE == SENSOR_RANDOM_H)
            strcpy((char *)reading.units, "%RH");

        if (SENSOR_TYPE == SENSOR_NET)
            sprintf((char*)reading.units,"%d", radioGetSavedLqiValue());

        // Add the name to the packet
        strncpy(reading.name, sensorInfo->name, NAME_LENGTH);

        // Get a sensor reading
        double val = sensorGetReading();
        sprintf((char *)str,"%f",val);
        strncpy((void *)reading.reading, (void *)str, 6);

        // Send it off
#if IPV6LOWPAN
        //If we have a report time of zero, don't actually send...
        if(frameInterval)
        {
            //We flag this data as possibly being sent to a remote address too
            sixlowpan_sensorPerSend(sizeof(sftSensorReading), (u8*)&reading);
        }
#else
        macDataRequest(DEFAULT_COORD_ADDR, sizeof(sftSensorReading), (u8*)&reading);
#endif

        sensorSentReading = true;
    }
}

/**
   This node has successfully sent a packet, and this function is
   called when that happens.
*/
void sensorPacketSendSucceed(void)
{
    LED_OFF(1);

    //IPv6LOWPAN Handles this itself
#if IPV6LOWPAN
    return;
#endif

    // Only handle this event if we are really waiting for a packet send result.
    if (!sensorSentReading)
        return;

    sensorSentReading = false;

    // If we have a data time interval, then wait a while and re-send data
    if (!frameInterval)
        return;

    if (!macConfig.sleeping)
        return;

    // Sleep or wait for the interval, then send again
    if (RUMSLEEP && NODETYPE == ENDDEVICE)
    {
        // Wait for the frame to get out, then go to sleep
        // Also must wait to receive and process a packet from
        // parent.
        macSetAlarm(50 + 150*(BAND == BAND900), sensorSleep);
    }
    else
    {
        macTimerEnd(sendReadingTimer);

        // no sleep, just wait for the timeout and send again
        if (frameInterval <= 650)
            // Less than 6.5 seconds
            sendReadingTimer = macSetAlarm(frameInterval * 100, sendReading);
        else
            // One or more seconds
            sendReadingTimer = macSetLongAlarm((frameInterval+5)/10, sendReading);
    }
}

/**
   This node has failed to send a packet.
*/
void sensorPacketSendFailed(void)
{
    LED_OFF(1);

    //IPv6LOWPAN Handles this itself
#if IPV6LOWPAN
    return;
#endif

    // Only handle this event if we are really waiting for a packet send result.
    if (!sensorSentReading)
        return;

    sensorSentReading = 0;

    // Wait a bit and try again.
    macTimerEnd(sendReadingTimer);
    sendReadingTimer = macSetAlarm(RETRY_WAIT_PERIOD, sendReading);
}

/**
   This node has lost its network connection.
*/
void sensorLostNetwork(void)
{
    LED_OFF(1);
    sensorSentReading = 0;
    // Do nothing more.  The node will try to re-associate, and if
    // it does, it will start sending data again.
}

/**
   @brief Send a sensor reading to coordinator.  Also begin timer to
   continuously send frames.
*/
static void sensorSendReading(sftRequestData *req)
{
    if (NODETYPE != COORD)
    {
        // setup sending of frames
        frameInterval = req->time;

        // For very low power nodes, don't set interval to < 1sec
        if (VLP && frameInterval < 10)
            frameInterval = 10;

        saveInterval();

        // Enable sleep mode so we will sleep between sending data frames.
        if (frameInterval)
        {
            macConfig.sleeping = true;
            debugMsgStr("\r\nSleeping interval = ");
            debugMsgInt(frameInterval);
            debugMsgCrLf();
        }

        // And start the sending process.
        macTimerEnd(sendReadingTimer);
        sendReadingTimer = macSetAlarm(10, sendReading);
    }
}
/** @} */

/**
   Set the name of the sensor name.

   @param name String that contains the new name.  The max length is
   @ref NAME_LENGTH bytes.
*/
static void sensorSetNodeNameRcv(char *name)
{
    if (NODETYPE != COORD)
    {
        // Sensor node, set the name
        strncpy(sensorInfo->name, name, NAME_LENGTH);
    }
}


/**
   @brief Handle a received packet.  The packet is a data packet
   addressed to this node.  This function applies to the coordinator
   and router/end nodes.

   @param frame Pointer to the frame payload.  See @ref ftData.
*/
void sensorRcvPacket(u8 *frame)
{
    if (NODETYPE == COORD)
    {
        // Sensor frame type is always first byte of frame
        switch (((sftRequest*)frame)->type)
        {
        case READING_FRAME:
            // Frame containing a node's sensor reading
            sensorRcvReading((sftSensorReading *)frame);
            break;
        case CAL_INFO_FRAME:
            // Frame containing a node's calibration information
            sensorRcvCalInfo((sftCalInfo*)frame);
            break;
        case RAW_DATA_FRAME:
            // Frame containing a node's raw data (A/D) readings
            sensorRcvRawData((sftRawData*)frame);
            break;
        default:
            break;
        }
    }
    else // Routers and end nodes
    {
        // Sensor frame type is always first byte of frame
        switch (((sftRequest*)frame)->type)
        {
        case REQ_READING_FRAME:
            // Frame containing a node's sensor reading
            sensorSendReading((sftRequestData *)frame);
            break;
        case CAL_REQ_INFO_FRAME:
            // Frame containing a node's calibration information
            sensorReplyCalInfo((sftRequest*)frame);
            break;
        case REQ_RAW_DATA_FRAME:
            // Request a frame containing a node's raw data (A/D) readings
            sensorReplyRawData((sftRequest*)frame);
            break;
        case CAL_CMD_FRAME:
            // Frame sent from coordinator commanding a node to perform a calibration
            sensorRcvCalCommand((sftCalCommand*)frame);
            break;
        case SET_NODE_NAME:
            sensorSetNodeNameRcv(((sftSetName *)frame)->name);
            break;
        default:
            break;
        }
    }
}

// TWI functions
#if (PLATFORM == SPITFIRE)
void twiInit(void)
{
    // Setup the TWI registers to enable the TWI interface
    TWBR = 0xff;     // pretty slow
    TWCR = (1<<TWINT);

    // write config bits
    twiWrite(1, 0x60);

    // write to temp register so that later reads will read temp
    twiWrite(0,0);
}

void twiWrite(u8 addr, u8 data)
{
    // write one byte of data to addr
    volatile u8 twsr;
    u8 count=0;

 twistart1:
    if (++count > 10)
        goto quit1;

    TWSR = 0;
    // first, send a start condition
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);// 0xa4;
    // wait for the start to happen
    while (!(TWCR & (1 << TWINT)))
        ;
    // Now check the status
    twsr = TW_STATUS;
    if (twsr != TW_START && twsr != TW_REP_START)
        goto twistart1;
    // Send slave address plus W bit:
    TWDR = 0x90;
    // Clear TWINT flag
    TWCR = (1<<TWINT)|(1<<TWEN);
    // wait for the transfer to finish
    while (!(TWCR & 1 << TWINT))
        ;
    // Check to see if slave acknowledged the address
    twsr = TW_STATUS;
    if (twsr == TW_MT_SLA_NACK)
        goto twistart1;
    if (twsr != TW_MT_SLA_ACK)
        goto quit1;
    // Send the address (TMP275 pointer register)
    TWDR = addr;
    // Clear TWINT flag
    TWCR = (1<<TWINT)|(1<<TWEN);
    // wait for the transfer to finish
    while (!(TWCR & 1 << TWINT))
        ;
    // Check to see if slave acknowledged the transfer
    if (TW_STATUS != TW_MT_DATA_ACK)
        goto quit1;
    // Send the data
    TWDR = data;
    // Clear TWINT flag
    TWCR = (1<<TWINT)|(1<<TWEN);
    // wait for the transfer to finish
    while (!(TWCR & 1 << TWINT))
        ;
 quit1:
    // Stop the TWI transfer
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

u16 twiRead(void)
{
    union {
        u8 bytes[2];
        u16 word;
    } tempData = { .word = 0 };
    volatile u8 twsr;
    u8 count=0;

 twistart2:
    if (++count > 10)
        goto quit2;

    //    TWCR = 0;

    // first, send a start condition
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    // wait for the start to happen
    while (!(TWCR & (1 << TWINT)))
        ;

    // Now check the status
    twsr = TW_STATUS;
    if (twsr != TW_START && twsr != TW_REP_START)
        goto twistart2;
    // Send slave address plus W bit:
    TWDR = 0x91;
    // Clear TWINT flag
    TWCR = (1<<TWINT)|(1<<TWEN);
    // wait for the transfer to finish
    while (!(TWCR & 1 << TWINT))
        ;

    // Check to see if slave acknowledged the address
    twsr = TW_STATUS;
    if (twsr == TW_MR_SLA_NACK)
        goto twistart2;
    if (twsr != TW_MR_SLA_ACK)
        goto quit2;

    // Command the slave to send the first byte of data
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    // wait for the transfer to finish
    while (!(TWCR & 1 << TWINT))
        ;
    // Read the data byte
    tempData.bytes[1] = TWDR;

    // Command the slave to send the second byte of data
    TWCR = (1<<TWINT)|(1<<TWEN);
    // Wait to receive data
    while (!(TWCR & 1 << TWINT))
        ;
    // Read the data byte
    tempData.bytes[0] = TWDR;

 quit2:
    // Stop the TWI transfer
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);

    // Return the data read
    return tempData.word;
}
#else  // PLATFORM == SPITFIRE
// Functions compiled out, make dummy definitions
#define twiInit()
#define twiWrite(a,b)
#endif // PLATFORM == SPITFIRE

#else // APP == SENSOR

void twiInit(void) {;}
void twiWrite(u8 addr, u8 data) {;}
char *sensorGetName(void) {return NULL;}
void sensorSendSetNodeName(u16 addr, char *name) {}

#endif // APP == SENSOR




/** @} */

