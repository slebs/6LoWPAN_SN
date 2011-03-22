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
  $Id: mac_scan.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 */

// Includes
#include <string.h>
#include <stdio.h>
#include "../inc/radio.h"
#include "../inc/mac.h"
#include "../inc/mac_scan.h"
#include "../inc/system.h"
#include "../inc/sleep.h"
#include "../inc/avr_timer.h"

#include "../inc/mac_event.h"
#include "../inc/deRFaddon/bmm.h"
#include "../inc/deRFaddon/uart.h"

/**
   @addtogroup mac
   @{
   @defgroup mac_scan MAC scanning system
   @{

   This module provides functions that scan the available channels.
   End nodes and routers scan to look for avaiable networks, while
   coordinator nodes scan to look for an empty channel to use.

   The coordinator node uses just one channel to establish a network,
   and all nodes on the network use that same channel.  The channel
   can be either a fixed channel @ref PAN_CHANNEL (by passing
   PAN_CHANNEL=13, the PAN will operate on channel 13), or the
   coordinator can choose the quietest free channel (by leaving
   PAN_CHANNEL unset.

   The router/end nodes scan all available channels, or, if @ref
   PAN_CHANNEL is set, only @ref PAN_CHANNEL will be scanned
   for network nodes.

   Scanning involves the following steps for each channel scanned.
   This is for an end node or router.

   -# Set the radio to the channel being scanned.
   -# Send a beacon request.
   -# Set a timeout timer for @ref SCANDURATION.
   -# If a beacon is received before the timer expires, save the
     relevent information about the node sending the beacon.
   -# When the timer expires, move on to the next channel.

   The coordinator performs a channel scan, not to look for available
   networks, but to avoid them, and to find the least-noisy channel.
   The coordinator scan measures the noise level on each channel, so
   there is an array of energy measurements, one for each channel.
   Here are the coordinator scan steps:

   -# Set the radio to the channel being scanned.
   -# Send a beacon request.
   -# Set a timeout timer for @ref SCANDURATION.
   -# If a beacon is received before the timer expires, add 500 noise
      points to the energy measurement for the current channel.
   -# Measure energy as many times as possible in the @ref
      SCANDURATION window.
   -# When all channels have been scanned, determine which channels
      have the minimum noise.  If more than one channel has minimal
      noise, randomly choose one of these channels to use for the
      network.
 */

//Globals


/// PAN Descriptor structure,
panDescriptor_t panDescriptor;

/// Flag to prevent re-entrancy in scanning.
static bool scanInProcess;
#define ALL_CHANNELS (0x80)
static u8 scanChannel=ALL_CHANNELS;

/**
   Structure used to keep track of how much energy is received on each
   scanned channel.  This is only used during the coordinator's energy scan.
   @see macFindClearChannel().
 */
typedef struct{
   u8 currentChannel;    ///< Channel that we're currently scanning
   u8 timerID;           ///< ID of channel scan timer, return value of @ref macSetAlarm
   u8 randomVal;         ///< Random value, used for choosing a channel
   u16 energy[MAX_CHANNEL+1]; ///< Accumulated energy value, one for each channel
} __attribute__((packed)) energy_t;

/// Energy accumulation structure
static energy_t energy;

/// Duration of scanning for each channel, in mSec
#define SCANDURATION (20 + (100*(BAND==BAND900)))  // Time (mSec) to scan one channel

/// Time to sleep between each channel scan (10ths of a second)
#define SCAN_SLEEP_TIME 2

//static void store_pandescriptors(void);
static void store_pandescriptors(uint8_t* pFrame);
void setScanDurationTimer(void);

/**
   Reports whether the MAC is currently scanning.  This applies to
   both coordinator scans and end/router node scans.

   @return True if MAC is scanning. False if MAC is not scanning.
 */
u8 macIsScanning(void)
{
   return scanInProcess;
}

/**
   Sets a specific single channel to scan, or all channels.  Note that
   @ref PAN_CHANNEL must not be set as a compiler command line
   argument in order for all channels to be scanned.

   @param channel The single channel to scan, or ALL_CHANNELS to scan
   all channels.
 */
void macSetScanChannel(u8 channel)
{
   scanChannel = channel;
}

/**
   Returns the channel set by @ref macSetScanChannel().

   @return The channel set by @ref macSetScanChannel().
 */
u8 macGetScanChannel(void)
{
   return scanChannel;
}

/**
   Send a beacon request frame.  The MAC must be previously
   initialized, and the channel set to a valid value.  The beacon
   request is normally called as part of a channel scan.
 */
static void sendBeaconRequest(void)
{
   uint8_t* pFrame = bmm_buffer_alloc();

   if(pFrame != NULL)
   {
      ftBeaconReq *brFrame = (ftBeaconReq*)(((rx_frame_t*)pFrame)->data);
      // Create a struct pointer to the global variable...

      // Fill in beacon request frame
      brFrame->fcf = FCF_BEACONREQ;
      brFrame->seq = macConfig.bsn++;
      brFrame->panid = PAN_ID;
      brFrame->broadcastAddr = BROADCASTADDR;
      brFrame->cmd = BEACON_REQUEST;

      ((rx_frame_t*)pFrame)->length = sizeof(ftBeaconReq);

      event_object_t event;
      event.event = MAC_EVENT_SEND;
      event.data = pFrame;
      event.callback = &setScanDurationTimer;

      // save Event
      mac_put_event(&event);
   }
}

/**
   Scan channels for coordinators. Reads the scanChannel variable, and
   scans either the channel in scanChannel, or all channels.  This is
   the scan function for routers and end nodes, searching for a
   prospective parent node.

   This function calls itself via a mac alarm callback. The timer
   callback will re-enter the function after the @ref SCANDURATION
   timeout.

   When the entire scan is complete, the mac_scanConfirm() function is called.
 */
void macScan(void)
{
   // Check for fixed channel
   if (PAN_CHANNEL != CHANNEL255)
      macSetScanChannel(PAN_CHANNEL);

   // Set up some varibles on the initial call to macScan()
   if (0xFF == macConfig.currentChannel)
   {
      scanInProcess = true;

      // Reset the variables.
      memset(&panDescriptor, 0, sizeof(panDescriptor_t));

      // logicalChannel is used as flag to show that we received a valid beacon
      panDescriptor.logicalChannel = 0xff;

      macConfig.currentChannel = MIN_CHANNEL; // First possible channel for RF23x

      // Check for a single pre-defined channel to scan.
      if (scanChannel != ALL_CHANNELS)
         macConfig.currentChannel = scanChannel;

      // Display the channel if it's a fixed channel
      if (PAN_CHANNEL != CHANNEL255)
      {
         debugMsgStr("Ch=0x");
         debugMsgInt(PAN_CHANNEL);
         debugMsgStr("\r\n");

      }
   }
   else
      // Not first time through, must sleep very low power nodes.
      if (VLP && (NODETYPE == ENDDEVICE))
         nodeSleep(SCAN_SLEEP_TIME);

   // See if we're done scanning
   if(macConfig.currentChannel > (CHINA_MODE ? 4 : MAX_CHANNEL) ||
         macConfig.currentChannel == scanChannel + 1)
   {
      // done scanning
      scanInProcess = false;
      macConfig.currentChannel = 0xFF;

      mac_scanConfirm();
      UART_PRINT(" -> DONE WITH SCANNING\r\n");
      return;
   }

   // Set the channel.
   macSetOperatingChannel(macConfig.currentChannel);

   // Send the beacon request
   sendBeaconRequest();

   // Beacon was send, increment channel to prepare for the next one.
   macConfig.currentChannel++;
}

void setScanDurationTimer(void)
{
   macSetAlarm(SCANDURATION, macScan);
}

/**
    @brief Record the pan description data from a received beacon
    frame in the global @ref panDescriptor structure. This function
    only stores coordinator information if the beaconing node is
    judged better than the previously stored node.  The criteria for
    determining the best node are:

    -# LQI value
    -# Hops to the coordinator
    -# RSSI of received beacon

    In @ref DEMO mode, the only criteria used is to pick the parent
    with the lowest RSSI reading.  This is done to encourage the
    formation of a multi-hop network for test and demo purposes.
 */
void mac_logPanDescriptors(uint8_t* pFrame)
{
   //ftBeacon *frame = (ftBeacon *)(mac_buffer_rx+1);
   //u8 lqi = ((rx_frame_t *)mac_buffer_rx)->lqi;

   ftBeacon *frame = (ftBeacon *)(((rx_frame_t*)pFrame)->data);
   u8 lqi = ((rx_frame_t*)pFrame)->lqi;

   if (!scanInProcess)
      return;

   if (NODETYPE == COORD)
   {
      // Energy scan, mark that this channel has a lot of interference
      energy.energy[energy.currentChannel] += 500;
   }
   else  // Router and end nodes
   {
      // Check the Beacon frame Superframe spec value.
      u16 previous_superframe = panDescriptor.superFrameSpec.superframe_data;
      panDescriptor.superFrameSpec.superframe_data = frame->superFrame;

      // Determine if association permit is true. We're looking for false.
      if(panDescriptor.superFrameSpec.superframe_struct.association_permit)
      {
         panDescriptor.superFrameSpec.superframe_data = previous_superframe;
         return;
      }

      if(DEMO)
      {
         // Demo mode, just pick closest node
         if (radioGetSavedRssiValue() > panDescriptor.rssi)
         {
            store_pandescriptors(pFrame);
         }
      }
      else
      {
         // Determine if the Beacon LQI value is stronger than the previous scan.
         if(panDescriptor.lqi < lqi)
         {
            store_pandescriptors(pFrame);
         }
         // If LQI is equal to previous stored, check for the least amount of hops to the coord.
         else if((panDescriptor.lqi == lqi)
               && (panDescriptor.hopsToCoord > frame->hops))
         {
            store_pandescriptors(pFrame);
         }
         // If LQI and hops are equal to the previous stored, check for the best RSSI value.
         else if((panDescriptor.lqi == lqi)
               && (panDescriptor.hopsToCoord == frame->hops)
               && (panDescriptor.rssi < radioGetSavedRssiValue()))
         {
            store_pandescriptors(pFrame);
         }
      }
   }
}

/**
    @brief This will store the actual beacon data into the pan
    descriptor list after the appropriate pan conditions have been
    met.  See @ref mac_logPanDescriptors().
 */
static void store_pandescriptors(uint8_t* pFrame)
{
   ftBeacon *frame = (ftBeacon *)(((rx_frame_t*)pFrame)->data);
   u8 lqi = ((rx_frame_t*)pFrame)->lqi;

   // Gather the pan descriptor data and retain the strongest link in the scan process.
   panDescriptor.coorAddrMode = frame->fcf >> 14;
   panDescriptor.coorPANId = frame->panid;
   panDescriptor.coordAddr = frame->addr;
   panDescriptor.hopsToCoord = frame->hops;
   panDescriptor.logicalChannel = macConfig.currentChannel - 1;
   panDescriptor.channelPage = CHANNEL_PAGE_0;
   panDescriptor.lqi = lqi;
   panDescriptor.rssi = radioGetSavedRssiValue();
}

/**
    @brief Trigger a call to @ref appScanConfirm(), since the scanning
    process is done.  If at least one valid beacon was received, then signal that
    success to the application.
 */
void mac_scanConfirm(void)
{
   // logicalChannel is used as flag to show that we received a valid beacon
   u8 gotbeacon = (panDescriptor.logicalChannel != 0xff);

   if (gotbeacon)
   {
      // Save the panDescriptor data to the PIB's.
      macConfig.panId = panDescriptor.coorPANId;
      macConfig.parentShortAddress = panDescriptor.coordAddr;

      // Need to reset the PAN ID in the radio.
      radioSetPanId(macConfig.panId);

      macSetOperatingChannel(panDescriptor.logicalChannel);

      // We are one hop more than our (prospective) parent
      macConfig.hopsToCoord = panDescriptor.hopsToCoord + 1;
   }
   // Confirm the scan, only if we have a valid coordinator
   appScanConfirm(gotbeacon);
}

/**
   Callback function, called from timer set in macFindClearChannel().
   This function tallies the results from an energy scan on one
   channel, and manages the channel change for the energy scan.
 */
static void energyScanCallback(void)
{
   if (NODETYPE == COORD)
   {
      // This function is a callback for macFindClearChannel()
      u8 maxChan = (CHINA_MODE ? 4 : MAX_CHANNEL);

      // Check to see if we're done
      if (energy.currentChannel >= maxChan)
      {
         // Done scanning, pick a channel
         scanInProcess = false;

         // Disable the energy detect interrupt
         u8 i;
         i = hal_register_read(RG_IRQ_MASK);
         hal_register_write(RG_IRQ_MASK, i & ~HAL_ED_READY_MASK);

         // Find lowest energy for a channel
         u16 bestEnergy = ~0;

         for (i=MIN_CHANNEL;i<=maxChan;i++)
         {
            // find MIN energy on a channel
            if (energy.energy[i] < bestEnergy)
               bestEnergy = energy.energy[i];
         }

         // Make a list of channels with this energy level
         // Use energy array as temp space.
         u8 ndx=0;
         u8 channels[maxChan];
         for (i=MIN_CHANNEL;i<=maxChan;i++)
         {
            if (energy.energy[i] == bestEnergy)
            {
               // save to list
               channels[ndx++] = i;
            }
         }


         // Now pick a random channel from the list
         if (ndx)
         {
            u8 bestCh = channels[energy.randomVal%ndx];

            // Call the app callback function
            appClearChanFound(bestCh);
         }
         return;
      }

      // Set new channel
      radioSetOperatingChannel(++energy.currentChannel);

      // Send beacon request
      sendBeaconRequest();

      // Start the energy detect scanning process (this will result in
      // an IRQ, see macEdCallback)
      hal_register_write(RG_PHY_ED_LEVEL, 0);

      // Set timer for next channel
      energy.timerID = macSetAlarm(SCANDURATION, energyScanCallback);
   }
}

/**
   @brief Callback function, called by the radio ISR function when the
   radio issues an energy measurement interrupt.  This function stores
   the energy measurement for later use.
 */
void macEdCallback(void)
{
   if (NODETYPE == COORD)
   {
      // We have received an energy detect IRQ (ED_READY)

      // Accumulate the ED values.
      u8 e = hal_register_read(RG_PHY_ED_LEVEL);
      energy.energy[energy.currentChannel] += e;
      hal_register_write(RG_PHY_ED_LEVEL, 0);
   }
}


/**
   @brief This function is used by the coordinator to find a clear
   channel to use for the PAN.  It automatically scans all channels,
   issues a beacon request on each channel, and listens for channel
   energy level.  When the scanning process is done, @ref
   appClearChanFound() is called with the least occupied channel
   number.

   If @ref PAN_CHANNEL is set to a valid channel, then scanning is not
   performed.
 */
void macFindClearChannel(void)
{
   if (NODETYPE == COORD)
   {
      if (PAN_CHANNEL != CHANNEL255)
      {
         // In this mode, don't scan
         appClearChanFound(PAN_CHANNEL);
      }
      else
      {
         u8 i;

         // Clear channels array
         for (i=0;i<MAX_CHANNEL+1;i++)
            energy.energy[i] = 0;

         // Set first channel
         energy.currentChannel = MIN_CHANNEL - 1;
         energy.timerID = 0;

         // Init radio
         macInit(0xff);
         radioSetTrxState(RX_ON);
         energy.randomVal = radioRandom(8);
         radioSetOperatingChannel(MIN_CHANNEL);


         // Scan through channels, using callback function
         // This is tricky, since we need to do both an energy
         // detect scan and a beacon scan
         // Steps in the process
         // 1. Issue a beacon request
         //    (If any beacon comes, knock this channel off list)
         // 2. Begin energy scan
         //    Using macAlarm, do a bunch of scans
         //      Issue scan command
         //      Process IRQ's for ED ready
         //      Average ED value
         // 3. When scan duration time is up, move to next channel

         // Enable the Energy Detect interrupt
         i = hal_register_read(RG_IRQ_MASK);
         hal_register_write(RG_IRQ_MASK, i | HAL_ED_READY_MASK);

         // Start the scanning process
         scanInProcess = true;

         energyScanCallback();
      }
   }
}

/** @} */
/** @} */
