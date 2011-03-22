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
  $Id: rum_application.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "../inc/radio.h"
#include "../inc/mac.h"
#include "../inc/mac_event.h"
#include "../inc/mac_start.h"
#include "../inc/mac_data.h"
#include "../inc/mac_scan.h"
#include "../inc/mac_associate.h"
#include "../inc/system.h"
#include "../inc/hal.h"
#include "../inc/sleep.h"
#include "../inc/sensors.h"
#include "../inc/avr_timer.h"

//#include "sixlowpan_wake.h"

#include "../inc/deRFaddon/uart.h"


/*
#if __arm__
// Utasker include
#include "config.h"

#include "tuip.h"
#include "hal_arm.h"
#define sixlowpan_hc01_gen_rs()
#define sixlowpan_init()
#define sixlowpan_application_init()
#define serial_send_frame(a,b,c)
//QUEUE_HANDLE  current_interface_handle;
#else // AVR
 */
#include "../inc/serial.h"
#define tuip_init_802154(a, b)
//#include <avr/eeprom.h>
// You can program the EEPROM with macaddress this way, using an ELF file
//u8 EEMEM macaddress[8] = {0xde, 0xed, 0xbe, 0xef, 0x11, 0x22, 0x33, 0x44};
//u8 EEMEM macaddress[8] = {0x01, 0x23, 0x45, 0x56, 0x78, 0x9a, 0xbc, 0xde};
//u8 EEMEM macaddress[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
#include "../inc/avr_sixlowpan.h"
#if (IPV6LOWPAN == 0)
#define sixlowpan_hc01_gen_rs(a)
#endif
void sixlowpan_application_init(void);
static u8 pingTimer;
static u8 streamMode=0;        ///< Are we in streaming mode?
extern tSerialFrame SerialFrame[];
//#endif
#define UDP_PORT_COMMANDS 0xF0B0
#define UDP_PORT_RESPONSE 0xF0B1

extern u8 simulateButton;
extern u16 frameInterval;

extern uint8_t destipAddr[];
extern uint8_t googleipAddr[];
extern uint8_t serveripAddr[];

/**
   @addtogroup app
   @{

   This is a sample application, intended to show how the Route Under
   MAC can be used to create a simple application.

   @section app_network_start Joining the network

   On startup, this application uses a number of callback functions to
   bring up the network.  A coordinator node executes this process:

   - Call macFindClearChannel(), which starts a channel scan to find a
     free channel to use for the new PAN.

   - When the scan is complete, the appClearChanFound() callback is
     called. This then initializes the MAC/radio using the channel
     found.

   When a router or end node starts, it connects to the network using
   these steps:

   - appStartScan() is called, which scans all available channels,
     issuing a beacon request frame on each channel.

   - When the scan is complete, appScanConfirm() is called with an
     argument that specifies whether the scan successfully found a
     network to join.

   - If the scan process found a network, then appAssociate() is
     called, which causes the node to send an association request
     packet to the node found in the channel scan.

   - When an association response packet is received, the
     appAssociateConfirm() function is called by the MAC.  At this
     point, the node is part of the network and can send data packets
     to other nodes.

   @section app_data Sending and Receiving Data

   To send data to another node, the application can call the
   macDataRequest() function.  The application must know the address
   of the destination node, unless the destination is the coordinator,
   which always uses the short address 0x0000 (See @ref
   DEFAULT_COORD_ADDR).

   When a data packet is received, it is routed on to its
   destination. If the final destination is this node's short
   address, then the MAC will call the appDataIndication()
   function, which can then process the incoming data.
 */

#define TRACKDEMO 0

// Application variables
static u8 failCount;  ///< Number of transmit-to-parent failures before re-scanning.
volatile u8 gotPanCoordPing;

// Function declarations
void appStartScan(void);
void ledcallback(void);
void sixlowpan_button(void);



//#if __arm__
///**
//    @brief This will toggle the led of choice on. Used by LED_ON().
//*/
//void ledOn(u8 val)
//{
//    if(1 == val) LED1ON();
//    else if(2 == val) LED2ON();
//    else if(3 == val) LED3ON();
//    else if(4 == val) LED4ON();
//}

///**
//    @brief This will toggle the led of choice off. Used by LED_OFF().
//*/
//void ledOff(u8 val)
//{
//    if(1 == val) LED1OFF();
//    else if(2 == val) LED2OFF();
//    else if(3 == val) LED3OFF();
//    else if(4 == val) LED4OFF();
//}
//#endif // __arm__


// These LED functions are used to provide address callbacks for macSetAlarm.
// Cannot use defines directly in callbacks.
void ledoff1(void)
{
   LED_OFF(1);
}

void ledoff2(void)
{
   LED_OFF(2);
}

void ledoff3(void)
{
   LED_OFF(3);
}

// This function can be used for diagnostic reasons. Not used in RUM application.
void blink1(void)
{
   LED_ON(1);
   macSetAlarm(25, ledoff1);

   //    macSetAlarm(500, blink1);

   //    macDataRequest(0, 7, (u8*)"Howdy\r\n");
}

/**
   Application callback function, called when the MAC receives a ping request
   packet addressed to this node.

   @param addr The short address of the node that sent the ping request
 */
void appPingReq(u16 addr)
{
   // We got a ping, send a response
   // Blip the LED
   LED_ON(1);

   macSetAlarm(/* LED_PING_DELAY */ 1 ,ledoff1);

   debugMsgStr("\r\nPing request from node ");
   debugMsgInt(addr);

   if (PLATFORM == RAVEN && SERIAL)
      // Tell 3290p
      serial_send_frame(REPORT_PING_BEEP, 2, (u8*)&addr);

   macPing(PING_RSP_FRAME, addr);
}

/**
   Application callback function, called when the MAC receives a ping
   response packet addressed to this node.

   @param addr The short address of the node that send the ping response
 */
void appPingRsp(u16 addr)
{
   debugMsgStr("\r\nPing response from ");
   debugMsgHex(addr);
   debugMsgStr(" LQI=");
   debugMsgInt(radioGetSavedLqiValue());
   debugMsgStr(" RSSI=");
   debugMsgInt(radioGetSavedRssiValue());
   debugMsgCrLf();

   if (PLATFORM == RAVEN && SERIAL)
      // Tell 3290p
      serial_send_frame(REPORT_PING_BEEP, 2, (u8*)&addr);

   // turn on the LED
   LED_ON(1);

   // and make it turn off after a litte bit
   macSetAlarm(LED_DELAY,ledoff1);
   //#if(__arm__)
   //    gotPanCoordPing = 1;
   //#endif
}

/**
   Application callback function.  Called when this node receives a
    ping response via IPv6
 */
void appSixlowpanPingResponse(void)
{
   if (PLATFORM == RAVEN)
      // Send the 3290 a frame to show that we got a ping response.
      serial_send_frame(REPORT_PING_BEEP, 0, NULL);
}

/**
   Application callback function, called when the MAC receives an ACK
   packet from a node that we sent a packet to.
 */
void appPacketSendSucceed(void)
{
   // Reset the failure count on a good packet to parent
   // (could also decrement failCount)

   if (NODETYPE != COORD)
   {
      // figure out which way we were sending when the failure occurred
      if (macConfig.lastDestAddr == macConfig.parentShortAddress)
         failCount = 0;

      // Tell sensor app
      if (APP == SENSOR)
         sensorPacketSendSucceed();

      // Tell IPv6 LOWPAN
      //        if (IPV6LOWPAN)
      //            sixlowpanSleep_packetSucceed();
   }
}

/**
   Application callback function, called when the MAC fails to send a
   packet due to a channel access failure (air is too busy).
 */
void appPacketSendAccessFail(void)
{
   // Tell sensor app
   if (APP == SENSOR)
      sensorPacketSendFailed();
}

/**
   Application callback function, called when the MAC fails to receive
   an ACK packet from a node that we sent a packet to.
 */
void appPacketSendFailed(void)
{

   //Tell 6LoWPAN
#if IPV6LOWPAN
   //        sixlowpanSleep_packetFailed();
#endif

   u8 parentFailed;
   if (NODETYPE != COORD)
   {
      // don't mess with assessing failures during scanning
      if (macIsScanning())
         return;

      debugMsgStr("\r\nFail=");
      debugMsgInt(failCount+1);

      parentFailed = (macConfig.lastDestAddr == macConfig.parentShortAddress);
      /* Special code: this prevents a situation where the
           coordinator goes away, and the nodes take a long time to
           realize that the network is gone.  Nodes Keep trying to
           re-associate with each other, but until a router loses a
           number of packets to its parent, it still thinks it's
           associated.  This code forces the issue by either verifying
           that the router is still connected, or forcing a failure.    */
      // Send an empty data packet
      if (parentFailed && NODETYPE == ROUTER)
         macDataRequest(macConfig.parentShortAddress, 0, NULL);

      // Don't have a cow until we have missed a few packets in a row
      if (++failCount < 8)
      {
         // Tell sensor app
         if (APP == SENSOR)
            sensorPacketSendFailed();
         return;
      }

      // A sent packet failed to reach its destination too many times
      // figure out which way we were sending when the failure occurred
      if (parentFailed)
      {
         // re-associate if we were sending upstream
         macConfig.associated = false;

         // It is possible to make the coord/end units re-connect
         // to the coordinator more quickly by not scanning all
         // available channels.  To do this, uncomment the following
         // line.
         macSetScanChannel(macConfig.currentChannel);

         if (APP == SENSOR)
         {
            sensorLostNetwork();
         }
         appStartScan(); // do not wait for new scan
         //macSetAlarm((radioRandom(8)+5) *10, appStartScan);
      }
      if (NODETYPE == ROUTER &&
            macIsChild(macConfig.lastDestAddr))
      {
         // Drop child from table if the failure was downstream
         macRemoveChild(macConfig.lastDestAddr);
         debugMsgStr("\r\nDropped child bcs packet send failed.");
      }
   }

}

#if DEBUG || DOXYGEN
/**
    @brief This is an array of radio register names for serial output
           used when rf2xx_reg_dump() is called.  See the radio
           datasheet for details.
 */
u8 rf2xx_reg_names[][16] =
      {"TRX_STATUS", "TRX_STATE", "TRX_CTRL_0", "TRX_CTRL_1", "PHY_TX_PWR",
            "PHY_RSSI", "PHY_ED_LEVEL", "PHY_CC_CCA", "CCA_THRES", "TRX_CTRL_2", "IRQ_MASK",
            "IRQ_STATUS", "VREG_CTRL", "BATMON", "XOSC_CTRL", "RX_SYN", "RF_CTRL_0", "XAH_CTRL_1",
            "FTN_CTRL", "RF_CTRL_1", "PLL_CF", "PLL_DCU", "PART_NUM", "VERSION_NUM", "MAN_ID_0",
            "MAN_ID_1", "SHORT_ADDR_0", "SHORT_ADDR_1", "PAN_ID_0", "PAN_ID_1",
            "IEEE_ADDR_0", "IEEE_ADDR_1", "IEEE_ADDR_2", "IEEE_ADDR_3", "IEEE_ADDR_4",
            "IEEE_ADDR_5", "IEEE_ADDR_6", "IEEE_ADDR_7", "XAH_CTRL_0", "CSMA_SEED_0",
            "CSMA_SEED_1", "CSMA_BE"};

/**
    @brief This is an array of radio register values to be used when
           rf2xx_reg_dump() is called.  See the radio datasheet for
           details.
 */
//u8 rf2xx_reg_enum[] =
//      {RG_TRX_STATUS, RG_TRX_STATE, RG_TRX_CTRL_0, RG_TRX_CTRL_1, RG_PHY_TX_PWR,
//            RG_PHY_RSSI, RG_PHY_ED_LEVEL, RG_PHY_CC_CCA, RG_CCA_THRES, RG_TRX_CTRL_2, RG_IRQ_MASK,
//            RG_IRQ_STATUS, RG_VREG_CTRL, RG_BATMON, RG_XOSC_CTRL, RG_RX_SYN, RG_RF_CTRL_0, RG_XAH_CTRL_1,
//            RG_FTN_CTRL, RG_RF_CTRL_0, RG_PLL_CF, RG_PLL_DCU, RG_PART_NUM, RG_VERSION_NUM, RG_MAN_ID_0,
//            RG_MAN_ID_1, RG_SHORT_ADDR_0, RG_SHORT_ADDR_1, RG_PAN_ID_0, RG_PAN_ID_1,
//            RG_IEEE_ADDR_0, RG_IEEE_ADDR_1, RG_IEEE_ADDR_2, RG_IEEE_ADDR_3, RG_IEEE_ADDR_4,
//            RG_IEEE_ADDR_5, RG_IEEE_ADDR_6, RG_IEEE_ADDR_7, RG_XAH_CTRL_0, RG_CSMA_SEED_0,
//            RG_CSMA_SEED_1, RG_CSMA_BE};
#endif

/**
    @brief Dumps the RF2xx register contents to serial port.

    Note: The serial output will only be available if the @ref DEBUG
    macro is defined as non-zero.
 */
void rf2xx_reg_dump(void)
{
//#if DEBUG
//   {
//      u8 i,j,k,val;
//      s8 str[40];
//
//      debugMsgStr("\r\n\r\nREG DUMP\r\n");
//
//      //k = sizeof(rf2xx_reg_enum);
//      k = 0;
//      for(i=0;i<k;i++)
//      {
//         val =  hal_register_read(rf2xx_reg_enum[i]);
//         sprintf((char*)str,"%-15s - %02X ", (char *)rf2xx_reg_names[i], (uint16_t)val);
//         debugMsgStr((char *)str);
//         for (j=7;j<8;j--)
//            // Print a bit
//            debugMsgChr(((val >> j)&1 ? '1' : '0' ));
//         debugMsgCrLf();
//      }
//
//      debugMsgStr("\r\n");
//   }
//#endif
}

/**
   Application function, sends a data frame to the coordinator, and
   schedules another call in two seconds, using the timer function.

   This can be used for testing the network.  To send a real data
   frame, use macDataRequest().
 */
void appSendDataFrame(void)
{
   // send data frames once per second
   if (NODETYPE != COORD)
      macDataRequest(0x00, 4, (u8*) ((NODETYPE == ENDDEVICE) ? "endd" : "rout"));

   // Send another data frame later
   macSetAlarm(1000, appSendDataFrame);
}

/**
   @brief Callback function, called when the MAC receives a data packet for
   this node.

   The data is available in mac_buffer_rx, which can be cast to a
   dataFrame_t struct like this:

   @code
   ftData *frame = (ftData *)(mac_buffer+1);
   @endcode
 */
void appDataIndication(void)
{
   // Write app code here -- This node has received a data frame.

   // Example code:

   // Find out the type of packet
   //ftData *frame = (ftData *)(mac_buffer_rx+1);

   /*
    if (frame->type == DATA_FRAME)
    {
        // Flash LED when we get a packet back
        LED_ON(1);
        // Turn off LED in a little bit, so it flashes
        macSetAlarm(LED_DELAY,ledoff1);

        // pass data to application
        if (APP == SENSOR)
            sensorRcvPacket(frame->payload);
        else
            // Default application
            if (DEBUG)
            {
                // Print data in frame to serial port (streaming mode)
                u8 n = *mac_buffer_rx - 16;
                // Terminate the string
                frame->payload[n] = 0;
                // and print it
                debugMsgStr((char *)(frame->payload));
            }
    }
    */

}

/**
   Example function, starts the network by scanning channels for a coordinator.

   This node will either scan all available channels, or just one
   channel if @ref macSetScanChannel() is called.  @see macScan().
 */
void sappStartScan(void)
{
   if(NODETYPE != COORD)
   {
      macInit(0xff);

      macScan();
   }

   if (IPV6LOWPAN == 1)
      sixlowpan_application_init();
}

/**
   Callback function, called when the MAC has associated a child of
   this node to the network.

   @param shortAddress The short address of the new child node that
   has been associated.  The MAC stores this address, so the
   application should not have to.
 */
void appChildAssociated(u16 shortAddress)
{
   // Blip the LED when we associate a child
   LED_ON(1);
   macSetAlarm(LED_DELAY,ledoff1);
}

/**
   Callback function, called when the MAC has associated a new node to
   a network. This is only called for the coordinator.

   @param shortAddress The short address assigned to the new node.  The
   MAC stores this address, so the application should not have to.
 */
void appNodeAssociated(u16 shortAddress)
{
}

/**
   Callback function, called when the MAC receives an association
   confirm packet addressed to this node, or the process timed out.

   @param success True if an association packet was received, or false
   if the association process timed out.
 */
void appAssociateConfirm(u8 success)
{
   if(NODETYPE != COORD)
   {
      if (success == SUCCESS)
      {
         //TODO added to reset status timer -> to recognize when node reassociates
         /*
         if(is_status_active())
         {
            status_timer_disable();
         }
         init_status_timer();
         status_timer_enable();
         */


         UART_PRINT("\r\nAssociated to ");
         UART_PRINT_HEX(macConfig.parentShortAddress);
         UART_PRINT(" as ");
         UART_PRINT_HEX(macConfig.shortAddress);
         /*
            LED_ON(1);
            macSetAlarm(LED_DELAY,ledoff1);
          */

         // For raven, notify other processor
         if (PLATFORM == RAVEN && SERIAL)
            serial_send_frame(REPORT_ASSOCIATED, 0, NULL);

         /* 6lowpan association */
         if (IPV6LOWPAN == 1)
            sixlowpan_hc01_gen_rs();

         // If we are auto-sending data, start that process.
         if (APP == SENSOR && NODETYPE != COORD)
         {
            sensorInit();
            sensorAutoSendStart();
         }

#if TRACKDEMO
         // Track Demo Application
         if (NODETYPE == ENDDEVICE)
         {
            // Re-associate every second
            macSetAlarm(1000, appStartScan);
         }
#endif
      }
      else
      {

         UART_PRINT("Association of %x failed\r\n", macConfig.shortAddress);

         //debugMsgStr("\r\nFailed to associate");

         if (VLP)
         {
            // Sleep for 10 seconds, try again
            nodeSleep(100);
            appStartScan();
         }
         else
            // Try again in one second
            macSetAlarm(1000, appStartScan);
      }
   }
}

/**
   Sample application function, associates this node to a network.
   This function is called after a successful @ref macScan.
 */
void appAssociate(void)
{
   debugMsgStr("\r\nAssociating to ");
   debugMsgHex(panDescriptor.coordAddr);
   debugMsgStr(" on ch ");
   debugMsgInt(panDescriptor.logicalChannel);
   debugMsgStr(", hops = ");
   debugMsgInt(panDescriptor.hopsToCoord);

   macAssociate(panDescriptor.coordAddr, panDescriptor.logicalChannel);
}

/**
   Callback function, called when the MAC has completed its channel scan.

   @param success True if @ref macScan found a network to connect to,
   or false if no networks were found.
 */
void appScanConfirm(u8 success)
{
   // Write app code here -- This (end) node has finished its scan,
   // and has receive a coordinator address.

   // Example code:
   // scan is done, turn off LED
   if(NODETYPE != COORD)
   {
      if (success)
      {
         // associate with coordinator
         macSetAlarm(10,appAssociate);

         debugMsgStr("\r\nScan good, select chan ");
         debugMsgInt(panDescriptor.logicalChannel);
      }
      else
      {
         // failure to find a network
         if (VLP)
         {
            // Try again after sleeping for 10 seconds
            nodeSleep(100);
            appStartScan();
         }
         else
            macSetAlarm(1000,appStartScan);
         debugMsgStr("\r\nScan bad");
      }
   }
}

/**
   Callback function, called when @ref macFindClearChannel()
   completes.

   u8 channel The clear channel selected for use by this PAN.
 */
void appClearChanFound(u8 channel)
{
   if (NODETYPE == COORD)
   {
      macInit(channel);
      macStartCoord();
#if (__AVR__)
      //debugMsgStr("\r\nStartup, I am coordinator on ch ");
      UART_PRINT("\r\nStartup, I am coordinator on ch ");
//#else // __arm__
      //        fnDebugMsg("\r\nStartup, I am coordinator on ch ");
#endif // __AVR__
      //debugMsgInt(channel);
      UART_PRINT_HEX(channel);
      UART_PRINT("\r\n");
      macConfig.associated = true;

      if (IPV6LOWPAN == 1)
         //Start uIP stack with 802.15.4 interface
         tuip_init_802154(macConfig.panId, macConfig.shortAddress);
   }
}

#if __AVR__
/**
   Verifies that the EEPROM contains valid data for the stored MAC
   address.  If the EEPROM is unprogrammed, then a random MAC address
   is written into EEPROM.

   Similarly, the sensor interval time is set to 2 seconds if the
   EEPROM is unprogrammed.
 */
void checkEeprom(void)
{
   //if (DEMO == 1)
   //if(1)
   //{
   u8 buf[8];
   u8 i,bad=1;

   halGetMacAddr(buf);
   for (i=0;i<8;i++)
      if (buf[i] != 0xff)
      {
         // Valid (non-fffffff) MAC addr
         bad = 0;
         break;
      }

   if (bad)
   {
      // create random MAC address and store it
      radioInit(0); // Needed for random to work.
      radioSetTrxState(RX_ON);
      for (i=0;i<8;i++)
         buf[i] = radioRandom(8);
      halPutMacAddr(buf);
   }

   halGetEeprom((u8*)offsetof(tEepromContents, dataSeconds),
         sizeof(typeof(((tEepromContents*)0)->dataSeconds)),
         buf);
   if (*((u16*)buf) == 0xffff)
   {
      // Bad timeout, set for 2 seconds
      *((u16*)buf) = 0x0014;
      halPutEeprom((u8*)offsetof(tEepromContents, dataSeconds),
            sizeof(typeof(((tEepromContents*)0)->dataSeconds)),
            buf);
   }
   //}
}
#endif // __AVR__

/**
   Sample application function, which initializes the application.
   This function is meant to be called on startup only.
 */
void appInit(void)
{
   // Init the mac
   LED_INIT();

   // Blip the LED once on powerup
   blink1();

#if (__AVR__)
   // If the EEPROM is cleared, init it to something useful
   checkEeprom();
#endif // __AVR__
   // Init the mac and stuff
   if (NODETYPE == COORD)
   {
      macFindClearChannel();
      // appClearChanFound() will be called
   }
   else
   {
      //debugMsgStr("\r\nStartup, I am router/end.");
      UART_PRINT("Startup, I am router/end.\r\n");

      // End node or router, start scanning for a coordinator
      appStartScan();
      // when scan is finished, appScanConfirm will be called.

      if (IPV6LOWPAN == 1)
         sixlowpan_init();
   }

   // Init the button
   BUTTON_SETUP();

   // Turn the power down depending if button is pressed on powerup
   if (!BUTTON_PRESSED() && DEMO)
   {
#if !TRACKDEMO
      // Reduce power to lowest setting
      if(BAND==BAND900)
         hal_register_write(RG_PHY_TX_PWR,0x06);
      else
         radioSetTxPowerLevel(TX_PWR_17_2DBM);

      // Reduce sensitivity a lot.
      hal_subregister_write(SR_RX_PDT_LEVEL, 0x02);
#endif
      UART_PRINT("Demo mode");
   }
   else
      UART_PRINT("Normal mode\r\n");

}

/**
   Sample application function. Sends a ping packet to the network
   coordinator.
 */
void doPing(void)
{
   hal_register_write(RG_CSMA_BE,0);
   //macDataRequest(DEFAULT_COORD_ADDR, 110, dataString);
   macPing(PING_REQ_FRAME, DEFAULT_COORD_ADDR);

   // Uncomment the next line to make the button unleash a torrent of pings
#if __AVR__
   //    pingTimer = macSetAlarm(5+(23*(BAND == BAND900)), doPing);
   pingTimer = macSetAlarm(500, doPing);
#endif
}

/**
    @brief This is used as the allNodes callback for repeating the function operation.
 */
void allNodesCB(void)
{
   allNodes(0,0);
}

/**
   When called, this will either Ping or request data from all associated nodes.
   A reading interval can also be set if desired.
 */
void allNodes(u8 func, u16 val)
{
   if (NODETYPE == COORD)
   {
      static u8 nodeNdx = 0; // Incremented each time through
      static u8 function;
      static u16 value;
      u8 nodeFound=0;
      associatedNodes_t *node;

      // Check to see if this is first time through
      if (func)
      {
         // Set up for repeated calls to this function
         function = func;
         value = val;
         nodeNdx = 1;
      }

      // See if we are supposed to do something
      if (!function)
         return;

      // Get a node from table
      node = macGetNode(nodeNdx);

      // Do the operation for connected nodes
      if (node->nodeType)
      {
         nodeFound = 1;
         // Do the operation on it
         switch (function)
         {
         case PING_ALL:
            // Send a ping to the node
            debugMsgStr("\r\nPinging node ");
            debugMsgInt(nodeNdx);
            macPing(PING_REQ_FRAME, nodeNdx);
            break;
         case REPORT_ALL:
            // Change report time
            if (APP==SENSOR)
            {
               debugMsgStr("\r\nInterval node ");
               debugMsgInt(nodeNdx);
               debugMsgStr(" = ");
               debugMsgInt(value);
               sensorRequestReading(nodeNdx, value);
            }
            break;
         }
      }

      // Prepare for next node in the list
      nodeNdx++;
      if (nodeNdx < MAXNODES)
         // Let's go again
         macSetAlarm(nodeFound ? 250 : 1, allNodesCB);
      else
         // All done, cancel any furthur action
         function = value = 0;
   }
}

/**
   Print prompts for debug mode.  This is in a separate function
   because we want a delay before printing the next prompt, in case
   there is some status data printed from the last command.
 */
void printPrompt(void)
{
   if (NODETYPE == COORD)
   {
      //#if (__AVR__)
      debugMsgStr("\r\nd=dump t=table i=info p=ping s=stream c=chan");
      debugMsgStr("\r\nr=reading n=name w=wake P=pause: ");
      /*
#else // __arm__
        fnDebugMsg("\r\n           802.15.4 Menu\r\n");
        fnDebugMsg("================================\r\n");
        fnDebugMsg("a.....IP address       A.....IPv6 Addresses\r\n");
        fnDebugMsg("b.....break            c.....chan\r\n");
        fnDebugMsg("C.....Calibration      d.....dump\r\n");
        fnDebugMsg("f.....filename         i.....info\r\n");
        fnDebugMsg("I.....New IP Addr      l.....log\r\n");
        fnDebugMsg("n.....name             o.....toggle readings\r\n");
        fnDebugMsg("p.....ping             Q.....Quit telnet\r\n");
        fnDebugMsg("r.....read interval    t.....table\r\n");
        fnDebugMsg("T.....Touch            w.....wake\r\n");
        fnDebugMsg("X.....Max TX power\r\n");
        fnDebugMsg("> ");
#endif // __AVR__
       */
   }
   else
      debugMsgStr("\r\nd=dump t=table i=info p=ping s=stream P=pause: ");
}

#if __AVR__
#define sicslowpan_hc01_gen_rs()

/**
   Sample application task loop function.  This function is meant to
   be called periodically.  It uses the serial port and button status
   for input, and implements a terminal interface for debugging use.
 */
void appTask(void)
{
   static u8 state=0;             // Used for button processing

   // perform periodical things.
   // check for button presses on RCB's
   if (BUTTON_PRESSED() ||
         (simulateButton &&
               IPV6LOWPAN == 1 &&
               NODETYPE != COORD))
   {
      if (!state)
      {
         if (pingTimer)
         {
            // stop pinging
            macTimerEnd(pingTimer);
            pingTimer = 0;
         }
         else
         {
            // ping the coordinator
            //debugMsgStr("\r\nPinging coord\r\n");
            UART_PRINT("Pinging coord\r\n");
            if (IPV6LOWPAN == 1)
            {
               simulateButton = 0;
               sixlowpan_button();

            }
            else
               doPing();
         }

         state = 1;
      }
   }
   else
      state = 0;

   if (DEBUG && SERIAL)
   {
      if (serial_ischar() &&
            (APP != SENSOR || !sensorCalBusy()))
      {
         u8 n;
         char ch;
         static u16 addr=0;
         char str[102];

         if (macConfig.busy)
            // try again when mac is not busy
            return;

         ch = serial_getchar();
         // Quit stream mode on Ctrl-t
         if (ch == 0x14)
            streamMode = 0;
         // In stream mode, send all serial data over the air.
         if (streamMode)
         {
            // Send the chars out over the air to dest
            n = 0;
            for(;;)
            {
               // Build a string of chars waiting in the queue
               str[n++] = ch;
               if (n >= 100)
                  break;
               if (serial_ischar())
                  ch = serial_getchar();
               else
                  break;
            }
            // And send it off to destination
            macDataRequest(addr, n, (u8*) str);
         }
         else
         {
            debugMsgCrLf();
            switch (ch)
            {
            case 'd':
               // reg dump
               rf2xx_reg_dump();
               break;
            case 't':
               // print table
               macPrintTree();
               break;
            case 'i':
               // print info
               sprintf(str,"\r\nshort = %04X\r\nparent = %04X\r\nroute=%04X\r\n",
                     macConfig.shortAddress,
                     macConfig.parentShortAddress,
                     macConfig.lastRoute);
               debugMsgStr(str);
               sprintf(str,"chan = %d\r\n", macConfig.currentChannel);
               debugMsgStr(str);
               debugMsgStr("PAN ID = 0x");
               debugMsgHex(macConfig.panId);
               debugMsgCrLf();
               u32 low = macConfig.longAddr;
               u32 high = macConfig.longAddr >> 32;
               sprintf(str,"long = 0x%08lX%08lX\r\n", high, low);
               debugMsgStr(str);

               sprintf(str,"assoc = %s\r\nHops = %04X\r\n",
                     macConfig.associated ? "true" : "false",
                           macConfig.hopsToCoord);
               debugMsgStr(str);
               sprintf(str,"rand = %02X\r\n",
                     radioRandom(8));
               debugMsgStr(str);
               // Radio part number
               u16 pn = radioGetPartnum();
               switch (pn)
               {
               case RF230:
                  pn = 230;
                  break;
               case RF231:
                  pn = 231;
                  break;
               case RF212:
                  pn = 212;
                  break;
               default:
                  // Just report whatever number the radio chip gives.
                  break;
               }
               debugMsgStr("Part=RF");
               sprintf(str,"%u, Rev=%d\r\n", pn,
                     hal_register_read(RG_VERSION_NUM));
               debugMsgStr(str);
               if (NODETYPE != COORD && APP == SENSOR)
               {
                  debugMsgStr("Name=");
                  debugMsgStr(sensorGetName());
                  debugMsgCrLf();
               }
               // Report compile options
               sprintf(str,"Sleep = %s\r\n", RUMSLEEP ? "Yes":"No");
               debugMsgStr(str);
               sprintf(str,"Freq = %dMHz\r\n", (int)(F_CPU / 1000000UL));
               debugMsgStr(str);
               if (APP == SENSOR)
               {
                  sprintf(str,"Interval = %d.%dsec\r\n", frameInterval/10, frameInterval%10);
                  debugMsgStr(str);
                  sprintf(str,"Sleep timed by %s timer\r\n", WDOG_SLEEP ? "WatchDog" : "32KHz");
                  debugMsgStr(str);
               }
               sprintf(str,"6LoWPAN = %s\r\n", IPV6LOWPAN ? "Yes":"No");
               debugMsgStr(str);
               sprintf(str,"Demo mode = %s\r\n", DEMO ? "Yes":"No");
               break;
               case 'p':
                  // ping
                  debugMsgStr("\r\nEnter short addr:");
                  serial_gets(str, 50, true);
                  addr = atoi(str);
                  macPing(PING_REQ_FRAME, addr);
                  break;
               case 's':
                  // Send data stream
                  debugMsgStr("\r\nStream mode to addr:");
                  serial_gets(str,50,true);
                  addr = atoi(str);
                  streamMode = 1;
                  break;
               case 'c':
                  // change coordinator channel
                  if (NODETYPE == COORD)
                  {
                     debugMsgStr("\r\nEnter new chan");
                     serial_gets(str, 50, true);
                     ch = atoi(str);
                     debugMsgStr("\r\nEnter new PANID");
                     serial_gets(str, 50, true);

                     // Re-do the init stuff.
                     macInit(ch);
                     macStartCoord();
                     debugMsgStr("\r\nStartup, I am coordinator.\r\n");
                     macConfig.associated = true;

                     // Set PANID
                     addr = atoi(str);
                     if (addr)
                     {
                        radioSetPanId(addr);
                        macConfig.panId = addr;
                     }
                  }
                  break;
               case 'r':
                  // Request reading from end node
                  if (NODETYPE == COORD && APP == SENSOR)
                  {
                     // get address and time
                     debugMsgStr("\r\nRequest data from node:");
                     serial_gets(str,50,true);
                     addr = atoi(str);
                     u16 time;
                     debugMsgStr("\r\nReport time (100mS intervals):");
                     serial_gets(str,50,true);
                     time = atoi(str);
                     // Grab pending frames, so we don't trigger sending req frame
                     macTask();
                     sensorRequestReading(addr, time);
                  }
                  break;
               case 'C':
                  // calibrate an end node
                  if (NODETYPE == COORD && APP == SENSOR)
                  {
                     // get address of node
                     debugMsgStr("\r\nCal which node:");
                     serial_gets(str,50,true);
                     addr = atoi(str);

                     // Get cal info from node
                     sensorRequestCalInfo(addr);
                  }
                  break;
               case 'S':
                  // sleep
                  if (NODETYPE != COORD)
                  {
                     for(;;)
                     {
                        u8 count;
                        nodeSleep(20);
                        macPing(PING_REQ_FRAME, DEFAULT_COORD_ADDR);
                        // Get the ping response before going back to sleep
                        delay_us(6000);
                        for (count=0;count<100;count++)
                           macTask();
                        // Send out the response string
                        delay_us(1000);
                     }
                  }
                  break;
               case 'n':
                  // Name a node
                  if (NODETYPE == COORD && APP == SENSOR)
                  {
                     debugMsgStr("\r\nName which node:");
                     serial_gets(str,50,true);
                     addr = atoi(str);
                     debugMsgStr("\r\nEnter name:");
                     serial_gets(str,50,true);
                     sensorSendSetNodeName(addr, str);
                  }
                  break;
               case 'w':
                  // wake an end node
                  if (NODETYPE == COORD)
                  {
                     debugMsgStr("\r\nWake which node:");
                     serial_gets(str,50,true);
                     addr = atoi(str);
                     // Must process any rx'd packets before running macWake...
                     macTask();
                     macWakeChildNode(addr);
                  }
                  break;
               case 'P':
                  // Pause serial display
                  debugMsgStr("\r\nPaused, press 'P' to unpause");
                  serial_toggle_pause();
                  break;
               case 'A':
                  // Do something to all nodes
                  if (NODETYPE == COORD)
                  {
                     debugMsgStr("\r\nAll nodes - (r)eading, (p)ping:");
                     serial_gets(str,50,true);
                     // Do the function
                     if (*str == 'p')
                        allNodes(PING_ALL,0);
                     if (*str == 'r' && APP == SENSOR)
                     {
                        debugMsgStr("\r\nReport time (100mS intervals):");
                        serial_gets(str,50,true);
                        allNodes(REPORT_ALL, atoi(str));
                     }
                  }
                  break;
               default:
                  break;
            }
            // Delay a bit to allow for other messages (ping resp) to print.
            macSetAlarm(250,printPrompt);
         }
      }
   }

   // Process serial port interface with raven 3290P processor
   if (!DEBUG && SERIAL && PLATFORM == RAVEN)
      if (serial_rcv_frame())
      {
         u8 *p;
         static u8 seq;

         // Got a frame, process it
         switch (SerialFrame->cmd)
         {
         case CMD_PING_COORD:
            // Send a ping packet to coordinator
            macPing(PING_REQ_FRAME, DEFAULT_COORD_ADDR);
            break;
         case CMD_PING_GOOGLE:
            if (IPV6LOWPAN)
            {
               // GOOGLE: 2001:4860:B006::68;

               // Get pointer to IPv6 address buffer
               p = sixlowpan_hc01_ping_setup_ipglobal(seq++);
               // Put Google's address in the buffer
               memcpy(p, googleipAddr, 16);
               // Now ping Google
               sixlowpan_hc01_ping_send();
            }
            break;
         case CMD_PING_SERVER:
            if (IPV6LOWPAN )
            {
               // Get pointer to IPv6 address buffer
               p = sixlowpan_hc01_ping_setup_ipglobal(seq++);
               // Put IPSO server's address in the buffer
               memcpy(p, serveripAddr, 16);
               // Now ping Google
               sixlowpan_hc01_ping_send();
            }
            break;
         case CMD_PING_NODE:
            if (IPV6LOWPAN && APP == IPSO)
            {
               // Get pointer to IPv6 address buffer
               p = sixlowpan_hc01_ping_setup_ipglobal(seq++);
               // Put other end node's address in the buffer
               memcpy(p, destipAddr, 16);
               // Now ping Google
               sixlowpan_hc01_ping_send();
            }
            break;
         case CMD_LED:
            // Turn on remote LED
            if (IPV6LOWPAN && APP == IPSO)
            {
               // Get pointer to IPv6 address buffer
               p = sixlowpan_hc01_udp_setup_ipglobal();
               // Put other end node's address in the buffer
               memcpy(p, destipAddr, 16);

               // Setup ports
               sixlowpan_hc01_udp_setup_ports(UDP_PORT_RESPONSE, UDP_PORT_COMMANDS);
               memcpy( sixlowpan_hc01_udp_get_payloadptr(),
                     (seq++ & 1) ? "A1": "A0" ,2);
               sixlowpan_hc01_udp_set_payloadsize(2);
               sixlowpan_hc01_udp_send();
            }
            break;
         default:
            break;
         }
      }
}
#endif

/** @} */
