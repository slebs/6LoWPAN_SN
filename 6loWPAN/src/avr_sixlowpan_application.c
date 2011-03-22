
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
 * $Id: avr_sixlowpan_application.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 */
/**
 * \file
 *         AVR 6LoWPAN / IP Stack Application Example
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../inc/avr_sixlowpan.h"
#include "../inc/avr_timer.h"
#include "../inc/mac.h"
#include "../inc/mac_data.h"
#include "../inc/mac_event.h"
#include "../inc/hal.h"
#include "../inc/system.h"
#include "../inc/radio.h"
#include "../inc/sensors.h"

//#include "sixlowpan_wake.h"
#include "../inc/bootloader.h"
#include <avr/boot.h>
#include "../inc/serial.h"
#include "../inc/deRFaddon/deRFapplication.h"

//include by Simon
#include "../inc/sensn/app_interface.h"

// No bootloader for SPITFIRE platform
#if PLATFORM == SPITFIRE
#define boot_program_page(address, spmBuf)
#define boot_copy_program(address)
#endif


#if (IPV6LOWPAN == 1) || defined(DOXYGEN)

/**
 * @addtogroup app
 * @{
 * @defgroup avr6lowpan_example AVR 6LoWPAN Example Application
 * @{
 *
 * Example 6LoWPAN Application. Provides three different examples: the
 * IPSO application which provides remote node control, the sensor example
 * which provides sensor reporting, and a TFTP file transfer example which
 * can be used to load new code in the AVR.
 */

/** Store destination address of commands */
uint8_t destipAddr[16] = {0x20, 0x01, 0x49, 0x78, 0x01, 0xE1, 0x00, 0x00, 0x02, 0x1C, 0x23, 0xFF, 0XFE, 0x2B, 0xBD, 0x6C};

/** Store destination address of server */
uint8_t googleipAddr[16] = {0x20, 0x01, 0x48, 0x60, 0xB0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0X00, 0x00, 0x00, 0x68};
uint8_t serveripAddr[16] = {0x20, 0x01, 0x05, 0xC0, 0x10, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0X00, 0x00, 0x11, 0x4D};
//uint8_t backupipAddr[16] = {0x2A, 0x01, 0x02, 0x40, 0xFE, 0x00, 0x00, 0x73, 0x00, 0x00, 0x00, 0x00, 0X00, 0x00, 0x00, 0x02};

/* Command to send remote node */
#define COMMAND_PING 0
#define COMMAND_UDP 1

static char remoteCommand[50];
static uint8_t remoteCommandLen = 0;
static uint8_t remoteAction = COMMAND_UDP; //TODO original: remoteAction = COMMAND_PING;
static uint8_t pingSequence = 0;


/* Response to Command*/
static char remoteCommandResponse[25];
static uint8_t remoteCommandResponseLen = 0;
static uint32_t remoteCommandResponseTime = 0;
static uint32_t PingSendTime = 0;

#define UDP_PORT_TFTP     69
#define UDP_PORT_COMMANDS 0xF0B0
#define UDP_PORT_RESPONSE 0xF0B1
#define UDP_PORT_ADMIN    0xF0B2
#define UDP_PORT_SENSOR   0xF0B3
#define UDP_PORT_DSKDEMO  0xF0B4

#if APP == IPSO
static char ledStatus = '0';
static double temperature = 22.3;
static double humidity = 9.5;
static double light = 25;
static uint8_t toggle;
#endif

#if APP == DSKIPDEMO
static uint8_t dskDestAddr[16];
static uint16_t dskDestPort;
#endif

uint8_t simulateButton = 0;

#if APP == SENSOR || defined(DOXYGEN)
static uint8_t * udpPayload;
static uint8_t   udpPayloadLen;
static uint8_t   udpPayloadMaxLen;
#endif

void sixlowpan_tftpData(uint8_t * data, uint8_t len);

/**
 * \brief This is the user callback when a ping response is received
 *
 * This function allows the user to know that a ping response was
 * received after sending out a ping request. Can be used to verify
 * node connectivity.
 *
 */
void sixlowpan_ping_usercall(uint8_t sequence)
{
   remoteCommandResponseTime = macGetTime();

   uint32_t timeDiff;

   timeDiff = remoteCommandResponseTime - PingSendTime;

   remoteCommandResponseLen =
         sprintf(remoteCommandResponse, "Ping took %ld mS\n",
               timeDiff);

   // Alert the application
   appSixlowpanPingResponse();
}

/**
 * \brief This is the user callback when a UDP packet is received
 *
 * This function handles the incoming packet, then possibly responds
 * if a response should be sent.
 *
 */
uint8_t sixlowpan_udp_usercall(uint16_t sourceport, uint16_t destport,
      uint8_t * payload, uint8_t payloadlen,
      uint8_t payloadmax, ipbuf_t * ipbuf,
      ftData* rxFrame) // added to get access to originAddr
      {
   /* Commands to the sensor are accepted on port 0xF0B0 */
   if (destport == UDP_PORT_COMMANDS)
   {
#if APP==IPSO || defined (DOXYGEN)
      char response[40];
      unsigned char respindex = 0;
      uint8_t * origpayload = payload;

      while(payloadlen)
      {
         switch(*payload)
         {

         case 'T': /* Temperature */
            response[respindex++] = 'T';
            respindex += sprintf(&response[respindex], "%1.1f", temperature);
            break;

         case 'L': /* Light Sensor */
            response[respindex++] = 'L';
            respindex += sprintf(&response[respindex], "%1.1f", light);
            break;

         case 'H': /* Humidity */
            response[respindex++] = 'H';
            respindex += sprintf(&response[respindex], "%1.1f", humidity);
            break;

         case 'A': /* LED Status / Set */

            //If next character involves \r or \n this is a query
            if ( (*(payload + 1) == '\r') || (*(payload + 1) == '\n') )
            {
               response[respindex++] = 'A';
               response[respindex++] = ledStatus;
               break;

               //Otherwise this is a command
            }
            else
            {
               payload++;
               payloadlen--;

               //'1' means turn LED on
               if ( *payload == '1')
               {
                  ledStatus = '1';
                  if((PLATFORM==RAVEN) && SERIAL)
                     serial_send_frame(REPORT_LED, 1, (u8 *)"1");
                  else
                     LED_ON(2);
               }
               //'0' means turn LED off
               else if ( *payload == '0')
               {
                  ledStatus = '0';
                  if((PLATFORM==RAVEN) && SERIAL)
                     serial_send_frame(REPORT_LED, 1, (u8 *)"0");
                  else
                     LED_OFF(2);
               }
               //Unknown command!
               else
               {
                  response[respindex++] = 0xFF;
                  response[respindex++] = 'A';
                  response[respindex++] = *payload;
               }
            }
            break;

         case '\r':
         case '\n':

            response[respindex++] = *payload;
            break;

         default:

            /* Print 0xFF followed by unknown command */
            response[respindex++] = 0xFF;
            response[respindex++] = *payload;
         }
         payloadlen--;
         payload++;
      }

      memcpy(origpayload, response, respindex);
      return respindex;

      /* Responses to commands are accepted on port 0xF0B1 */
   }
   else if (destport == UDP_PORT_RESPONSE)
   {
      memcpy(remoteCommandResponse, payload, payloadlen);
      remoteCommandResponseLen = payloadlen;
      remoteCommandResponseTime = macGetTime();

      return 0;


      /* Administration Commands are accepted on port 0xF0B2 */
   }
   else if (destport == UDP_PORT_ADMIN)
   {
      uint8_t i, tempbyte;

      switch (*payload)
      {
      case 'S': /**** Set destination IP address for the server */
         //Check payload is long enough to contain a full IP address
         if (payloadlen < 40)
         {
            strcpy((char *)payload, "Length Error\r\n");
            return 14;
         }

         sscanf((char *)(payload + 1), "%x:%x:%x:%x:%x:%x:%x:%x\r\n", (unsigned int *)&serveripAddr[0], (unsigned int *)&serveripAddr[2], (unsigned int *)&serveripAddr[4],
               (unsigned int *)&serveripAddr[6], (unsigned int *)&serveripAddr[8], (unsigned int *)&serveripAddr[10],
               (unsigned int *)&serveripAddr[12], (unsigned int *)&serveripAddr[14]);

         /* Switch to proper byte order */
         for(i = 0; i < 16; i += 2)
         {
            tempbyte = serveripAddr[i + 1];
            serveripAddr[i+1] = serveripAddr[i];
            serveripAddr[i] = tempbyte;
         }

         break;

      case 'D': /**** Set destination IP address for button press */

         //Check if we are sending to 'server' address
         if (*(char *)(payload + 1) == 's')
         {
            memcpy(destipAddr, serveripAddr, 16);
            break;
         }

         //Check payload is long enough to contain a full IP address
         if (payloadlen < 40)
         {
            strcpy((char *)payload, "Length Error\r\n");
            return 14;
         }

         sscanf((char *)(payload + 1), "%x:%x:%x:%x:%x:%x:%x:%x\r\n", (unsigned int *)&destipAddr[0], (unsigned int *)&destipAddr[2], (unsigned int *)&destipAddr[4],
               (unsigned int *)&destipAddr[6], (unsigned int *)&destipAddr[8], (unsigned int *)&destipAddr[10],
               (unsigned int *)&destipAddr[12], (unsigned int *)&destipAddr[14]);

         /* Switch to proper byte order */
         for(i = 0; i < 16; i += 2)
         {
            tempbyte = destipAddr[i + 1];
            destipAddr[i+1] = destipAddr[i];
            destipAddr[i] = tempbyte;
         }

         break;

      case 'B': /**** Set action to perform on button press */

         switch( *(payload + 1) )
         {
         case 'S': /* Send this to node: */

            remoteCommandLen = payloadlen - 2;
            memcpy(remoteCommand, payload + 2, remoteCommandLen);

            /* If string ends with \n with no \r infront
             * of it, we put one there, as the user probably wanted one! */
            if ((*(payload + payloadlen - 1) == '\n') && (*(payload + payloadlen - 2) != '\r'))
            {
               *(remoteCommand + remoteCommandLen - 1) = '\r';
               *(remoteCommand + remoteCommandLen) = '\n';

               remoteCommandLen++;
            }
            remoteAction = COMMAND_UDP;
            break;

         case 'P': /* Ping this node */
            remoteAction = COMMAND_PING;
            break;

         default:
            strcpy((char *)payload, "Unknown Command\r\n");
            return 17;
            break;
         }

         break;

         case 'G': /**** Get response from last command sent to remote node */

            if (remoteCommandResponseLen == 0)
            {
               strcpy((char *)payload, "No response\r\n");
               return 13;
            }

            uint8_t timestampoffset = sprintf((char *)payload, "[%ld]  ", remoteCommandResponseTime);

            memcpy(payload + timestampoffset, remoteCommandResponse, remoteCommandResponseLen);
            return remoteCommandResponseLen + timestampoffset;

            break;

         case 'C': /**** Clear response from last commmand sent to remote node */

            remoteCommandResponseLen = 0;
            break;

         case 'H': /**** Hit button (simulate it) */
            simulateButton = 1;
            break;

         case 'w': /**** WAKE UP */
            if (!VLP)
            {
               macConfig.sleeping = false;
               strcpy((char *)payload, "awake\r\n");
               return 7;
            }
            else
            {
               strcpy((char *)payload, "VLP mode (wake disabled)\r\n");
               return 26;
            }


         default:
            strcpy((char *)payload, "Unknown Command\r\n");
            return 17;

      }

      strcpy((char *)payload, "OK\r\n");
      return 4;
#else
      strcpy((char *)payload, "IPSO app disabled\r\n");
      return 19;
#endif

   }
   /* SENSOR Application */
   else if (destport == UDP_PORT_SENSOR)
   {
#if APP == SENSOR || defined(DOXYGEN)
      /* Reset user payload, and call sensor receive */
      udpPayload = payload;
      udpPayloadLen = 0;
      udpPayloadMaxLen = payloadmax;
      sensorRcvPacket(payload);

      udpPayload = NULL;

      return udpPayloadLen;
#else // APP == IPSO
      if((PLATFORM==RAVEN) && SERIAL)
      {
         u8 index = 0;
         while(payload[index] != 0x0A)
            if(index++ > 10)
               break;
         payload[index] = 0;
         serial_send_frame(REPORT_TEXT_MSG, strlen((char *)payload)+1, (u8 *)payload);
         strcpy((char *)payload, "OK\r\n");
         return 4;
      }
      else
      {
         strcpy((char *)payload, "Disabled.\r\n");
         return 11;
      }
#endif

   }

   /* DSK001 Demo Application */
   else if (destport == UDP_PORT_DSKDEMO)
   {
#if APP==DSKIPDEMO || defined (DOXYGEN)

#if PLATFORM != DSK001
      strcpy((char *)payload, "WRONG PLATFORM\r\n");
      return 14;
#endif

      if (ipbuf->srcmode == SIXLOWPAN_IPHC_DAM_I)
      {
         memcpy(dskDestAddr, ipbuf->srcptr, 16);
         dskDestPort = sourceport;
         strcpy((char *)payload, "OK\r\n");
         return 4;
      }
      else
      {
         strcpy((char *)payload, "Error\r\n");
         return 7;
      }


#endif

   }
   /* TFTP Port */
   else if (destport == UDP_PORT_TFTP)
   {

      static uint16_t expectedBlock;
      uint8_t * tftpPayload = payload;

#if RUMSLEEP
      /* Sleeping enabled - take out of sleep mode if not VLP.
       * If VLP switch to a faster timeout */
      if (VLP)
      {
         SIXLOWPAN_PERIODIC_TIME = 2;
         SIXLOWPAN_PERIODIC_APP_TIME = 0;
      }
      else
      {
         macConfig.sleeping = false;
      }
#endif

      /* Only support write request in binary: Opcode = 2 */
      if ((*(tftpPayload + 0) == 0x00) &&
            (*(tftpPayload + 1) == 0x02))
      {
         //Point to filename
         tftpPayload += 2;

         //Find end of filename
         while (*tftpPayload)
            tftpPayload++;

         //Point to file transfer mode
         tftpPayload++;

         //Check we are using binary file transfer mode
         if ((*(tftpPayload + 0) == 'o') &&
               (*(tftpPayload + 1) == 'c') &&
               (*(tftpPayload + 2) == 't') &&
               (*(tftpPayload + 3) == 'e') &&
               (*(tftpPayload + 4) == 't') &&
               (*(tftpPayload + 5) == 0x00))
         {

            //Point to options
            tftpPayload += 6;

            //We only accept requests with a size of
            //64 bytes per data node!
            if ((*(tftpPayload + 0) == 'b') &&
                  (*(tftpPayload + 1) == 'l') &&
                  (*(tftpPayload + 2) == 'k') &&
                  (*(tftpPayload + 3) == 's') &&
                  (*(tftpPayload + 4) == 'i') &&
                  (*(tftpPayload + 5) == 'z') &&
                  (*(tftpPayload + 6) == 'e') &&
                  (*(tftpPayload + 7) == 0x00) &&
                  (*(tftpPayload + 8) == '6') &&
                  (*(tftpPayload + 9) == '4') &&
                  (*(tftpPayload + 10) == 0x00))
            {
               //Reset payload
               tftpPayload = payload;

               *tftpPayload++ = 0x00;
               *tftpPayload++ = 0x06;
               *tftpPayload++ = 'b';
               *tftpPayload++ = 'l';
               *tftpPayload++ = 'k';
               *tftpPayload++ = 's';
               *tftpPayload++ = 'i';
               *tftpPayload++ = 'z';
               *tftpPayload++ = 'e';
               *tftpPayload++ = 0x00;
               *tftpPayload++ = '6';
               *tftpPayload++ = '4';
               *tftpPayload++ = 0x00;

               //Reset this
               sixlowpan_tftpData(NULL, 0);

               expectedBlock = 1;

               return 13;
            }
         }
      }
      /* Data Packet: Opcode = 3 */
      else if ((*(tftpPayload + 0) == 0x00) &&
            (*(tftpPayload + 1) == 0x03))
      {
         uint16_t actualBlock;

         actualBlock =  *(tftpPayload + 3);
         actualBlock += (*(tftpPayload + 2) << 8);

         //Received the proper data block
         if (actualBlock == expectedBlock)
         {
            *(tftpPayload + 1) = 4; //Opcode 4 = ACK
            expectedBlock++;

            //Check if we are done
            sixlowpan_tftpData(tftpPayload + 4, payloadlen - 4);
            return 4;
         }
         //Our ACK must have got lost?
         else if (actualBlock < expectedBlock)
         {
            *(tftpPayload + 1) = 4; //Opcode 4 = ACK
            return 4;
         }
         //Block from the future... something bad happened
         else
         {
            *(tftpPayload + 1) = 5; //OpCode 5 = error
            *(tftpPayload + 2) = 0; //Error code = 0 (undefined)
            *(tftpPayload + 3) = 0;
            *(tftpPayload + 4) = 0; //No string sorry
            return 5;
         }
      }

      return 0;

   }
   //TODO added by Dresden Elektronik to implement own user application
   // End Node/Router Node Port address
   else if(destport == UDP_PORT_END_ROUTER)
   {
      process_endnode_udp_packet(payload, rxFrame->originAddr);
      return 0; //send nothing back
   }

   //TODO added by Dresden Elektronik to implement own user application
   // Coordinator Node Port address
   else if(destport == UDP_PORT_COORD)

   {
      process_coord_udp_packet(payload, rxFrame->originAddr);
      return 0; //send nothing back
   }

   // TODO Implemented by Simon to implement own behaviour on incoming Messages on specified UDP PORT
   else if(destport == UDP_PORT_SENSN_COORD)
     {
        process_coord_udp_packet_SN(payload, payloadlen, rxFrame->originAddr);
        return 0; //send nothing back
     }
   else if(destport == UDP_PORT_SENSN_END_ROUTER)
        {
           process_endnode_udp_packet_SN(payload, payloadlen, rxFrame->originAddr);
           return 0; //send nothing back
        }

   /* Unknown port */
   else
   {
      strcpy((char *)payload, "Unknown port\r\n");
      return 14;
   }

   return payloadlen;
      }

/**
 * \brief Processes incomming TFTP Data
 *
 * This function writes the data to the proper place in
 * flash memory.
 */
void sixlowpan_tftpData(uint8_t * data, uint8_t len)
{
   static uint32_t address;
   static uint16_t byteCnt;
   static uint8_t  spmBuf[SPM_PAGESIZE];
   uint8_t         lastPacket = 0;
   uint8_t         haveRoom = 0;


   if (data == NULL)
   {
      address = BOOTLOADER_INITIAL_ADDR;
      byteCnt = 0;
      return;
   }

   //Check we have room
   if ((address + SPM_PAGESIZE) > MAX_BOOTSIZE)
   {
      haveRoom = 0;
   }
   else
   {
      haveRoom = 1;
   }

   //Check for last packet
   if (len != 64)
   {
      lastPacket = 1;
   }

   //Copy data over
   while(len)
   {
      spmBuf[byteCnt] = *data;
      data++;
      len--;
      byteCnt++;
   }

   //Check for full
   if (((byteCnt >= SPM_PAGESIZE) || (lastPacket)) && haveRoom)
   {
      //        boot_program_page(address, spmBuf);
      address += byteCnt;
      byteCnt = 0;
   }


   //Copy over to main FLASH if we ended
   if (lastPacket)
   {
      //boot_verify();

      //Give time for us to ACK last packet
      //        bootloaderEndAddress = address;
      //        macSetAlarm(100, boot_copy_program);
   }

}

/**
 * \brief 6lowpan Periodic Function Call
 *
 * This function is called periodically, in this case it is used
 * to send back sensor data as needed. The function waits
 *
 * If no application (@ref APP) is defined, then this function does
 * nothing.
 */
void sixlowpan_application_periodic(void)
{
#if APP == SENSOR || defined(DOXYGEN)
   sendReading();
#elif APP == IPSO
   uint8_t * addr_ptr;
   uint8_t strlength;

   if((APP==IPSO) && (SENSOR_TYPE==SENSOR_RANDOM_IPSO))
   {
      // Just send faked random data for demo
      radioSetTrxState(RX_AACK_ON); // Needed for good random number
      temperature = 70 + ((s8)radioRandom(8)/1000.0);
      humidity = 29 + ((s8)radioRandom(8)/1000.00);
      light = 270 + ((s8)radioRandom(8)/1000.0);
      //        UNCOMMENT FOR CELCIUS
      //        temperature = 20 + ((s8)radioRandom(8)/1000.0);
      //        humidity = 40 + ((s8)radioRandom(8)/1000.00);
      //        light = 60 + ((s8)radioRandom(8)/1000.0);

      // Bound ranges to typical office environment.
      if(temperature>70.1)
         temperature = 70.1;
      else if(temperature<69.9)
         temperature = 69.9;

      if(humidity>29.11)
         humidity = 29.11;
      else if(humidity<28.95)
         humidity = 28.95;

      if(light>270.1)
         light = 270.1;
      else if(light<269.9)
         light = 269.9;
      //        UNCOMMENT FOR CELCIUS
      //        if(temperature>25.1)
      //            temperature = 25.1;
      //        else if(temperature<16.9)
      //            temperature = 16.9;
      //
      //        if(humidity>42.11)
      //            humidity = 42.11;
      //        else if(humidity<39.95)
      //            humidity = 39.95;
      //
      //        if(light>61.1)
      //            light = 61.1;
      //        else if(light<58.9)
      //            light = 58.9;
   }

   //    if(toggle)
   //    {
   toggle = 0;
   /* Send to IPSO Server */
   addr_ptr = sixlowpan_hc01_udp_setup_ipglobal();
   memcpy(addr_ptr, serveripAddr, 16);

   /* Send on port 0xF0B0 */
   sixlowpan_hc01_udp_setup_ports(UDP_PORT_RESPONSE, UDP_PORT_COMMANDS);

   strlength = sprintf((char *)sixlowpan_hc01_udp_get_payloadptr(), "T%1.1f\r\nH%1.2f\r\nL%1.1f\r\n", temperature, humidity, light);
   sixlowpan_hc01_udp_set_payloadsize(strlength);
   sixlowpan_hc01_udp_send();
   //    }
   //    else
   //    {
   //        toggle =1;
   //        /* Send to backup IPSO Server */
   //        addr_ptr = sixlowpan_hc01_udp_setup_ipglobal();
   //        memcpy(addr_ptr, backupipAddr, 16);
   //
   //        /* Send on port 0xF0B0 */
   //        sixlowpan_hc01_udp_setup_ports(UDP_PORT_RESPONSE, UDP_PORT_COMMANDS);
   //
   //        strlength = sprintf((char *)sixlowpan_hc01_udp_get_payloadptr(), "T%1.1f\r\nH%1.2f\r\nL%1.1f\r\n", temperature, humidity, light);
   //        sixlowpan_hc01_udp_set_payloadsize(strlength);
   //        sixlowpan_hc01_udp_send();
   //    }

#elif APP == DSKIPDEMO

   static uint8_t waitForOn = 0;
   uint8_t * addr_ptr;
   uint8_t strlength;
   static uint32_t oldAccelSq;

   /* If we have port, set this up */
   if (!dskDestPort)
      return;

   if (!waitForOn)
   {
      addr_ptr = sixlowpan_hc01_udp_setup_ipglobal();
      memcpy(addr_ptr, dskDestAddr, 16);

      sixlowpan_hc01_udp_setup_ports(UDP_PORT_DSKDEMO, dskDestPort);

      HAL_INIT_ADC();
      HAL_ACCEL_INIT();
      HAL_ACCEL_ON();

      //We use AVCC as AREF, not the 1.1V reference
      ADMUX = 0x40;

      //Wait for the accelerometer to turn on (takes a few ms)
      macSetAlarm(4, sixlowpan_application_periodic);
      waitForOn = 1;
   }
   else
   {
      waitForOn = 0;

      HAL_SELECT_ACCELZ();

      for(uint8_t i = 0; i < 2; i++)
      {
         HAL_SAMPLE_ADC();
         HAL_WAIT_ADC();
      }

      int16_t zaccel = HAL_READ_ADC() - 512;

      HAL_SELECT_ACCELY();
      for(uint8_t i = 0; i < 2; i++)
      {
         HAL_SAMPLE_ADC();
         HAL_WAIT_ADC();
      }

      int16_t yaccel = HAL_READ_ADC() - 512;

      HAL_SELECT_ACCELX();
      for(uint8_t i = 0; i < 2; i++)
      {
         HAL_SAMPLE_ADC();
         HAL_WAIT_ADC();
      }

      HAL_ACCEL_OFF();

      int16_t xaccel = HAL_READ_ADC() - 512;

      int32_t totalAccelSq;

      totalAccelSq = ((int32_t)zaccel*(int32_t)zaccel) + ((int32_t)yaccel*(int32_t)yaccel) + ((int32_t)xaccel*(int32_t)xaccel);

      SIXLOWPAN_PERIODIC_TIME = 4;


      //strlength = sprintf((char *)sixlowpan_hc01_udp_get_payloadptr(), "%ld\r\n", totalAccelSq);
      //strlength = sprintf((char *)sixlowpan_hc01_udp_get_payloadptr(), "X: %d Y:%d Z:%d\r\n", xaccel, yaccel, zaccel);

      if ((totalAccelSq < 15000) || (totalAccelSq > 25000))
      {
         strcpy((char *)sixlowpan_hc01_udp_get_payloadptr(), "Coin falling\r\n");
         strlength = 14;
      }
      else if (zaccel < -10)
      {
         strcpy((char *)sixlowpan_hc01_udp_get_payloadptr(), "HEADS\r\n");
         strlength = 7;
      }
      else if (zaccel > 10)
      {
         strcpy((char *)sixlowpan_hc01_udp_get_payloadptr(), "TAILS\r\n");
         strlength = 7;
      }
      else
      {
         strcpy((char *)sixlowpan_hc01_udp_get_payloadptr(), "EDGE\r\n");
         strlength = 6;
      }

      sixlowpan_hc01_udp_set_payloadsize(strlength);
      sixlowpan_hc01_udp_send();
   }

#endif
}

/**
 * \brief Initilize the 6lowpan application
 *
 */
void sixlowpan_application_init(void)
{

#if APP == DSKIPDEMO
   /* Ever 2 seconds check for disk activity */
   SIXLOWPAN_PERIODIC_TIME = 20 ;
   SIXLOWPAN_PERIODIC_APP_TIME = 1;
#elif ((APP == IPSO) && (SENSOR_TYPE != SENSOR_RANDOM_IPSO))
   SIXLOWPAN_PERIODIC_APP_TIME = 15;
#elif ((APP == IPSO) && (SENSOR_TYPE == SENSOR_RANDOM_IPSO))
   SIXLOWPAN_PERIODIC_APP_TIME = 1;
#endif

   return;
}

/**
 * \brief Called when button on physical device is pressed, or a command
 *        to simulate pressing the button is sent.
 *
 * This function performs the requested command. It is either to send
 * a UDP packet to a remote node, or ping a remote node.
 */
void sixlowpan_button(void)
{
   uint8_t * addr_ptr;

   /* Reset response */
   remoteCommandResponseLen = 0;

   if (remoteAction == COMMAND_UDP)
   {
      addr_ptr = sixlowpan_hc01_udp_setup_ipglobal();
   }
   else
   {
      addr_ptr = sixlowpan_hc01_ping_setup_ipglobal(++pingSequence);
   }

   memcpy(addr_ptr, destipAddr, 16);

   if (remoteAction == COMMAND_UDP)
   {
      sixlowpan_hc01_udp_setup_ports(UDP_PORT_RESPONSE, UDP_PORT_COMMANDS);
      memcpy( sixlowpan_hc01_udp_get_payloadptr(), remoteCommand, remoteCommandLen);
      sixlowpan_hc01_udp_set_payloadsize(remoteCommandLen);
      sixlowpan_hc01_udp_send();
   }
   else
   {
      PingSendTime = macGetTime();
      sixlowpan_hc01_ping_send();
   }


}

#if APP == SENSOR || defined(DOXYGEN)

/**
 * \brief Called to send back a UDP packet to the source.
 * \param len Length of data
 * \param data Pointer to data
 *
 * This function is used to respond to requests from the coordinator.
 * For example calibration setup, etc.
 */
void sixlowpan_sensorReturn(u8 len, u8 * data)
{
   if ((udpPayload) && (len <= udpPayloadMaxLen))
   {
      memcpy(udpPayload, data, len);
      udpPayloadLen = len;
   }
}

/**
 * \brief Called to send a UDP packet to a specific end-node.
 * \param addr Short address of node to send to
 * \param len Length of data
 * \param data Pointer to data
 *
 * This function is used to respond to send data from the coordinator
 * to end nodes / routers.
 */
void sixlowpan_sensorSend(u16 addr, u8 len, u8 * data)
{
   sixlowpan_hc01_udp_setup_iplocal(addr);
   sixlowpan_hc01_udp_setup_ports(UDP_PORT_SENSOR, UDP_PORT_SENSOR);
   memcpy( sixlowpan_hc01_udp_get_payloadptr(), data, len);
   sixlowpan_hc01_udp_set_payloadsize(len);
   sixlowpan_hc01_udp_send();
}

/**
 * \brief Called to send a UDP packet to the periodic address.
 * \param len Length of data
 * \param data Pointer to data
 *
 * This function is used to send a periodic sensor reading.
 */
void sixlowpan_sensorPerSend(u8 len, u8 * data)
{
   sixlowpan_hc01_udp_setup_iplocal(DEFAULT_COORD_ADDR);
   sixlowpan_hc01_udp_setup_ports(UDP_PORT_SENSOR, UDP_PORT_SENSOR);
   memcpy( sixlowpan_hc01_udp_get_payloadptr(), data, len);
   sixlowpan_hc01_udp_set_payloadsize(len);
   sixlowpan_hc01_udp_send();
}
#endif

/** @}
 *  @} */
#endif

