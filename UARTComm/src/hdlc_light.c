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
 * @file hdlc_light.c
 *
 * @brief HDLC Layer function.
 *
 * This file implements HDLC techniques. Here it adds frame tagging support.
 * A frame is tagged with an start and end sequence. To make sure this sequence
 * do not occur again in the frame all occurrences are replaced and tagged.
 * This is working on both directions - receive and transmit.
 *
 * $Id: hdlc_light.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-01-13
 */

/* === Includes ============================================================= */

#include <stdint.h>
#include <stdio.h>

#include "../inc/hdlc_light.h"
#include "../inc/uart.h"

/* === Macros =============================================================== */

#define PROTO_IGNORE_CRC

/*
 * Boolean definitions
 */
#define TFALSE                     (0x00)
#define TTRUE                      (0x01)

/*
 * Constant definitions
 */
#define PROTO_MAX_DEV                     2

// enable low level debugging
//#define PROTO_LOW_LEVEL_DBG
//#define PROTO_IGNORE_CRC

// define the communication flags
#define FR_END       (uint8_t)0xC0
#define FR_ESC       (uint8_t)0xDB
#define T_FR_END     (uint8_t)0xDC
#define T_FR_ESC     (uint8_t)0xDD
#define ASC_FLAG     0x01

// our send structure
// we use a flagging method with an END flag and an ESC flag
// each byte equal to the END or ESC flag within the data are
// flagged and a frame and a unflagged END is used as a sync
// byte, that is, if the receiver gets a unflagged END byte the
// receiver is flushed and everything received is discarded
// additionally we send a crc as the last two bytes of a frame
//
// a frame always looks like
//
//  END | tCommand | CRC low | CRC high | END
// the crc is the sum of all byte unflagged and without the crc itself
// the low byte is send as the complement + 1, and the high byte is send as
// is with a one added
// the receiver just looks at frame end for the last 2 byte and both should
// be equal to the high byte of its own calculated crc

/* === Globals ============================================================== */

/*
 * Local type definitions
 */
typedef struct stProtocol_s
{
  uint8_t        u8Escaped;
  uint16_t       u16Crc;
  uint8_t        u8Options;
  tGetCFN      pGetC;
  tIsCFN       pIsC;
  tPutCFN      pPutC;
  tFlushFN      pFlush;
  tPacketFN    pPacket;
  uint8_t*       pBuffer;
  uint16_t       u16BufferLen;
  uint16_t       u16BufferPos;

} tProtocol;

/*
 * Local variables
 */
static uint8_t bInit = TFALSE;
static tProtocol arDevices[PROTO_MAX_DEV];

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
  * @brief Initialize the PROTOCOL module
  *
  *
  * @return        void
  *
  * @author
  * @date          05.05.2006 11:23
  *
  */
void protocol_init(void)
{
   uint8_t  i;

   for (i = 0; i < PROTO_MAX_DEV; i++)
   {
      arDevices[i].u8Options = 0;
      arDevices[i].u8Escaped = 0;
      arDevices[i].pGetC = NULL;
      arDevices[i].pIsC = NULL;
      arDevices[i].pPutC = NULL;
      arDevices[i].pPacket = NULL;

      arDevices[i].pBuffer = NULL;
      arDevices[i].u16BufferLen = 0;
      arDevices[i].u16BufferPos = 0;
   }

   bInit = TTRUE;
}

/**
  * @brief Close the PROTOCOL module
  *
  *
  * @return        void
  *
  * @author
  * @date          05.05.2006 11:23
  *
  */
void protocol_exit(void)
{
   uint8_t  i;

   for (i = 0; i < PROTO_MAX_DEV; i++)
   {
      arDevices[i].u8Options = 0;
   }

   bInit = TFALSE;
}

/**
  * @brief Add a device to the PROTOCOL module
  *
  *
  * @param         u8Options  Options
  * @param         pGetC      the GetChar function of the device
  * @param         pIsC       the IsKey function of the device
  * @param         pPutC      the PutChar function of the device
  * @param         pFlush     the Flush function of the device
  * @param         pPacket    the pointer to buffer area (packet)
  *
  * @return        the instance handle on success, PROTO_NO_PROTOCOL else
  *
  * @author
  * @date          05.05.2006 11:23
  *
  */
uint8_t protocol_add(uint8_t u8Options, tGetCFN pGetC, tIsCFN pIsC, tPutCFN pPutC, tFlushFN pFlush, tPacketFN pPacket)
{
   uint8_t  i;

   // get a free device
   for (i = 0; i < PROTO_MAX_DEV; i++)
   {
      if (arDevices[i].u8Options == 0)
      {
         if (pGetC && pIsC && pPutC && pPacket && (u8Options != 0))
         {
            arDevices[i].u8Options = u8Options;
            arDevices[i].u8Escaped = 0;

            arDevices[i].pBuffer = NULL;
            arDevices[i].u16BufferLen = 0;
            arDevices[i].u16BufferPos = 0;

            arDevices[i].pGetC = pGetC;
            arDevices[i].pIsC = pIsC;
            arDevices[i].pPutC = pPutC;
            arDevices[i].pFlush = pFlush;
            arDevices[i].pPacket = pPacket;
            return i;
         }
         break;
      }
   }
   return PROTO_NO_PROTOCOL;
}

/**
  * @brief Remove a device from the PROTOCOL module
  *
  *
  * @param         u8Instance the instance to remove
  * @return        tbool      TTRUE on succes, TFALSE else
  *
  * @author
  * @date          05.05.2006 11:23
  *
  */
tbool protocol_remove(uint8_t u8Instance)
{
   if (bInit && (u8Instance < PROTO_MAX_DEV))
   {
      arDevices[u8Instance].u8Options = 0;
      return TTRUE;
   }
   return TFALSE;
}

/**
  * @brief Set the receive buffer for a device
  *
  *
  * @param         u8Instance  the instance to set the buffer for
  * @param         pBuffer     the pointer to the buffer
  * @param         u16Len      the size of the buffer
  *
  * @return        tbool      TTRUE on success
  *
  * @author
  * @date          05.05.2006 11:23
  *
  */
tbool protocol_set_buffer(uint8_t u8Instance, uint8_t* pBuffer, uint16_t u16Len)
{
   if (bInit && (u8Instance < PROTO_MAX_DEV))
   {
      if (pBuffer && (u16Len > 0))
      {
         arDevices[u8Instance].pBuffer = pBuffer;
         arDevices[u8Instance].u16BufferLen = u16Len;
      }
      else
      {
         arDevices[u8Instance].pBuffer = NULL;
         arDevices[u8Instance].u16BufferLen = 0;
      }

      arDevices[u8Instance].u16BufferPos = 0;
      return TTRUE;
   }
   return TFALSE;
}

/**
  * @brief send a binary data packet - use escape technique and apply a crc
  *
  *
  * @param         u8Instance the protocol instance to send a packet for
  * @param         pData      pointer to the data buffer to send
  * @param         u16Len     length from packet
  *
  * @return        void
  *
  * @author
  * @date          05.05.2006 11:23
  *
  */
void protocol_send(uint8_t u8Instance, uint8_t* pData, uint16_t u16Len)
{
   if (bInit && (u8Instance < PROTO_MAX_DEV) && pData && (u16Len > 0))
   {
      if (arDevices[u8Instance].u8Options & PROTO_TX)
      {
         uint8_t c;
         tProtocol* pDev = &arDevices[u8Instance];
         uint16_t i = 0;
         uint16_t u16Crc = 0;

         // put an end before the packet
         pDev->pPutC(FR_END);
#if HDLC_DEBUG_ENABLE
         UART_PRINT("OUT[%d] ", u8Instance);
#endif
         while (i < u16Len)
         {
            c = pData[i++];
            u16Crc += c;
#if HDLC_DEBUG_ENABLE
            UART_PRINT("%02X " , c);
#endif
            switch (c)
            {
            case FR_ESC:
               pDev->pPutC(FR_ESC);
               pDev->pPutC(T_FR_ESC);
               break;
            case FR_END:
               pDev->pPutC(FR_ESC);
               pDev->pPutC(T_FR_END);
               break;
            default:
               pDev->pPutC(c);
               break;
            }
         }

         c = (~u16Crc + 1) & 0xFF;
         if (c == FR_ESC)
         {
            pDev->pPutC(FR_ESC);
            pDev->pPutC(T_FR_ESC);
         }
         else if (c == FR_END)
         {
            pDev->pPutC(FR_ESC);
            pDev->pPutC(T_FR_END);
         }
         else
         {
            pDev->pPutC(c);
         }

         c = ((~u16Crc + 1) >> 8)   & 0xFF;
         if (c == FR_ESC)
         {
            pDev->pPutC(FR_ESC);
            pDev->pPutC(T_FR_ESC);
         }
         else if (c == FR_END)
         {
            pDev->pPutC(FR_ESC);
            pDev->pPutC(T_FR_END);
         }
         else
         {
            pDev->pPutC(c);
         }
#if HDLC_DEBUG_ENABLE
         UART_PRINT(("\n"));
#endif
         // tie off the packet
         pDev->pPutC(FR_END);

         if (pDev->pFlush)
         {
            pDev->pFlush();
         }
      }
   }
}

/**
  * @brief receive a binary data packet - use escape technique and check the crc
  *
  *
  * @param         u8Instance      the the device instance
  * @return        void
  *
  * @author
  * @date          05.05.2006 11:23
  *
  */
void protocol_receive(uint8_t u8Instance)
{
   if (bInit && (u8Instance < PROTO_MAX_DEV))
   {
     tProtocol* pDev = &arDevices[u8Instance];
      if ((pDev->u8Options & PROTO_RX) && pDev->pIsC())
      {
         uint8_t c;
         if (pDev->u8Options & PROTO_TX_ON_RX)
         {
            // enable TX
            pDev->u8Options |= PROTO_TX;
         }
#if HDLC_DEBUG_ENABLE
         UART_PRINT("IN[%d] ", u8Instance);
#endif
         do
         {
            c = pDev->pGetC();
#if HDLC_DEBUG_ENABLE
            UART_PRINT("%02X " , c);
#endif
            switch (c)
            {
            case FR_END:
               if (pDev->u8Escaped)
               {
                  pDev->u16BufferPos = 0;
                  pDev->u16Crc = 0;
               }
               else
               {
                  if (pDev->u16BufferPos >= 2)
                  {
                     tbool bCRCok = TFALSE;
                     // Checksum bytes are added to the checksum pDev->u16Crc - substract them here
                     pDev->u16Crc -= pDev->pBuffer[pDev->u16BufferPos - 1];
                     pDev->u16Crc -= pDev->pBuffer[pDev->u16BufferPos - 2];
                     if ((((~(pDev->u16Crc) + 1) & 0xFF) == pDev->pBuffer[pDev->u16BufferPos - 2]) &&
                        ((((~(pDev->u16Crc) + 1) >> 8) & 0xFF) == pDev->pBuffer[pDev->u16BufferPos - 1]))
                     {
                        bCRCok = TTRUE;
                     }
#ifdef PROTO_IGNORE_CRC
                     if (!bCRCok)
                     {
#if HDLC_DEBUG_ENABLE
                        UART_PRINT((("PROTO: ignoring invalid CRC")));
#endif
                     }
                     if (1)   // handle the packet even for invalid CRC
#else
                     if (bCRCok)
#endif
                     {
                        if (pDev->pPacket)
                        {
                           pDev->pPacket(&pDev->pBuffer[0], (uint16_t)(pDev->u16BufferPos - 2));
                        }
                        else
                        {
#if HDLC_DEBUG_ENABLE
                           UART_PRINT((("PROTO: skip packet - no packet handler")));
#endif
                        }
                     }
                     else
                     {
#if HDLC_DEBUG_ENABLE
                        UART_PRINT((("PROTO: CRC error")));
#endif
                     }
                  }
                  pDev->u16BufferPos = 0;
                  pDev->u16Crc = 0;
               }
               pDev->u8Escaped &= ~ASC_FLAG;
               return;
            case FR_ESC:
               pDev->u8Escaped |= ASC_FLAG;
               return;
            }

            if (pDev->u8Escaped & ASC_FLAG)
            {
               // translate the 2 byte escape sequence back to original char
               pDev->u8Escaped &= ~ASC_FLAG;

               switch (c)
               {
               case T_FR_ESC: c = FR_ESC; break;
               case T_FR_END: c = FR_END; break;
               default: break;
               }
            }

            // we reach here with every byte for the buffer
            // BUG: checksum bytes are added but should not be
            if (pDev->pBuffer && (pDev->u16BufferPos < pDev->u16BufferLen))
            {
               pDev->pBuffer[pDev->u16BufferPos++] = c;
               pDev->u16Crc += c;
            }
         }
         while(pDev->pIsC());
#if HDLC_DEBUG_ENABLE
         UART_PRINT(("\n"));
#endif
      }
   }
}

/* EOF */
