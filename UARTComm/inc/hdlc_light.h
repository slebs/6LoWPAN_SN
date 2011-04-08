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
 * @file hdlc_light.h
 *
 * @brief Header file for HDLC Layer
 *
 * $Id: hdlc_light.h,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-01-13
 */

#ifndef HDLC_LIGHT_H_
#define HDLC_LIGHT_H_


/**
 * @name Constant definitions
 * @{
 */
#define PROTO_RX                          (0x02)  ///< receive enabled
#define PROTO_TX                          (0x04)  ///< transmit enabled
#define PROTO_TX_ON_RX                    (0x08)  ///< transmit auto enabled on receive
#define PROTO_NO_PROTOCOL                 (0xFF)
/** @} */

/**
 * @name Type definitions
 * @{
 */
typedef int16_t  (* tGetCFN)(void);
typedef uint8_t  (* tIsCFN)(void);
typedef uint8_t (* tPutCFN)(uint8_t c);
typedef uint8_t  (* tFlushFN)(void);
typedef uint8_t  (* tPacketFN)(uint8_t* pData, uint16_t u16Len);

typedef uint8_t tbool;

#define  HDLC_DEBUG_ENABLE          (0)

/** @} */

/*
 * Prototypes
 */
void protocol_init(void);
void protocol_exit(void);
uint8_t protocol_add(uint8_t u8Options, tGetCFN pGetC, tIsCFN pIsC, tPutCFN pPutC, tFlushFN pFlush, tPacketFN pPacket);
tbool protocol_remove(uint8_t u8Instance);
tbool protocol_set_buffer(uint8_t u8Instance, uint8_t* pBuffer, uint16_t u16Len);
void protocol_send(uint8_t u8Instance, uint8_t* pData, uint16_t u16Len);
void protocol_receive(uint8_t u8Instance);

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif /* HDLC_LIGHT_H_ */
