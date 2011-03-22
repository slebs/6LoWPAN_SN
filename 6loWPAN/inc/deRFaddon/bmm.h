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
 * @file bmm.h
 *
 * @brief Header file for Buffer Management Module
 *
 * $Id: bmm.h,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-03-18
 */

#ifndef BMM_H_
#define BMM_H_

/* === Macros =============================================================== */

#include "../../inc/mac.h"

#define DEBUG_BUFFER                (1)   ///< Enable (1) or Disable (0) the Buffer management module debug

#define BMM_DEBUG_ENABLE            (0)   ///< Enable (1) or Disable (0) the Buffer management UART print mode (print some info's anytime a buffer is allocated or freed)

#define TOTAL_NUMBER_OF_BUFS        (15)  ///< Set the number of buffers that should be used (a number >5 is recommended)

#define BUFFER_SIZE                 (sizeof(buffer_t))   ///< Size of one buffer area (holds one frame)

/**
 * @brief structure that hold a frame (receiving and transmitting frames)
 */
typedef struct
{
   rx_frame_t data;  ///< structure that hold the frame (see mac.h)
} buffer_t;


void bmm_buffer_init(void);
uint8_t* bmm_buffer_alloc(void);
void bmm_buffer_free(uint8_t* buf);
uint8_t number_of_free_buffers(void);

#endif /* BMM_H_ */
