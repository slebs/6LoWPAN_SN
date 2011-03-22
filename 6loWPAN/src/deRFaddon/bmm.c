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
 * @file bmm.c
 *
 * @brief Buffer Management Module
 *
 * This file implements the Buffer Management Module. This module provides methods
 * to allocate free memory. This is used as buffer pool for transmitting and
 * receiving frames. The number of allocated buffers can be set with TOTAL_NUMBER_OF_BUFS.
 *
 *
 * $Id: bmm.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2009-03-18
 */

/* === Includes ============================================================= */

#include <stdint.h>

#include "../../inc/deRFaddon/bmm.h"

#if BMM_DEBUG_ENABLE
#include "../../inc/deRFaddon/uart.h"
#endif // BMM_DEBUG_ENABLE

/* === Macros =============================================================== */

/* === Globals ============================================================== */

/**
 * @brief Common Buffer pool holding the buffer user area
 *
 */
static uint8_t buf_pool[(TOTAL_NUMBER_OF_BUFS * BUFFER_SIZE)];

/**
 * @brief map that contains pointer to free buffers
 *
 */
static uint8_t* free_map[TOTAL_NUMBER_OF_BUFS];

#ifdef DEBUG_BUFFER
uint8_t counter;
#if BMM_DEBUG_ENABLE
uint32_t id_counter;
typedef struct
{
   uint8_t* pFrame;
   uint32_t id;
} index_map_t;
index_map_t index_map[TOTAL_NUMBER_OF_BUFS];
#endif // BMM_DEBUG_ENABLE
#endif // DEBUG_BUFFER

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
 * @brief Initialize buffer pool. Must called inside main initialization
 * routine (before first frame transmission starts).
 *
 */
void bmm_buffer_init(void)
{
   uint8_t index = 0;
   for(; index < TOTAL_NUMBER_OF_BUFS; index++)
   {
      free_map[index] = buf_pool + (sizeof(buffer_t) * index);
   }
#if DEBUG_BUFFER
   counter = TOTAL_NUMBER_OF_BUFS;
#if BMM_DEBUG_ENABLE
   id_counter = 0;
   uint8_t i = 0;
   for(; i < TOTAL_NUMBER_OF_BUFS; i++)
   {
      index_map[i].pFrame = NULL;
      index_map[i].id = 0;
   }
#endif // BMM_DEBUG_ENABLE
#endif
}

/**
 * @brief Allocates new buffer.
 *
 * Allocates buffer if available. If there is no free buffer, NULL is returned.
 *
 * @return pointer to free buffer area, or NULL if no free buffer is available
 */
uint8_t* bmm_buffer_alloc()
{
   uint8_t index = 0;
   uint8_t* pFreeSpace = NULL;

   AVR_ENTER_CRITICAL_REGION();
   for(; index < TOTAL_NUMBER_OF_BUFS; index++)
   {
      if(free_map[index] != NULL)
      {
         pFreeSpace = free_map[index];
         free_map[index] = NULL;
#if DEBUG_BUFFER
         counter--;
#if BMM_DEBUG_ENABLE
         id_counter++; // count up ID
         if(id_counter > 255)
         {
            id_counter = 0;
         }
         uint8_t i = 0;
         for(; i < TOTAL_NUMBER_OF_BUFS; i++)
         {
            if(index_map[i].pFrame == NULL)
            {
               index_map[i].pFrame = pFreeSpace;
               index_map[i].id = id_counter;
               break;
            }
         }
         UART_PRINT(" b_a (%u - %u)\n", counter, id_counter);
#endif // BMM_DEBUG_ENABLE
#endif
         break;
      }
   }
   AVR_LEAVE_CRITICAL_REGION();

   return pFreeSpace;
}

/**
 * @brief Free's allocated buffer.
 *
 * Free allocated buffer area by simply add to free buffer list
 *
 * @param buf pointer to buffer area which should be freed
 */
void bmm_buffer_free(uint8_t* buf)
{
   uint8_t index = 0;

   AVR_ENTER_CRITICAL_REGION();
   for(; index < TOTAL_NUMBER_OF_BUFS; index++)
   {
      if(free_map[index] == NULL)
      {
         free_map[index] = buf;
         break;
      }
   }

#if DEBUG_BUFFER
   counter++;
#if BMM_DEBUG_ENABLE
   uint32_t temp_id = 0;
   uint8_t i = 0;
   for(; i < TOTAL_NUMBER_OF_BUFS; i++)
   {
      if(index_map[i].pFrame == buf)
      {
         index_map[i].pFrame = NULL;
         temp_id = index_map[i].id;
         index_map[i].id = 0;
         break;
      }
   }
   UART_PRINT(" b_f (%u - %u)\n", counter, temp_id);
#endif // BMM_DEBUG_ENABLE
#endif

   AVR_LEAVE_CRITICAL_REGION();
}

/**
 * @brief Returns number of free buffers.
 *
 * The number of free buffers is returned if DEBUG_BUFFER is true.
 *
 * @return Number of free buffers (DEBUG_BUFFER == true). Return 0 if DEBUG_BUFFER == false.
 */
uint8_t number_of_free_buffers(void)
{
#if DEBUG_BUFFER
   return counter;
#endif
   return 0;
}





/* EOF */

