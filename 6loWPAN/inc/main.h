/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      This is the main file for the Raven LCD application. Contains binary
 *      command definitions.
 *
 * \par Application note:
 *      User's Guide to be published.
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Name: deRFdevelopmentKitV1-0-4 $
 * $Revision: 1.1.2.1 $
 * $RCSfile: main.h,v $
 * $Date: 2010/09/16 08:21:09 $  \n
 * $Id: main.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
 ******************************************************************************/
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

#ifndef __MAIN_H__
#define __MAIN_H__

/// @name Logical defines
/// @{
#define BOOL    char
#define FALSE   0
#define TRUE    (!false)

/** This macro will protect any subsequent code from interrupts. */
#define AVR_ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )

/** This macro ends a protected block of code and must always be used
    in conjunction with @ref AVR_ENTER_CRITICAL_REGION.  */
#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

#define PING_ATTEMPTS       (4)
/// @}

/// @name These are GUI to Radio Binary commands.
/// @{
#define NULL_CMD                      (0)
#define CMD_TEMP                     (0x80)
#define CMD_PING_COORD               (0x81)
#define CMD_PING_GOOGLE              (0x82)
#define CMD_PING_SERVER              (0x83)
#define CMD_PING_NODE                (0x84)
#define CMD_LED                      (0x85)
/// @}

/// @name These are the Radio to GUI binary commands.
/// @{
#define REPORT_PING                   (0xC0)
#define REPORT_PING_BEEP              (0xC1)
#define REPORT_TEXT_MSG               (0xC2)
#define REPORT_ASSOCIATED             (0xC3)
#define REPORT_LED                    (0xC4)
/// @}


#endif // __MAIN_H__
