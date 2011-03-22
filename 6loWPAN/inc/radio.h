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
  $Id: radio.h,v 1.1.2.1 2010/09/16 08:21:09 ele Exp $
*/

#ifndef RADIO_H
#define RADIO_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include "../inc/stdbool.h"
#include "../inc/rum_types.h"

/*============================ MACROS ========================================*/
#define RF230_REVA                              ( 1 )
#define RF230_REVB                              ( 2 )
#define SUPPORTED_MANUFACTURER_ID               ( 31 )
#ifdef SINGLE_CHIP
#define SUPPORTED_INTERRUPT_MASK                ( 0x5C )
#else
#define SUPPORTED_INTERRUPT_MASK                ( 0x0C )
#endif
#define RF2xx_MAX_TX_FRAME_LENGTH               ( 127 ) //!< 127 Byte PSDU.

#define TX_PWR_3DBM                             ( 0 )
#define TX_PWR_17_2DBM                          ( 15 )

#define BATTERY_MONITOR_HIGHEST_VOLTAGE         ( 15 )
#define BATTERY_MONITOR_VOLTAGE_UNDER_THRESHOLD ( 0 )
#define BATTERY_MONITOR_HIGH_VOLTAGE            ( 1 )
#define BATTERY_MONITOR_LOW_VOLTAGE             ( 0 )

#define FTN_CALIBRATION_DONE                    ( 0 )
#define PLL_DCU_CALIBRATION_DONE                ( 0 )
#define PLL_CF_CALIBRATION_DONE                 ( 0 )
/*============================ TYPEDEFS ======================================*/

/*! \brief  This macro defines the start value for the RADIO_* status constants.
 *
 *          It was chosen to have this macro so that the user can define where
 *          the status returned from the TAT starts. This can be useful in a
 *          system where numerous drivers are used, and some range of status codes
 *          are occupied.
 *
 *  \see radioStatusT
 *  \ingroup radio
 */
#define RADIO_STATUS_START_VALUE                  ( 0x40 )

/*! \brief  This enumeration defines the possible return values for the TAT API
 *          functions.
 *
 *          These values are defined so that they should not collide with the
 *          return/status codes defined in the IEEE 802.15.4 standard.
 *
 *  \ingroup radio
 */
typedef enum{
    RADIO_SUCCESS = RADIO_STATUS_START_VALUE,  //!< The requested service was performed successfully.
    RADIO_UNSUPPORTED_DEVICE,         //!< The connected device is not an Atmel AT86RF230.
    RADIO_INVALID_ARGUMENT,           //!< One or more of the supplied function arguments are invalid.
    RADIO_TIMED_OUT,                  //!< The requested service timed out.
    RADIO_WRONG_STATE,                //!< The end-user tried to do an invalid state transition.
    RADIO_BUSY_STATE,                 //!< The radio transceiver is busy receiving or transmitting.
    RADIO_STATE_TRANSITION_FAILED,    //!< The requested state transition could not be completed.
    RADIO_CCA_IDLE,                   //!< Channel in idle. Ready to transmit a new frame.
    RADIO_CCA_BUSY,                   //!< Channel busy.
    RADIO_TRX_BUSY,                   //!< Transceiver is busy receiving or transmitting data.
    RADIO_BAT_LOW,                    //!< Measured battery voltage is lower than voltage threshold.
    RADIO_BAT_OK,                     //!< Measured battery voltage is above the voltage threshold.
    RADIO_CRC_FAILED,                 //!< The CRC failed for the actual frame.
    RADIO_CHANNEL_ACCESS_FAILURE,     //!< The channel access failed during the auto mode.
    RADIO_NO_ACK,                     //!< No acknowledge frame was received.
}radio_status_t;

/// @name Transaction status codes
/// @{
#define TRAC_SUCCESS                0
#define TRAC_SUCCESS_DATA_PENDING   1
#define TRAC_SUCCESS_WAIT_FOR_ACK   2
#define TRAC_CHANNEL_ACCESS_FAILURE 3
#define TRAC_NO_ACK                 5
#define TRAC_INVALID                7
/// @}


/*! \brief  This enumeration defines the possible modes available for the
 *          Clear Channel Assessment algorithm.
 *
 *          These constants are extracted from the datasheet.
 *
 *  \ingroup radio
 */
/*
typedef enum {
    CCA_ED                    = 0,    //!< Use energy detection above threshold mode.
    CCA_CARRIER_SENSE         = 1,    //!< Use carrier sense mode.
    CCA_CARRIER_SENSE_WITH_ED = 2     //!< Use a combination of both energy detection and carrier sense.
}radio_cca_mode_t;
*/

/*! \brief  This enumeration defines the possible CLKM speeds.
 *
 *          These constants are extracted from the RF230 datasheet.
 *
 *  \ingroup radio
 */
typedef enum{
    CLKM_DISABLED      = 0,
    CLKM_1MHZ          = 1,
    CLKM_2MHZ          = 2,
    CLKM_4MHZ          = 3,
    CLKM_8MHZ          = 4,
    CLKM_16MHZ         = 5
}radio_clkm_speed_t;

typedef void (*radio_rx_callback) (u16 data);
/*============================ PROTOTYPES ====================================*/
radio_status_t radioInit(bool calRcOsc);

u8             radioGetPartnum(void);

u8             radioGetSavedRssiValue(void);
u8             radioGetSavedLqiValue(void);
u8             radioGetOperatingChannel(void);
radio_status_t radioSetOperatingChannel(u8 channel);
u8             radioGetTxPowerLevel(void);
radio_status_t radioSetTxPowerLevel(u8 powerLevel);

u8             radioGetCcaMode(void);
u8             radioGetEdThreshold(void);
radio_status_t radioGetRssiValue(u8 *rssi);

u8             radioBatmonGetVoltageThreshold(void);
u8             radioBatmonGetVoltageRange(void);
radio_status_t radioBatmonConfigure(bool range, u8 voltageThreshold);
radio_status_t radioBatmonGetStatus(void);

u8             radioGetClockSpeed(void);
radio_status_t radioSetClockSpeed(bool direct, u8 clockSpeed);

u8             radioIsBusy(void);
u8             radioGetTrxState(void);
radio_status_t radioSetTrxState(u8 newState);
radio_status_t radioEnterSleepMode(void);
radio_status_t radioLeaveSleepMode(void);
void           radioResetStateMachine(void);
void           radioResetTrx(void);

void           radioUseAutoTxCrc(bool autoCrcOn);
radio_status_t radioSendData(u8 dataLength, u8 *data);

void           radioSetDeviceRole(bool iAmCoordinator);
void           radioSetPanId(u16 newPanId);
void           radioSetShortAddress(u16 newShortAddress);
void           radioSetExtendedAddress(u8 *extendedAddress);

void           radioSetup900(void);

u8             radioRandom(u8 bits);

//TODO added to get saved ed level
int8_t         radioGetSavedEDValue(void);

#endif
/*EOF*/
