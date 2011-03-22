/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with th
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
  $Id: radio.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 */

/*============================ INCLUDE =======================================*/
#include <stdlib.h>
#include <stdio.h>
#include "../inc/mac.h"
#include "../inc/radio.h"
#include "../inc/mac_event.h"
#include "../inc/mac_scan.h"
#include "../inc/system.h"
#include "../inc/hal.h"

//TODO remove later
#include "../inc/deRFaddon/uart.h"
#include "../inc/deRFaddon/link_quality.h"

#include "../inc/deRFaddon/bmm.h"
/**
   @addtogroup radio
   @{

   The radio interface is a driver for the Atmel 802.15.4 radio chips.
   This code works with the AT86RF230, 'RF231, and 'RF212 chips.

   This driver code is fairly modular, and can be modified to be used
   without the RUM MAC layer that lies on top.

   The radio interface involves an SPI port and several digital I/O
   lines.  See the radio chip datasheet for details.
 */


/*============================ TYPEDEFS ======================================*/

/** @brief  This enumeration defines the necessary timing information for the
    AT86RF230 radio transceiver. All times are in microseconds.

    These constants are extracted from the datasheet.
 */
typedef enum{
   TIME_TO_ENTER_P_ON               = 510, ///< Transition time from VCC is applied to P_ON.
         TIME_P_ON_TO_TRX_OFF             = 510, ///< Transition time from P_ON to TRX_OFF.
         TIME_SLEEP_TO_TRX_OFF            = 880, ///< Transition time from SLEEP to TRX_OFF.
         TIME_RESET                       = 6,   ///< Time to hold the RST pin low during reset
         TIME_ED_MEASUREMENT              = 140, ///< Time it takes to do a ED measurement.
         TIME_CCA                         = 140, ///< Time it takes to do a CCA.
         TIME_PLL_LOCK                    = 150, ///< Maximum time it should take for the PLL to lock.
         TIME_FTN_TUNING                  = 25,  ///< Maximum time it should take to do the filter tuning.
         TIME_NOCLK_TO_WAKE               = 6,   ///< Transition time from *_NOCLK to being awake.
         TIME_CMD_FORCE_TRX_OFF           = 1,   ///< Time it takes to execute the FORCE_TRX_OFF command.
         TIME_TRX_OFF_TO_PLL_ACTIVE       = 180, ///< Transition time from TRX_OFF to: RX_ON, PLL_ON, TX_ARET_ON and RX_AACK_ON.
         TIME_STATE_TRANSITION_PLL_ACTIVE = 1,   ///< Transition time from PLL active state to another.
         TIME_RESET_TRX_OFF               = 37,  ///< Transition time from RESET to TRX_OFF
}radio_trx_timing_t;
/*============================ VARIABLES =====================================*/
u8 rx_mode;         ///< Flag: are we in RX mode?

static u8 rssi_val;

static u8 ed_val;  //TODO added to save ed level value - by Dresden Elektronik 03.02.10
static u8 lastLQI; //TODO added to save lqi level value - by Dresden Elektronik 22.03.10

/* Constant defines for the LQI calculation */
// ATMEGA128RFA1
/*
#define RSSI_BASE_VAL                   (-91)
#define ED_THRESHOLD                    (30)
#define ED_MAX_VAL                      (-RSSI_BASE_VAL - ED_THRESHOLD)
#define LQI_MAX                         (3)

//AT86RF230B
#define RSSI_BASE_VAL                   (-91)
#define ED_THRESHOLD                    (60)
#define ED_MAX                          (-RSSI_BASE_VAL - ED_THRESHOLD)
#define LQI_MAX                         (3)

//AT86RF231
#define RSSI_BASE_VAL                   (-90)
#define ED_THRESHOLD                    (60)
#define ED_MAX                          (-RSSI_BASE_VAL - ED_THRESHOLD)
#define LQI_MAX                         (3)

//AT86RF212
#define RSSI_BASE_VAL                   (-97)
#define ED_THRESHOLD                    (60)
#define ED_MAX                          (-RSSI_BASE_VAL - ED_THRESHOLD)
#define LQI_MAX                         (3)
*/

/*============================ PROTOTYPES ====================================*/
static bool isSleeping(void);
void radioRxStartEvent(u8 const frame_length);
void radioTrxend_event(void);


/**
   @brief  Initialize the radio chip.

   If the initialization is successful the radio transceiver will be
   in TRX_OFF state.

   @note This function must be called prior to any of the other
   functions in this file! Can be called from any transceiver state.

   @note There is a parameter (SR_CCA_ED_THRES) that sets the energy
   threshold for determining if a channel is clear before the radio
   can send a packet.  If the threshold is set too low, then in the
   presence of interference the radio may never get a chance to send a
   packet, and will be cut off from the network.  If the threshold is
   set too high, then each node may "yell over" the other nodes and
   disrupt other network connections.  The default value is 7, and a
   value of 2 was useful for testing the initial 2.4GHz version of the
   network.  Interference caused problems in the 900MHz band, and the
   threshold was raised back up to 7.  This parameter may be
   auto-tuned by application software in the future.

   @param cal_rc_osc If true, the radio's accurate clock is used to
   calibrate the CPU's internal RC oscillator.

   @retval RADIO_SUCCESS The radio transceiver was successfully
   initialized and put into the TRX_OFF state.

   @retval RADIO_UNSUPPORTED_DEVICE The connected device is not an
   Atmel AT86RF230 radio transceiver.

   @retval RADIO_TIMED_OUT The radio transceiver was not able to
   initialize and enter TRX_OFF state within the specified time.
 */
radio_status_t radioInit(bool cal_rc_osc)
{
   radio_status_t init_status = RADIO_SUCCESS;

   delay_us(TIME_TO_ENTER_P_ON);

   //Initialize Hardware Abstraction Layer.
   hal_init();

   radioResetTrx(); //Do HW reset of radio transeiver.

   //Force transition to TRX_OFF.
   hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);

   delay_us(TIME_P_ON_TO_TRX_OFF); //Wait for the transition to be complete.

   hal_register_write(RG_IRQ_MASK, SUPPORTED_INTERRUPT_MASK);

   // Set the CCA ED threshold really low
   hal_subregister_write(SR_CCA_ED_THRES, DEMO ? 2 : 7);

   // calibrate oscillator
   if (cal_rc_osc && SERIAL && (PLATFORM != RAVENUSB))
   {
      calibrate_rc_osc();
   }
   return init_status;
}

/**
   @brief Returns the radio part number.  The value returned is shown
   in the radio datasheet.  This value is valid any time after
   radioInit() is called.

   @return @ref RF230
   @return @ref RF231
   @return @ref RF212
 */
u8 radioGetPartnum(void)
{
   static u8 radio_part_number;

   if (!radio_part_number)
      radio_part_number = hal_register_read(RG_PART_NUM);
   return radio_part_number;
}

/**
   @brief Callback function, called when the radio receives an
   RX_START interrupt.  This function reads the radio's RSSI reading
   for the frame that is being received.

   @note This function is called from an interrupt - beware of
   re-entrancy problems.

   @param frame_length The length of the frame that is being received.
 */
void radioRxStartEvent(u8 const frame_length)
{
   // save away RSSI
   rssi_val =  hal_subregister_read( SR_RSSI );

   //UART_PRINT("  -> frame length: %d and RSSI: %d\r\n", frame_length, rssi_val);

   macConfig.busy = false;
}

/**
   @brief Retrieves the saved RSSI (Received Signal Strength
   Indication) value.  The value returned is the RSSI at the time of
   the RX_START interrupt.

   @return The RSSI value, which ranges from 0 to 28, and can be used
   to calculate the RSSI in dBm:

   Input Signal Strength (in dBm) = -90dBm + (3 * RSSI - 1)
 */
u8 radioGetSavedRssiValue(void)
{
   return rssi_val;
}

/**
   @brief Retrieves the saved LQI (Link Quality Indication) value.
   The value returned is the LQI for the last packet received.

   @return The LQI value, which ranges from 0 to 255.
 */
u8 radioGetSavedLqiValue(void)
{
   return lastLQI;
}

/**
   @brief Retrieves the saved ED (Energy Detection) value.
   The value returned is the ED for the last packet received.

   @return The ED value, which ranges from 0 to 84.
 */
//TODO added to get saved ed level value in dB
int8_t radioGetSavedEDValue(void)
{
   return (RSSI_BASE_VAL + ed_val);
}

/**
 * @brief Normalize LQI
 *
 * This function normalizes the LQI value based on the ED and
 * the originally appended LQI value.
 *
 * @param lqi Measured LQI
 * @param ed_value Read ED value
 *
 * @return The calculated LQI value
 */
static inline uint8_t normalize_lqi(uint8_t lqi, uint8_t ed_value)
{
    uint16_t link_quality;
    uint8_t lqi_star;

    if (ed_value > ED_MAX)
    {
        ed_value = ED_MAX;
    }
    else if (ed_value == 0)
    {
        ed_value = 1;
    }

    lqi_star = lqi >> 6;
    link_quality = (uint16_t)lqi_star * (uint16_t)ed_value * 255 / (ED_MAX * LQI_MAX);

    if (link_quality > 255)
    {
        return 255;
    }
    else
    {
        return (uint8_t)link_quality;
    }
}

/**
   @brief Callback function, called when the radio has received a
   TRX_END interrupt.
 */
#ifndef SINGLE_CHIP
void radioTrxEndEvent(void)
{
   volatile u8 status;

   //if (rx_mode)
   volatile u8 state = radioGetTrxState();
   if((state == BUSY_RX_AACK) || (state == RX_ON) || (state == BUSY_RX) || (state == RX_AACK_ON))
   {
      //TODO read ED level - by Dresden Elektronik
      ed_val = hal_register_read(RG_PHY_ED_LEVEL);

      /* radio has received frame, store it away */
      uint8_t* pFrame = hal_frame_read();

      if(pFrame != NULL)
      {
         // normalize incoming LQI value
         lastLQI = normalize_lqi(((rx_frame_t*)pFrame)->lqi, ed_val);
         ((rx_frame_t*)pFrame)->lqi = lastLQI;

#if (NODETYPE != ENDDEVICE)
         check_and_save_quality_values(pFrame);
#endif

         event_object_t event;
         event.event = 0;
         event.data = 0;

         // data_frame points to data area of rx_frame_t, mac_buffer_rx points to start of rx_frame_t
         uint8_t *data_frame = ((rx_frame_t*)pFrame)->data;

         // Figure out which kind of frame we have
         //u16 fcf = mac_buffer_rx[1] + mac_buffer_rx[2]*0x100;
         u16 fcf = data_frame[0] + data_frame[1]*0x100;

         // Dump any broadcast frames, except for Beacon/BeaconReq
         if (data_frame[5] == 0xff && // Broadcast src addr
               data_frame[6] == 0xff &&
               (fcf & 0xf0)) // Beacon and beacon request have this nibble zero
         {
            bmm_buffer_free(pFrame); // free buffer
            // Don't bother processing this broadcast frame
            return;
         }

         // Look at fcf
         switch (fcf)
         {
         case FCF_BEACONREQ:
            // Beacon request
            if ((NODETYPE == ROUTER && macConfig.associated) || NODETYPE == COORD)
            {
               event.event = MAC_EVENT_BEACON_REQ;
               event.data = pFrame;
            }
            break;
         case FCF_BEACON:
            // Beacon
            // Only report beacon frames if we're scanning
            if (macIsScanning())
            {
               event.event = MAC_EVENT_SCAN;
               event.data = pFrame;
            }
            break;
         case FCF_DATA:
            // Data
            event.event = MAC_EVENT_RX;
            event.data = pFrame;
            break;
         case FCF_ASSOC_REQ_DIRECT:
            // Association Request, direct
            if (NODETYPE != ENDDEVICE)
            {
               event.event = MAC_EVENT_ASSOCIATION_REQUEST;
               event.data = pFrame;
            }
            break;
         case FCF_ASSOC_RESP_DIRECT:
            // Association response, direct
            if (NODETYPE != COORD)
            {
               event.event = MAC_EVENT_ASSOCIATION_RESPONSE;
               event.data = pFrame;
            }
            break;
         case FCF_MAC_CMD:
            // MAC command frames
            switch (((ftRouting*)(((rx_frame_t*)pFrame)->data))->cmd)
            {
            case 1:
               // Association request
               if (NODETYPE != ENDDEVICE)
               {
                  event.event = MAC_EVENT_ASSOCIATION_REQUEST;
                  event.data = pFrame;
               }
               break;
            case 2:
               // Association response
               if (NODETYPE != COORD)
               {
                  event.event = MAC_EVENT_ASSOCIATION_RESPONSE;
                  event.data = pFrame;
               }
               break;
            case ROUTING_PACKET:
               // Routing packet
               if (NODETYPE != COORD)
               {
                  event.event = MAC_EVENT_ROUTE;
                  event.data = pFrame;
               }
               break;
            default:
               break;
            }
            default:
               break;
         }
         if (event.event == 0)
         {
            bmm_buffer_free(pFrame); // free buffer
         }
         else
         {
            mac_put_event(&event);
         }
      }
   }
   //if (!rx_mode)
   else if ((state == BUSY_TX) || (state == BUSY_TX_ARET) || (state == TX_ARET_ON) || (state == PLL_ON))
   {
      // Not busy any more
      macConfig.busy = false;

      // transmit mode, put end-of-transmit event in queue
      event_object_t event;
      event.event = 0;
      event.data = 0;

      status = hal_subregister_read(SR_TRAC_STATUS);

      switch(status)
      {
      case TRAC_SUCCESS:
      case TRAC_SUCCESS_DATA_PENDING:
         event.event = MAC_EVENT_ACK;
         break;
      case TRAC_CHANNEL_ACCESS_FAILURE:
         event.event = MAC_EVENT_ACCESS;
         break;
      case TRAC_NO_ACK:
         event.event = MAC_EVENT_NACK;
         break;
      case TRAC_SUCCESS_WAIT_FOR_ACK:
         // should only happen in RX mode
      case TRAC_INVALID:
         // should never happen here
      default:
         break;
      }
      if (event.event)
      {
         mac_put_event(&event);
      }

      // Put radio back into receive mode.
      radioSetTrxState(RX_AACK_ON);
   }
}
#endif // SINGLE_CHIP

#ifdef SINGLE_CHIP
void radioTrxEndEvent(void)
{
   volatile u8 status;

   // Not busy any more
   macConfig.busy = false;

   // transmit mode, put end-of-transmit event in queue
   event_object_t event;
   event.event = 0;
   event.data = 0;

   status = hal_subregister_read(SR_TRAC_STATUS);

   switch(status)
   {
   case TRAC_SUCCESS:
      //UART_PRINT("TRX END Trac Success\r\n");
      event.event = MAC_EVENT_ACK;
      break;
   case TRAC_SUCCESS_DATA_PENDING:
      //UART_PRINT("TRX END Trac Success Data Pending\r\n");
      event.event = MAC_EVENT_ACK;
      break;
   case TRAC_CHANNEL_ACCESS_FAILURE:
      //UART_PRINT("TRX END Trac Channel Access Failure\r\n");
      event.event = MAC_EVENT_ACCESS;
      break;
   case TRAC_NO_ACK:
      //UART_PRINT("TRX END Trac No Ack\r\n");
      event.event = MAC_EVENT_NACK;
      break;
   case TRAC_SUCCESS_WAIT_FOR_ACK:
      //UART_PRINT("TRX END Trac Success wait for Ack\r\n");
      // should only happen in RX mode
      break;
   case TRAC_INVALID:
      //UART_PRINT("TRX END Trac invalid\r\n");
      // should never happen here
      break;
   default:
      break;
   }

   if (event.event)
      mac_put_event(&event);

   // Put radio back into receive mode.
   radioSetTrxState(RX_AACK_ON);
}
void radioRxEndEvent(void)
{
   //TODO read ED level - by Dresden Elektronik
   ed_val = hal_register_read(RG_PHY_ED_LEVEL);

   /* radio has received frame, store it away */
   uint8_t* pFrame = hal_frame_read();

   if(pFrame != NULL)
   {
      lastLQI = normalize_lqi(((rx_frame_t*)pFrame)->lqi, ed_val);
      ((rx_frame_t*)pFrame)->lqi = lastLQI;

#if (NODETYPE != ENDDEVICE)
      check_and_save_quality_values(pFrame);
#endif

      event_object_t event;
      event.event = 0;
      event.data = 0;
      event.callback = 0;

      uint8_t *data_frame = ((rx_frame_t*)pFrame)->data;

      // Figure out which kind of frame we have
      u16 fcf = data_frame[0] + data_frame[1]*0x100;

      if (data_frame[5] == 0xff && // Broadcast src addr
            data_frame[6] == 0xff &&
            (fcf & 0xf0)) // Beacon and beacon request have this nibble zero
      {
         bmm_buffer_free(pFrame); // free buffer
         // Don't bother processing this broadcast frame
         return;
      }

      // Look at fcf
      switch (fcf)
      {
      case FCF_BEACONREQ:
         // Beacon request
         if ((NODETYPE == ROUTER && macConfig.associated) || NODETYPE == COORD)
         {
            event.event = MAC_EVENT_BEACON_REQ;
            event.data = pFrame;
         }
         break;
      case FCF_BEACON:
         // Beacon
         // Only report beacon frames if we're scanning
         if (macIsScanning())
         {
            event.event = MAC_EVENT_SCAN;
            event.data = pFrame;
         }
         break;
      case FCF_DATA:
         // Data
         event.event = MAC_EVENT_RX;
         event.data = pFrame;
         break;
      case FCF_ASSOC_REQ_DIRECT:
         // Association Request, direct
         if (NODETYPE != ENDDEVICE)
         {
            event.event = MAC_EVENT_ASSOCIATION_REQUEST;
            event.data = pFrame;
         }
         break;
      case FCF_ASSOC_RESP_DIRECT:
         // Association response, direct
         if (NODETYPE != COORD)
         {
            event.event = MAC_EVENT_ASSOCIATION_RESPONSE;
            event.data = pFrame;
         }
         break;
      case FCF_MAC_CMD:
         // MAC command frames
         switch (((ftRouting*)(((rx_frame_t*)pFrame)->data))->cmd)
         {
         case 1:
            // Association request
            if (NODETYPE != ENDDEVICE)
            {
               event.event = MAC_EVENT_ASSOCIATION_REQUEST;
               event.data = pFrame;
            }
            break;
         case 2:
            // Association response
            if (NODETYPE != COORD)
            {
               event.event = MAC_EVENT_ASSOCIATION_RESPONSE;
               event.data = pFrame;
            }
            break;
         case ROUTING_PACKET:
            // Routing packet
            if (NODETYPE != COORD)
            {
               event.event = MAC_EVENT_ROUTE;
               event.data = pFrame;
            }
            break;
         default:
            break;
         }
         default:
            break;
      }
      if (event.event == 0)
      {
         bmm_buffer_free(pFrame); // free buffer
      }
      else
      {
         mac_put_event(&event);
      }
   }
}
#endif



/*! \brief  This function will return the channel used by the radio transceiver.
 *
 *  \return Current channel, 11 to 26.
 *
 *  \ingroup radio
 */
u8 radioGetOperatingChannel(void)
{
   return hal_subregister_read(SR_CHANNEL);
}

/*! \brief This function will change the operating channel.
 *
 *  \param  channel New channel to operate on. Must be between 11 and 26.
 *
 *  \retval RADIO_SUCCESS New channel set.
 *  \retval RADIO_WRONG_STATE Transceiver is in a state where the channel cannot
 *                          be changed (SLEEP).
 *  \retval RADIO_INVALID_ARGUMENT Channel argument is out of bounds.
 *  \retval RADIO_TIMED_OUT The PLL did not lock within the specified time.
 *
 *  \ingroup radio
 */
radio_status_t radioSetOperatingChannel(u8 channel)
{
   /*Do function parameter and state check.*/

   if (CHINA_MODE)
   {
      u8 val;

      // Enable direct frequency setting
      // Ch1 = 780MHz
      // Ch2 = 782MHz
      // Ch3 = 784MHz
      // Ch4 = 786MHz
      hal_subregister_write(SR_CC_BAND, 4);
      if (channel < 1 || channel > 4)
         return RADIO_INVALID_ARGUMENT;

      val = 9 + 2*channel;
      hal_register_write(RG_CC_CTRL_0, val);
      return RADIO_SUCCESS;
   }
   else // Not china mode
   {
      if (((s8)channel < MIN_CHANNEL && MIN_CHANNEL) ||
            (channel > MAX_CHANNEL))
         return RADIO_INVALID_ARGUMENT;

      if (isSleeping() == true)
         return RADIO_WRONG_STATE;

      if (radioGetOperatingChannel() == channel)
         return RADIO_SUCCESS;

      /*Set new operating channel.*/
      hal_subregister_write(SR_CHANNEL, channel);

      //Read current state and wait for the PLL_LOCK interrupt if the
      //radio transceiver is in either RX_ON or PLL_ON.
      u8 trx_state = radioGetTrxState();

      if ((trx_state == RX_ON) ||
            (trx_state == PLL_ON))
         delay_us(TIME_PLL_LOCK);

      radio_status_t channel_set_status = RADIO_TIMED_OUT;

      //Check that the channel was set properly.
      if (radioGetOperatingChannel() == channel)
         channel_set_status = RADIO_SUCCESS;

      return channel_set_status;
   }

}


/*! \brief This function will change the output power level.
 *
 *  \param  power_level New output power level in the "TX power settings"
 *                      as defined in the radio transceiver's datasheet.
 *
 *  \retval RADIO_SUCCESS New output power set successfully.
 *  \retval RADIO_INVALID_ARGUMENT The supplied function argument is out of bounds.
 *  \retval RADIO_WRONG_STATE It is not possible to change the TX power when the
 *                          device is sleeping.
 *
 *  \ingroup radio
 */
radio_status_t radioSetTxPowerLevel(u8 power_level)
{

   /*Check function parameter and state.*/
   if (power_level > TX_PWR_17_2DBM)
      return RADIO_INVALID_ARGUMENT;

   if (isSleeping() == true)
      return RADIO_WRONG_STATE;

   /*Set new power level*/
   hal_subregister_write(SR_TX_PWR, power_level);

   return RADIO_SUCCESS;
}

#if (NODETYPE == COORD)
/*! \brief This function will read and return the output power level.
 *
 *  \return 0 to 15 Current output power in "TX power settings" as defined in
 *          the radio transceiver's datasheet
 *
 *  \ingroup radio
 */
u8 radioGetTxPowerLevel(void)
{
   return hal_subregister_read(SR_TX_PWR);
}


/*! \brief This function returns the current CCA mode used.
 *
 *  \return CCA mode currently used, 0 to 3.
 *
 *  \ingroup radio
 */
u8 radioGetCcaMode(void)
{
   return hal_subregister_read(SR_CCA_MODE);
}

/*! \brief This function returns the current ED threshold used by the CCA algorithm.
 *
 *  \return Current ED threshold, 0 to 15.
 *
 *  \ingroup radio
 */
u8 radioGetEdThreshold(void)
{
   return hal_subregister_read(SR_CCA_ED_THRES);
}

/*! \brief This function returns the current threshold volatge used by the
 *         battery monitor (BATMON_VTH).
 *
 *  \note This function can not be called from P_ON or SLEEP. This is ensured
 *        by reading the device state before calling this function.
 *
 *  \return Current threshold voltage, 0 to 15.
 *
 *  \ingroup radio
 */
u8 radioBatmonGetVoltageThreshold(void)
{
   return hal_subregister_read(SR_BATMON_VTH);
}

/*! \brief This function returns if high or low voltage range is used.
 *
 *  \note This function can not be called from P_ON or SLEEP. This is ensured
 *        by reading the device state before calling this function.
 *
 *  \retval 0 Low voltage range selected.
 *  \retval 1 High voltage range selected.
 *
 *  \ingroup radio
 */
u8 radioBatmonGetVoltageRange(void)
{
   return hal_subregister_read(SR_BATMON_HR);
}

/*! \brief This function is used to configure the battery monitor module
 *
 *  \param range True means high voltage range and false low voltage range.
 *  \param voltage_threshold The datasheet defines 16 voltage levels for both
 *                          low and high range.
 *  \retval RADIO_SUCCESS Battery monitor configured
 *  \retval RADIO_WRONG_STATE The device is sleeping.
 *  \retval RADIO_INVALID_ARGUMENT The voltage_threshold parameter is out of
 *                               bounds (Not within [0 - 15]).
 *  \ingroup radio
 */
radio_status_t radioBatmonConfigure(bool range, u8 voltage_threshold)
{

   /*Check function parameters and state.*/
   if (voltage_threshold > BATTERY_MONITOR_HIGHEST_VOLTAGE)
      return RADIO_INVALID_ARGUMENT;

   if (isSleeping() == true)
      return RADIO_WRONG_STATE;

   /*Write new voltage range and voltage level.*/
   if (range == true)
      hal_subregister_write(SR_BATMON_HR, BATTERY_MONITOR_HIGH_VOLTAGE);
   else
      hal_subregister_write(SR_BATMON_HR, BATTERY_MONITOR_LOW_VOLTAGE);

   hal_subregister_write(SR_BATMON_VTH, voltage_threshold);

   return RADIO_SUCCESS;
}

/*! \brief This function returns the status of the Battery Monitor module.
 *
 *  \note This function can not be called from P_ON or SLEEP. This is ensured
 *        by reading the device state before calling this function.
 *
 *  \retval RADIO_BAT_LOW Battery voltage is below the programmed threshold.
 *  \retval RADIO_BAT_OK Battery voltage is above the programmed threshold.
 *
 *  \ingroup radio
 */
radio_status_t radioBatmonGetStatus(void)
{

   radio_status_t batmon_status = RADIO_BAT_LOW;

   if (hal_subregister_read(SR_BATMON_OK) !=
         BATTERY_MONITOR_VOLTAGE_UNDER_THRESHOLD)
      batmon_status = RADIO_BAT_OK;

   return batmon_status;
}

/*! \brief This function returns the current clock setting for the CLKM pin.
 *
 *  \retval CLKM_DISABLED CLKM pin is disabled.
 *  \retval CLKM_1MHZ CLKM pin is prescaled to 1 MHz.
 *  \retval CLKM_2MHZ CLKM pin is prescaled to 2 MHz.
 *  \retval CLKM_4MHZ CLKM pin is prescaled to 4 MHz.
 *  \retval CLKM_8MHZ CLKM pin is prescaled to 8 MHz.
 *  \retval CLKM_16MHZ CLKM pin is not prescaled. Output is 16 MHz.
 *
 *  \ingroup radio
 */
u8 radioGetClockSpeed(void) // function not used yet
{
#ifdef SINGLE_CHIP
   return 0; // there is no clcok speed setting available on 128rfa1
#else
   return hal_subregister_read(SR_CLKM_CTRL);
#endif
}
#endif // if (COORD)

/*! \brief This function returns the Received Signal Strength Indication.
 *
 *  \note This function should only be called from the: RX_ON and BUSY_RX. This
 *        can be ensured by reading the current state of the radio transceiver
 *        before executing this function!
 *  \param rssi Pointer to memory location where RSSI value should be written.
 *  \retval RADIO_SUCCESS The RSSI measurement was successful.
 *  \retval RADIO_WRONG_STATE The radio transceiver is not in RX_ON or BUSY_RX.
 *
 *  \ingroup radio
 */
radio_status_t radioGetRssiValue(u8 *rssi)
{

   u8 current_state = radioGetTrxState();
   radio_status_t retval = RADIO_WRONG_STATE;

   /*The RSSI measurement should only be done in RX_ON or BUSY_RX.*/
   if ((current_state == RX_ON) ||
         (current_state == BUSY_RX))
   {
      *rssi = hal_subregister_read(SR_RSSI);
      retval = RADIO_SUCCESS;
   }

   return retval;
}

/*! \brief This function changes the prescaler on the CLKM pin.
 *
 *  \param direct   This boolean variable is used to determine if the frequency
 *                  of the CLKM pin shall be changed directly or not. If direct
 *                  equals true, the frequency will be changed directly. This is
 *                  fine if the CLKM signal is used to drive a timer etc. on the
 *                  connected microcontroller. However, the CLKM signal can also
 *                  be used to clock the microcontroller itself. In this situation
 *                  it is possible to change the CLKM frequency indirectly
 *                  (direct == false). When the direct argument equlas false, the
 *                  CLKM frequency will be changed first after the radio transceiver
 *                  has been taken to SLEEP and awaken again.
 *  \param clock_speed This parameter can be one of the following constants:
 *                     CLKM_DISABLED, CLKM_1MHZ, CLKM_2MHZ, CLKM_4MHZ, CLKM_8MHZ
 *                     or CLKM_16MHZ.
 *
 *  \retval RADIO_SUCCESS Clock speed updated. New state is TRX_OFF.
 *  \retval RADIO_INVALID_ARGUMENT Requested clock speed is out of bounds.
 *
 * \ingroup radio
 */
radio_status_t radioSetClockSpeed(bool direct, u8 clock_speed)
{
#ifdef SINGLE_CHIP
   // nothing to do -> there is no clock speed selection on 128rfa1 available
#else
   /*Check function parameter and current clock speed.*/
   if (clock_speed > CLKM_16MHZ)
      return RADIO_INVALID_ARGUMENT;

   /*Select to change the CLKM frequency directly or after returning from SLEEP.*/
   if (direct == false)
      hal_subregister_write(SR_CLKM_SHA_SEL, 1);
   else
      hal_subregister_write(SR_CLKM_SHA_SEL, 0);

   hal_subregister_write(SR_CLKM_CTRL, clock_speed);
#endif
   return RADIO_SUCCESS;
}

/*! \brief  This function return the Radio Transceivers current state.
 *
 *  \retval     P_ON               When the external supply voltage (VDD) is
 *                                 first supplied to the transceiver IC, the
 *                                 system is in the P_ON (Poweron) mode.
 *  \retval     BUSY_RX            The radio transceiver is busy receiving a
 *                                 frame.
 *  \retval     BUSY_TX            The radio transceiver is busy transmitting a
 *                                 frame.
 *  \retval     RX_ON              The RX_ON mode enables the analog and digital
 *                                 receiver blocks and the PLL frequency
 *                                 synthesizer.
 *  \retval     TRX_OFF            In this mode, the SPI module and crystal
 *                                 oscillator are active.
 *  \retval     PLL_ON             Entering the PLL_ON mode from TRX_OFF will
 *                                 first enable the analog voltage regulator. The
 *                                 transceiver is ready to transmit a frame.
 *  \retval     BUSY_RX_AACK       The radio was in RX_AACK_ON mode and received
 *                                 the Start of Frame Delimiter (SFD). State
 *                                 transition to BUSY_RX_AACK is done if the SFD
 *                                 is valid.
 *  \retval     BUSY_TX_ARET       The radio transceiver is busy handling the
 *                                 auto retry mechanism.
 *  \retval     RX_AACK_ON         The auto acknowledge mode of the radio is
 *                                 enabled and it is waiting for an incomming
 *                                 frame.
 *  \retval     TX_ARET_ON         The auto retry mechanism is enabled and the
 *                                 radio transceiver is waiting for the user to
 *                                 send the TX_START command.
 *  \retval     RX_ON_NOCLK        The radio transceiver is listening for
 *                                 incomming frames, but the CLKM is disabled so
 *                                 that the controller could be sleeping.
 *                                 However, this is only true if the controller
 *                                 is run from the clock output of the radio.
 *  \retval     RX_AACK_ON_NOCLK   Same as the RX_ON_NOCLK state, but with the
 *                                 auto acknowledge module turned on.
 *  \retval     BUSY_RX_AACK_NOCLK Same as BUSY_RX_AACK, but the controller
 *                                 could be sleeping since the CLKM pin is
 *                                 disabled.
 *  \retval     STATE_TRANSITION   The radio transceiver's state machine is in
 *                                 transition between two states.
 *
 *  \ingroup radio
 */
u8 radioGetTrxState(void)
{
   return hal_subregister_read(SR_TRX_STATUS);
}

/*! \brief  This function checks if the radio transceiver is sleeping.
 *
 *  \retval     true    The radio transceiver is in SLEEP or one of the *_NOCLK
 *                      states.
 *  \retval     false   The radio transceiver is not sleeping.
 *
 *  \ingroup radio
 */
static bool isSleeping(void)
{
   bool sleeping = false;

   //The radio transceiver will be at SLEEP or one of the *_NOCLK states only if
   //the SLP_TR pin is high.
   if (hal_get_slptr() != 0)
      sleeping = true;

   return sleeping;
}

/*! \brief  This function will change the current state of the radio
 *          transceiver's internal state machine.
 *
 *  \param     new_state        Here is a list of possible states:
 *             - RX_ON        Requested transition to RX_ON state.
 *             - TRX_OFF      Requested transition to TRX_OFF state.
 *             - PLL_ON       Requested transition to PLL_ON state.
 *             - RX_AACK_ON   Requested transition to RX_AACK_ON state.
 *             - TX_ARET_ON   Requested transition to TX_ARET_ON state.
 *
 *  \retval    RADIO_SUCCESS          Requested state transition completed
 *                                  successfully.
 *  \retval    RADIO_INVALID_ARGUMENT Supplied function parameter out of bounds.
 *  \retval    RADIO_WRONG_STATE      Illegal state to do transition from.
 *  \retval    RADIO_BUSY_STATE       The radio transceiver is busy.
 *  \retval    RADIO_TIMED_OUT        The state transition could not be completed
 *                                  within resonable time.
 *
 *  \ingroup radio
 */
radio_status_t radioSetTrxState(u8 new_state)
{
   u8 original_state;

   /*Check function paramter and current state of the radio transceiver.*/
   if (!((new_state == TRX_OFF)    ||
         (new_state == RX_ON)      ||
         (new_state == PLL_ON)     ||
         (new_state == RX_AACK_ON) ||
         (new_state == TX_ARET_ON)))
      return RADIO_INVALID_ARGUMENT;

   if (isSleeping() == true)
      return RADIO_WRONG_STATE;

   // Wait for radio to finish previous operation
   while (radioIsBusy())
      ;

   // For RF230, don't use auto mode while scanning, because no RX_START
   // will be issued.
   if (radioGetPartnum() == RF230 &&
         macIsScanning() &&
         new_state == RX_AACK_ON)
      new_state = RX_ON;

   original_state = radioGetTrxState();

   if (new_state == original_state)
      return RADIO_SUCCESS;

   //At this point it is clear that the requested new_state is:
   //TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON.

   //The radio transceiver can be in one of the following states:
   //TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON, TX_ARET_ON.
   if(new_state == TRX_OFF)
      radioResetStateMachine(); //Go to TRX_OFF from any state.
   else
   {
      //It is not allowed to go from RX_AACK_ON or TX_AACK_ON and directly to
      //TX_AACK_ON or RX_AACK_ON respectively. Need to go via RX_ON or PLL_ON.
      if ((new_state == TX_ARET_ON) &&
            (original_state != PLL_ON))
      {
         //First do intermediate state transition to PLL_ON, then to TX_ARET_ON.
         //The final state transition to TX_ARET_ON is handled after the if-else if.
         hal_subregister_write(SR_TRX_CMD, PLL_ON);
#ifdef __AVR__
         if (original_state == RX_AACK_ON)
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
         else
            delay_us(TIME_TRX_OFF_TO_PLL_ACTIVE);
#endif
      }
      else if ((new_state == RX_AACK_ON) &&
            (original_state != PLL_ON))
      {
         //First do intermediate state transition to PLL_ON, then to RX_AACK_ON.
         hal_subregister_write(SR_TRX_CMD, PLL_ON);
#ifdef __AVR__
         if (original_state == TX_ARET_ON)
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
         else
            delay_us(TIME_TRX_OFF_TO_PLL_ACTIVE);
#endif
      }

      //Any other state transition can be done directly.
      hal_subregister_write(SR_TRX_CMD, new_state);

      //When the PLL is active most states can be reached in 1us. However, from
      //TRX_OFF the PLL needs time to activate.
      if (original_state == TRX_OFF)
      {
#ifdef __AVR__
         delay_us(TIME_TRX_OFF_TO_PLL_ACTIVE);
#endif
      }
      else
      {
#ifdef __AVR__
         delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
#endif
      }
   } // end: if(new_state == TRX_OFF) ...

   /*Verify state transition.*/
   radio_status_t set_state_status = RADIO_TIMED_OUT;

   if (radioGetTrxState() == new_state)
      set_state_status = RADIO_SUCCESS;

   return set_state_status;
}

/*! \brief  This function will put the radio transceiver to sleep.
 *
 *  \retval    RADIO_SUCCESS          Sleep mode entered successfully.
 *  \retval    RADIO_TIMED_OUT        The transition to TRX_OFF took too long.
 *
 *  \ingroup radio
 */
radio_status_t radioEnterSleepMode(void)
{
   if (isSleeping() == true)
      return RADIO_SUCCESS;

   radioResetStateMachine(); //Force the device into TRX_OFF.

   delay_us(TIME_RESET_TRX_OFF);

   radio_status_t enter_sleep_status = RADIO_TIMED_OUT;

   if (radioGetTrxState() == TRX_OFF)
   {
      //Enter Sleep.
      hal_set_slptr_high();
      enter_sleep_status = RADIO_SUCCESS;
   }

   return enter_sleep_status;
}

/*! \brief  This function will take the radio transceiver from sleep mode and
 *          put it into the TRX_OFF state.
 *
 *  \retval    RADIO_SUCCESS          Left sleep mode and entered TRX_OFF state.
 *  \retval    RADIO_TIMED_OUT        Transition to TRX_OFF state timed out.
 *
 *  \ingroup radio
 */
radio_status_t radioLeaveSleepMode(void)
{
   //Check if the radio transceiver is actually sleeping.
   if (isSleeping() == false)
      return RADIO_SUCCESS;

   hal_set_slptr_low();
   delay_us(TIME_SLEEP_TO_TRX_OFF);

   radio_status_t leave_sleep_status = RADIO_TIMED_OUT;

   //Ensure CLKM is OFF
   radioSetClockSpeed(true, CLKM_DISABLED);

   //Ensure that the radio transceiver is in the TRX_OFF state.
   if (radioGetTrxState() == TRX_OFF)
      leave_sleep_status = RADIO_SUCCESS;

   return leave_sleep_status;
}

/*! \brief  This function will reset the state machine (to TRX_OFF) from any of
 *          its states, except for the SLEEP state.
 *
 *  \ingroup radio
 */
void radioResetStateMachine(void)
{
   hal_set_slptr_low();
   delay_us(TIME_NOCLK_TO_WAKE);
   hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
   delay_us(TIME_CMD_FORCE_TRX_OFF);
}

/*! \brief  This function will reset all the registers and the state machine of
 *          the radio transceiver.
 *
 *  \ingroup radio
 */
void radioResetTrx(void)
{
   hal_set_rst_low();
   hal_set_slptr_low();
   delay_us(TIME_RESET);
   hal_set_rst_high();
}

/*! \brief  This function will enable or disable automatic CRC during frame
 *          transmission.
 *
 *  \param  auto_crc_on If this parameter equals true auto CRC will be used for
 *                      all frames to be transmitted. The framelength must be
 *                      increased by two bytes (16 bit CRC). If the parameter equals
 *                      false, the automatic CRC will be disabled.
 *
 *  \ingroup radio
 */
void radioUseAutoTxCrc(bool auto_crc_on)
{
   if (BAND == BAND2400)
   {
      if (radioGetPartnum() == RF230)
         hal_subregister_write(SR_TX_AUTO_CRC_ON_230, auto_crc_on == true);
      if (radioGetPartnum() == RF231)
         hal_subregister_write(SR_TX_AUTO_CRC_ON_231, auto_crc_on == true);
   }
   /*
    * if SINGLE CHIP is used this is the default setting
    * SR_TX_AUTO_CRC_ON is mapped to correct register place
    */
   else
      hal_subregister_write(SR_TX_AUTO_CRC_ON, auto_crc_on == true);
}

/**
   Check whether the radio chip is busy. This can be used to make sure
   the radio is free to send another packet.  If the radio is busy and
   the MAC is directed to send another packet, then radioSendData()
   will simply wait for the radio to be free before proceeding.

   @return true  Radio is busy.
   @return false Radio is free to use.
 */
u8 radioIsBusy(void)
{
   u8 state;

   state = radioGetTrxState();
   return (state == BUSY_RX_AACK ||
         state == BUSY_TX_ARET ||
         state == BUSY_TX ||
         state == BUSY_RX ||
         state == BUSY_RX_AACK_NOCLK);
}

/*! \brief  This function will download a frame to the radio
 *          transceiver's transmit buffer and send it.  If the radio
 *          is currently busy receiving or transmitting, this function
 *          will block processing waiting for the radio to finish its
 *          current operation.  You can avoid blocking by checking
 *          that radioIsBusy() return false before calling this
 *          function.
 *
 *  \param  data_length Length of the frame to be transmitted. 1 to 128 bytes are the valid lengths.
 *          The data lenght is padded by two to account for the auto-CRC, which is used in every frame.
 *  \param  *data   Pointer to the data to transmit
 *
 *  \retval RADIO_SUCCESS Frame downloaded and sent successfully.
 *  \retval RADIO_INVALID_ARGUMENT If the dataLength is 0 byte or more than 127
 *                               bytes the frame will not be sent.
 *  \retval RADIO_WRONG_STATE It is only possible to use this function in the
 *                          PLL_ON and TX_ARET_ON state. If any other state is
 *                          detected this error message will be returned.
 *
 *  \ingroup radio
 */
radio_status_t radioSendData(u8 data_length, u8 *data)
{
   //UART_PRINT("radioSendData()\r\n");
   // Check function parameters and current state.
   if (data_length > RF2xx_MAX_TX_FRAME_LENGTH)
      return RADIO_INVALID_ARGUMENT;

   // Wait for radio to get unbusy
   while (radioIsBusy())
      ;

   // Put radio in TX_ARET_ON state
   do
   {
      radioSetTrxState(TX_ARET_ON);
   } while (radioGetTrxState() != TX_ARET_ON);

   // save last destination address, which is needed
   // to process a send failure later
   macConfig.lastDestAddr = ((ftData*)data)->destAddr;

   /*Do frame transmission.*/
   hal_frame_write(data, data_length+2); //Then write data to the frame buffer.

   return RADIO_SUCCESS;
}

/*! \brief  This function will set the I_AM_COORD sub register.
 *
 *  \param[in] i_am_coordinator If this parameter is true, the associated
 *                              coordinator role will be enabled in the radio
 *                              transceiver's address filter.
 *                              False disables the same feature.
 *  \ingroup radio
 */
void radioSetDeviceRole(bool i_am_coordinator)
{
   hal_subregister_write(SR_I_AM_COORD, i_am_coordinator);
}

/*! \brief  This function will set the PANID used by the address filter.
 *
 *  \param  new_pan_id Desired PANID. Can be any value from 0x0000 to 0xFFFF
 *
 *  \ingroup radio
 */
void radioSetPanId(u16 new_pan_id)
{

   u8 pan_byte = new_pan_id & 0xFF; // Extract new_pan_id_7_0.
   hal_register_write(RG_PAN_ID_0, pan_byte);

   pan_byte = (new_pan_id >> 8*1) & 0xFF;  // Extract new_pan_id_15_8.
   hal_register_write(RG_PAN_ID_1, pan_byte);
}

/*! \brief  This function will set the short address used by the address filter.
 *
 *  \param  new_short_address Short address to be used by the address filter.
 *
 *  \ingroup radio
 */
void radioSetShortAddress(u16 new_short_address)
{

   u8 short_address_byte = new_short_address & 0xFF; // Extract short_address_7_0.
   hal_register_write(RG_SHORT_ADDR_0, short_address_byte);

   short_address_byte = (new_short_address >> 8*1) & 0xFF; // Extract short_address_15_8.
   hal_register_write(RG_SHORT_ADDR_1, short_address_byte);
}

/*! \brief  This function will set a new extended address to be used by the
 *          address filter.
 *
 *  \param  extended_address Extended address to be used by the address filter.
 *
 *  \ingroup radio
 */
void radioSetExtendedAddress(u8 *extended_address)
{
   u8 i;

   for (i=0;i<8;i++)
   {
      hal_register_write(RG_IEEE_ADDR_0+i, *extended_address++);
   }
}


/**
   @brief Returns a random number, composed of bits from the radio's
   random number generator.  Note that the radio must be in a receive
   mode for this to work, otherwise the library rand() function is
   used.

   @param bits Number of bits of random data to return.  This function
   will return an even number of random bits, equal to or less than bits.
 */
u8 radioRandom(u8 bits)
{
   if ((NODETYPE != ENDDEVICE) || APP)
   {
      volatile u8 val=0;
      volatile u8 regval;
      u8 i;

      i = radioGetTrxState();
      if ((radioGetPartnum() == RF231 ||    // RF231
            radioGetPartnum() == RF212 ) &&      // RF212
            (i == RX_ON ||
                  i == RX_AACK_ON))       // Must be in rx to get random numbers
      {
         // Random number generator on-board
         // has two random bits each read
         for (i=0;i<bits/2;i++)
         {
            regval = hal_subregister_read(SR_RND_VALUE);
            val = (val << 2) | regval;
         }
         return val;
      }
      else
         // use library function.
         return rand();
   }
   else
      return 0;
}

/**
   @name RF-212 Functions
   @{
   These functions are only available for the RF-212 chips.
 */

void radioSetBoost(u8 boost)
{
   if (BAND == BAND900)
   {
   }
}

void radioSetModulation(u8 modulation)
{
   // create an enum of monulation types.
   if (BAND == BAND900)
   {
   }
}

void radioSetup900(void)
{
   if (BAND == BAND900)
   {
      radioSetTrxState(TRX_OFF);
      // Set datarate for RF212
      if (CHINA_MODE)
      {
         // IEEE says only OQPSK RC at 250KBPS is valid
         hal_register_write(RG_TRX_CTRL_2, OQPSK_RC_250);
         // Max power
         hal_register_write(RG_PHY_TX_PWR, 0xe7); // Plus 5dBm
         // Set GC offset for O-QPSK.
         hal_subregister_write(SR_GC_TX_OFFS, 2);
      }
      else
      {
         // Choose one of many data rates (set in mac.h)
         hal_register_write(RG_TRX_CTRL_2, DATA_RATE_212);

         if (PLATFORM == DSK001)
         {
            // Max power
            hal_register_write(RG_PHY_TX_PWR, 0x41); //Plus 3 dBm
         }
         else
         {
            hal_register_write(RG_PHY_TX_PWR, 0xC0); //Plus 10 dBm
         }


         // Set GC offset
         if ((DATA_RATE_212 == BPSK_40) ||
               (DATA_RATE_212 == BPSK_20))
         {
            hal_subregister_write(SR_GC_TX_OFFS, 3); //Set to 3 for BPSK
         }
         else
         {
            hal_subregister_write(SR_GC_TX_OFFS, 2); //Set to 2 for OQPSK
         }
      }
      // Turn up the backoff times
      hal_register_write(RG_CSMA_BE, 0xfa);
   }
}

/** @} */

/** @} */
/*EOF*/
