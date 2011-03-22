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
  $Id: hal_avr.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 */

/*============================ INCLUDE =======================================*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../inc/mac.h"
#include "../inc/radio.h"

#include "../inc/mac_event.h"
#include "../inc/deRFaddon/bmm.h"
#include "../inc/deRFaddon/uart.h"


/*============================ MACROS ========================================*/

void radioRxStartEvent(u8 const frame_length);

/**
   @ingroup radio
   @{
 */

/*
 * Macros defined for the radio transceiver's access modes.
 *
 * These functions are implemented as macros since they are used very often and
 * we want to remove the function call overhead.
 */
#define HAL_DUMMY_READ         (0x00) //!< Dummy value for the SPI.

#define HAL_TRX_CMD_RW         (0xC0) //!< Register Write (short mode).
#define HAL_TRX_CMD_RR         (0x80) //!< Register Read (short mode).
#define HAL_TRX_CMD_FW         (0x60) //!< Frame Transmit Mode (long mode).
#define HAL_TRX_CMD_FR         (0x20) //!< Frame Receive Mode (long mode).
#define HAL_TRX_CMD_SW         (0x40) //!< SRAM Write.
#define HAL_TRX_CMD_SR         (0x00) //!< SRAM Read.
#define HAL_TRX_CMD_RADDRM     (0x7F) //!< Register Address Mask.

#define HAL_CALCULATED_CRC_OK   (0) //!< CRC calculated over the frame including the CRC field should be 0.
/*============================ TYPDEFS =======================================*/
/*============================ VARIABLES =====================================*/
/*Flag section.*/

/*============================ PROTOTYPES ====================================*/
void radioTrxEndEvent(void);
void macEdCallback(void);
#ifdef SINGLE_CHIP
void radioRxEndEvent(void);
#endif

extern u8 rx_mode;

/*============================ IMPLEMENTATION ================================*/


void hal_spi_init(void)
{
#ifdef SINGLE_CHIP
   // do nothing -> there is no SPI connection
#else
   /*SPI Specific Initialization.*/
   //Set SCK and MOSI as output.
   HAL_DDR_SPI  |= (1 << HAL_DD_SCK) | (1 << HAL_DD_MOSI);
   HAL_PORT_SPI |= (1 << HAL_DD_SCK); //Set CLK high
   // Setup slave select pin as output, and high
   HAL_DDR_SS  |= (1 << SSPIN);
   HAL_PORT_SS |= (1 << SSPIN);

   //Enable SPI module and master operation.
   SPCR         = (1 << SPE) | (1 << MSTR);
   //Enable doubled SPI speed in master mode.
   SPSR         = (1 << SPI2X);
#endif // SINGLE_CHIP
}

/*! \brief  This function initializes the Hardware Abstraction Layer.
 *
 */
void hal_init(void)
{
#ifdef SINGLE_CHIP
   // do nothing -> there is no external transceiver
   DRTRAM0 = _BV(ENDRT);
   DRTRAM1 = _BV(ENDRT);
   DRTRAM2 = _BV(ENDRT);
   DRTRAM3 = _BV(ENDRT);
#else
   /*IO Specific Initialization.*/
   DDR_SLP_TR |= (1 << SLP_TR); //Enable SLP_TR as output.
   DDR_RST    |= (1 << RST);    //Enable RST as output.
#endif // do nothing

   hal_spi_init();
   hal_enable_trx_interrupt();    //Enable interrupts from the radio transceiver.
}

/*! \brief  This function reads data from one of the radio transceiver's registers.
 *
 *  \param  address Register address to read from. See datasheet for register
 *                  map.
 *
 *  \see Look at the at86rf23x_registermap.h file for register address definitions.
 *
 *  \returns The actual value of the read register.
 *
 */
u8 hal_register_read(u16 address)
{
#ifdef SINGLE_CHIP
   return (*(volatile uint8_t *)(address));
#else
   //Add the register read command to the register address.
   //    address &= HAL_TRX_CMD_RADDRM;
   address |= HAL_TRX_CMD_RR;

   u8 register_value = 0;

   AVR_ENTER_CRITICAL_REGION();

   HAL_SS_LOW(); //Start the SPI transaction by pulling the Slave Select low.

   /*Send Register address and read register content.*/
   SPDR = (u8)address;
   while ((SPSR & (1 << SPIF)) == 0) {;}
   register_value = SPDR;

   SPDR = register_value;
   while ((SPSR & (1 << SPIF)) == 0) {;}
   register_value = SPDR;

   HAL_SS_HIGH(); //End the transaction by pulling the Slave Select High.

   AVR_LEAVE_CRITICAL_REGION();

   return register_value;
#endif
   //return 0;
}

/*! \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *
 *  \see Look at the at86rf23x_registermap.h file for register address definitions.
 *
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 *
 */
void hal_register_write(u16 address, u8 value)
{
#ifdef SINGLE_CHIP
   (*(volatile uint8_t *)(address)) = (value);
#else
   //Add the Register Write command to the address.
   address = HAL_TRX_CMD_RW | (HAL_TRX_CMD_RADDRM & (u8)address);

   AVR_ENTER_CRITICAL_REGION();

   HAL_SS_LOW(); //Start the SPI transaction by pulling the Slave Select low.

   /*Send Register address and write register content.*/
   SPDR = (u8)address;
   while ((SPSR & (1 << SPIF)) == 0) {;}
   SPDR;  // Dummy read of SPDR

   SPDR = value;
   while ((SPSR & (1 << SPIF)) == 0) {;}
   SPDR;  // Dummy read of SPDR

   HAL_SS_HIGH(); //End the transaction by pulling the Slave Slect High.

   AVR_LEAVE_CRITICAL_REGION();

   // Set the rx_mode variable based on how we set the radio
   if (((u8)address & ~HAL_TRX_CMD_RW) == RG_TRX_STATE)
   {
      // set rx_mode flag based on mode we're changing to
      value &= 0x1f;   // Mask for TRX_STATE register
      if (value == RX_ON ||
            value == RX_AACK_ON)
         rx_mode = true;
      else
         rx_mode = false;
   }
#endif
}

/*! \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf23x_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 *
 */
u8 hal_subregister_read(u16 address, u8 mask, u8 position)
{
   //Read current register value and mask out subregister.
   u8 register_value = hal_register_read(address);
   register_value &= mask;
   register_value >>= position; //Align subregister value.

   return register_value;
}

/*! \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf23x_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 *
 */
void hal_subregister_write(u16 address, u8 mask, u8 position,
      u8 value)
{
   //Read current register value and mask area outside the subregister.
   u8 register_value = hal_register_read(address);
   register_value &= ~mask;

   //Start preparing the new subregister value. shift in place and mask.
   value <<= position;
   value &= mask;

   value |= register_value; //Set the new subregister value.

   //Write the modified register value.
   hal_register_write(address, value);
}

/*! \brief  This function will upload a frame from the radio transceiver's frame
 *          buffer.
 *
 *          If the frame currently available in the radio transceiver's frame buffer
 *          is out of the defined bounds. Then the frame length, lqi value and crc
 *          be set to zero. This is done to indicate an error.
 *
 */
uint8_t* hal_frame_read(void)
{
   uint8_t* pFrame = bmm_buffer_alloc();

   if(pFrame != NULL)
   {
      rx_frame_t *rx_frame = (rx_frame_t*)pFrame;

#ifdef SINGLE_CHIP
      AVR_ENTER_CRITICAL_REGION();

      volatile uint8_t *pSrc = (volatile uint8_t *)0x180;
      uint8_t frame_length = hal_register_read(RG_TST_RX_LENGTH);
      if ((frame_length >= HAL_MIN_FRAME_LENGTH) && (frame_length <= HAL_MAX_FRAME_LENGTH))
      {
         // read length and save frame content -> lqi is NOT included in frame length byte
         rx_frame->length = frame_length;
         //memcpy(rx_data, (void *)pSrc, frame_length);
         memcpy(rx_frame->data, (void *)pSrc, frame_length-1);
         // save LQI /
         //rx_frame->lqi = *(pSrc + (frame_length + 1));
         rx_frame->lqi = *(pSrc + frame_length);
      }
      else
      {
         rx_frame->length = 0;
         rx_frame->lqi    = 0;
         rx_frame->crc    = false;
         bmm_buffer_free(pFrame); // free allcoated buffer
         pFrame = NULL; // set buffer pointer to NULL, that next app do not use it
      }
      AVR_LEAVE_CRITICAL_REGION();
#else
      uint8_t* rx_data = &rx_frame->data[0];

      AVR_ENTER_CRITICAL_REGION();

      HAL_SS_LOW();

      //Send frame read command.
      SPDR = HAL_TRX_CMD_FR;
      while ((SPSR & (1 << SPIF)) == 0) {;}
      u8 frame_length = SPDR;

      //Read frame length.
      SPDR = frame_length;
      while ((SPSR & (1 << SPIF)) == 0) {;}
      frame_length = SPDR;

      //Check for correct frame length.
      if ((frame_length >= HAL_MIN_FRAME_LENGTH) && (frame_length <= HAL_MAX_FRAME_LENGTH))
      {
         rx_frame->length = frame_length; //Store frame length.

         //Upload frame buffer to data pointer. Calculate CRC.
         SPDR = frame_length;
         while ((SPSR & (1 << SPIF)) == 0)
            ;

         do
         {
            u8 tempData = SPDR;
            SPDR = 0;       // dummy write

            //*rx_data++ = tempData;
            *rx_data = tempData;
            rx_data++;
            while ((SPSR & (1 << SPIF)) == 0)
               ;
         } while (--frame_length > 0);

         //Read LQI value for this frame.
         rx_frame->lqi = SPDR;

         HAL_SS_HIGH();
      }
      else
      {
         HAL_SS_HIGH();

         if (rx_frame)
         {
            rx_frame->length = 0;
            rx_frame->lqi    = 0;
            rx_frame->crc    = false;
            bmm_buffer_free(pFrame); // free allocated buffer
            pFrame = NULL; // set buffer pointer to NULL, that next app do not use it
         }
      }

      AVR_LEAVE_CRITICAL_REGION();

#endif // SINGLE_CHIP
   }
   return pFrame;
}

/*! \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 *
 */
void hal_frame_write(u8 *write_buffer, u8 length)
{
#ifdef SINGLE_CHIP
   volatile uint8_t *pDst = (volatile uint8_t *)0x180;

   AVR_ENTER_CRITICAL_REGION();

   //Toggle the SLP_TR pin to initiate the frame transmission.
   hal_set_slptr_high();
   hal_set_slptr_low();

   *pDst = length;
   pDst++;
   memcpy((void *)pDst, write_buffer, length);

   AVR_LEAVE_CRITICAL_REGION();
#else

   length &= HAL_TRX_CMD_RADDRM; //Truncate length to maximum frame length.

   AVR_ENTER_CRITICAL_REGION();

   //Toggle the SLP_TR pin to initiate the frame transmission.
   hal_set_slptr_high();
   hal_set_slptr_low();

   HAL_SS_LOW(); //Initiate the SPI transaction.

   /*SEND FRAME WRITE COMMAND AND FRAME LENGTH.*/
   SPDR = HAL_TRX_CMD_FW;
   while ((SPSR & (1 << SPIF)) == 0) {;}
   SPDR; // Dummy read of SPDR

   SPDR = length;
   while ((SPSR & (1 << SPIF)) == 0) {;}
   SPDR;  // Dummy read of SPDR

   // Download to the Frame Buffer.
   do
   {
      SPDR = *write_buffer++;
      --length;

      while ((SPSR & (1 << SPIF)) == 0)
         ;

      SPDR;  // Dummy read of SPDR
   } while (length > 0);

   HAL_SS_HIGH(); //Terminate SPI transaction.

   AVR_LEAVE_CRITICAL_REGION();

#endif //SINGLE_CHIP
}

/*! \brief Read SRAM
 *
 * This function reads from the SRAM of the radio transceiver.
 *
 * \param address Address in the TRX's SRAM where the read burst should start
 * \param length Length of the read burst
 * \param data Pointer to buffer where data is stored.
 *
 */
void hal_sram_read(u8 address, u8 length, u8 *data)
{
   AVR_ENTER_CRITICAL_REGION();

   HAL_SS_LOW(); //Initiate the SPI transaction.

   /*Send SRAM read command.*/
   SPDR = HAL_TRX_CMD_SR;
   while ((SPSR & (1 << SPIF)) == 0)
      ;
   SPDR;  // Dummy read of SPDR

   /*Send address where to start reading.*/
   SPDR = address;
   while ((SPSR & (1 << SPIF)) == 0)
      ;
   SPDR;  // Dummy read of SPDR

   /*Upload the chosen memory area.*/
   do
   {
      SPDR = HAL_DUMMY_READ;
      while ((SPSR & (1 << SPIF)) == 0) {;}
      *data++ = SPDR;
   } while (--length > 0);

   HAL_SS_HIGH();

   AVR_LEAVE_CRITICAL_REGION();
}

/*! \brief Write SRAM
 *
 * This function writes into the SRAM of the radio transceiver.
 *
 * \param address Address in the TRX's SRAM where the write burst should start
 * \param length  Length of the write burst
 * \param data    Pointer to an array of bytes that should be written
 *
 */
void hal_sram_write(u8 address, u8 length, u8 *data)
{
   AVR_ENTER_CRITICAL_REGION();

   HAL_SS_LOW();

   /*Send SRAM write command.*/
   SPDR = HAL_TRX_CMD_SW;
   while ((SPSR & (1 << SPIF)) == 0) {;}
   SPDR;  // Dummy read of SPDR

   /*Send address where to start writing to.*/
   SPDR = address;
   while ((SPSR & (1 << SPIF)) == 0)
      ;
   SPDR;  // Dummy read of SPDR

   /*Upload the chosen memory area.*/
   do
   {
      SPDR = *data++;
      while ((SPSR & (1 << SPIF)) == 0)
         ;
      SPDR;  // Dummy read of SPDR
   } while (--length > 0);

   HAL_SS_HIGH();

   AVR_LEAVE_CRITICAL_REGION();
}

//This #if compile switch is used to provide a "standard" function body for the
//doxygen documentation.
#if defined(DOXYGEN)
/*! \brief ISR for the radio IRQ line, triggered by the input capture.
 *  This is the interrupt service routine for timer1.ICIE1 input capture.
 *  It is triggered of a rising edge on the radio transceivers IRQ line.
 */
void RADIO_VECT(void);
#else  /* !DOXYGEN */
#ifndef SINGLE_CHIP
ISR(RADIO_VECT)
{
   /*Read Interrupt source.*/
   u8 interrupt_source = hal_register_read(RG_IRQ_STATUS);

   if (interrupt_source & HAL_TRX_END_MASK)
   {
      radioTrxEndEvent();
   }

   // Energy detect event
   if (interrupt_source & HAL_ED_READY_MASK)
   {
      macEdCallback();
   }



   /* Handle the incomming interrupt. Prioritized.
       Other Interrupts: 
       HAL_TRX_UR_MASK
       HAL_PLL_UNLOCK_MASK
       HAL_PLL_LOCK_MASK
    */
   if ((interrupt_source & HAL_RX_START_MASK))
   {
      /*Read Frame length and call rx_start callback.*/
      HAL_SS_LOW();

      SPDR = HAL_TRX_CMD_FR;
      while ((SPSR & (1 << SPIF)) == 0) {;}
      SPDR;

      SPDR = 0; // Send dummy byte so we can read one byte back
      while ((SPSR & (1 << SPIF)) == 0) {;}
      u8 frame_length = SPDR;

      HAL_SS_HIGH();

      radioRxStartEvent(frame_length);
   }
   else if (interrupt_source & HAL_BAT_LOW_MASK)
   {
      //Disable BAT_LOW interrupt to prevent interrupt storm. The interrupt
      //will continously be signaled when the supply voltage is less than the
      //user defined voltage threshold.
      u8 trx_isr_mask = hal_register_read(RG_IRQ_MASK);
      trx_isr_mask &= ~HAL_BAT_LOW_MASK;
      hal_register_write(RG_IRQ_MASK, trx_isr_mask);
   }
   else
      ; // unknown ISR

}
#endif

#ifdef SINGLE_CHIP
// TRX24_RX_START_vect
ISR(RADIO_VECT1)
{
   //UART_PRINT("RX_START IRQ\r\n");
   uint8_t frame_length = hal_register_read(RG_TST_RX_LENGTH);
   radioRxStartEvent(frame_length);
}

// TRX24_RX_END_vect
ISR(RADIO_VECT2)
{
   //UART_PRINT("RX_END IRQ\r\n");
   //radioTrxEndEvent();
   radioRxEndEvent();
}

// TRX24_TX_END_vect
ISR(RADIO_VECT3)
{
   //UART_PRINT("TX_END IRQ\r\n");
   radioTrxEndEvent();
}

// TRX24_CCA_ED_DONE_vect
ISR(RADIO_VECT4)
{
   //UART_PRINT("ED_DONE IRQ\r\n");
   macEdCallback();
}

// BAT_LOW_vect
ISR(RADIO_VECT5)
{
   //UART_PRINT("BAT_LOW IRQ\r\n");
   //Disable BAT_LOW interrupt to prevent interrupt storm. The interrupt
   //will continously be signaled when the supply voltage is less than the
   //user defined voltage threshold.

   //u8 trx_isr_mask = hal_register_read(RG_IRQ_MASK);
   //trx_isr_mask &= ~HAL_BAT_LOW_MASK;
   //hal_register_write(RG_IRQ_MASK, trx_isr_mask);
}

// PLL_UNLOCK_vect
ISR(RADIO_VECT6)
{
   //UART_PRINT("PLL UNLOCK IRQ\r\n");
}

// PLL_LOCK_vect
ISR(RADIO_VECT7)
{
   //UART_PRINT("PLL LOCK IRQ\r\n");
}

// AWAKE_vect
ISR(RADIO_VECT8)
{
   //UART_PRINT("AWAKE IRQ\r\n");
}
#endif


/**
    @brief Calibrate the internal RC oscillator

    This function calibrates the internal RC oscillator, based
    on either the 1 MHz clock supplied by the AT86RF2xx. In order to
    verify the calibration result you can program the CKOUT fuse
    and monitor the CPU clock on an I/O pin.

    @return TRUE if calibrate passed; FALSE if calibrate failed.
 */
bool calibrate_rc_osc(void)
{
   if (SERIAL && F_CPU >= 2000000UL) // only need for making serial port work
   {
      bool success = false;
      static bool once=0;

      if (once)
         // Only calibrate once
         return true;

      once = true;

      // Use the 1 MHz CLK_M from the AT86RF230.
      volatile u16 temp;
      u16 counter;
      u8 osccal_saved;
      u8 tccr2b, tccr1b, tccr1a;

      // in the following line, 1000000ULL represents the 1MHz input signal
      // from the radio.  265 is the number of counts to overflow 8-bit
      // timer 2.  32 is the divide by 32 prescaler for timer 1.  F_CPU is
      // the main clock frequency.
#define TARGETVAL ((1000000ULL * 256 * 32) / F_CPU)

      // Timer 1 (16-bit) is run from the 1MHz RF23x CLKM signal, which gives
      //   1uS/count.  This timer measures how long it takes for
      //   timer2 to overflow
      // Timer 2 (8-bit) is run from the main CPU clock divided by
      //   32.  This means the clock is 31250Hz at 1MHz, or 250KHz at 8MHz.
      //   Target time is 1024uS (8MHz) or 8192uS (1MHz).

      osccal_saved = OSCCAL;
      cli();

      radioSetClockSpeed(true, CLKM_1MHz);

      // Save current values of timer status.
      tccr2b = TCCR2B;
      tccr1b = TCCR1B;
      tccr1a = TCCR1A;

      // Stop timers 1 and 2.
      // Set timer 1 to normal mode (no CTC, no PWM, just count).
      TCCR2B = 0;
      TCCR1B = 0;
      TCCR1A = 0;

      for (counter = 0; counter < 1000;  counter++)
      {
         // Timer 2 driven from clock divided by 32
         TCCR2B = (1 << CS21) | (1 << CS20);
         // Timer 1 driven with external clock
         TCCR1B = (1 << CS12) | (1 << CS11);

         // Clear pending timer 1 and 2 interrupts, and clear the
         // counters.
         TCNT2 = 0;
         TCNT1 = 0;
         TIFR1 = 0xFF;
         TIFR2 = 0xFF;

         // Wait for timer 2 to overflow.
         while (!(TIFR2 & (1 << TOV2)))
            ;

         temp = TCNT1;


         // Stop timer 1.  Now, TCNT1 contains the number of 1MHz RF2xx cycles
         // counted while timer 2 was counting CPU cycles
         TCCR1B = 0;
         TCCR2B = 0;

         if (temp < (u16)(0.995 * TARGETVAL))
         {
            // Too fast, slow down
            OSCCAL--;
         }
         else if (temp > (u16)(1.005 * TARGETVAL))
         {
            // Too slow, speed up
            OSCCAL++;
         }
         else
         {
            // We are within +/- 0.5 % of our target frequency, so we're
            // done.
            success = true;
            break;
         }
      }

      radioSetClockSpeed(true, CLKM_DISABLED);

      // restore timer status regs
      TCCR2B = tccr2b;
      TCCR1B = tccr1b;
      TCCR1A = tccr1a;
      if (!success)
      {
         // We failed, therefore restore previous OSCCAL value.
         OSCCAL = osccal_saved;
      }

      return success;
   }
   else
      return 0;
}

/**
   General-purpose function to read data out of eeprom

   @param offset The offset in EEPROM of the start of the data block
   @param length The length in bytes of the data block
   @param dest  Pointer to the area in memory to place the data block
 */
void halGetEeprom(u8 *addr, u8 length, u8 *dest)
{
   AVR_ENTER_CRITICAL_REGION();
   eeprom_read_block (dest, addr, length);
   AVR_LEAVE_CRITICAL_REGION();
}

/**
   General-purpose function to write data to eeprom

   @param offset The offset in EEPROM of the start of the data block
   @param length The length in bytes of the data block
   @param src  Pointer to the area in memory which contains the data block
 */
void halPutEeprom(u8 *addr, u8 length, u8 *src)
{
   AVR_ENTER_CRITICAL_REGION();
   eeprom_write_block (src, addr, length);
   AVR_LEAVE_CRITICAL_REGION();
}

/**
   Initialize the AVR clock speed.
 */
void halSetupClock(void)
{
   // Set clock speed based on F_CPU flag
   if (F_CPU == 4000000UL)
   {
      AVR_ENTER_CRITICAL_REGION();
      CLKPR = 1 << CLKPCE;  // Set the change-enable flag
      CLKPR = 1;            // Set for divide-by-two, or 4MHz
      AVR_LEAVE_CRITICAL_REGION();
   }
   if (F_CPU == 2000000UL)
   {
      AVR_ENTER_CRITICAL_REGION();
      CLKPR = 1 << CLKPCE;  // Set the change-enable flag
      CLKPR = 2;            // Set for divide-by-four, or 2MHz
      AVR_LEAVE_CRITICAL_REGION();
   }
   if (F_CPU == 1000000UL)
   {
      AVR_ENTER_CRITICAL_REGION();
      CLKPR = 1 << CLKPCE;  // Set the change-enable flag
      CLKPR = 3;            // Set for divide-by-eight, or 1MHz
      AVR_LEAVE_CRITICAL_REGION();
   }

}


/** @} */

#   endif /* defined(DOXYGEN) */

/*EOF*/
