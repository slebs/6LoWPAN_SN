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
 * @file data.c
 *
 * @brief Evaluation and generation of Data frame messages.
 *
 * This file implements methods for data message generation and evaluation.
 * This message type is responsible for changing inputs and outputs on current platform.
 * A new message is generated if one of the following changes occur:
 *    - the state of an input pin changes
 *    - the temperature change about +-0.5° degree
 *    - supply voltage change about +-0.1 volt
 * The temperature, supply voltage and all inputs are checked periodically.
 * This time is adjustable. Update time for temperature and supply voltage is changed via
 * ITERATION_TIME_TEMP_VCC (in milliseconds) and input checking via ITERATION_TIME_INPUT_IO (in ms)
 *
 *
 *
 * $Id: data.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-01-18
 */

/* === Includes ============================================================= */

#include <stdint.h>
#include <string.h>

#include "../../inc/avr_timer.h"

#include "../../inc/deRFaddon/util.h"
#include "../../inc/deRFaddon/commands.h"
#include "../../inc/deRFaddon/deRFapplication.h"
#include "../../inc/deRFaddon/io_access.h"
#include "../../inc/deRFaddon/uart.h"



/* === Macros =============================================================== */

/*
 * defines the iteration time for checking temperature and vcc (in ms)
 */
#define ITERATION_TIME_TEMP_VCC        (5000)

/*
 * defines the iteration time for checking input I/O's (in ms)
 */
#define ITERATION_TIME_INPUT_IO        (20)

/*
 * Defines the temperature difference between old value and new measured value, in milli degree.
 *
 * If using Single Chip, the internal temperature sensor is chosen. It has a
 * max. accuracy of 1° degree. So difference should be greater than 1° degree.
 */
#ifdef SINGLE_CHIP
#define TEMPERATURE_DIFFERENCE         (1200)
#else
#define TEMPERATURE_DIFFERENCE         (500)
#endif //SINGLE_CHIP

/*
 * Defines the supply voltage difference between old value and new mesured value, in micro volt.
 */
#define VCC_DIFFERENCE                 (100000)

/* === Globals ============================================================== */

// hold the content of new incoming DATA_FRAME message
deRFprotocol_t dataConfiguration;

#if NODETYPE != COORD
static int32_t actual_temp = 0;
static uint32_t actual_vcc = 0;
#endif

/* === Prototypes =========================================================== */

void set_output(uint8_t protocol_port, uint8_t protocol_bit, uint8_t hardware_port, uint8_t hardware_bit);
bool read_io(uint8_t port, uint8_t pin);

/* === Implementation ======================================================= */

/**
 * @brief Process an incoming data frame.
 *
 * A data frame contains instructions how to change the I/O's of this node. The frame has a
 * defined structure (see payloadDataFrame_t) where every single I/O has it's direction and
 * value. Direction means if this I/O is input or output and value mean if the I/O is set or
 * unset.
 * I/O changes depends on platform. With use of deRFmega128 radio modules not all I/O's can
 * be served, because there are less pins than e.g. a RCB231 has.
 *
 * @param   pFrame  pointer to data packet
 */
void evaluate_data_frame(uint8_t* pFrame)
{
#if NODETYPE != COORD
   // copy incoming data configuration to be able recognizes changes later (e.g. button pressed, ...)
   memcpy((uint8_t*)&dataConfiguration, pFrame, sizeof(dataConfiguration));

   payloadDataFrame_t* dataFrame = (payloadDataFrame_t*)&dataConfiguration.payload;

#ifdef RCB_BREAKOUT
   uint8_t dataRCB = dataFrame->digitalData[0];

   set_output(PROTOCOL_PORT_PA0, PROTOCOL_BIT_PA0, PORT_PA0, BIT_PA0);
   set_output(PROTOCOL_PORT_PA1, PROTOCOL_BIT_PA1, PORT_PA1, BIT_PA1);
   set_output(PROTOCOL_PORT_PA2, PROTOCOL_BIT_PA2, PORT_PA2, BIT_PA2);
   set_output(PROTOCOL_PORT_PA3, PROTOCOL_BIT_PA3, PORT_PA3, BIT_PA3);
   set_output(PROTOCOL_PORT_PA4, PROTOCOL_BIT_PA4, PORT_PA4, BIT_PA4);
   set_output(PROTOCOL_PORT_PA5, PROTOCOL_BIT_PA5, PORT_PA5, BIT_PA5);
   set_output(PROTOCOL_PORT_PA6, PROTOCOL_BIT_PA6, PORT_PA6, BIT_PA6);
   set_output(PROTOCOL_PORT_PE7, PROTOCOL_BIT_PE7, PORT_PE7, BIT_PE7);

   (dataRCB & 0x02) ? led_set(PLATFORM_RCB, RCB_LED_0, PLATFORM_LED_ON) : led_set(PLATFORM_RCB, RCB_LED_0, PLATFORM_LED_OFF);
   (dataRCB & 0x04) ? led_set(PLATFORM_RCB, RCB_LED_1, PLATFORM_LED_ON) : led_set(PLATFORM_RCB, RCB_LED_1, PLATFORM_LED_OFF);
   (dataRCB & 0x08) ? led_set(PLATFORM_RCB, RCB_LED_2, PLATFORM_LED_ON) : led_set(PLATFORM_RCB, RCB_LED_2, PLATFORM_LED_OFF);

#else

   uint8_t dataRCB = dataFrame->digitalData[0];
   uint8_t dataSTBOnboard = dataFrame->digitalData[1];

   (dataSTBOnboard & 0x01) ? led_set(PLATFORM_STB, STB_LED_0, PLATFORM_LED_ON) : led_set(PLATFORM_STB, STB_LED_0, PLATFORM_LED_OFF);
   (dataSTBOnboard & 0x02) ? led_set(PLATFORM_STB, STB_LED_1, PLATFORM_LED_ON) : led_set(PLATFORM_STB, STB_LED_1, PLATFORM_LED_OFF);

   /*
    * Changes for RCB212 and RCB231 Devices
    * PORT_I2C_SCL - not set, because this Pin is Transceiver Interrupt
    */
#if (PLATFORM == RCB212) || (PLATFORM == RCB231)
   set_output(PROTOCOL_PORT_PE4, PROTOCOL_BIT_PE4, PORT_PE4, BIT_PE4);
   set_output(PROTOCOL_PORT_PE5, PROTOCOL_BIT_PE5, PORT_PE5, BIT_PE5);
   set_output(PROTOCOL_PORT_PD5, PROTOCOL_BIT_PD5, PORT_PD5, BIT_PD5);
   set_output(PROTOCOL_PORT_PD7, PROTOCOL_BIT_PD7, PORT_PD7, BIT_PD7);
   set_output(PROTOCOL_PORT_PB6, PROTOCOL_BIT_PB6, PORT_PB6, BIT_PB6);
   set_output(PROTOCOL_PORT_PB7, PROTOCOL_BIT_PB7, PORT_PB7, BIT_PB7);

   set_output(PROTOCOL_PORT_X4_REL1, PROTOCOL_BIT_X4_REL1, PORT_X4_REL1, BIT_X4_REL1);
   set_output(PROTOCOL_PORT_X4_REL2, PROTOCOL_BIT_X4_REL2, PORT_X4_REL2, BIT_X4_REL2);
   set_output(PROTOCOL_PORT_I2C_SDA, PROTOCOL_BIT_I2C_SDA, PORT_I2C_SDA, BIT_I2C_SDA);
   set_output(PROTOCOL_PORT_UART_RXD, PROTOCOL_BIT_UART_RXD, PORT_UART_RXD, BIT_UART_RXD);
   set_output(PROTOCOL_PORT_UART_TXD, PROTOCOL_BIT_UART_TXD, PORT_UART_TXD, BIT_UART_TXD);
#endif

   /*
    * Changes for RCB230 Devices
    * nothing changed - we can use all pins
    */
#if PLATFORM == RCB230
   set_output(PROTOCOL_PORT_PE4, PROTOCOL_BIT_PE4, PORT_PE4, BIT_PE4);
   set_output(PROTOCOL_PORT_PE5, PROTOCOL_BIT_PE5, PORT_PE5, BIT_PE5);
   set_output(PROTOCOL_PORT_PD5, PROTOCOL_BIT_PD5, PORT_PD5, BIT_PD5);
   set_output(PROTOCOL_PORT_PD7, PROTOCOL_BIT_PD7, PORT_PD7, BIT_PD7);
   set_output(PROTOCOL_PORT_PB6, PROTOCOL_BIT_PB6, PORT_PB6, BIT_PB6);
   set_output(PROTOCOL_PORT_PB7, PROTOCOL_BIT_PB7, PORT_PB7, BIT_PB7);

   set_output(PROTOCOL_PORT_X4_REL1, PROTOCOL_BIT_X4_REL1, PORT_X4_REL1, BIT_X4_REL1);
   set_output(PROTOCOL_PORT_X4_REL2, PROTOCOL_BIT_X4_REL2, PORT_X4_REL2, BIT_X4_REL2);
   set_output(PROTOCOL_PORT_I2C_SCL, PROTOCOL_BIT_I2C_SCL, PORT_I2C_SCL, BIT_I2C_SCL);
   set_output(PROTOCOL_PORT_I2C_SDA, PROTOCOL_BIT_I2C_SDA, PORT_I2C_SDA, BIT_I2C_SDA);
   set_output(PROTOCOL_PORT_UART_RXD, PROTOCOL_BIT_UART_RXD, PORT_UART_RXD, BIT_UART_RXD);
   set_output(PROTOCOL_PORT_UART_TXD, PROTOCOL_BIT_UART_TXD, PORT_UART_TXD, BIT_UART_TXD);
#endif

   /*
    * Changes for RCBSINGLE Devices
    * PORT_PE4 - not set, on STB Pin PE4 it's set to RSTON and on deRFtoRCBAdapter its routed to EXT0.17 which is #WR on STB, on deRFtoRCBAdapter it's also one LED
    * PORT_PE5 - not set, on STB its routed to PE5 AND #RD Pin, on deRFtoRCBAdapter it's the Button
    * PORT_PD7 - not set, on STB its routed to PD7 AND PC7, but PC7 is also enable pin for STB I/O's (USB_CE/IO_CE)
    * PORT_PB6 - not set, on STB Pin PB6 it's set to GND and on deRFtoRCBAdapter its routed to EXT1.29 which is PA6 on STB
    */
#if PLATFORM == RCBSINGLE
   // PD5 is routed on STB to PD5 AND PC5 (but this doesn't matter)
   set_output(PROTOCOL_PORT_PD5, PROTOCOL_BIT_PD5, PORT_PD5, BIT_PD5);
   // on Single Chip Platform PB7 is PG1
   set_output(PROTOCOL_PORT_PB7, PROTOCOL_BIT_PB7, PORT_PB7, BIT_PB7);

   set_output(PROTOCOL_PORT_X4_REL1, PROTOCOL_BIT_X4_REL1, PORT_X4_REL1, BIT_X4_REL1);
   set_output(PROTOCOL_PORT_X4_REL2, PROTOCOL_BIT_X4_REL2, PORT_X4_REL2, BIT_X4_REL2);
   set_output(PROTOCOL_PORT_I2C_SCL, PROTOCOL_BIT_I2C_SCL, PORT_I2C_SCL, BIT_I2C_SCL);
   set_output(PROTOCOL_PORT_I2C_SDA, PROTOCOL_BIT_I2C_SDA, PORT_I2C_SDA, BIT_I2C_SDA);
   set_output(PROTOCOL_PORT_UART_RXD, PROTOCOL_BIT_UART_RXD, PORT_UART_RXD, BIT_UART_RXD);
   set_output(PROTOCOL_PORT_UART_TXD, PROTOCOL_BIT_UART_TXD, PORT_UART_TXD, BIT_UART_TXD);
#endif

   (dataRCB & 0x02) ? led_set(PLATFORM_RCB, RCB_LED_0, PLATFORM_LED_ON) : led_set(PLATFORM_RCB, RCB_LED_0, PLATFORM_LED_OFF);
   (dataRCB & 0x04) ? led_set(PLATFORM_RCB, RCB_LED_1, PLATFORM_LED_ON) : led_set(PLATFORM_RCB, RCB_LED_1, PLATFORM_LED_OFF);
   (dataRCB & 0x08) ? led_set(PLATFORM_RCB, RCB_LED_2, PLATFORM_LED_ON) : led_set(PLATFORM_RCB, RCB_LED_2, PLATFORM_LED_OFF);

#endif // RCB_BREAKOUT
#endif // NODETYPE != COORD
}

/**
 * @brief Helper function to set specific output on specified port.
 *
 * This function set all outputs which defined as outputs on data frame. Depending on actual
 * data frame the I/O is checked (if output) and than set to value defined in data frame.
 *
 * @param   protocol_port  defines the byte aligned position of output pin inside data frame
 * @param   protocol_bit   defines the bit position of output pin inside the data frame port
 * @param   hardware_port  select the port where output pin is connected to
 * @param   hardware_bit   select the bit on port 'hardware_port' where output pin is connected
 */
void set_output(uint8_t protocol_port, uint8_t protocol_bit, uint8_t hardware_port, uint8_t hardware_bit)
{
#if NODETYPE != COORD
   payloadDataFrame_t* dataFrame = (payloadDataFrame_t*)&dataConfiguration.payload;
   // check if output
   if((dataFrame->digitalIODirection[protocol_port] & _BV(protocol_bit)) == 0x00)
   {
      if((dataFrame->digitalData[protocol_port] & _BV(protocol_bit)) != 0x00)
      {// switch on
         switch(hardware_port)
         {
         case SELECT_PORT_A:
            PORTA &= ~_BV(hardware_bit);
            DDRA  |=  _BV(hardware_bit);
            break;
         case SELECT_PORT_B:
            PORTB &= ~_BV(hardware_bit);
            DDRB  |=  _BV(hardware_bit);
            break;
         case SELECT_PORT_D:
            PORTD &= ~_BV(hardware_bit);
            DDRD  |=  _BV(hardware_bit);
            break;
         case SELECT_PORT_E:
            PORTE &= ~_BV(hardware_bit);
            DDRE  |=  _BV(hardware_bit);
            break;
         case SELECT_PORT_G:
            PORTG &= ~_BV(hardware_bit);
            DDRG  |=  _BV(hardware_bit);
            break;
         }
      }
      else
      { // switch off
         switch(hardware_port)
         {
         case SELECT_PORT_A:
            PORTA |= _BV(hardware_bit);
            DDRA  |= _BV(hardware_bit);
            break;
         case SELECT_PORT_B:
            PORTB |= _BV(hardware_bit);
            DDRB  |= _BV(hardware_bit);
            break;
         case SELECT_PORT_D:
            PORTD |= _BV(hardware_bit);
            DDRD  |= _BV(hardware_bit);
            break;
         case SELECT_PORT_E:
            PORTE |= _BV(hardware_bit);
            DDRE  |= _BV(hardware_bit);
            break;
         case SELECT_PORT_G:
            PORTG |= _BV(hardware_bit);
            DDRG  |= _BV(hardware_bit);
            break;
         }
      }
   }
#endif // NODETYPE != COORD
}

/**
 * @brief Check specified data protocol pin on corresponding hardware port pin.
 *
 * @param   protocol_port  defines the byte aligned position of output pin inside data frame
 * @param   protocol_bit   defines the bit position of output pin inside the data frame port
 * @param   hardware_port  select the port where output pin is connected to
 * @param   hardware_bit   select the bit on port 'hardware_port' where output pin is connected
 */
uint8_t check_input(uint8_t protocol_port, uint8_t protocol_bit, uint8_t hardware_port, uint8_t hardware_bit)
{
   uint8_t stateChanged = 0;
#if NODETYPE != COORD

   payloadDataFrame_t* dataFrame = (payloadDataFrame_t*)&dataConfiguration.payload;

   if((dataFrame->digitalIODirection[protocol_port] & _BV(protocol_bit)) != 0x00)
   {
      if(read_io(hardware_port, hardware_bit))
      {
         if((dataFrame->digitalData[protocol_port] & _BV(protocol_bit)) == 0x00)
         { // if zero, bit was not set -> we have to set it
            dataFrame->digitalData[protocol_port] |= _BV(protocol_bit);
            stateChanged = 1;
         }
      }
      else
      {
         if((dataFrame->digitalData[protocol_port] & _BV(protocol_bit)) != 0x00)
         { // if not zero, bit was set -> we have to reset it
            dataFrame->digitalData[protocol_port] &= ~_BV(protocol_bit);
            stateChanged = 1;
         }
      }
   }
#endif // NODETYPE != COORD
   return stateChanged;
}

/**
 * @brief Read input status from selected pin
 *
 * @param   port  defines the port where the pin ic located
 * @param   pin   defines the pin
 *
 * @return        true, when input pin is active, otherwise false
 */
bool read_io(uint8_t port, uint8_t pin)
{
   bool result = false;
#if NODETYPE != COORD
   switch(port)
   {
   case SELECT_PORT_A:
      PORTA |= (1 << pin);
      DDRA &= ~(1 << pin);
      if(PINA & (1 << pin)) { result = true; }
      break;
   case SELECT_PORT_B:
      PORTB |= (1 << pin);
      DDRB &= ~(1 << pin);
      if(PINB & (1 << pin)) { result = true; }
      break;
   case SELECT_PORT_D:
      PORTD |= (1 << pin);
      DDRD &= ~(1 << pin);
      if(PIND & (1 << pin)) { result = true; }
      break;
   case SELECT_PORT_E:
      PORTE |= (1 << pin);
      DDRE &= ~(1 << pin);
      if(PINE & (1 << pin)) { result = true; }
      break;
   case SELECT_PORT_G:
      PORTG |= (1 << pin);
      DDRG &= ~(1 << pin);
      if(PING & (1 << pin)) { result = true; }
      break;
   }
#endif // NODETYPE != COORD
   return result;
}

/**
 * @brief Checks periodically all inputs.
 *
 * If any change occurs, a new data message is generated and is transmitted to coordinator.
 * This function is called after timer with preloaded value ITERATION_TIME_INPUT_IO expired.
 */
void check_io_components(void)
{
#if NODETYPE != COORD
   uint8_t stateChanged = 0;

   payloadDataFrame_t* dataFrame     = (payloadDataFrame_t*)&dataConfiguration.payload;

   /*
    * RCB Input Components -> just button
    */
   if(!button_pressed(PLATFORM_RCB))
   {
      // if button not pressed but it was before -> change data
      if((dataFrame->digitalData[RCB_BUTTON_PORT_POSITION] & _BV(RCB_BUTTON_BIT_POSITION)) != 0x00)
      {
         dataFrame->digitalData[RCB_BUTTON_PORT_POSITION] &= ~_BV(RCB_BUTTON_BIT_POSITION);
         stateChanged++;
      }
   }
   // button is not pressed OR not ANY MORE
   else
   {
      // if button pressed and it was not before -> change data
      if((dataFrame->digitalData[RCB_BUTTON_PORT_POSITION] & _BV(RCB_BUTTON_BIT_POSITION)) == 0x00)
      {
         dataFrame->digitalData[RCB_BUTTON_PORT_POSITION] |= _BV(RCB_BUTTON_BIT_POSITION);
         stateChanged++;
      }
   }

#ifdef RCB_BREAKOUT
   stateChanged += check_input(PROTOCOL_PORT_PA0, PROTOCOL_BIT_PA0, PORT_PA0, BIT_PA0);
   stateChanged += check_input(PROTOCOL_PORT_PA1, PROTOCOL_BIT_PA1, PORT_PA1, BIT_PA1);
   stateChanged += check_input(PROTOCOL_PORT_PA2, PROTOCOL_BIT_PA2, PORT_PA2, BIT_PA2);
   stateChanged += check_input(PROTOCOL_PORT_PA3, PROTOCOL_BIT_PA3, PORT_PA3, BIT_PA3);
   stateChanged += check_input(PROTOCOL_PORT_PA4, PROTOCOL_BIT_PA4, PORT_PA4, BIT_PA4);
   stateChanged += check_input(PROTOCOL_PORT_PA5, PROTOCOL_BIT_PA5, PORT_PA5, BIT_PA5);
   stateChanged += check_input(PROTOCOL_PORT_PA6, PROTOCOL_BIT_PA6, PORT_PA6, BIT_PA6);
   stateChanged += check_input(PROTOCOL_PORT_PE7, PROTOCOL_BIT_PE7, PORT_PE7, BIT_PE7);
#else
   /*
    * STB Predefined Input Components -> just button
    */
   if(!button_pressed(PLATFORM_STB))
   {
      // if button not pressed but it was before -> change data
      if((dataFrame->digitalData[STB_BUTTON_PORT_PROTOCOL] & _BV(STB_BUTTON_BIT_POSITION)) != 0x00)
      {
         dataFrame->digitalData[STB_BUTTON_PORT_PROTOCOL] &= ~_BV(STB_BUTTON_BIT_POSITION);
         stateChanged++;
      }
   }
   // button is not pressed OR not ANY MORE
   else
   {
      // if button pressed and it was not before -> change data
      if((dataFrame->digitalData[STB_BUTTON_PORT_PROTOCOL] & _BV(STB_BUTTON_BIT_POSITION)) == 0x00)
      {
         dataFrame->digitalData[STB_BUTTON_PORT_PROTOCOL] |= _BV(STB_BUTTON_BIT_POSITION);
         stateChanged++;
      }
   }

   /*
    * Changes for RCB212 and RCB231 Devices
    * PORT_I2C_SCL - not set, because this Pin is Transceiver Interrupt
    */
#if (PLATFORM == RCB212) || (PLATFORM == RCB231)
   stateChanged += check_input(PROTOCOL_PORT_PE4, PROTOCOL_BIT_PE4, PORT_PE4, BIT_PE4);
   stateChanged += check_input(PROTOCOL_PORT_PE5, PROTOCOL_BIT_PE5, PORT_PE5, BIT_PE5);
   stateChanged += check_input(PROTOCOL_PORT_PD5, PROTOCOL_BIT_PD5, PORT_PD5, BIT_PD5);
   stateChanged += check_input(PROTOCOL_PORT_PD7, PROTOCOL_BIT_PD7, PORT_PD7, BIT_PD7);
   stateChanged += check_input(PROTOCOL_PORT_PB6, PROTOCOL_BIT_PB6, PORT_PB6, BIT_PB6);
   stateChanged += check_input(PROTOCOL_PORT_PB7, PROTOCOL_BIT_PB7, PORT_PB7, BIT_PB7);

   stateChanged += check_input(PROTOCOL_PORT_I2C_SDA, PROTOCOL_BIT_I2C_SDA, PORT_I2C_SDA, BIT_I2C_SDA);
   stateChanged += check_input(PROTOCOL_PORT_UART_RXD, PROTOCOL_BIT_UART_RXD, PORT_UART_RXD, BIT_UART_RXD);
   stateChanged += check_input(PROTOCOL_PORT_UART_TXD, PROTOCOL_BIT_UART_TXD, PORT_UART_TXD, BIT_UART_TXD);
#endif

   /*
    * Changes for RCB230 Devices
    * nothing changed - we can use all pins
    */
#if PLATFORM == RCB230
   stateChanged += check_input(PROTOCOL_PORT_PE4, PROTOCOL_BIT_PE4, PORT_PE4, BIT_PE4);
   stateChanged += check_input(PROTOCOL_PORT_PE5, PROTOCOL_BIT_PE5, PORT_PE5, BIT_PE5);
   stateChanged += check_input(PROTOCOL_PORT_PD5, PROTOCOL_BIT_PD5, PORT_PD5, BIT_PD5);
   stateChanged += check_input(PROTOCOL_PORT_PD7, PROTOCOL_BIT_PD7, PORT_PD7, BIT_PD7);
   stateChanged += check_input(PROTOCOL_PORT_PB6, PROTOCOL_BIT_PB6, PORT_PB6, BIT_PB6);
   stateChanged += check_input(PROTOCOL_PORT_PB7, PROTOCOL_BIT_PB7, PORT_PB7, BIT_PB7);

   stateChanged += check_input(PROTOCOL_PORT_I2C_SCL, PROTOCOL_BIT_I2C_SCL, PORT_I2C_SCL, BIT_I2C_SCL);
   stateChanged += check_input(PROTOCOL_PORT_I2C_SDA, PROTOCOL_BIT_I2C_SDA, PORT_I2C_SDA, BIT_I2C_SDA);
   stateChanged += check_input(PROTOCOL_PORT_UART_RXD, PROTOCOL_BIT_UART_RXD, PORT_UART_RXD, BIT_UART_RXD);
   stateChanged += check_input(PROTOCOL_PORT_UART_TXD, PROTOCOL_BIT_UART_TXD, PORT_UART_TXD, BIT_UART_TXD);
#endif

   /*
    * Changes for RCBSINGLE Devices
    * PORT_PE4 - not set, on STB Pin PE4 it's set to RSTON and on deRFtoRCBAdapter its routed to EXT0.17 which is #WR on STB, on deRFtoRCBAdapter it's also one LED
    * PORT_PE5 - not set, on STB its routed to PE5 AND #RD Pin, on deRFtoRCBAdapter it's the Button
    * PORT_PD7 - not set, on STB its routed to PD7 AND PC7, but PC7 is also enable pin for STB I/O's (USB_CE/IO_CE)
    * PORT_PB6 - not set, on STB Pin PB6 it's set to GND and on deRFtoRCBAdapter its routed to EXT1.29 which is PA6 on STB
    */
#if PLATFORM == RCBSINGLE
   stateChanged += check_input(PROTOCOL_PORT_PD5, PROTOCOL_BIT_PD5, PORT_PD5, BIT_PD5);
   stateChanged += check_input(PROTOCOL_PORT_PB7, PROTOCOL_BIT_PB7, PORT_PB7, BIT_PB7);

   stateChanged += check_input(PROTOCOL_PORT_I2C_SDA, PROTOCOL_BIT_I2C_SDA, PORT_I2C_SDA, BIT_I2C_SDA);
   stateChanged += check_input(PROTOCOL_PORT_I2C_SCL, PROTOCOL_BIT_I2C_SCL, PORT_I2C_SCL, BIT_I2C_SCL);
   stateChanged += check_input(PROTOCOL_PORT_UART_RXD, PROTOCOL_BIT_UART_RXD, PORT_UART_RXD, BIT_UART_RXD);
   stateChanged += check_input(PROTOCOL_PORT_UART_TXD, PROTOCOL_BIT_UART_TXD, PORT_UART_TXD, BIT_UART_TXD);
#endif


#endif

   if(stateChanged > 0)
   {
      dataConfiguration.command = COMMAND_DATA_FRAME_RESPONSE;
      dataConfiguration.option = NO_OPTION;
      dataFrame->mac = macConfig.longAddr;

      // refresh actual temperature
      payloadDataFrame_t* dataFrame = (payloadDataFrame_t*)&dataConfiguration.payload;
      dataFrame->analogData[0] = actual_temp;
      dataFrame->analogData[1] = actual_vcc;

      send_data_wireless(DEFAULT_COORD_ADDR, (uint8_t*)&dataConfiguration, sizeof(deRFprotocol_t) ,UDP_PORT_END_ROUTER, UDP_PORT_COORD);
   }

   macSetAlarm(ITERATION_TIME_INPUT_IO, check_io_components);

#endif // NODETYPE != COORD
}

/**
 * @brief Checks periodically temperature and voltage of node.
 *
 * If any changes recognized, a message is generated and the actual temperature and voltage
 * is send out over wireless interface. This function is called after timer with preloaded
 * value ITERATION_TIME_TEMP_VCC expired.
 */
void check_temp_and_vcc(void)
{
#if NODETYPE != COORD
   uint8_t stateChanged = 0;

   payloadDataFrame_t* dataFrame = (payloadDataFrame_t*)&dataConfiguration.payload;

   init_adc();

   //SingleChip does not use STB temperature sensor -> no need to activate STB temp sensor
#ifdef SINGLE_CHIP
   uint32_t volt = get_vcc();
   int32_t temp = temp_get_degrcelc();
#else
   stb_status_temperature(STB_TEMPERATURE_ON);
   uint32_t volt = get_vcc();
   int32_t temp = temp_get_degrcelc();
   stb_status_temperature(STB_TEMPERATURE_OFF);
#endif

   //check if temperature changed -> only send changes about +-TEMPERATURE_DIFFERENCE milli degree
   if((actual_temp + TEMPERATURE_DIFFERENCE) <= temp || (actual_temp - TEMPERATURE_DIFFERENCE) > temp)
   {
      actual_temp = temp;
      stateChanged++;
   }

   //check if voltage changed -> only send changes about +-VCC_DIFFERENCE micro volt
   if((actual_vcc + VCC_DIFFERENCE) <= volt || (actual_vcc - VCC_DIFFERENCE) > volt)
   {
      actual_vcc = volt;
      stateChanged++;
   }

   if(stateChanged > 0)
   {
      dataConfiguration.command = COMMAND_DATA_FRAME_RESPONSE;
      dataConfiguration.option = NO_OPTION;
      dataFrame->mac = macConfig.longAddr;

      dataFrame->analogData[0] = actual_temp;
      dataFrame->analogData[1] = actual_vcc;

      send_data_wireless(DEFAULT_COORD_ADDR, (uint8_t*)&dataConfiguration, sizeof(deRFprotocol_t) ,UDP_PORT_END_ROUTER, UDP_PORT_COORD);
   }

   // call again after ITERATION_TIME_TEMP_VCC timeouts
   macSetAlarm(ITERATION_TIME_TEMP_VCC, check_temp_and_vcc);
#endif // NODETYPE != COORD
}



/* EOF */

