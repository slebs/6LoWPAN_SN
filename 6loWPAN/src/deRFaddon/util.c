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
 * @file util.c
 *
 * @brief Helper functions for ADC access and calculation.
 *
 * This file implements helper functions to get access to Analog Digital Converter. Also functions
 * to convert ADC results into temperature and supply voltage values.
 *
 * $Id: util.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 *
 * @author     Dresden Elektronik: http://www.dresden-elektronik.de
 * @author     Support email: wireless@dresden-elektronik.de
 * @date       2010-01-25
 */


/* === Includes ============================================================= */

#include <avr/io.h>
#include <util/delay.h>

#include "../../inc/deRFaddon/util.h"

/* === Macros =============================================================== */

#define VOLTAGE_MEASUREMENT         (1)
#define TEMPERATURE_MEASUREMENT     (2)

/* === Globals ============================================================== */

/* === Prototypes =========================================================== */

/* === Implementation ======================================================= */

/**
  * @brief Return ADC measured value depending on measurement method. This could
  * be wether temperature measurement or supply voltage measurement.
  *
  * @param     measurement  defines which should be measured (TEMPERATURE_MEASUREMENT, VOLTAGE_MEASUREMENT)
  * @return    measured ADC value
  *
  */
uint32_t pwr_read_adc (uint8_t measurement)
{
#ifdef SINGLE_CHIP
  if(measurement == VOLTAGE_MEASUREMENT)
  {
    return 0; // Single Chip does not provide an external VCC reference on ADC
  }
  else if(measurement == TEMPERATURE_MEASUREMENT)
  {
    ADMUX = (1 << MUX3) | (1 << MUX0);   // preselect temperatur sensor
    ADCSRB |= (1 << MUX5);               // select temperature sensor
    ADMUX |= _BV(REFS1) | _BV(REFS0);    // reference is 1.6V intern
  }
#else
  if(measurement == VOLTAGE_MEASUREMENT)
  {
    ADMUX = _BV(MUX1) | _BV(MUX2) | _BV(MUX3) | _BV(MUX4); // 1.1V internal reference channel
    ADMUX |= (1<<REFS0);                                   // reference is 3,3V VCC
  }
  else if(measurement == TEMPERATURE_MEASUREMENT)
  {
    ADMUX = _BV(MUX1) | _BV(MUX0); // external temperature sensor
    ADMUX |= (1<<REFS0);           // reference is 3,3V VCC
  }
#endif
  
  ADCSRA |= (1<<ADEN);             // enable ADC
  ADCSRA |= (1<<ADSC);             // single ADC conversion
  while ( ADCSRA & (1<<ADSC) ) {;} // wait until conversion is completed

  return ADC;
}

/**
  * @brief Initialize ADC for Measurement Temperature and VCC
  *
  * As reference the 3,3V system power is used. So you have a more accurate
  * reference than the internal of the AVR. The full range of the ADC can
  * be used.
  *
  */
void init_adc (void)
{
  PRR0 &= ~(1 << PRADC);                 // power up ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // divider 64 (250 kHz clock frequency)
}

/**
  * @brief Gives back the actual temperature.
  *
  * Uses a table with precalculated voltage measures and does linear
  * interpolation between two sampling points.
  *
  * Using SingleChip the internal temperature sensor is chosen
  *
  * @return    measured temperature in milli degrees celsius
  *
  */
int32_t temp_get_degrcelc (void)
{
  static int32_t temp;

#ifdef SINGLE_CHIP
  uint32_t t = pwr_read_adc(TEMPERATURE_MEASUREMENT);
  temp = (int32_t)(((float)1.13*(float)t - 272.8) * 1000);
#else
  static struct _ttable    // table with precalculated volatages
  {
    int32_t temp; int32_t millivolts;
  } ttable[] =
  {
    { -40000 ,  86   } ,
    { -35000 ,  120  } ,
    { -30000 ,  164  } ,
    { -25000 ,  221  } ,
    { -20000 ,  292  } ,
    { -15000 ,  380  } ,
    { -10000 ,  487  } ,
    { -50000 ,  612  } ,
    { 0      ,  756  } ,
    { 5000   ,  916  } ,
    { 10000  ,  1091 } ,
    { 15000  ,  1274 } ,
    { 20000  ,  1462 } ,
    { 25000  ,  1650 } ,
    { 30000  ,  1832 } ,
    { 35000  ,  2006 } ,
    { 40000  ,  2166 } ,
    { 45000  ,  2313 } ,
    { 50000  ,  2445 } ,
    { 55000  ,  2562 } ,
    { 60000  ,  2665 } ,
    { 65000  ,  2754 } ,
    { 70000  ,  2830 } ,
    { 75000  ,  2897 } ,
  };

  int32_t i, volt;

  pwr_read_adc(3);                               // dummy read -> init ADC
  volt = pwr_read_adc(TEMPERATURE_MEASUREMENT);  // read ADC RAW value -> convert to millivolt
  volt  = (3223UL*volt);                         // 3300mV within 1024 steps are max. 3223mV
  volt /= 1000;

  for (i = 1; i < (sizeof(ttable)/sizeof(ttable[0])-1); i++)
  {
    if (volt < ttable[i].millivolts)             // look for a fitting table entry
    {
      i--;
      // linear Interpolation

      float interpol = 5.0 * (float)(volt - ttable[i].millivolts)
                       / (float)(ttable[i+1].millivolts - ttable[i].millivolts);

      temp = ttable[i].temp + (int32_t)(interpol * 1000);
      /*
      temp = ((int32_t)ttable[i].temp
              + 5000 * (int32_t) (volt - ttable[i].millivolts)
              / (int32_t)((ttable[i+1].millivolts - ttable[i].millivolts)*1000)
              );                              // convert to millivolt
      */
      break;
    }
  }
#endif
  return temp; // give back the temperature
}

/**
  * @brief Gives back the actual supply voltage.
  *
  * SingleChip does not provide supply voltage connection to ADC,
  * this is solved with use of BATMON voltage calculation
  *
  * @return    measured voltage value
  *
  */
uint32_t get_vcc (void)
{
   static uint32_t volt;

// Single Chip does not provide conversion via ADC, so the Batmon is choosen instead
#ifdef SINGLE_CHIP
   uint8_t val;
   uint8_t batmon = BATMON;        // backup

   BATMON &= ~_BV(BAT_LOW_EN);     // disable the interrupt

   // linear search, starting at the maximum value
   // The high range switch bit is interpreted as the MSB
   val = 0x1F;
   do{
       BATMON = (BATMON & ~0x1F) | val;
       _delay_us(1); // wait until conversion is completed
   }while( !(BATMON & _BV(BATMON_OK)) && (--val > 0) );

   BATMON = batmon;        // restore previuos value

   if( val & 0x10 ) // BATMON_HR is set
   {        
       volt = (uint32_t)(2550000UL+(val&0x0F)*75UL); // result in uV
   }
   else
   {
       volt = (uint32_t)(1700000UL+val*50UL);        // result in uV
   }
#else
  pwr_read_adc(VOLTAGE_MEASUREMENT); // dummy read -> init ADC
  volt = pwr_read_adc(VOLTAGE_MEASUREMENT); // read RAW ADC data
  volt = ((float)(1024 * 1.1) / (float)volt) * 1000000; // 1.1V reference selection
#endif

  return volt;
}


/* EOF */

