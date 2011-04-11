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
 $Id: main.c,v 1.1.2.1 2010/09/16 08:21:10 ele Exp $
 */

#include <stdlib.h>
#include <stdio.h>
#include "../inc/stdbool.h"
#include <avr/interrupt.h>
#include <string.h>

#include "../inc/radio.h"
#include "../inc/mac.h"
#include "../inc/mac_event.h"
#include "../inc/serial.h"
#include "../inc/mac_start.h"
#include "../inc/mac_data.h"
#include "../inc/mac_associate.h"
#include "../inc/system.h"
#include "../inc/avr_timer.h"
#include "../inc/avr_sixlowpan.h"

#include "../inc/deRFaddon/usb.h"
#include "../inc/deRFaddon/uart.h"
#include "../inc/deRFaddon/util.h"
#include "../inc/deRFaddon/deRFapplication.h"
#include "../inc/deRFaddon/data.h"
#include "../inc/deRFaddon/io_access.h"
#include "../inc/deRFaddon/link_quality.h"
#include "../inc/deRFaddon/bmm.h"

// include by simon
#include "../inc/sensn/app_interface.h"

#ifdef STATUS_DEBUG
#include "../inc/deRFaddon/status.h"
#endif

#include <avr/wdt.h>

extern void sixlowpan_button(void);

/* On restart WDT will be enabled with shortest timeout if previously
 enabled. Must turn it off before anything else, attempting to
 disable within the main() function will take too long. */

void disable_wdt(void)
__attribute__((naked))
__attribute__((section(".init3")));
void disable_wdt(void) {
	MCUSR = 0;
	wdt_disable();
}

/**
 @brief Main function of the program.
 */
int main(void) {
	// make sure external memory interface is enabled (only ATmega1281)
	XRAM_ENABLE();

	// Init USB (for serial communication)
	usb_init();

	// Init Buffer Management Module (buffer for rx and tx messages)
	bmm_buffer_init();

	// switch off LED's
	led_set(PLATFORM_RCB, RCB_LED_0, PLATFORM_LED_OFF);
	led_set(PLATFORM_RCB, RCB_LED_1, PLATFORM_LED_OFF);
	led_set(PLATFORM_RCB, RCB_LED_2, PLATFORM_LED_OFF);

	// Setup clock speed
	halSetupClock();
	if (NODETYPE != COORD) {
		set_i2c_params();
	}
#if defined(UART_DEBUG) || defined(COMMUNICATION_UART)
	//uart_init(19200);
	//uart_init(115200);
	uart_init(9600);
#endif

	// init HDLC layer (frame tagging)
	hdlc_init();

	// init serial
	if (SERIAL) {
		serial_init(NULL);
	}

	// Init the timer system for the MAC
	timerInit();

	LED_INIT();

	// Init the (sample) application rum_application
	//appInit();

	// Init sensor_network application
	app_init();
	macSetAlarm(1000, print_sensor_data);

#if defined(ROUTERNODE) || defined(ENDNODE)
	//check_io_components(); // is called via macSetAlarm() every 20 ms
	check_temp_and_vcc(); // is called via macSetAlarm() every 5000 ms

#ifdef STATUS_DEBUG
	evaluate_status_request(); // is called via macSetAlarm() every 10000ms
#endif

#endif // ROUTERNODE || ENDNODE
#ifdef COORDNODE
	send_quality_request(); // is called via macSetAlarm() every 5000 ms
#endif

#ifdef STATUS_DEBUG
	init_status_timer(); // init timestamp (free running timer) unit
	status_timer_enable(); // start timestamp unit
#endif

	UART_PRINT("Init completed\r\n");

	// Main forever loop for the application.
	for (;;) {
		// Turn interrupts on if they are off.  They should always be on
		// at this point in the loop, so just turn them on every time, in
		// case interrupts were inadvertently turned off elsewhere.
		sei();

		// Task functions called from main loop.  Either add your own task loop
		// or edit the example appTask().
		//appTask();

		//LoopTask for Appinterface (apps' periodic tasks should be called int loopTask() function
		loopTask();

		//Stack tasks are down here
		macTask();

		// added Task to check for incoming protocol messages - by Dresden Elektronik
#if defined(COMMUNICATION_UART) || defined(COMMUNICATION_USB)
		wired_packet_task();
#endif

		// send out debug messages over UART interface
		// is made up as task to prevent time interferences
		UART_QUEUED_TASK();

	}
	return 0;
}
