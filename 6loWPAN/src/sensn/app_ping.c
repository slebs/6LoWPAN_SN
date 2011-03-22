#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../../inc/sensn/app_interface.h"
#include "../../inc/sensn/commands.h"
#include "../../inc/avr_sixlowpan.h"
#include "../../inc/avr_timer.h"
#include "../../inc/mac.h"
#include "../../inc/mac_scan.h"
#include "../../inc/mac_data.h"
#include "../../inc/mac_event.h"
#include "../../inc/rum_types.h"
#include "../../inc/hal.h"
#include "../../inc/system.h"
#include "../../inc/radio.h"
#include "../../inc/sensors.h"
#include "../../inc/avr_sixlowpan.h"
#include "../../inc/mac_associate.h"
#include "../../inc/deRFaddon/deRFapplication.h"
#include "../../inc/deRFaddon/uart.h"
#include "../../inc/deRFaddon/io_access.h"

u16 pingdelay = 0;
uint16_t dest_addr_ping;
uint16_t iterations;
u8 cvar = 0;
u8 utimer;
bool do_ = false;

/*
 * starts/stops infinity ping
 */
void ping_button_ev() {
	if (do_ == true) {
		do_ = false;
		macTimerEnd(utimer);

	} else {
		do_ = true;
		if (NODETYPE == COORD) {

			sendPing(DEFAULT_FIRST_NODE_ADDR, 0);
		} else {
			sendPing(DEFAULT_COORD_ADDR, 0);
		}
	}
}

void pingMacro() {
	sendPing(dest_addr_ping, iterations);
}

/*
 * function sends a periodic ping request over UDP to the dest_addr_ping
 * @param dest_addr_ping Address the ping request is sent
 */
void sendPing(uint16_t dest, uint16_t iter) {
	dest_addr_ping = dest;
	iterations = iter;
	// Setup a Ping Request packet to be send to the destination address
	deRFprotocol_t frame;
	frame.command = COMMAND_PING_REQUEST;
	frame.option = NO_OPTION;

	if (NODETYPE == COORD) {
		pingdelay = macGetTime();
		send_data_wireless(dest, (uint8_t *) &frame, sizeof(deRFprotocol_t),
				UDP_PORT_SENSN_COORD, UDP_PORT_SENSN_END_ROUTER);
	}

	if (NODETYPE == ENDDEVICE || NODETYPE == ROUTER) {
		pingdelay = macGetTime();
		send_data_wireless(dest, (uint8_t *) &frame, sizeof(deRFprotocol_t),
				UDP_PORT_SENSN_END_ROUTER, UDP_PORT_SENSN_COORD);
		UART_PRINT("Send Ping Request to 0x%u \r\n", dest);
	}
	// cvar repeats
	if (iter == 0) {
		utimer = macSetAlarm(500, pingMacro);
	} else {
		if (++cvar < iter) {
			utimer = macSetAlarm(500, pingMacro);
		} else {
			cvar = 0;
		}
	}
}

void app_ping_coord_process(uint8_t* pUDPpacket, int16_t originAddr) {
	if (*pUDPpacket == COMMAND_PING_REQUEST) {
		UART_PRINT("Coord got Ping request\r\n");
		LED_TOGGLE(LED_WORKING);

		//setup response frame TODO response with a modified frame not a new one
		deRFprotocol_t frame;
		payloadPingFrame_t* pingFrame = (payloadPingFrame_t*) &frame.payload;
		frame.command = COMMAND_PING_RESPONSE;
		frame.option = NO_OPTION;
		pingFrame->mac = macConfig.longAddr;

		// send ping response to origin device/node
		send_data_wireless(originAddr, (uint8_t *) &frame,
				sizeof(deRFprotocol_t), UDP_PORT_SENSN_COORD,
				UDP_PORT_SENSN_END_ROUTER);
	}
	if (*pUDPpacket == COMMAND_PING_RESPONSE) {
		pingdelay = macGetTime() - pingdelay;
		LED_TOGGLE(LED_WORKING);
		UART_PRINT("coord got a ping response, ping took: %ums\r\n", pingdelay);
	}

}

void app_ping_device_process(uint8_t* pUDPpacket, int16_t originAddr) {
	if (*pUDPpacket == COMMAND_PING_REQUEST) {
		UART_PRINT("Node/Router got Ping request\r\n");
		LED_TOGGLE(LED_WORKING);

		//setup response frame TODO response with a modified frame not a new one
		deRFprotocol_t frame;
		frame.command = COMMAND_PING_RESPONSE;
		frame.option = NO_OPTION;

		// send ping response to origin device/node
		send_data_wireless(originAddr, (uint8_t *) &frame,
				sizeof(deRFprotocol_t), UDP_PORT_SENSN_END_ROUTER,
				UDP_PORT_SENSN_COORD);
	}
	if (*pUDPpacket == COMMAND_PING_RESPONSE) {
		pingdelay = macGetTime() - pingdelay;
		LED_TOGGLE(LED_WORKING);
		UART_PRINT("Node/Router got a ping response, ping took: %ums\r\n", pingdelay);
	}
}
