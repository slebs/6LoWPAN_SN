/*
 * sensor_network.c
 *
 *  Created on: Nov 18, 2010
 *      Author: simon
 */

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

u8 buttondebounce = 0; // stat for buttondebounce

u8 stat = 0;
int count = 0;
char mac_buf[MAX_FRAME_LENGTH_SN];

//MACRO for debounce delay
void setButtonDebounce(void) {
	buttondebounce = 0;
}

/**
 function, starts the network by scanning channels for a coordinator.

 This node will either scan all available channels, or just one
 channel if @ref macSetScanChannel() is called.  @see macScan().
 */
void appStartScan(void) {
	if (NODETYPE != COORD) {
		macInit(0xff);
		macScan();
	}
}

void alive_pulse() {
	LED_TOGGLE(LED_ALIVE);
	macSetAlarm(1000, alive_pulse);
}

/*
 * Initials the network
 */
void app_init() {

#ifdef SENSORNETWORK
	UART_PRINT("SENSORNETWORK defined\r\n");
	i2c_init();
#endif

#ifdef APP_PERF
	UART_PRINT("APP_PERF defined\r\n");
#endif
#ifdef APP_PING
	UART_PRINT("APP_PING defined\r\n");
#endif

#if (__AVR__)
	// If the EEPROM is cleared, init it to something useful
	checkEeprom();
#endif // __AVR__
	alive_pulse();
	BUTTON_SETUP();

	if (NODETYPE == ENDDEVICE || NODETYPE == ROUTER) {
		appStartScan();
		if (IPV6LOWPAN == 1)
			sixlowpan_init();
		if (NODETYPE == ENDDEVICE) {
			UART_PRINT("--Device is EndNode\r\n");
		} else {
			UART_PRINT("--Device is Router\r\n");
		}

		//		UART_PRINT("--some information:\r\n\t-sizeof(int)= %d\ Bytes\r\n\t-sizeof(uint8_t)= %d\r\n\t"
		//				"-sizeof(uint16_t)= %d\r\n\t-sizeof(uint32_t)= %d \r\n--end of information\r\n"
		//				,sizeof(int),sizeof(uint8_t),sizeof(uint16_t),sizeof(uint32_t));
	}
	if (NODETYPE == COORD) {
		macFindClearChannel();
		sixlowpan_init();
		UART_PRINT("--Device is Coordinator\r\n");
	}

}

/*
 * periodically called function in main
 */
void loopTask(void) {
	/*
	 * Here the applications loop tasks should be called
	 */
#ifdef APP_PERF
	perf_loop_task();
#endif

#ifdef SENSORNETWORK
	fh_com_looptask();
#endif

	/*
	 * Button Press
	 * Here the applications button press logic should be called
	 */
	if (button_pressed(PLATFORM_RCB) && buttondebounce == 0) {

#ifdef APP_PERF
		perf_button_ev();
#endif

#ifdef APP_PING
		ping_button_ev();
#endif

		// code to debbounce the button and avoid multiple button press
		buttondebounce = 1;
		macSetAlarm(200, setButtonDebounce);
	}
}

// TODO Implement behavior on incoming UDP packets for the Coordinator
/*
 * param: pUDPpacket Pointer zur Payload
 * param: payloadlen
 * param: address packet originates from
 */
void process_coord_udp_packet_SN(uint8_t* pUDPpacket, uint8_t payloadlen,
		uint16_t originAddr) {

	/*
	 * Process incoming Ping packets
	 */
	if ((*pUDPpacket == COMMAND_PING_REQUEST) || (*pUDPpacket
			== COMMAND_PING_RESPONSE)) {
		app_ping_coord_process(pUDPpacket, originAddr);
	} else if (*pUDPpacket == COMMAND_COORD_DATA_RESPONSE) {
		app_fh_com_process_data_res(pUDPpacket);
	} else {

#ifdef APP_PERF
		compare_test_data(pUDPpacket, payloadlen);
#else
		UART_PRINT("-----Coord hat einen String oder ein unbekanntes Paket empfangen\r\n");
		UART_PRINT("-----Payloadlen: %u\r\n",payloadlen);
		UART_PRINT("-----originAddr: 0x%u\r\n", originAddr);
#endif
	}
}
// Test
/* Implements behavior on incoming UDP packets for Endnodes/Routers
 *
 * param: pUDPpacket pointer zur payload
 * param: payloadlen
 * param: address packet originates from
 */
void process_endnode_udp_packet_SN(uint8_t* pUDPpacket, uint8_t payloadlen,
		uint16_t originAddr) {

	/*
	 * Process incoming Ping packets
	 */
	if ((*pUDPpacket == COMMAND_PING_REQUEST) || (*pUDPpacket
			== COMMAND_PING_RESPONSE)) {
		app_ping_device_process(pUDPpacket, originAddr);
	} else

	if (*pUDPpacket == COMMAND_COORD_DATA_REQUEST) {
		app_fh_com_process_data_req(pUDPpacket);
	} else
	/*
	 * Process incoming unknown package and print it as String
	 */
	{
		UART_PRINT("-----ENDNODE hat ein unbekanntes Paket empfangen\r\n");
		UART_PRINT("-----Payloadlen: %u\r\n",payloadlen);
		UART_PRINT("-----originAddr: 0x%u\r\n", originAddr);
		memcpy(mac_buf, pUDPpacket, payloadlen);
		UART_PRINT("-----%s\r\n",mac_buf);
	}
}
/**
 * @brief Send out a UDP Packet over wireless interface.
 *
 * @param   destAddr    short address of destination node
 * @param   pData       pointer to data packet
 * @param   len         length of data packet
 * @param   srcUDPPort  UDP port of source node
 * @param   destUDPPort UDP port of destination node
 */
void send_SN_data_wireless(uint16_t destAddr, uint8_t* pData, uint8_t len,
		uint16_t srcUDPPort, uint16_t destUDPPort) {
	sixlowpan_hc01_udp_setup_iplocal(destAddr);
	sixlowpan_hc01_udp_setup_ports(srcUDPPort, destUDPPort);

	memcpy(sixlowpan_hc01_udp_get_payloadptr(), pData, len);
	sixlowpan_hc01_udp_set_payloadsize(len);
	sixlowpan_hc01_udp_send();
}

