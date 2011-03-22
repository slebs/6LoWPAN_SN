/*
 * Application sends UDP packets with a 79Byte String to the Coordinator.
 * The Coordinator evaluates the String and calculates and prints thetransfer rate to UART
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../../inc/sensn/app_interface.h"
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

#define P_COUNT        (50)
#define P_TEST         (1000)
char mac_buf[MAX_PERFTEST_FRAME_LENGTH]; // Max payload length
char s_send[MAX_PERFTEST_FRAME_LENGTH];

uint16_t pro_counter = 0;
uint16_t co_counter = 0;
uint16_t time_from;
uint16_t time_test;
uint16_t netto_troughput;
bool do__ = false;
u8 utimer;

void perf_init() {
	do__ = false;
}

void perf_loop_task() {
	if (do__) {
		send_test_data();
	}
}

void perf_button_ev() {

	if (NODETYPE == COORD) {
		UART_PRINT("APP_PERF COORD:Button pressed");
	} else if (NODETYPE == ENDDEVICE) {
		if (do__ == true) {
			do__ = false;
			UART_PRINT("stop throughput test\r\n");
			macTimerEnd(utimer);
			LED_OFF(LED_WORKING);

		} else {
			UART_PRINT("start throughput test\r\n");
			do__ = true;
			send_test_data();

		}
	}

}
/*
 * sends test string in a UDP packet to coordinator
 */
void send_test_data() {
	//	char s_append[15];
	//	uint8_t len;
	// UART_PRINT("\r\nSende %d. Paket\r\n", ++count);
	LED_TOGGLE(LED_WORKING);
	//sprintf(s_send, "Bin das %d. Paket von 0x%d (shortadress)  ", ++count,
	//		macConfig.shortAddress);
	// Teststring to be send periodic
	sprintf(
			s_send,
			"Lorem ipsum dolor sit amet, consectetuers adipiscing elit. Aenaaaaaaean commodo ligul");
	//	len = strlen(s_send);
	//	sprintf(s_append, "L\x84nge: %d\r\n", (len + 11));
	//	strcat(s_send, s_append);
	//	UART_PRINT("String to be send:  %s",s_send);
	//	UART_PRINT("strlen of s_send: %d\r\n",strlen(s_send));

	send_data_wireless(DEFAULT_COORD_ADDR, (uint8_t *) &s_send, strlen(s_send),
			UDP_PORT_SENSN_END_ROUTER, UDP_PORT_SENSN_COORD);
}

void compare_test_data(uint8_t* pUDPpacket, uint8_t payloadlen) {
	//Check integrity of the data in a very simple way
	memcpy(mac_buf, pUDPpacket, payloadlen);

	if (strcmp(
			mac_buf,
			"Lorem ipsum dolor sit amet, consectetuers adipiscing elit. Aenaaaaaaean commodo ligul")
			== 0) {

		//measure time from first successfully received frame
		if (pro_counter == 1) {
			time_from = macGetTime();
		}
		//UART_PRINT(" ");
		//UART_PRINT("successfully received packet No. %u\r\n",pro_counter);

		if (++pro_counter >= P_COUNT) {
			time_test = macGetTime() - time_from;

			// evaluate time and co_counter
			LED_TOGGLE(LED_WORKING);
			//UART_PRINT("\033[2J\r\n");
			UART_PRINT("Received %u Frames in : %ums\r\n",pro_counter, time_test);
			netto_troughput = (((float) MAX_PERFTEST_FRAME_LENGTH * pro_counter)
					/ time_test) * 1000;
			UART_PRINT("Netto troughput was %u Bytes/s\r\n",netto_troughput);
			UART_PRINT("Es wurden %u Pakete erfolgreich empfangen\r\nDefekte Pakete: %u\r\n",pro_counter,co_counter);
			UART_PRINT("%u\r\n", netto_troughput);
			pro_counter = 0;
			co_counter = 0;
		}

	} else {
		co_counter++;
		UART_PRINT("defect frame\r\n");
	}

	//UART_PRINT("-----%s\r\n",mac_buf);

}
