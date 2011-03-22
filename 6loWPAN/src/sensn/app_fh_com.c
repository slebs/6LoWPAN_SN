/*
 * app_fh_interface.c
 *
 *  Created on: 19.03.2011
 *      Author: Kevin
 */
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/interrupt.h>
#include "../../inc/mac_associate.h"

#include "../../inc/deRFaddon/hdlc_light.h"
#include "../../inc/deRFaddon/io_access.h"
#include "../../inc/deRFaddon/uart.h"
#include "../../inc/sensn/commands.h"
#include "../../inc/sensn/app_interface.h"

//------------------------------Initialisierungen--------------------------------------

static volatile uint8_t uart_str_complete = 0; // 1 .. String komplett empfangen
static volatile uint8_t uart_str_count = 0;
static unsigned volatile char uart_string[UART_MAXSTRLEN];

unsigned char raute_pos;

char command[20];
char paraBuffer[80];

//------------------------------RX Interrupt Service Routine---------------------------
ISR(USART0_RX_vect)
{
	//	rxBuffer[rxPtr++] = (unsigned char) UDR0;
	//	printf("---Char received---\n");
	unsigned char nextChar;

	// Daten aus dem Puffer lesen
	nextChar = UDR0;
	if (uart_str_complete == 0) { // wenn uart_string gerade in Verwendung, neues Zeichen verwerfen

		// Daten werden erst in string geschrieben, wenn nicht String-Ende/max Zeichenl�nge erreicht ist/string gerade verarbeitet wird
		if (nextChar != '\n' && nextChar != '\r' && uart_str_count
				< UART_MAXSTRLEN - 1) {
			uart_string[uart_str_count] = nextChar;
			uart_str_count++;
			//			printf("---Char received---\n");
		} else {
			uart_string[uart_str_count] = '\0';
			uart_str_count = 0;
			uart_str_complete = 1;
			// UART_PRINT("%s ---  %d\n", uart_string, uart_str_complete);
		}
	}
}
//-------------------------------------------------------------------------------------

void fh_com_looptask() {

	if (uart_str_complete == 1) {
		//Bearbeitung der Data-Link Befehle
		command[0] = 0;
		paraBuffer[0] = 0;
		uart_str_complete = 0;

		//	Mit UART-String
		raute_pos = (unsigned char) strlen(
				(char*) strstr((char *) uart_string, "#")) - strlen(
				(char *) uart_string);
		if (raute_pos == 255)
			raute_pos = 0;

		command[19] = 0;
		paraBuffer[19] = 0;

		sscanf((char*) &uart_string[raute_pos], "%s %s\n", command, paraBuffer);

		UART_PRINT("command: %s\n", command);
		UART_PRINT("paraBuffer %s\n", paraBuffer);

		if (strcmp(command, "#getdata") == 0) {
			if (macIsChild(atoi(paraBuffer)) == false) {
				UART_PRINT("Node nicht im Netzwerk\r\n");
			} else {
				send_SN_data_request(atoi(paraBuffer));
			}
		}

		if (strcmp(command, "#getecho") == 0) {

			if (macIsChild(atoi(paraBuffer)) == false) {
							UART_PRINT("Node nicht im Netzwerk\r\n");
						} else {
							uint16_t addr = atoi(paraBuffer);
							sendPing(addr,4);
						}
		}

		if (strcmp(command, "#getnodes") == 0) {
			//TODO Implement
		}

		if (strcmp(command, "#getnodeaddress") == 0) {
					printf("#address %d\n",macConfig.shortAddress);
				}
	}

}
/*
 * send data request
 * coord --> node (addr)
 */
void send_SN_data_request(uint16_t addr) {
	UART_PRINT("send_data_request to %d\r\n", addr);
	SN_data_frame_t pdata;
	pdata.command = COMMAND_COORD_DATA_REQUEST;
	pdata.length = 0;

	send_SN_data_wireless(addr, (uint8_t *) &pdata, sizeof(SN_data_frame_t),
			UDP_PORT_SENSN_COORD, UDP_PORT_SENSN_END_ROUTER);
}

/*
 * process incoming data request on node
 * function sends back the sensor data to coord
 * node --> coord#
 */
void app_fh_com_process_data_req(uint8_t* pUDPpacket) {
	int i = 0;
	UART_PRINT("NODE: got data request\r\n");
	char* u = get_sensor_data();
	SN_data_frame_t pdata;
	pdata.command = COMMAND_COORD_DATA_RESPONSE;
	for (i = 0; i < strlen(u); i++) {
		pdata.payload[i] = u[i];
		UART_PRINT("%c",u[i]);
	}
	pdata.length = strlen(u);

	send_SN_data_wireless(DEFAULT_COORD_ADDR, (uint8_t *) &pdata,
			sizeof(SN_data_frame_t), UDP_PORT_SENSN_END_ROUTER,
			UDP_PORT_SENSN_COORD);
}

void app_fh_com_process_data_res(uint8_t* pUDPpacket) {
	char * payload;
	UART_PRINT("COORD: got data response\r\n");
	SN_data_frame_t * frame = (SN_data_frame_t *) pUDPpacket;
	payload = frame->payload;
	payload[frame->length] = '\0';
	printf("#BOData\n%s\n#EOData\n", payload);
}

