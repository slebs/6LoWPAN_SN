/*
 * app_fh_interface.c
 *
 *  Created on: 19.03.2011
 *      Author: Kevin
 */

//------------------------------------------------------------------------
//|                                INCLUDES                              |
//------------------------------------------------------------------------
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

//------------------------------------------------------------------------
//|                                  INITS                               |
//------------------------------------------------------------------------

static volatile uint8_t uart_str_complete = 0; // 1 -> if string complete
static volatile uint8_t uart_str_count = 0;
static unsigned volatile char uart_string[UART_MAXSTRLEN];

unsigned char raute_pos;

char command[20];
char paraBuffer[80];

//------------------------------------------------------------------------
//|                      RX INTERRUPT SERVICE ROUTINE                    |
//------------------------------------------------------------------------
ISR(USART0_RX_vect)
{

	unsigned char nextChar;

	// read data form rx buffer
	nextChar = UDR0;
	// if uart-string in useage, discard new char
	if (uart_str_complete == 0) {

		// write data to string
		// if end of string or max of character are not reached
		// and string is not in use
		if (nextChar != '\n' && nextChar != '\r' && uart_str_count
				< UART_MAXSTRLEN - 1) {
			uart_string[uart_str_count] = nextChar;
			uart_str_count++;
		} else {
			uart_string[uart_str_count] = '\0';
			uart_str_count = 0;
			uart_str_complete = 1;
		}
	}
}

//------------------------------------------------------------------------
//|                               SUBROUTINES                            |
//------------------------------------------------------------------------

void fh_com_looptask() {

	if (uart_str_complete == 1) {
		//process data-link commands
		command[0] = 0;
		paraBuffer[0] = 0;
		uart_str_complete = 0;

		//find the position of the rhombus in the string
		raute_pos = (unsigned char) strlen(
				(char*) strstr((char *) uart_string, "#")) - strlen(
				(char*) uart_string);
		if (raute_pos == 255)
			raute_pos = 0;

		command[19] = 0;
		paraBuffer[19] = 0;

		//separate uart sting into command and paraBuffer
		sscanf((char*) &uart_string[raute_pos], "%s %s\n", command, paraBuffer);

		UART_PRINT("command: %s\r\nparaBuffer %s\r\n", command, paraBuffer);

#ifdef COORDNODE
		//handle different commands for different procedures
		if (strcmp(command, "#getdata") == 0) {
			if (macIsChild(atoi(paraBuffer)) == false) {
				UART_PRINT("Node nicht im Netzwerk\r\n");
			} else {
				send_SN_data_request(atoi(paraBuffer));
			}
		}

		if (strcmp(command, "#getnodes") == 0) {
			associatedNodes_t* nodes = (associatedNodes_t*) getChildTable();
			associatedNodes_t* node;

			uint8_t i;
			printf("#BONL\r\n");
			for (i = 1; i < MAXNODES; i++) {
				node = &nodes[i];

				if ((node->nodeType) == ENDDEVICE) {
					// fixme
					//printf("#shortaddr:%d,#longaddr:%llu\r\n", i, node->nodeLongAddress);
					printf("#shortaddr:%d #longaddr:%s\r\n", i,
							"00:12:f0:41:82:f4");
				}
			}
			printf("#EONL\r\n");
		}
#endif

		if (strcmp(command, "#getecho") == 0) {

			uint16_t addr = atoi(paraBuffer);
			sendPing(addr, 1);

		}
		if (strcmp(command, "#getnodeaddress") == 0) {
			printf("#address %d\n", macConfig.shortAddress);
		}

		if (strcmp(command, "#getintervall") == 0) {

			//TODO implement getintervall

		}
		if (strcmp(command, "#setintervall") == 0) {

			//TODO implement setintervall

		}
		if (strcmp(command, "#gettime") == 0) {

			//TODO implement gettime

		}
		if (strcmp(command, "#settime") == 0) {

			//TODO implement settime

		}
	}

}

/*
 * send data request
 * coord --> node (addr)
 */
void send_SN_data_request(uint16_t addr) {
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
	//UART_PRINT("NODE: got SN_data_request\r\n");
	char* u = get_sensor_data();
	SN_data_frame_t pdata;
	pdata.command = COMMAND_COORD_DATA_RESPONSE;
	pdata.length = strlen(u);

	for (i = 0; i < strlen(u); i++) {
		pdata.payload[i] = u[i];
		UART_PRINT("%c",u[i]);
	}

	send_SN_data_wireless(DEFAULT_COORD_ADDR, (uint8_t *) &pdata,
			sizeof(SN_data_frame_t), UDP_PORT_SENSN_END_ROUTER,
			UDP_PORT_SENSN_COORD);
}

void app_fh_com_process_data_res(uint8_t* pUDPpacket) {
	char * payload;
	SN_data_frame_t * frame = (SN_data_frame_t *) pUDPpacket;
	payload = frame->payload;
	payload[frame->length] = '\0';
	printf("#BOData\r\n%s\n#EOData\r\n", payload);
}
