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
			printf("%s ---  %d\n", uart_string,uart_str_complete);
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
			printf("#BOData\n");
//				send_data_wireless((uint16_t) paraBuffer,"hallo")
			printf("bin die daten\n");
			//TODO Implement Wireless_UART
			printf("#EOData\n");
		}

		if (strcmp(command, "#getecho") == 0) {
			//TODO Implement PING
		}

		if (strcmp(command, "#getnodes") == 0) {
			//TODO Implement
		}

	}

}
