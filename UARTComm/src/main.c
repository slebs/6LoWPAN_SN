/*
 * main.c
 *
 *  Created on: 16.03.2011
 *      Author: Kevin
 */

#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "../inc/hdlc_light.h"
#include "../inc/io_access.h"
#include "../inc/uart.h"
#include "../inc/commands.h"
#include "../inc/tmp75.h"
#include "../inc/i2cmaster.h"
#include "../inc/TWI_Master.h"

//------------------------------Initialisierungen--------------------------------------
#define UART_MAXSTRLEN		256

typedef struct {
	unsigned int timestamp;
	int temp_buf[2];
} data;

static unsigned char  messageBuf[3];

static unsigned char volatile rxBuffer[UART_MAXSTRLEN]; //RX-Buffer
static unsigned char volatile rxPtr = 0; //Receive Pointer f�r RX-Buffer
static unsigned char volatile rdPtr = 0; //Readout Pointer f�r RX-Buffer

static volatile uint8_t uart_str_complete = 0; // 1 .. String komplett empfangen
static volatile uint8_t uart_str_count = 0;
static unsigned volatile char uart_string[UART_MAXSTRLEN];

unsigned char iic_data;

static FILE mystdout = FDEV_SETUP_STREAM(uart_putc,NULL,_FDEV_SETUP_WRITE);

//-------------------------------------------------------------------------------------

//------------------------------RX Interrupt Service Routine---------------------------
ISR(USART0_RX_vect)
{
	//	rxBuffer[rxPtr++] = (unsigned char) UDR0;
	//	printf("---Char received---\n");
	unsigned char nextChar;

	// Daten aus dem Puffer lesen
	nextChar = UDR0;
	if (uart_str_complete == 0) { // wenn uart_string gerade in Verwendung, neues Zeichen verwerfen

		// Daten werden erst in string geschrieben, wenn nicht String-Ende/max Zeichenlänge erreicht ist
		// string gerade verarbeitet wird
		if (nextChar != '\n' && nextChar != '\r' && uart_str_count
				< UART_MAXSTRLEN - 1) {
			uart_string[uart_str_count] = nextChar;
			uart_str_count++;
			//			printf("---Char received---\n");
		} else {
			uart_string[uart_str_count] = '\0';
			uart_str_count = 0;
			uart_str_complete = 1;
			printf("%s ---  %d", uart_string, uart_str_complete);
		}
	}
}
//-------------------------------------------------------------------------------------

void reject_rxBuffer() {
	rdPtr = rxPtr;
}

//char _getKey() {
//	unsigned char out;
//	return out = rxBuffer[rdPtr++];
//}
//
//void getLine(char *string, unsigned char n) {
//	unsigned char cnt = 0;
//	char zeichen;
//
//	do {
//		zeichen = rxBuffer[rdPtr++];
//		if (zeichen == '\r') {
//			zeichen = '\n';
//			rdPtr++;
//		}
//		*string++ = zeichen;
//		cnt++;
//	} while (cnt < n - 1 && (zeichen != '\n'));
//
//	//	*string = 0;
//}

void TMP275_init(int i2c_adr) {
	messageBuf[0] = i2c_adr; // Write to TMP275
	messageBuf[1] = 0x01; // Pointer Reg. => Config. Reg
	messageBuf[2] = 0x70; // Write to Config.Reg for 12 Bit Resolution
	TWI_Start_Transceiver_With_Data(messageBuf, 3);
	printf(" blablab");

	messageBuf[0] = i2c_adr; // Write to TMP275
	messageBuf[1] = 0x00; // Pointer Reg. back to Temp. Reg
	TWI_Start_Transceiver_With_Data(messageBuf, 2);
	printf(" blblublu");
}

long int TMP275_read(int i2c_adr) {
	long int temp;

	messageBuf[0] = i2c_adr + 1; // read to TMP275

	TWI_Start_Transceiver_With_Data(messageBuf, 3);

	TWI_Get_Data_From_Transceiver(messageBuf, 3);

	temp = (signed char) messageBuf[1] << 8;
	temp |= messageBuf[2];
	temp = temp * 1000 / 256;

	return (temp);
}

//---------------------------------Main---------------------------------------------
int main(void) {
	stdout = &mystdout;
	//	unsigned int i, j;
	//	float temp;

	unsigned char raute_pos;

	char command[20];
	char paraBuffer[80];

	uart_init(9600);
	printf("\r\n---TWI Initialisieren---\r\n");
//	TWI_Master_Initialise();
	i2c_init();
	tmp75_write_config(0x70);

	// Init used I2C-ICs
	printf("\r\n---TMP275 Initialisieren---\r\n");
//	TMP275_init(0x92);

	_delay_ms(20);
	sei();

	printf("\r\n---Initialisierungen abgeschlossen---\r\n");
	//	printf("Config Register %d\r\n", tmp75_read_config());

	//Hauptprogramm
	while (1) {
		do {
			led_set(PLATFORM_RCB, RCB_LED_1, PLATFORM_LED_TOGGLE);
			led_set(PLATFORM_RCB, RCB_LED_2, PLATFORM_LED_TOGGLE);
//			int temp = TMP275_read(0x92);
			uint16_t temp = tmp75_read_temp();
			double t = temp/(16.0);
			printf("Temperatur betraegt %.3f\r\n",t);

			_delay_ms(500);
		} while (uart_str_complete == 0);

		if (uart_str_complete == 1) {
			//Bearbeitung der Data-Link Befehle
			command[0] = 0;
			paraBuffer[0] = 0;
			uart_str_complete = 0;

			//	Mit Pointer
			//			printf("RX-Buffer: %s\n", rxBuffer);
			//			getLine(cmdBuffer, sizeof(cmdBuffer));
			//
			//			raute_pos = (unsigned char) strlen(strstr(cmdBuffer, "#"))
			//					- strlen(cmdBuffer);
			//			if (raute_pos == 255)
			//				raute_pos = 0;

			//	Mit UART-String
			raute_pos = (unsigned char) strlen(
					(char*) strstr((char *) uart_string, "#")) - strlen(
					(char *) uart_string);
			if (raute_pos == 255)
				raute_pos = 0;

			command[19] = 0;
			paraBuffer[19] = 0;

			sscanf((char*) &uart_string[raute_pos], "%s %s\n", command,
					paraBuffer);

			//			printf("command: %s\n", command);
			//			printf("paraBuffer %s\n", paraBuffer);

			if (strcmp(command, "#getdata") == 0) {
				printf("#BOData\n");

				printf("uhhhhh wir sind die Daten!!\n");
				//TODO Implement Wireless_UART
				printf("#EOData\n");
			}

			else if (strcmp(command, "#getecho") == 0) {
				//TODO Implement PING
			}

			else if (strcmp(command, "#getnodes") == 0) {
				//TODO Implement
			} else {
				printf("unkown command");
			}

		}
	}// Ende Hauptprogramm
	return 0;
}//Ende Main()
//-------------------------------------------------------------------------------------
//EOF

