/*
 * twi_interface.c
 *
 *  Created on: Mar 22, 2011
 *      Author: simon
 */

/*
 * TODO Pluch to implement TWI interface
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../inc/sensn/app_interface.h"
#include "../../inc/deRFaddon/uart.h"
#include "../../inc/sensn/tmp75.h"
#include "../../inc/avr_timer.h"

static char rv[80];

/*
 * return the sensor data ready to send to the coordinator
 */
char* get_sensor_data() {

	sprintf(rv, "Sensor Data from node: %d : Temp: %2.2f", macConfig.shortAddress,
			((float) tmp75_read_temp() / 16.0));
	UART_PRINT("%s",rv);
	return rv;

}
void print_sensor_data() {
	sprintf(rv, "Sensor Data from node: %d : Temp: %3.3f\r\n",
			macConfig.shortAddress, ((float) tmp75_read_temp() / 16.0));
	UART_PRINT("%s",rv);
	macSetAlarm(1000, print_sensor_data);
}

void set_i2c_params() {
	sei();
	i2c_init();
	tmp75_write_config(0x70);
}
