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

static char rv[25];

/*
 * return the sensor data ready to send to the coordinator
 */
char* get_sensor_data() {
	// dummie return for testing

	sprintf(rv, "Sensor Data from node: %d : Temp: %ld",
			macConfig.shortAddress, tmp75_read_temp() / 16);
	return rv;

}

void set_i2c_params() {
	sei();
	i2c_init();
	tmp75_write_config(0x70);
}
