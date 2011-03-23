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

static char rv[25];

/*
 * return the sensor data ready to send to the coordinator
 */
char* get_sensor_data() {
	// dummie return for testing
	sprintf(rv, "Sensor Data from node: %d", macConfig.shortAddress);
	return rv;

}

