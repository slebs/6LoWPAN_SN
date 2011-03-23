/*
 * twi_interface.c
 *
 *  Created on: Mar 22, 2011
 *      Author: simon
 */

/*
 * TODO Pluch to implement TWI interface
 */

#include "../../inc/sensn/app_interface.h"

/*
 * return the sensor data ready to send to the coordinator
 */
char* get_sensor_data(){
	// dummie return for testing
	char* rv = "";
	sprintf(rv,"Sensor Data from node: %d",macConfig.shortAddress);
	return rv;
}
