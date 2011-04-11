/*
 * app_interface.h
 *
 *  Created on: Feb 18, 2011
 *      Author: simon
 */

#ifndef SENSOR_NETWORK_H_
#define SENSOR_NETWORK_H_

#include "../../inc/mac.h"
/** Maximum frame length */
#define MAX_FRAME_LENGTH_SN			(79)
/** Maximum available frame payload*/
#define MAX_PAYLOAD_LENGTH_SN     (MAX_FRAME_LENGTH_SN - 2) //(Length - (command + option))
/*
 * Struct defined as data frame for sensor network application
 */
typedef struct {
	uint8_t command; ///< defines the command, which selects the service
	uint8_t length; ///< defines the payload length
	char payload[MAX_PAYLOAD_LENGTH_SN]; ///< payload, depends on command
}__attribute__((packed)) SN_data_frame_t;

// Define LEDs with speaking names
#define LED_ALIVE (RCB_LED_2)		// LED green
#define LED_ASSOCIATED (RCB_LED_1)	// LED yellow
#define LED_WORKING (RCB_LED_0)		// LED red
/*
 * Application UDP port definitions
 * ports.
 */
/** UDP Port to talk with coordinator */
#define UDP_PORT_SENSN_COORD        (0xF0BD)
/** UDP Port to talk with router/end node */
#define UDP_PORT_SENSN_END_ROUTER   (0xF0BE)

/** Maximum frame length for app_perf*/
#define MAX_PERFTEST_FRAME_LENGTH         (86) // does not work (max 79)
/*
 * Application FH-Interface
 */
#define UART_MAXSTRLEN		(256)

// LoopTask
void loopTask(void);

void app_init(void);
void checkEeprom(void);

void process_endnode_udp_packet_SN(uint8_t* pUDPpacket, uint8_t payloadlen,
		uint16_t originAddr);
void process_coord_udp_packet_SN(uint8_t* pUDPpacket, uint8_t payloadlen,
		uint16_t originAddr);

// functions for APP_PING
void pingMacro();
void sendPing(uint16_t dest_addr_ping, uint16_t iter);
void send_test_data();
void ping_button_ev();
void app_ping_device_process(uint8_t* pUDPpacket, int16_t originAddr);
void app_ping_coord_process(uint8_t* pUDPpacket, int16_t originAddr);
void send_SN_data_wireless(uint16_t destAddr, uint8_t* pData, uint8_t len,
		uint16_t srcUDPPort, uint16_t destUDPPort);

//functions for APP_PERF
void perf_init();
void perf_loop_task();
void perf_button_ev();
void evaluate_troughput_test();
void compare_test_data(uint8_t* pUDPpacket, uint8_t payloadlen);

//functions for APP_FH_COM
void fh_com_looptask();
void send_SN_data_request(uint16_t addr);
void app_fh_com_process_data_res(uint8_t* pUDPpacket);
void app_fh_com_process_data_req(uint8_t* pUDPpacket);

//functions for twi_interface
char* get_sensor_data();
void set_i2c_params();
void i2c_init();
void print_sensor_data();
//functions for sensornetwork
/*
 * dummie
 */
#endif /* SENSOR_NETWORK_H_ */
