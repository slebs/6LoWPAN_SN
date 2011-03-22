/*
 * commands.h
 *
 *  Created on: Mar 11, 2011
 *      Author: simon
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

/*
 * COMMANDS Section
 * Commands can be handled in apps.
 * New commands have to be designed for new apps
 * DO NOT use the defined Commands!
 */

#define NO_OPTION                               (0x00)

/** @brief new ping frame which request a ping response */
#define COMMAND_PING_REQUEST                    (0x01)
/** @brief ping response frame (require ping request) */
#define COMMAND_PING_RESPONSE                   (0x02)

/** @brief Data request frame from Coordinator */
#define COMMAND_COORD_DATA_REQUEST				(0x03)
/** @brief Data response frame from Node */
#define COMMAND_COORD_DATA_RESPONSE				(0x04)

/** @brief Stop response frame from Node */
#define COMMAND_STOP_REQUEST					(0x05)

#endif /* COMMANDS_H_ */
