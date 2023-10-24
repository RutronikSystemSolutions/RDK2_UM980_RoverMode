/*
 * nmea_packet.h
 *
 *  Created on: 19 Oct 2023
 *      Author: jorda
 */

#ifndef UM980_NMEA_PACKET_H_
#define UM980_NMEA_PACKET_H_

#include <stdint.h>

typedef enum
{
	PACKET_TYPE_GGA,
	PACKET_TYPE_COMMAND_ACK,
	PACKET_TYPE_UNKNOWN
} nmea_packet_type_t;

/**
 * @brief Given a buffer containing a valid NMEA packet, return the type of the packet
 *
 * @param [in] buffer Buffer containing the packet. Example: $GNGGA,023634.00,4004.73871635,N,11614.19729418,E,1,28,0.7,61.0988,M,-8.4923,M,,*58
 * @param [in] len Length of the buffer
 *
 * @retval The type of the packet
 */
nmea_packet_type_t nmea_packet_get_type(uint8_t* buffer, uint16_t len);


#endif /* UM980_NMEA_PACKET_H_ */
