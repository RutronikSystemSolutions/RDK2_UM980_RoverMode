/*
 * nmea_packet.c
 *
 *  Created on: 19 Oct 2023
 *      Author: jorda
 */

#include "nmea_packet.h"

#include <string.h>

nmea_packet_type_t nmea_packet_get_type(uint8_t* buffer, uint16_t len)
{
	if ((len > 6) && (strncmp((char*)buffer, "$GNGGA", 6) == 0))
	{
		return PACKET_TYPE_GGA;
	}
	if ((len > 6) && (strncmp((char*)buffer, "$GPGGA", 6) == 0))
	{
		return PACKET_TYPE_GGA;
	}
	if ((len > 6) && (strncmp((char*)buffer, "$GBGGA", 6) == 0))
	{
		return PACKET_TYPE_GGA;
	}
	if ((len > 6) && (strncmp((char*)buffer, "$GLGGA", 6) == 0))
	{
		return PACKET_TYPE_GGA;
	}
	if ((len > 6) && (strncmp((char*)buffer, "$GAGGA", 6) == 0))
	{
		return PACKET_TYPE_GGA;
	}
	if ((len > 6) && (strncmp((char*)buffer, "$GQGGA", 6) == 0))
	{
		return PACKET_TYPE_GGA;
	}
	if ((len > 8) && (strncmp((char*)buffer, "$command", 8) == 0))
	{
		return PACKET_TYPE_COMMAND_ACK;
	}
	return PACKET_TYPE_UNKNOWN;
}


