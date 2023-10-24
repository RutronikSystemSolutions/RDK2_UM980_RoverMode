/*
 * rtcm_packet.c
 *
 *  Created on: 23 Oct 2023
 *      Author: jorda
 */

#include "rtcm_packet.h"


uint16_t rtcm_packet_get_variable_size(uint8_t* buffer)
{
	return ((buffer[1] & 3) << 8) | buffer[2];
}

uint16_t rtcm_packet_get_type(uint8_t* buffer)
{
	return ((buffer[3]) << 4) | ((buffer[4] & 0xF0) >> 4);
}


