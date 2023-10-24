/*
 * rtcm_packet.h
 *
 *  Created on: 23 Oct 2023
 *      Author: jorda
 */

#ifndef UM980_RTCM_PACKET_H_
#define UM980_RTCM_PACKET_H_

#include <stdint.h>

uint16_t rtcm_packet_get_variable_size(uint8_t* buffer);

uint16_t rtcm_packet_get_type(uint8_t* buffer);

#endif /* UM980_RTCM_PACKET_H_ */
