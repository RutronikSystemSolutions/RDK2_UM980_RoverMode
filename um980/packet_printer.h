/*
 * packet_printer.h
 *
 *  Created on: 19 Oct 2023
 *      Author: jorda
 */

#ifndef UM980_PACKET_PRINTER_H_
#define UM980_PACKET_PRINTER_H_


#include "gga_packet.h"

void packet_printer_print_gga(um980_gga_packet_t* packet);

void packet_printer_print_rtcm(uint8_t* buffer, uint16_t len);

#endif /* UM980_PACKET_PRINTER_H_ */
