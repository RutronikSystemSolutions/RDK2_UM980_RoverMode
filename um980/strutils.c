/*
 * strutils.c
 *
 *  Created on: 20 Oct 2023
 *      Author: jorda
 */

#include "strutils.h"

int get_segment_address_and_length(uint8_t* buffer, uint16_t len, uint8_t delim, uint16_t index, uint16_t* seg_addr, uint16_t* seg_len)
{
	uint16_t counter = 0;
	uint16_t current_start = 0;
	for(uint16_t i = 0; i < len; ++i)
	{
		if (buffer[i] == delim)
		{
			counter++;
			if (counter == (index + 1))
			{
				*seg_addr = current_start;
				*seg_len = (i - current_start);
				return 0;
			}
			current_start = i + 1;
		}
	}

	// Special case, we reach the end of the buffer
	counter++;
	if (counter == (index + 1))
	{
		*seg_addr = current_start;
		*seg_len = (len - current_start);
		return 0;
	}

	return -1;
}

int get_segment_count(uint8_t* buffer, uint16_t len, uint8_t delim)
{
	uint16_t count = 0;
	for(uint16_t i = 0; i < len; ++i)
	{
		if (buffer[i] == delim) count++;
	}
	return (count + 1);
}

