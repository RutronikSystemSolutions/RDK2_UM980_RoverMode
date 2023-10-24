/*
 * um980_app.h
 *
 *  Created on: 19 Oct 2023
 *      Author: jorda
 */

#ifndef UM980_UM980_APP_H_
#define UM980_UM980_APP_H_

#include <stdint.h>

#include "packet_handler.h"

/**
 * @brief Initializes the module
 *
 * Store the HAL function and flush the RX receiving buffer
 */
void um980_app_init(um980_app_uart_readable_func_t uart_readable,
		um980_app_uart_read_func_t uart_read,
		um980_app_uart_write_func_t uart_write,
		um980_app_get_uticks get_uticks);

/**
 * @brief In case of error, call this function to cleanup the internal buffer
 */
void um980_app_reset();

/**
 * @brief Make the UM980 quiet (start generation correction and position messages)
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_unlog();

/**
 * @brief Start the generation of GGA output messages (contains position, time, ...)
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_start_gga_generation();

/**
 * @brief Set the UM980 as a base mode
 *
 * In base mode, the module first measures its position during 60seconds.
 * It can send correction data (using RTCM messages) to rover
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_set_mode_base();

/**
 * @brief Set the UM980 to rover mode
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_set_mode_rover();

/**
 * @brief Tell the UM980 to generate correction messages of type rtcm_number
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_start_correction_generation(uint16_t rtcm_number, uint16_t period);


/**
 * @brief Cyclic call to be called to catch the messages sent by the UM980
 *
 * Check if a message (NMEA or RTCM) is available and read it
 *
 * @retval 0 Success
 * @retval != 0 Error
 */
int um980_app_do();

#endif /* UM980_UM980_APP_H_ */
