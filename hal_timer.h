/*
 * hal_timer.h
 *
 *  Created on: 11 Sep 2023
 *      Author: jorda
 */

#ifndef HAL_HAL_TIMER_H_
#define HAL_HAL_TIMER_H_

#include <stdint.h>

int hal_timer_init();

uint32_t hal_timer_get_uticks(void);

#endif /* HAL_HAL_TIMER_H_ */
