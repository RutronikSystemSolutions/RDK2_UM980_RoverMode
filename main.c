/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK2_UM980_RoverMode application for ModusToolbox.
*
* Related Document: See README.md
*
*  Created on: 2023-10-23
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address:  Industriestraße 2, 75228 Ispringen, Germany
*  Author: ROJ030
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "um980/um980_app.h"
#include "hal_timer.h"

cyhal_uart_t um980_uart_obj;

/* Initialize the ARDUINO UART configuration structure */
const cyhal_uart_cfg_t um980_uart_config =
{
    .data_bits = 8,
    .stop_bits = 1,
    .parity = CYHAL_UART_PARITY_NONE,
    .rx_buffer = NULL,
    .rx_buffer_size = 0
};

uint32_t um980_app_uart_readable_func(void)
{
	return cyhal_uart_readable(&um980_uart_obj);
}

int um980_app_uart_read_func(uint8_t* buffer, uint16_t size)
{
	size_t toread = (size_t)size;
	cy_rslt_t result = cyhal_uart_read (&um980_uart_obj, buffer, &toread);
	if (result != CY_RSLT_SUCCESS) return -1;
	return (int)toread;
}

int um980_app_uart_write_func(uint8_t* buffer, uint16_t size)
{
	size_t towrite = (size_t)size;
	cy_rslt_t result = cyhal_uart_write(&um980_uart_obj, buffer, &towrite);
	if (result != CY_RSLT_SUCCESS) return -1;
	return (int)towrite;
}

/**
 * @brief Error function to be called if something wrong happens
 */
static void handle_error(void)
{
     // Disable all interrupts.
    __disable_irq();
    CY_ASSERT(0);
}

int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    // Initialize the device and board peripherals
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }

    // Enable interrupts
    __enable_irq();

    // Enable debug output via KitProg UART
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    printf("-------------------------------- \r\n");
    printf("Start RDK2_UM980_RoverMode \r\n");
    printf("-------------------------------- \r\n");

    // Initialize the UART block enabling the communication with the UM980
	result = cyhal_uart_init(&um980_uart_obj, ARDU_TX, ARDU_RX, NC, NC, NULL, &um980_uart_config);
	if (result != CY_RSLT_SUCCESS)
	{
		handle_error();
	}
	printf("UART initialized.\r\n");

	int retval = hal_timer_init();
	if (retval != 0)
	{
		printf("Error while initializing timer. \r\n");
		for(;;){}
	}
	printf("Timer initialized.\r\n");

	um980_app_init(um980_app_uart_readable_func, um980_app_uart_read_func, um980_app_uart_write_func, hal_timer_get_uticks);

	// Stop message generation (correction and position)
	// Wait until successful - First call might fail because of strange startup behavior
	for(;;)
	{
		retval = um980_app_unlog();
		if (retval != 0)
		{
			printf("um980_app_unlog error %d ! \r\n", retval);
			Cy_SysLib_Delay(1000);
			um980_app_reset();
		}
		else break;
	}

	// Set rover mode (in contrary, in base mode, the system is considered to have a non-moving location)
	retval = um980_app_set_mode_rover();
	if (retval != 0)
	{
		printf("um980_app_set_mode_rover error : %d ! \r\n", retval);
		handle_error();
	}

	// Request position every seconds
	retval = um980_app_start_gga_generation();
	if (retval != 0)
	{
		printf("um980_app_start_gga_generation error: %d ! \r\n", retval);
		handle_error();
	}

	printf("UM980 initialized. Start to generate data. \r\n");

    for (;;)
    {
    	retval = um980_app_do();
    	if (retval != 0)
		{
			printf("Error um980_app_do() : %d \r\n", retval);
			handle_error();
		}
    }
}
