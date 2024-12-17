/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
 /** @file
  *  app_queue.c
 *
 */

#ifdef RUN_SOD_ONLY
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <time.h>
#include "cyabs_rtos.h"
#include "cy_result.h"
#include "cy_log.h"

#include "cy_buffer_pool.h"
/******************************************************
 *                      Macros
 ******************************************************/
/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
cy_queue_t app_free_input_buffer_queue;
cy_queue_t app_free_output_buffer_queue;
cy_queue_t app_usb_data_buffer_queue;
/******************************************************
 *               Function Definitions
 ******************************************************/

cy_rslt_t cy_afe_app_initialize_free_input_buffer_queue(uint8_t no_of_buffers, uint8_t size_of_buffer)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_rtos_init_queue(&app_free_input_buffer_queue, no_of_buffers, size_of_buffer);
    if (CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err(result, "Failed to initialize queue to store input free buffers");
        return result;
    }

    return result;
}

cy_rslt_t cy_afe_app_initialize_free_output_buffer_queue(uint8_t no_of_buffers, uint8_t size_of_buffer)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_rtos_init_queue(&app_free_output_buffer_queue, no_of_buffers, size_of_buffer);
    if (CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err(result, "Failed to initialize queue to store output free buffers");
        return result;
    }

    return result;
}

cy_rslt_t cy_afe_app_initialize_data_buffer_queue(uint8_t no_of_buffers, uint8_t size_of_buffer)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_rtos_init_queue(&app_usb_data_buffer_queue, no_of_buffers, size_of_buffer);
    if (CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err(result, "Failed to initialize queue to store the AFE output buffer");
        return result;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_afe_app_push_free_input_buffer_to_queue(cy_buffer_t buffer, bool is_from_isr)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_rtos_put_queue(&app_free_input_buffer_queue, &buffer, CY_RTOS_NEVER_TIMEOUT, is_from_isr);
    if (CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err_no_isr(result, "Failed to push the free buffer to queue");
        return result;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_afe_app_push_free_output_buffer_to_queue(cy_buffer_t buffer, bool is_from_isr)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_rtos_put_queue(&app_free_output_buffer_queue, &buffer, CY_RTOS_NEVER_TIMEOUT, is_from_isr);
    if (CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err_no_isr(result, "Failed to push the free buffer to queue");
        return result;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_afe_app_get_free_input_buffer_from_queue(cy_buffer_t* buffer, bool is_from_isr)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_buffer_t free_buffer;

    result = cy_rtos_get_queue(&app_free_input_buffer_queue, &free_buffer, CY_RTOS_NEVER_TIMEOUT, is_from_isr);
    if(CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err_no_isr(result, "Failed to get the free buffer from queue");
        return result;
    }

    *buffer = free_buffer;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_afe_app_get_free_output_buffer_from_queue(cy_buffer_t* buffer, bool is_from_isr)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_buffer_t free_buffer;

    result = cy_rtos_get_queue(&app_free_output_buffer_queue, &free_buffer, CY_RTOS_NEVER_TIMEOUT, is_from_isr);
    if(CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err(result, "Failed to get the free buffer from queue");
        return result;
    }

    *buffer = free_buffer;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_afe_app_push_output_buffer_to_process_queue(cy_buffer_t buffer, bool is_from_isr)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_rtos_put_queue(&app_usb_data_buffer_queue, &buffer, CY_RTOS_NEVER_TIMEOUT, is_from_isr);
    if (CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err(result, "Failed to push the buffer to send over USB");
        return result;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_afe_app_get_output_buffer_from_process_queue(cy_buffer_t* buffer, bool is_from_isr)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_buffer_t afe_output_buffer;

    result = cy_rtos_get_queue(&app_usb_data_buffer_queue, &afe_output_buffer, CY_RTOS_NEVER_TIMEOUT, is_from_isr);
    if (CY_RSLT_SUCCESS != result)
    {
        //cy_afe_app_log_err_no_isr(result, "Failed to get the buffer to send over USB");
        return result;
    }

    *buffer = afe_output_buffer;

    return CY_RSLT_SUCCESS;
}
#endif
