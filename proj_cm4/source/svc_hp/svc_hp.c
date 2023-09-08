/******************************************************************************
 * File Name:   svc_hp.c
 *
 * Description: This file contains the high performance staged voice 
 * control(lp svc) configuration and constants.
 *
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 *******************************************************************************/

#include "svc_hp.h"

/*******************************************************************************
 * Function Name: cy_svc_hp_pcm_data_callback
 ********************************************************************************
 * Summary:
 * This callback provides audio data to be used for inferencing on CM4.
 * Parameters:
 *  frame_buffer: Pointer to the circular buffer.
 *  frame_count: Number of frames available for inferencing.
 *  callback_user_arg: NULL
 *
 * Return:
 *  cy_rslt_t
 *
 *******************************************************************************/
/*cy_svc_hp_data_callback_t*/cy_rslt_t cy_svc_hp_pcm_data_callback(
    CY_SVC_DATA_T *frame_buffer,
    uint32_t frame_count,
    void *callback_user_arg)
{
    svc_hp_audio_data_t svc_hp_pcm_data = {0};
    svc_hp_pcm_data.data_ptr = frame_buffer;
    svc_hp_pcm_data.frame_count = frame_count;

    return(cy_rtos_put_queue(&audio_data_queue_handle, &svc_hp_pcm_data, 
                      CY_RTOS_NEVER_TIMEOUT, is_in_isr()));
}

/*******************************************************************************
 * Function Name: svc_init
 ********************************************************************************
 * Summary:
 * This function sets the CM4 SVC configurations and initilizes.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t
 *
 *******************************************************************************/
cy_rslt_t svc_init(void)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cy_svc_hp_config_t init_svc = {0};

    init_svc.data_callback = (cy_svc_hp_data_callback_t)cy_svc_hp_pcm_data_callback;

    ret_val = cy_svc_hp_init(&init_svc);
    if (ret_val != CY_RSLT_SUCCESS)
    {
        printf("cy_svc_hp_init fail\n");
    }
    return ret_val;
}
