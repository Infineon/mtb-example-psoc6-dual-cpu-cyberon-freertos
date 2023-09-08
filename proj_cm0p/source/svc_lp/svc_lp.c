/******************************************************************************
 * File Name:   svc_lp.c
 *
 * Description: This file contains the low power staged voice control(lp svc) 
 * configuration and constants.
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

#include "svc_lp.h"


/*******************************************************************************
 * Function Name: svc_init
 ********************************************************************************
 * Summary:
 * This function sets the CM0p SVC configurations and initilizes.
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
    cy_svc_lp_config_t init_svc = {0};

    init_svc.stage_config_list = CY_SVC_ENABLE_LPWWD | 
                                 CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS | 
                                 CY_SVC_ENABLE_ASR_STATE_TRANSITIONS;
    init_svc.callback_user_arg = NULL;
    init_svc.event_callback =  svc_lp_cyb_app_cbk;
    init_svc.audio_input_type = CY_SVC_AUDIO_INPUT_TYPE_MONO;
    init_svc.sample_rate = CY_SVC_SAMPLE_RATE_16KHZ;
    init_svc.single_frame_time_ms = CY_SVC_SUPPORTED_FRAME_TIME_MS;
    init_svc.total_circular_buf_size = NUMBER_OF_FRAMES_MULTIPLIER * 
                                       FRAME_SIZE * CIRCULAR_BUFFER_SIZE_IN_SEC;
    init_svc.is_lpwwd_external = true;
    init_svc.lpwwd_external_data_callback = cy_svc_lp_app_data_callback_t_lpwwd_external;
    init_svc.pre_roll_frame_count_from_lpwwd_detect_frame = NUMBER_OF_FRAMES_MULTIPLIER 
                                                            * PRE_ROLL_SIZE_IN_SEC;
    ret_val = cy_svc_lp_init(&init_svc);
    if (ret_val != CY_RSLT_SUCCESS)
    {
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r cy_svc_lp_init fail \n\r");
    }
    return ret_val;
}



/*******************************************************************************
 * Function Name: cy_svc_lp_app_data_callback_t_lpwwd_external
 ********************************************************************************
 * Summary:
 * This callback allows user to call user specific inference engine for wake word detection.
 *
 * Parameters:
 *  frame_buffer: Pointer to the circular buffer.
 *  frame_count: Number of frames available for inferencing.
 *  callback_user_arg: NULL
 *  lpwwd_state: CM0p SVC expect the inference status for a wake word detection through lpwwd_state
 *
 * Return:
 *  cy_rslt_t
 *
 *******************************************************************************/
cy_rslt_t cy_svc_lp_app_data_callback_t_lpwwd_external(
                                   CY_SVC_DATA_T *frame_buffer,
                                   uint32_t frame_count,
                                   void *callback_user_arg,
                                   cy_svc_lp_external_lpwwd_state_t *lpwwd_state)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    int cyb_status = LPWW_DETECTION_IN_PROGRESS;

    if (NULL == lpwwd_state)
    {
        ret_val = CY_RSLT_SVC_BAD_ARG;
    }
    else
    {
        /* Feed frames in Cyberon and get the WWD status. */ 
        custom_one_stage_asr_process(frame_buffer, FRAME_SIZE, &cyb_status);

        if (LPWW_DETECTION_IN_PROGRESS == cyb_status)
        {
            *lpwwd_state = CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_IN_PROGRESS;
        }
        else if (LPWW_DETECTION_SUCCESFUL == cyb_status)
        {
            *lpwwd_state = CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_SUCCESS;
        }
        else if (LPWW_DETECTION_FAILED == cyb_status)
        {
            *lpwwd_state = CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_FAILED;
        }
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}

/*******************************************************************************
 * Function Name: svc_lp_cyb_app_cbk
 ********************************************************************************
 * Summary:
 * Gets the the events from the SVC
 *
 * Parameters:
 *  cy_svc_event_t event: event
 *  void *callback_user_arg: User callback to provide extra information
 * 
 * Return:
 *  cy_rslt_t: Returns the status
 *
 *******************************************************************************/
cy_rslt_t svc_lp_cyb_app_cbk(
                              cy_svc_event_t event,
                              void *callback_user_arg)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}

