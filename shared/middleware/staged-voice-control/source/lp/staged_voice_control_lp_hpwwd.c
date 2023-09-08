/*
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
 */

/**
 * @file staged_voice_control_lp_lpwwd.c
 *
 */

#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_hpwwd.h"

/*******************************************************************************
 *                              Macros
 ******************************************************************************/

/*******************************************************************************
 *                              Constants
 ******************************************************************************/

/*******************************************************************************
 *                              Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                              Function Declarations
 ******************************************************************************/


cy_rslt_t svc_lp_hpwwd_reset(
        svc_lp_instance_t *lp_instance)
{
    if (!(lp_instance->init_params.stage_config_list
            & CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS))
    {
        return CY_RSLT_SUCCESS;
    }

    lp_instance->hpwwd_trigger_data_final_address = NULL;
    lp_instance->post_hpwwd_pending_frame_counter_to_hp = 0;
    lp_instance->post_wwd_frame_count_req_by_hp = 0;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t svc_lp_asr_reset(
        svc_lp_instance_t *lp_instance)
{
    if (!(lp_instance->init_params.stage_config_list
            & CY_SVC_ENABLE_ASR_STATE_TRANSITIONS))
    {
        return CY_RSLT_SUCCESS;
    }

    lp_instance->stream_requested_on_asr_processing_query_state = false;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t svc_lp_hpwwd_reset_and_high_components(
        svc_lp_instance_t *lp_instance)
{
    (void) svc_lp_hpwwd_reset(lp_instance);
    (void) svc_lp_asr_reset(lp_instance);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t svc_lp_asr_reset_and_high_components(
        svc_lp_instance_t *lp_instance)
{
    (void) svc_lp_asr_reset(lp_instance);

    return CY_RSLT_SUCCESS;
}

#endif
