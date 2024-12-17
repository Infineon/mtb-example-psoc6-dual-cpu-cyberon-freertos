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

/**
 * @file staged_voice_control_lp_aad.c
 *
 */

#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_aad.h"
#include "staged_voice_control_lp_sod.h"
#include "staged_voice_control_lp_lpwwd.h"
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


cy_rslt_t svc_lp_aad_reset(svc_lp_instance_t *lp_instance)
{
    lp_instance->stats.frame_counter_received_after_last_aad_dbg = 0;

    /**
     * Resetting the last frame start address, as there is a gap
     * in the feed, old frames address stored is of no use.
     */
    lp_instance->last_processed_frame_start_address = NULL;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t svc_lp_aad_reset_and_high_components(svc_lp_instance_t *lp_instance)
{
    svc_lp_aad_reset(lp_instance);

    /**
     * Reset the higher stage states if any exists.
     */
    (void) svc_lp_sod_reset(lp_instance);
    (void) svc_lp_lpwwd_reset(lp_instance);
    (void) svc_lp_hpwwd_reset(lp_instance);
    (void) svc_lp_asr_reset(lp_instance);

    if (CY_SVC_ENABLE_SOD & lp_instance->init_params.stage_config_list)
    {
        (void) cy_sod_reset(lp_instance->sod_handle);
    }
    return CY_RSLT_SUCCESS;
}

cy_rslt_t svc_lp_auto_aad_detection_check(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if (true == lp_instance->enable_auto_detect_frame_feed_discontinuity)
    {
        cy_time_t cur_time = 0;
        (void) cy_rtos_get_time(&cur_time);

        if (0 == lp_instance->stats.last_frame_feed_time_ms)
        {
            cy_svc_log_info("First Feed after boot");
            svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY);
        }
        else if (cur_time > lp_instance->stats.last_frame_feed_time_ms)
        {
            if ((cur_time - lp_instance->stats.last_frame_feed_time_ms)
                    > lp_instance->auto_frame_feed_discontinuity_timeout_ms)
            {
                svc_lp_trigger_state(lp_instance,
                        SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY);
            }
        }
        else
        {
            /**
             * Normal execution time. During continuous feed time.
             */
            ;
        }

        lp_instance->stats.last_frame_feed_time_ms = cur_time;
    }

    if (CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY
            == lp_instance->current_stage)
    {
        svc_lp_trigger_state(lp_instance,
                SVC_TRIGGER_ACOUSTIC_ACTIVITY_DETECTED);
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}

#endif
