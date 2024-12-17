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
 * @file staged_voice_control_lp_lpwwd_external.c
 *
 */

#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_process_data.h"

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
static cy_rslt_t svc_lp_handle_lpwwd_external_detected_state(
        svc_lp_instance_t *lp_instance,
        uint8_t *data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    (void) svc_lp_trigger_state(lp_instance,
            SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED);

    lp_instance->lpwwd_detected = true;
    cy_svc_log_dbg("LPWWD is detected");

    svc_lp_start_circular_buf_update_on_transition_to_hp(lp_instance, data,
            SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED);

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}


cy_rslt_t svc_lp_process_data_lpwwd_external(
        svc_lp_instance_t *lp_instance,
        uint8_t *data)
{
    cy_svc_lp_external_lpwwd_state_t lpwwd_detected =
            CY_SVC_EXTERNAL_LPWWD_WWD_STATE_INVALID;
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if (CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION
            == lp_instance->current_stage)
    {
        if (false == lp_instance->lpwwd_detected)
        {
            if (NULL != lp_instance->init_params.lpwwd_external_data_callback)
            {
                lp_instance->stats.lpwwd_feed_counter_dbg++;

                ret_val = lp_instance->init_params.lpwwd_external_data_callback(
                        (CY_SVC_DATA_T*) data, 1,
                        lp_instance->init_params.callback_user_arg,
                        &lpwwd_detected);
                if (CY_RSLT_SUCCESS != ret_val)
                {
                    cy_svc_log_err(ret_val, "data_callback failed");
                }

                switch(lpwwd_detected)
                {
                    case CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_IN_PROGRESS:
                        break;

                    case CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_FAILED:
                    {
                        (void) svc_lp_trigger_state(lp_instance,
                                SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED);

                        cy_svc_log_dbg("ELPWWD NDETD [Addr:0x%08X,DET:%d:NDET:%d]", data,
                                lp_instance->stats.lpwwd_detect_counter_dbg,
                                lp_instance->stats.lpwwd_not_detect_counter_dbg);
                        break;
                    }
                    case CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_SUCCESS:
                    {
                        svc_lp_handle_lpwwd_external_detected_state(lp_instance,
                                data);

                        cy_svc_log_dbg("ELPWWD DETD [Addr:0x%08X,DET:%d:NDET:%d]", data,
                                lp_instance->stats.lpwwd_detect_counter_dbg,
                                lp_instance->stats.lpwwd_not_detect_counter_dbg);
                        break;
                    }
                    case CY_SVC_EXTERNAL_LPWWD_WWD_STATE_INVALID:
                    case CY_SVC_EXTERNAL_LPWWD_WWD_STATE_MAX:
                    {
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
        }
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}

#endif
