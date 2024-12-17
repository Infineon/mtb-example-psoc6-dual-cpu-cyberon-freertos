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
 * @file staged_voice_control_lp_lpwwd.c
 *
 */
#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_process_data.h"
#include "staged_voice_control_lp_lpwwd_external.h"
#include "staged_voice_control_lp_lpwwd_internal.h"
#include "staged_voice_control_lp_private.h"
#include "staged_voice_control_lp_aad.h"
#include "staged_voice_control_lp_hpwwd.h"

/*******************************************************************************
 *                              Macros
 ******************************************************************************/
#define DEFAULT_SOD_LOOK_BACK_SIZE_FRAME_COUNT_FOR_LPWWD (10) //100ms.

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

static cy_rslt_t svc_lp_lpwwd_process_data_get_preshift_buffer(
        svc_lp_instance_t *lp_instance,
        uint8_t *data,
        uint8_t **pp_pre_shift_buffer)
{
    uint8_t *pre_shift_buffer = NULL;
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    unsigned int sod_detect_delay_look_back_frame_counter =
    DEFAULT_SOD_LOOK_BACK_SIZE_FRAME_COUNT_FOR_LPWWD;

    if (NULL == pp_pre_shift_buffer)
    {
        goto CLEAN_RETURN;
    }
    *pp_pre_shift_buffer = NULL;

    if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
    {
        if (0 < lp_instance->init_params.sod_onset_detect_max_late_hit_delay_ms)
        {
            sod_detect_delay_look_back_frame_counter =
                    lp_instance->init_params.sod_onset_detect_max_late_hit_delay_ms
                            / 10;
        }

        /**
         * This check ensures the pre-shift buffer needed for LPWWD is valid
         * and available. This check will avoid providing garbage when we dont
         * have enough data on start from AAD.
         */
        if (lp_instance->stats.frame_counter_received_after_last_aad_dbg
                < sod_detect_delay_look_back_frame_counter)
        {
            //cy_svc_log_dbg("Feed is not sufficient to start lpwwd");
            ret_val = CY_RSLT_SUCCESS;
            goto CLEAN_RETURN;
        }

        svc_lp_get_circular_buf_pre_shift_buffer_from_any_address(lp_instance,
                data, sod_detect_delay_look_back_frame_counter,
                &pre_shift_buffer);

        *pp_pre_shift_buffer = pre_shift_buffer;
    }
    else
    {
        *pp_pre_shift_buffer = data;
    }
    ret_val = CY_RSLT_SUCCESS;

    CLEAN_RETURN: return ret_val;
}

cy_rslt_t svc_lp_process_data_lpwwd(
        svc_lp_instance_t *lp_instance,
        uint8_t *data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    uint8_t *pre_shift_buffer = NULL;

    (void) svc_lp_lpwwd_process_data_get_preshift_buffer(lp_instance, data,
            &pre_shift_buffer);
    if(NULL == pre_shift_buffer)
    {
        ret_val = CY_RSLT_SUCCESS;
    }
    else
    {
        if (true == lp_instance->init_params.is_lpwwd_external)
        {
            ret_val = svc_lp_process_data_lpwwd_external(lp_instance,
                    pre_shift_buffer);
        }
        else
        {
#ifdef ENABLE_IFX_LPWWD
            ret_val = svc_lp_process_data_lpwwd_internal(lp_instance,
                    pre_shift_buffer);
#else
            ret_val = CY_RSLT_SUCCESS;
#endif
        }
    }
    return ret_val;
}

cy_rslt_t svc_lp_lpwwd_init(
        svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *init)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if(true == init->is_lpwwd_external)
    {
        ret_val = CY_RSLT_SUCCESS;
    }
    else
    {
#ifdef ENABLE_IFX_LPWWD
        ret_val = svc_lp_lpwwd_internal_init(lp_instance, init);
        if (CY_RSLT_SUCCESS != ret_val)
        {
            cy_svc_log_err(ret_val, "svc_lp_lpwwd_internal_init fail");
            // goto CLEAN_RETURN;
            // return ret_val;
        }
#else
        ret_val = CY_RSLT_SUCCESS;
        // return ret_val;
#endif
    }
    // CLEAN_RETURN: return ret_val;
    return ret_val;
}


cy_rslt_t svc_lp_lpwwd_reset(
        svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if (!(lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD))
    {
        return CY_RSLT_SUCCESS;
    }

    if (true == lp_instance->init_params.is_lpwwd_external)
    {
        ret_val = CY_RSLT_SUCCESS;
    }
    else
    {
#ifdef ENABLE_IFX_LPWWD
        ret_val = svc_lp_lpwwd_internal_reset(lp_instance);
#else
        ret_val = CY_RSLT_SUCCESS;
#endif
    }

    lp_instance->lpwwd_detected = false;
    lp_instance->post_wwd_frame_count_req_by_hp = 0;
    lp_instance->post_lpwwd_received_frame_count = 0;

    return ret_val;
}

cy_rslt_t svc_lp_lpwwd_reset_and_high_components(
        svc_lp_instance_t *lp_instance)
{
    (void) svc_lp_lpwwd_reset(lp_instance);
    /**
     * Reset the higher stage states if any exists.
     */
    (void) svc_lp_hpwwd_reset(lp_instance);
    (void) svc_lp_asr_reset(lp_instance);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t svc_lp_lpwwd_deinit(
        svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if (true == lp_instance->init_params.is_lpwwd_external)
    {
        ret_val = CY_RSLT_SUCCESS;
    }
    else
    {
#ifdef ENABLE_IFX_LPWWD
        ret_val = svc_lp_lpwwd_internal_deinit(lp_instance);
#else
        ret_val = CY_RSLT_SUCCESS;
#endif
    }

    return ret_val;
}

cy_rslt_t svc_lp_lpwwd_get_wwd_identified(
        svc_lp_instance_t *lp_instance,
        cy_svc_buffer_info_t *buff_info)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if(NULL == buff_info)
    {
        ret_val = CY_RSLT_SVC_BAD_ARG;
        goto CLEAN_RETURN;
    }
    *buff_info = 0;

#ifdef LPWWD_WAKE_WORD_MODEL
    if(LPWWD_WAKE_WORD_MODEL >= 10 && LPWWD_WAKE_WORD_MODEL <20)
    {
        *buff_info = CY_SVC_BUF_INFO_OK_INFINEON_WWD;
    }
    else if(LPWWD_WAKE_WORD_MODEL >= 20 && LPWWD_WAKE_WORD_MODEL <30)
    {
        *buff_info = CY_SVC_BUF_INFO_ALEXA_WWD;
    }
    else if(LPWWD_WAKE_WORD_MODEL >= 30 && LPWWD_WAKE_WORD_MODEL <40)
    {
        *buff_info = CY_SVC_BUF_INFO_OK_GOOGLE_WWD;
    }
    else
    {
        *buff_info = CY_SVC_BUF_INFO_OK_INFINEON_WWD;
    }
#else
    *buff_info = CY_SVC_BUF_INFO_OK_INFINEON_WWD;
#endif

    ret_val = CY_RSLT_SUCCESS;
    CLEAN_RETURN:
    return ret_val;
}


#endif
