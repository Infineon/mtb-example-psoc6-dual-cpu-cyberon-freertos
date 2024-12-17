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
 * @file staged_voice_control_lp_sod.c
 *
 */

#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_sod.h"
#include "staged_voice_control_lp_lpwwd.h"
#include "staged_voice_control_lp_hpwwd.h"
#include "staged_voice_control_lp_private.h"

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

cy_rslt_t svc_lp_sod_init(svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *init)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cy_sod_config_params config_params = {0};

    config_params.sampling_rate = CY_SOD_SAMPLE_RATE_16000Hz;
    config_params.input_frame_size = CY_SOD_FRAME_SIZE_MONO_SAMPLES;
    config_params.sensitivity = init->sod_sensitivity;
    config_params.onset_gap_setting_ms = init->sod_onset_gap_setting_ms;

    ret_val = cy_sod_init(&config_params, &lp_instance->sod_handle);

    return ret_val;
}

cy_rslt_t svc_lp_sod_deinit(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    ret_val = cy_sod_deinit(&lp_instance->sod_handle);
    lp_instance->sod_handle = NULL;
    return ret_val;
}

cy_rslt_t svc_lp_sod_check_if_any_data_to_be_sent_to_hp(
        svc_lp_instance_t *lp_instance,
        cy_sod_status_t sod_status,
        uint8_t *data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if (!(lp_instance->init_params.stage_config_list
            & CY_SVC_ENABLE_LPWWD)
            && ((lp_instance->init_params.stage_config_list
                    & CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS)
                    || (lp_instance->init_params.stage_config_list
                            & CY_SVC_ENABLE_ASR_STATE_TRANSITIONS)))
    {
        /**
         * SOD is detected, LPWWD is not enabled by the user, Hence
         * directly sending the buffer information to SVC HP after
         * SVC has detected SOD.
         */
        cy_svc_log_info(
                "LPWWD disabled, HPWWD/ASR enabled. Direct fwd data to HP");
        svc_lp_start_circular_buf_update_on_transition_to_hp(
                lp_instance, data, SVC_TRIGGER_SPEECH_ONSET_DETECTED);
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}

/**
 * Create the required resource for the instance.
 *
 * @param[in]  lp_instance             Staged voice control module instance
 * @param[in]  create_domain        Create low power domain configuration
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_lp_sod_process(svc_lp_instance_t *lp_instance, uint8_t *data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    bool b_speech_detect_required = false;
    cy_sod_status_t sod_status = CY_SOD_STATUS_INVALID;

    if ((CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION
            == lp_instance->current_stage)
            || (CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION
                    == lp_instance->current_stage))
    {
        b_speech_detect_required = true;
    }

    ret_val = cy_sod_process(lp_instance->sod_handle,
            b_speech_detect_required,
            (int16_t*) data,
            &sod_status);

    if(false == lp_instance->sod_detected)
    {
        if (CY_SOD_STATUS_DETECTED == sod_status)
        {
            /**
             * First time Speech detected.
             */
            lp_instance->sod_detected = true;

            (void) svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_SPEECH_ONSET_DETECTED);

#ifdef ENABLE_TIMELINE_MARKER
            (void) svc_lp_trigger_audio_timeline_marker_update(
                    SVC_TRIGGER_SPEECH_ONSET_DETECTED);
#endif

            cy_svc_log_dbg("SOD DETD [0x%08X,Cnt:%d]", data,
                    lp_instance->stats.sod_detect_counter_dbg);

#ifdef SIMULATE_SOD_IPC_TRIGGER_TEST
            svc_lp_app_ipc_send_command_to_trigger_event_from_hp(
                    CY_SVC_SET_STATE_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTED);
#endif

            if (!(lp_instance->init_params.stage_config_list
                    & CY_SVC_ENABLE_LPWWD))
            {
                if ((lp_instance->init_params.stage_config_list
                        & CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS)
                        || (lp_instance->init_params.stage_config_list
                                & CY_SVC_ENABLE_ASR_STATE_TRANSITIONS))
                {
                    (void) svc_lp_start_circular_buf_update_on_transition_to_hp(
                            lp_instance, data,
                            SVC_TRIGGER_SPEECH_ONSET_DETECTED);
                }
            }
        }
        else if (CY_SOD_STATUS_NOT_DETECTED == sod_status)
        {
            ;
        }
    }
    else
    {
        if(CY_SOD_STATUS_DETECTED == sod_status )
        {

            (void) svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_SPEECH_ONSET_DETECTED);

#ifdef ENABLE_TIMELINE_MARKER
            (void) svc_lp_trigger_audio_timeline_marker_update(
                    SVC_TRIGGER_SPEECH_ONSET_DETECTED);
#endif

            cy_svc_log_dbg("SOD REDETD, Addr:0x%08X,Cnt:%d", data,
                    lp_instance->stats.sod_detect_counter_dbg);
        }
        else if(CY_SOD_STATUS_NOT_DETECTED == sod_status )
        {
            ;
            //Nothing to be done for SOD onset not detected. SOD talks only
            //about onset of a speech.
        }
        else
        {
            //Speech detection is not required.
            ;
        }
        if (!(lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD))
        {
            if ((lp_instance->init_params.stage_config_list
                    & CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS)
                    || (lp_instance->init_params.stage_config_list
                            & CY_SVC_ENABLE_ASR_STATE_TRANSITIONS))
            {
                svc_lp_send_ipc_event_circular_buffer_update(lp_instance, data,
                        lp_instance->circular_shared_buffer->frame_size_in_bytes,
                        0, 0);
            }
        }
    }

    return ret_val;
}


/**
 * Create the required resource for the instance.
 *
 * @param[in]  lp_instance             Staged voice control module instance
 * @param[in]  create_domain        Create low power domain configuration
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_lp_sod_reset (svc_lp_instance_t *lp_instance)
{
    lp_instance->sod_detected = false;

    return CY_RSLT_SUCCESS;
}

/**
 * Create the required resource for the instance.
 *
 * @param[in]  lp_instance             Staged voice control module instance
 * @param[in]  create_domain        Create low power domain configuration
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_lp_sod_reset_and_high_components(svc_lp_instance_t *lp_instance)
{
    (void) svc_lp_sod_reset(lp_instance);
    /**
     * Reset the higher stage states if any exists.
     */
    (void) svc_lp_lpwwd_reset(lp_instance);
    (void) svc_lp_hpwwd_reset(lp_instance);
    (void) svc_lp_asr_reset(lp_instance);

    return CY_RSLT_SUCCESS;
}


#endif
