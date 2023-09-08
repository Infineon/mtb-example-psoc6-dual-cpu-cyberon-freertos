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
 * @file staged_voice_control_lp_stages.c
 *
 */
#ifdef ENABLE_SVC_LP_MW
#include "staged_voice_control_lp_private.h"
#include "staged_voice_control_lp_aad.h"
#include "staged_voice_control_lp_stages.h"
#include "staged_voice_control_lp_log_utils.h"
#include "staged_voice_control_lp_stats.h"


/*******************************************************************************
 *                              Macros
 ******************************************************************************/

#if 0
#define SET_CUR_STAGE(__X__) lp_instance->current_stage=__X__;  \
        cy_svc_log_info("SetCurrStage:%d, func:%s, Line:%d",__X__,__FUNCTION__,__LINE__) ;
#else
#if 0
#define SET_CUR_STAGE(__X__) \
        cy_svc_log_info("SetCurrStage[%d->%d]",lp_instance->current_stage,__X__) ; \
        lp_instance->current_stage=__X__;
#else
#define SET_CUR_STAGE(__X__)  lp_instance->current_stage=__X__;
#endif
#endif

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

cy_rslt_t svc_lp_send_event_to_app_on_stage_change(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t state_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cy_svc_event_t event = CY_SVC_EVENT_INVALID;

    switch (state_trigger)
    {
        case SVC_TRIGGER_INVALID:
        {
            break;
        }
        case SVC_TRIGGER_INSTANCE_INIT_DONE:
        {
            break;
        }
        case SVC_TRIGGER_PUT_TO_DEEP_SLEEP:
        {
            break;
        }
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_DETECTED:
        {
            event = CY_SVC_EVENT_ACOUSTIC_ACTIVITY_DETECTED;
            break;
        }
        case SVC_TRIGGER_SPEECH_ONSET_DETECTED:
        {
            event = CY_SVC_EVENT_SPEECH_ONSET_DETECTED;
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED:
        {
            event = CY_SVC_EVENT_LOW_POWER_WAKEUP_WORD_DETECTED;
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            event = CY_SVC_EVENT_LOW_POWER_WAKEUP_WORD_NOT_DETECTED;
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED:
        {
            event = CY_SVC_EVENT_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTED;
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            event = CY_SVC_EVENT_HIGH_PERFORMANCE_WAKEUP_WORD_NOT_DETECTED;
            break;
        }
        case SVC_TRIGGER_ASR_DETECTED:
        {
            event = CY_SVC_EVENT_ASR_DETECTED;
            break;
        }
        case SVC_TRIGGER_ASR_NOT_DETECTED:
        {
            event = CY_SVC_EVENT_ASR_NOT_DETECTED;
            break;
        }
        case SVC_TRIGGER_ASR_PROCESSING_COMPLETED:
        {
            event = CY_SVC_EVENT_ASR_PROCESSING_COMPLETED;
            break;
        }
        case SVC_TRIGGER_MAX:
        {
            break;
        }
        default:
        {
            break;
        }
    }


    if (CY_SVC_EVENT_INVALID != event)
    {
        if (NULL != lp_instance->init_params.event_callback)
        {
            ret_val = lp_instance->init_params.event_callback(event,
                    lp_instance->init_params.callback_user_arg);

#ifdef READABLE_SVC_LOG

            if (CY_RSLT_SUCCESS != ret_val)
            {
                cy_svc_log_err(ret_val, "Send [%s] fail",
                        svc_lp_events_to_printable_string(event));
            }
            else
            {
                cy_svc_log_info("Sent [%s]",
                        svc_lp_events_to_printable_string(event));
            }

#else
            if (CY_RSLT_SUCCESS != ret_val)
            {
                cy_svc_log_err(ret_val, "Send event to app fail");
            }
            else
            {
                cy_svc_log_info("Sent event[%d] to app", event);
            }
#endif
        }
        else
        {
            cy_svc_log_info("user callback is not registered");
        }
    }
    else
    {
        cy_svc_log_info("event to app is not required");
    }
    return ret_val;
}



static cy_rslt_t svc_lp_trigger_on_stage5_wait_for_processing_query(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch (stage_trigger)
    {
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY);
            (void) svc_lp_aad_reset_and_high_components(lp_instance);
            /**
             * TODO: Flush the data Q, if needed. To be added after experimenting
             * real time.
             */
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
            ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ASR_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ASR_PROCESSING_COMPLETED:
        {
            if (CY_SVC_ENABLE_ASR_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
                {
                    SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
                    ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
                }
                else if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD)
                {
                    SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
                    ret_val = svc_lp_lpwwd_reset_and_high_components(lp_instance);
                }
            }
            else
            {
                cy_svc_log_err(ret_val, "Invalid stage configuration 0x%x",
                        lp_instance->init_params.stage_config_list);
            }
            break;
        }
        default:
        {
            cy_svc_log_err(ret_val,
                    "Invalid Trigger on stage5 (wait for processing query) %d",
                    stage_trigger);
            break;
        }
    }
    return ret_val;
}


static cy_rslt_t svc_trigger_on_stage4_wait_for_asr_detection(
        svc_lp_instance_t* lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch (stage_trigger)
    {
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY);
            (void) svc_lp_aad_reset_and_high_components(lp_instance);
            /**
             * TODO: Flush the data Q, if needed. To be added after experimenting
             * real time.
             */
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
            ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ASR_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ASR_DETECTED:
        {
            if (CY_SVC_ENABLE_ASR_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_ASR_PROCESSING_QUERY_DETECTED);
                ret_val = CY_RSLT_SUCCESS;
            }
            else
            {
                cy_svc_log_err(ret_val, "Invalid stage configuration 0x%x",
                        lp_instance->init_params.stage_config_list);
                /*
                 * Return error, as this is a invalid stage handling,
                 * ASR is not enabled and invalid trigger
                 * */
            }
            break;
        }
        case SVC_TRIGGER_ASR_NOT_DETECTED:
        {
            if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
                ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            }
            else if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
                ret_val = svc_lp_lpwwd_reset_and_high_components(lp_instance);
            }
            break;
        }
        default:
        {
            cy_svc_log_err(ret_val,
                    "Invalid Trigger on stage4 (wait for asr detection) %d",
                    stage_trigger);
            break;
        }
    }
    return ret_val;
}

static cy_rslt_t svc_lp_trigger_on_stage3_wait_for_hpwwd_detection(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch (stage_trigger)
    {
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY);
            (void) svc_lp_aad_reset_and_high_components(lp_instance);
            /**
             * TODO: Flush the data Q, if needed. To be added after experimenting
             * real time.
             */
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
            ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ASR_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED:
        {
            if (CY_SVC_ENABLE_ASR_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);
                ret_val = CY_RSLT_SUCCESS;
            }
            else if (CY_SVC_ENABLE_SOD
                    & lp_instance->init_params.stage_config_list)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
                ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            }
            else if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
                ret_val = svc_lp_lpwwd_reset_and_high_components(lp_instance);
            }
            else
            {
                cy_svc_log_err(ret_val, "Invalid stage configuration 0x%x",
                        lp_instance->init_params.stage_config_list);
                /*
                 * Return error, as this is a invalid stage handling,
                 * ASR is not enabled and invalid trigger
                 * */
            }
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
                ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            }
            else if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
                ret_val = svc_lp_lpwwd_reset_and_high_components(lp_instance);
            }
            else
            {
                cy_svc_log_err(ret_val, "Invalid stage configuration 0x%x",
                        lp_instance->init_params.stage_config_list);
            }
            break;
        }
        default:
        {
            cy_svc_log_err(ret_val,
                    "Invalid Trigger on stage3 (wait for hpwwd detection) %d",
                    stage_trigger);
            break;
        }
    }

    return ret_val;
}

static cy_rslt_t svc_lp_trigger_on_stage2_wait_for_lpwwd_detection(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch (stage_trigger)
    {
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY);
            (void) svc_lp_aad_reset_and_high_components(lp_instance);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
            ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ASR_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED:
        {
            if (CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                /* Go to stage 3 */
                SET_CUR_STAGE(
                        CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION);
                ret_val = CY_RSLT_SUCCESS;
            }
            /**
             * Assumption, if HPWWD and ASR is enabled, then assumed that
             * HPWWD will be the first module and its output will be given to ASR
             * Hence the order of "if" check is done this way.
             */
            else if (CY_SVC_ENABLE_ASR_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                /* Go to stage 4 */
                SET_CUR_STAGE(
                        CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);

                ret_val = CY_RSLT_SUCCESS;
            }
            else if (CY_SVC_ENABLE_SOD
                    & lp_instance->init_params.stage_config_list)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
                ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            }
            else
            {
                cy_svc_log_err(ret_val, "Invalid stage configuration 0x%x",
                        lp_instance->init_params.stage_config_list);
                /*
                 * Return error, as this is a invalid stage handling,
                 * LPWWD, HPWWD and ASR is not enabled. Note sure what to do
                 * wit the data and the stage.
                 * */
            }
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
                ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            }
            else if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
                ret_val = svc_lp_lpwwd_reset_and_high_components(lp_instance);
            }
            break;
        }
        case SVC_TRIGGER_SPEECH_ONSET_DETECTED:
        {
            /**
             * Special case: Speech is re-detected at the stage2. LPWWD is enabled
             * and also the current stage is waiting for LPWWD. Hence directly
             * performing lpwwd reset and there is no change in stage is required.
             * LPWWD
             */
            cy_svc_log_dbg("Reset LPWWD due to SOD re-trigger");
            ret_val = svc_lp_lpwwd_reset_and_high_components(lp_instance);
            break;
        }
        default:
        {
            cy_svc_log_err(ret_val,
                    "Invalid Trigger on stage2 (wait for lpwwd detection) %d",
                    stage_trigger);
            break;
        }
    }
    return ret_val;
}

static cy_rslt_t svc_lp_trigger_on_stage1_wait_for_speech_detection(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch (stage_trigger)
    {
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY);
            (void) svc_lp_aad_reset_and_high_components(lp_instance);

            /**
             * TODO: Flush the data Q, if needed. To be added after experimenting
             * real time.
             */
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
            ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ASR_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_SPEECH_ONSET_DETECTED:
        {
            if (CY_SVC_ENABLE_LPWWD
                    & lp_instance->init_params.stage_config_list)
            {
                /* Go to Stage 2 */
                SET_CUR_STAGE(
                        CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);

//                cy_svc_log_dbg("Reset LPWWD due to SOD trigger");
                ret_val = svc_lp_lpwwd_reset_and_high_components(lp_instance);
            }
            /**
             * Assumption, if HPWWD and ASR is enabled, then assumed that
             * HPWWD will be the first module and its output will be given to ASR
             * Hence the order of "if" check is done this way.
             */
            else if (CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                /* Go to Stage 3 */
                SET_CUR_STAGE(
                        CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION);

                /**
                 * Enabling the direct HPWWD detection as LPWWD is skipped, so
                 * that data can be flowed from LP to HP
                 */
                ret_val = CY_RSLT_SUCCESS;
            }
            else if (CY_SVC_ENABLE_ASR_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                /* Go to Stage 4 */
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);
                ret_val = CY_RSLT_SUCCESS;
            }
            else if (CY_SVC_ENABLE_SOD
                    & lp_instance->init_params.stage_config_list)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
                ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            }
            else
            {
                cy_svc_log_err(ret_val, "Invalid stage configuration 0x%x",
                        lp_instance->init_params.stage_config_list);
                /*
                 * Return error, as this is a invalid stage handling,
                 * LPWWD, HPWWD and ASR is not enabled. Note sure what to do
                 * wit the data and the stage.
                 * */
            }
            break;
        }
        default:
        {
            cy_svc_log_err(ret_val,
                    "Invalid Trigger on stage1 (wait for speech detection) %d",
                    stage_trigger);
            break;
        }
    }
    return ret_val;
}


static cy_rslt_t svc_lp_trigger_on_stage0_wait_for_acoustic_activity(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch (stage_trigger)
    {
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        {
            /**
             * Nothing to be done, as already the stage is AAD only.
             */
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_DETECTED:
        {
            if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
                ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            }
            else if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD)
            {
                SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
                ret_val = svc_lp_lpwwd_reset_and_high_components(lp_instance);
            }
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION);
            ret_val = svc_lp_sod_reset_and_high_components(lp_instance);
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        case SVC_TRIGGER_ASR_DETECTION_FORCEFULLY:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }
        default:
        {
            cy_svc_log_err(ret_val,
                    "Invalid Trigger on stage0 (wait for acoustic activity) %d",
                    stage_trigger);
            break;
        }
    }
    return ret_val;
}

static cy_rslt_t svc_lp_handle_trigger_on_idle_state(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch (stage_trigger)
    {
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        case SVC_TRIGGER_INSTANCE_INIT_DONE:
        {
            SET_CUR_STAGE(CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY);
            (void) svc_lp_aad_reset_and_high_components(lp_instance);
            ret_val = CY_RSLT_SUCCESS;
            break;
        }

        default:
        {
            ret_val = CY_RSLT_SVC_INVALID_STAGE_TRIGGER;
            cy_svc_log_err(ret_val,
                    "Invalid Trigger on stage (idle/invalid) %d", stage_trigger);
            break;
        }
    }
    return ret_val;
}


cy_rslt_t svc_lp_trigger_stage_from_lp_app_set_stage (
        svc_lp_instance_t *lp_instance,
        cy_svc_stage_t   set_stage)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if(lp_instance->current_stage == set_stage)
    {
        cy_svc_log_info("CurStage and SetStage [%d] are same already",
                set_stage);
        ret_val = CY_RSLT_SUCCESS;
    }
    else
    {
        switch (set_stage)
        {
            case CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY:
            {
                ret_val = svc_lp_trigger_state(lp_instance,
                        SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY);
                break;
            }
            case CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION:
            {
                ret_val = svc_lp_trigger_state(lp_instance,
                        SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY);
                break;
            }
            case CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION:
            {
                ret_val = svc_lp_trigger_state(lp_instance,
                        SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY);
                break;
            }
            case CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION:
            {
                ret_val = svc_lp_trigger_state(lp_instance,
                        SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY);
                break;
            }
            case CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT:
            {
                ret_val = svc_lp_trigger_state(lp_instance,
                        SVC_TRIGGER_ASR_DETECTION_FORCEFULLY);
                break;
            }
            case CY_SVC_STAGE_ASR_PROCESSING_QUERY_DETECTED:
            {
                /**
                 * Set stage from ASR processing Query completed from M33 APP
                 * is not a valid scenario. Hence returning failure.
                 */
                ret_val = CY_RSLT_SVC_INVALID_STAGE_TRIGGER;
                break;
            }
            default:
            {
                break;
            }
        }
    }

    return ret_val;
}

cy_rslt_t svc_lp_trigger_state_from_hp_set_state(
        svc_lp_instance_t *lp_instance,
        cy_svc_set_state_t  set_state,
        void *set_state_info)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch(set_state)
    {
        case CY_SVC_SET_STATE_HIGH_PERFORMANCE_DETECT_IN_PROGRESS:
        {
            if (CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION
                    == lp_instance->current_stage)
            {
                cy_svc_hp_set_state_hpwwd_det_in_prog_info_t *hpwwd_state_info =
                        set_state_info;

                if ((hpwwd_state_info->action & CY_SVC_SEND_POST_WWD_BUFFER)
                        && (hpwwd_state_info->post_wwd_frame_count))
                {
                    cy_svc_log_info("PostWWD FrameCount needed:%d",
                            hpwwd_state_info->post_wwd_frame_count);

                    lp_instance->post_wwd_frame_count_req_by_hp =
                            hpwwd_state_info->post_wwd_frame_count;

                    svc_lp_start_circular_buf_update_on_transition_to_hp(
                            lp_instance, NULL,
                            SVC_TRIGGER_SEND_POST_WWD_HPWWD_DET_IN_PROGRESS);
                    ret_val = CY_RSLT_SUCCESS;
                }
            }
            else
            {
                ret_val = CY_RSLT_SVC_INVALID_STAGE;
            }
            break;
        }
        case CY_SVC_SET_STATE_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTED:
        {
            if (CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                ret_val = svc_lp_trigger_state(lp_instance,
                        SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED);

                svc_lp_start_circular_buf_update_on_transition_to_hp(
                        lp_instance, NULL,
                        SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED);
            }

            break;
        }
        case CY_SVC_SET_STATE_HIGH_PERFORMANCE_WAKEUP_WORD_NOT_DETECTED:
        {
            ret_val = svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_NOT_DETECTED);
            break;
        }
        case CY_SVC_SET_STATE_ASR_DETECTED:
        {
            cy_svc_hp_set_state_asr_detected_info_t *asr_dected_info =
                    set_state_info;

            ret_val = svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_ASR_DETECTED);

            if (CY_SVC_STAGE_ASR_PROCESSING_QUERY_DETECTED
                    == lp_instance->current_stage)
            {
                if (CY_SVC_STREAM_DATA_REQUEST & asr_dected_info->action)
                {
                    lp_instance->stream_requested_on_asr_processing_query_state =
                            true;
                    cy_svc_log_info("Stream requested on ASR detected state");
                }
                else
                {
                    lp_instance->stream_requested_on_asr_processing_query_state =
                            false;
                    cy_svc_log_info(
                            "Stream not requested on ASR detected state");
                }
            }
            else
            {
                ret_val = CY_RSLT_SVC_INVALID_STAGE;
            }
            break;
        }
        case CY_SVC_SET_STATE_ASR_NOT_DETECTED:
        {
            ret_val = svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_ASR_NOT_DETECTED);
            break;
        }
        case CY_SVC_SET_STATE_ASR_PROCESSING_COMPLETED:
        {
            ret_val = svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_ASR_PROCESSING_COMPLETED);
            lp_instance->stream_requested_on_asr_processing_query_state =
                    false;
            break;
        }
        default:
        {
            break;
        }
    }

    return ret_val;
}
    

#ifdef ENABLE_TIMELINE_MARKER
cy_rslt_t svc_lp_trigger_audio_timeline_marker_update (
        svc_stage_trigger_t stage_trigger)
{
    switch(stage_trigger)
    {
        case SVC_TRIGGER_INSTANCE_INIT_DONE:
        case SVC_TRIGGER_PUT_TO_DEEP_SLEEP:
        {
            break;
        }
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        {
            break;
        }
        case SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY:
        {
            break;
        }
        case  SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            break;
        }
        case  SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY:
        {
            break;
        }
        case SVC_TRIGGER_ASR_DETECTION_FORCEFULLY:
        {
            break;
        }
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_DETECTED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_AAD_DETECTED, NULL);
            break;
        }
        case SVC_TRIGGER_SPEECH_ONSET_DETECTED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_SOD_DETECTED, NULL);
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_LPWWD_DETECTED, NULL);
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_LPWWD_NOT_DETECTED, NULL);
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_HPWWD_DETECTED, NULL);
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_HPWWD_NOT_DETECTED, NULL);
            break;
        }
        case SVC_TRIGGER_ASR_DETECTED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_ASR_DETECTED, NULL);
            break;
        }
        case SVC_TRIGGER_ASR_NOT_DETECTED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_ASR_NOT_DETECTED, NULL);
            break;
        }
        case SVC_TRIGGER_ASR_PROCESSING_COMPLETED:
        {
            cy_audio_timeline_marker_update_status(
                    CY_AUDIO_TIMELINE_ASR_PROCESS_COMPLETED, NULL);
            break;
        }
        case SVC_TRIGGER_MAX:
        default:
        {
            break;
        }
    }

    return CY_RSLT_SUCCESS;
}
#endif
cy_rslt_t svc_lp_trigger_state(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cy_svc_stage_t temp_stage = CY_SVC_STAGE_INVALID;

    temp_stage = lp_instance->current_stage;

    (void) svc_lp_stats_update_and_and_print(lp_instance, stage_trigger);

    /**
     * Handle the trigger depends on the current SVC stage.
     */
#if 0
#ifdef READABLE_SVC_LOG
    cy_svc_log_info("TRIG ->%s, CStage:%s",
            svc_lp_state_trigger_to_printable_string(stage_trigger),
            svc_lp_stage_to_printable_string(temp_stage));
#else
    cy_svc_log_info("trigger_state: CurStage:%d StateTrigger:%d",
            temp_stage, stage_trigger);
#endif
#endif

    switch (lp_instance->current_stage)
    {
        case CY_SVC_STAGE_INVALID:
        {
            ret_val = svc_lp_handle_trigger_on_idle_state(lp_instance,
                    stage_trigger);
            break;
        }
        case CY_SVC_STAGE_UNKNOWN:
        {
            break;
        }
        case CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY:
        {
            ret_val = svc_lp_trigger_on_stage0_wait_for_acoustic_activity(
                    lp_instance, stage_trigger);
            break;
        }
        case CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION:
        {
            ret_val = svc_lp_trigger_on_stage1_wait_for_speech_detection(
                    lp_instance, stage_trigger);
            break;
        }
        case CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION:
        {
            ret_val = svc_lp_trigger_on_stage2_wait_for_lpwwd_detection(
                    lp_instance, stage_trigger);
            break;
        }
        case CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION:
        {
            ret_val = svc_lp_trigger_on_stage3_wait_for_hpwwd_detection(
                    lp_instance, stage_trigger);
            break;
        }
        case CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT:
        {
            ret_val = svc_trigger_on_stage4_wait_for_asr_detection(
                    lp_instance, stage_trigger);
            break;
        }
        case CY_SVC_STAGE_ASR_PROCESSING_QUERY_DETECTED:
        {
            ret_val = svc_lp_trigger_on_stage5_wait_for_processing_query(
                    lp_instance, stage_trigger);
            break;
        }
        case CY_SVC_STAGE_MAX:
        default:
        {
            break;
        }
    }

    if (temp_stage != lp_instance->current_stage)
    {
        if (CY_RSLT_SUCCESS != ret_val)
        {
#ifdef READABLE_SVC_LOG
            cy_svc_log_err(ret_val, "TRIG Fail->[%d:%s],[%s->%s]",
                    stage_trigger,
                    svc_lp_state_trigger_to_printable_string(stage_trigger),
                    svc_lp_stage_to_printable_string(temp_stage),
                    svc_lp_stage_to_printable_string(lp_instance->current_stage));
#else
            cy_svc_log_err(ret_val,
                    "StateTrigger:%d fail,[%d->%d]"
                    stage_trigger,temp_stage, lp_instance->current_stage);
#endif
        }
        else
        {
#ifdef READABLE_SVC_LOG
            cy_svc_log_info("TRIG Succ->[%d:%s],[%s->%s]",
                    stage_trigger,
                    svc_lp_state_trigger_to_printable_string(stage_trigger),
                    svc_lp_stage_to_printable_string(temp_stage),
                    svc_lp_stage_to_printable_string(lp_instance->current_stage));
#else
            cy_svc_log_info("StateTrigger:%d,[%d->%d]",
                    stage_trigger,temp_stage, lp_instance->current_stage);
#endif
        }

        /**
         * Send event change event to application.
         *
         * Intentionally ignoring the error code returned by application. Error has
         * been printed, the application has to handle.
         */
        (void) svc_lp_send_event_to_app_on_stage_change(lp_instance,
                stage_trigger);
    }
    else
    {
        if (CY_RSLT_SUCCESS != ret_val)
        {
#ifdef READABLE_SVC_LOG
            cy_svc_log_info("TRIG Fail->%s,StageNoChg CStage:%s",
                    svc_lp_state_trigger_to_printable_string(stage_trigger),
                    svc_lp_stage_to_printable_string(lp_instance->current_stage));
#else
            cy_svc_log_err(ret_val, "StateTrigger:%d fail, StageNoChg:%d",
                    stage_trigger, lp_instance->current_stage);
#endif
        }
        else
        {
#ifdef READABLE_SVC_LOG
            cy_svc_log_info("TRIG Succ->%s,StageNoChg CStage:%s",
                    svc_lp_state_trigger_to_printable_string(stage_trigger),
                    svc_lp_stage_to_printable_string(lp_instance->current_stage));
#else
            cy_svc_log_info("StateTrigger:%d, StageNoChg:%d",
                    stage_trigger, lp_instance->current_stage);
#endif
        }
    }

#ifdef ENABLE_TIMELINE_MARKER
    if (SVC_TRIGGER_SPEECH_ONSET_DETECTED != stage_trigger)
    {
        svc_lp_trigger_audio_timeline_marker_update(stage_trigger);
    }
#endif

    return ret_val;
}

#endif
