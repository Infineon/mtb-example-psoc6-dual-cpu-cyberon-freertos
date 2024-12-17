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
 * @file staged_voice_control_lp_log_utils.c
 *
 */

#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_sod.h"
#include "cy_sod.h"

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
#ifdef READABLE_SVC_LOG

const char* svc_lp_state_trigger_to_printable_string(
        svc_stage_trigger_t stage_trigger)
{
    char *str = NULL;

    switch (stage_trigger)
    {
        case SVC_TRIGGER_INVALID:
            str = "TR_INVALID";
            break;

        case SVC_TRIGGER_INSTANCE_INIT_DONE:
            str = "TR_INIT_DONE";
            break;

        case SVC_TRIGGER_PUT_TO_DEEP_SLEEP:
            str = "TR_PUT_DEEP_SLP";
            break;

        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
            str = "TR_AAD_DET_FORCE";
            break;
        case SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY:
            str = "TR_WAIT_SOD_DET_FORCE";
            break;

        case SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY:
            str = "TR_WAIT_LPWWD_DET_FORCE";
            break;

        case SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY:
            str = "TR_WAIT_HPWWD_DET_FORCE";
            break;
        case SVC_TRIGGER_ASR_DETECTION_FORCEFULLY:
            str = "TR_ASR_DET_FORCE";
            break;
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_DETECTED:
            str = "TR_AAD_DETD";
            break;
        case SVC_TRIGGER_SPEECH_ONSET_DETECTED:
            str = "TR_SOD_DETD";
            break;
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED:
            str = "TR_LPWWD_DETD";
            break;
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED:
            str = "TR_LPWWD_NDETD";
            break;
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED:
            str = "TR_HPWWD_DETD";
            break;
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_NOT_DETECTED:
            str = "TR_HPWWD_NDETD";
            break;
        case SVC_TRIGGER_ASR_DETECTED:
            str = "TR_ASR_DETD";
            break;
        case SVC_TRIGGER_ASR_NOT_DETECTED:
            str = "TR_ASR_NDETD";
            break;
        case SVC_TRIGGER_ASR_PROCESSING_COMPLETED:
            str = "TR_ASR_PROC_DONE";
            break;
        case SVC_TRIGGER_MAX:
            str = "TR_MAX";
            break;
        default:
            str = "TR_UKN";
            break;
    }
    return str;
}

const char* svc_lp_stage_to_printable_string(cy_svc_stage_t current_stage)
{
    char *str = NULL;
    switch (current_stage)
    {
        case CY_SVC_STAGE_INVALID:
            str = "STG_INV";
            break;

        case CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY:
            str = "STG_WAIT_AAD";
            break;

        case CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION:
            str = "STG_WAIT_SOD";
            break;

        case CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION:
            str = "STG_WAIT_LPWWD";
            break;

        case CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION:
            str = "STG_WAIT_HPWWD";
            break;

        case CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT:
            str = "STG_WAIT_ASR";
            break;

        case CY_SVC_STAGE_ASR_PROCESSING_QUERY_DETECTED:
            str = "STG_WAIT_ASR_PROC";
            break;

        case CY_SVC_STAGE_MAX:
            str = "STG_MAX";
            break;

        default:
            str = "STG_UKN";
            break;
    }
    return str;
}


const char* svc_lp_events_to_printable_string(cy_svc_event_t event)
{
    char *str = NULL;
    switch (event)
    {
        case CY_SVC_EVENT_INVALID:
        {
            str = "ET_INV";
            break;
        }
        case CY_SVC_EVENT_ACOUSTIC_ACTIVITY_DETECTED:
        {
            str = "ET_AAD_DETD";
            break;
        }
        case CY_SVC_EVENT_LOW_NOISE_DETECTED:
        {
            str = "ET_LNOISE_DETD";
            break;
        }
        case CY_SVC_EVENT_SPEECH_ONSET_DETECTED:
        {
            str = "ET_SOD_DETD";
            break;
        }
        case CY_SVC_EVENT_LOW_POWER_WAKEUP_WORD_DETECTED:
        {
            str = "ET_LPWWD_DETD";
            break;
        }
        case CY_SVC_EVENT_LOW_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            str = "ET_LPWWD_NDETD";
            break;
        }
        case CY_SVC_EVENT_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTED:
        {
            str = "ET_HPWWD_DETD";
            break;
        }
        case CY_SVC_EVENT_HIGH_PERFORMANCE_WAKEUP_WORD_NOT_DETECTED:
        {
            str = "ET_HPWWD_NDETD";
            break;
        }
        case CY_SVC_EVENT_ASR_DETECTED:
        {
            str = "ET_ASR_DETD";
            break;
        }
        case CY_SVC_EVENT_ASR_NOT_DETECTED:
        {
            str = "ET_ASR_NDETD";
            break;
        }
        case CY_SVC_EVENT_ASR_PROCESSING_COMPLETED:
        {
            str = "ET_ASR_PROC_DONE";
            break;
        }
        case CY_SVC_EVENT_MAX:
        {
            str = "ET_MAX";
            break;
        }
        default:
        {
            str = "ET_UKN";
            break;
        }
    }
    return str;
}
#endif

#endif
