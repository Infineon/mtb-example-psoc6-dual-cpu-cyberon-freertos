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
 * @file staged_voice_control_lp_log_utils.c
 *
 */

#ifdef ENABLE_SVC_LP_MW
#include "staged_voice_control_lp_private.h"
#include "staged_voice_control_lp_stats.h"

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

cy_rslt_t svc_lp_stats_update_and_and_print(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t trigger)
{
    bool print_stats = false;

    switch (trigger)
    {
        case SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY:
        {
            print_stats = true;
            break;
        }
        case SVC_TRIGGER_PRINT_STATISTICS:
        {
            print_stats = true;
            break;
        }
        case SVC_TRIGGER_SPEECH_ONSET_DETECTED:
        {
            lp_instance->stats.sod_detect_counter_dbg++;

            if (lp_instance->stats.sod_detect_counter_dbg
                    % PRINT_STATS_ON_EVERY_SOD_DETECT_COUNTER == 0)
            {
                print_stats = true;
            }
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED:
        {
            lp_instance->stats.lpwwd_detect_counter_dbg++;

            if (lp_instance->stats.lpwwd_detect_counter_dbg
                    % PRINT_STATS_ON_EVERY_LPWWD_DETECT_COUNTER == 0)
            {
                print_stats = true;
            }
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            lp_instance->stats.lpwwd_not_detect_counter_dbg++;

            if (lp_instance->stats.lpwwd_not_detect_counter_dbg
                    % PRINT_STATS_ON_EVERY_LPWWD_NOT_DETECT_COUNTER == 0)
            {
                print_stats = true;
            }
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED:
        {
            lp_instance->stats.hpwwd_detect_counter_dbg++;
            if (lp_instance->stats.hpwwd_detect_counter_dbg
                    % PRINT_STATS_ON_EVERY_HPWWD_DETECT_COUNTER == 0)
            {
                print_stats = true;
            }
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_NOT_DETECTED:
        {
            lp_instance->stats.hpwwd_not_detect_counter_dbg++;
            if (lp_instance->stats.hpwwd_not_detect_counter_dbg
                    % PRINT_STATS_ON_EVERY_HPWWD_NOT_DETECT_COUNTER == 0)
            {
                print_stats = true;
            }
            break;
        }
        case SVC_TRIGGER_ASR_DETECTED:
        {
            lp_instance->stats.asr_detect_counter_dbg++;
            if (lp_instance->stats.asr_detect_counter_dbg
                    % PRINT_STATS_ON_EVERY_ASR_DETECT_COUNTER == 0)
            {
                print_stats = true;
            }
            break;
        }
        case SVC_TRIGGER_ASR_NOT_DETECTED:
        {
            lp_instance->stats.asr_not_detect_counter_dbg++;
            if (lp_instance->stats.asr_not_detect_counter_dbg
                    % PRINT_STATS_ON_EVERY_ASR_NOT_DETECT_COUNTER == 0)
            {
                print_stats = true;
            }
            break;
        }
        case SVC_TRIGGER_ASR_PROCESSING_COMPLETED:
        {
            lp_instance->stats.asr_process_completed_counter_dbg++;
            if (lp_instance->stats.asr_process_completed_counter_dbg
                    % PRINT_STATS_ON_EVERY_ASR_PROCESS_COMPLETED_COUNTER == 0)
            {
                print_stats = true;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if (true == print_stats)
    {
        cy_svc_log_info(
                "STATS:Feed(%d:%d:%d:%d)SOD(%d)LPW(%d:%d:%d)HPW(%d,%d:%d)ASR(%d:%d:%d)",
                lp_instance->stats.frame_counter_received_after_last_aad_dbg,
                lp_instance->stats.svc_lp_feed_post_fail_counter_dbg,
                lp_instance->stats.crc_check_fail_counter_dbg,
                lp_instance->stats.last_frame_feed_time_ms,
                lp_instance->stats.sod_detect_counter_dbg,
                lp_instance->stats.lpwwd_feed_counter_dbg,
                lp_instance->stats.lpwwd_detect_counter_dbg,
                lp_instance->stats.lpwwd_not_detect_counter_dbg,
                lp_instance->stats.hpwwd_feed_counter_dbg,
                lp_instance->stats.hpwwd_detect_counter_dbg,
                lp_instance->stats.hpwwd_not_detect_counter_dbg,
                lp_instance->stats.asr_detect_counter_dbg,
                lp_instance->stats.asr_process_completed_counter_dbg,
                lp_instance->stats.asr_not_detect_counter_dbg)
    }

    return CY_RSLT_SUCCESS;
}

#endif
