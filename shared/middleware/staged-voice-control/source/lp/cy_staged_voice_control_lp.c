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
 * @file cy_staged_voice_control_lp.c
 *
 * @brief Staged voice control API implementation for library
 * running in low power domain (M33).
 *
 */

#include "cy_staged_voice_control.h"
#include "staged_voice_control_lp_private.h"

#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_resource.h"
#include "staged_voice_control_lp_process_data.h"
#include "staged_voice_control_lp_resource.h"
#ifndef ENABLE_MIC_INPUT_FEED
#ifdef ENABLE_USB_DBG_OUTPUT
#include "audio_usb_send_utils.h"
#endif
#endif

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

/**
 * Initializes the staged voice control module in low power device, creates the
 * required resources based on the input configuration, the resource could be
 * buffer, internal task, IPC communication, etc.,
 *
 * @param[in]  init                 Staged voice control module init
 *                                  configuration parameter in low power domain.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_init(cy_svc_lp_config_t *init)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    svc_lp_instance_t *lp_instance = NULL;

    if (NULL == init)
    {
        ret_val = CY_RSLT_SVC_BAD_ARG;
        cy_svc_log_err(ret_val, "NULL params %p", init);
        goto CLEAN_RETURN;
    }

    lp_instance = svc_lp_get_instance();

    if (true == lp_instance->init_done)
    {
        ret_val = CY_RSLT_SVC_ALREADY_INITIALIZED;
        cy_svc_log_err(ret_val, "cy_svc_lp_init already initialized");
        goto CLEAN_RETURN;
    }

    cy_mem_get_allocated_memory("Start => cy_svc_lp_init");

    ret_val = svc_lp_create_resource_for_instance(lp_instance, init);
    if (CY_RSLT_SUCCESS == ret_val)
    {
        cy_svc_log_info("cy_svc_lp_init success, SVC ret base: 0x%x",
                CY_RSLT_SVC_ERR_BASE);
        lp_instance->init_done = true;
        lp_instance->api_set_allowed = true;
    }
    else
    {
        memset(lp_instance, 0, sizeof(*lp_instance));
        cy_svc_log_err(ret_val, "cy_svc_lp_init fail");
    }

    cy_mem_get_allocated_memory("End => cy_svc_lp_init");

    CLEAN_RETURN: return ret_val;
}

/**
 * Get the current stage of staged voice control module.
 *
 * @param[out]  stage                staged voice control module's current stage
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_get_current_stage(cy_svc_stage_t *stage)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    svc_lp_instance_t *lp_instance = NULL;

    lp_instance = svc_lp_get_instance();

    if (false == lp_instance->init_done)
    {
        ret_val = CY_RSLT_SVC_NOT_INITIALIZED;
        cy_svc_log_err(ret_val, "SVC LP is not yet initialized");
        goto CLEAN_RETURN;
    }

    if(false == lp_instance->api_set_allowed)
    {
        ret_val = CY_RSLT_SVC_INVALID_STATE;
        cy_svc_log_err_on_no_isr(ret_val, "get stage not allowed");
        goto CLEAN_RETURN;
    }

    if (NULL == stage)
    {
        ret_val = CY_RSLT_SVC_BAD_ARG;
        cy_svc_log_err_on_no_isr(ret_val, "Null argument %p", stage);
        goto CLEAN_RETURN;
    }

    /**
     * Mutex protection is not added intentionally, as all the cy_svc_lp_set_stage
     * set stage apis are synchronized with process. Hence it may not be required
     */
    *stage = lp_instance->current_stage;
    ret_val = CY_RSLT_SUCCESS;

    CLEAN_RETURN: return ret_val;
}

/**
 * SVC-LP application can force the stage in SVC module.
 *
 * @param[in] stage                 Required stage to set.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_set_stage(cy_svc_stage_t stage)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    svc_lp_instance_t *lp_instance = NULL;
    bool valid_stage = false;

    lp_instance = svc_lp_get_instance();

    cy_svc_log_info("SetStage:%d", stage);

    if (false == lp_instance->init_done)
    {
        ret_val = CY_RSLT_SVC_NOT_INITIALIZED;
        cy_svc_log_err(ret_val, "SVC LP is not yet initialized");
        goto CLEAN_RETURN;
    }

    if(false == lp_instance->api_set_allowed)
    {
        ret_val = CY_RSLT_SVC_INVALID_STATE;
        cy_svc_log_err_on_no_isr(ret_val, "Set stage not allowed");
        goto CLEAN_RETURN;
    }

    switch (stage)
    {
        case CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY:
        {
            cy_svc_log_err(ret_val,
                    "TODO: Post AAD wait data id to data Q. Empty the data Q");
            /**
             * TODO: Post a AAD wait data id to data Q. Empty the data Q till
             * and then fallback to AAD stage.
             *
             * TODO: To be discussed, this API for this requirement
             * to be synchronous or async and it depends on the application
             * requirement on timings to put LP core to deep sleep.
             */
            valid_stage = true;
            break;
        }
        case CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION:
        {
            if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
            {
                valid_stage = true;
            }
            break;
        }
        case CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION:
        {
            if (lp_instance->init_params.stage_config_list
                    & CY_SVC_ENABLE_LPWWD)
            {
                valid_stage = true;
            }
            break;
        }

        case CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION:
        {
            if (lp_instance->init_params.stage_config_list
                    & CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS)
            {
                valid_stage = true;
            }
            break;
        }
        case CY_SVC_STAGE_ASR_PROCESSING_QUERY_DETECTED:
        case CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT:
        {
            if (lp_instance->init_params.stage_config_list
                    & CY_SVC_ENABLE_ASR_STATE_TRANSITIONS)
            {
                valid_stage = true;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if (false == valid_stage)
    {
        ret_val = CY_RSLT_SVC_BAD_ARG;
        cy_svc_log_err(ret_val, "Invalid stage[%d], cfg-lp[0x%x]", stage,
                lp_instance->init_params.stage_config_list);
        goto CLEAN_RETURN;
    }

    ret_val = svc_lp_post_to_cmd_q_to_set_stage_from_lp_app(lp_instance, stage);
    if (CY_RSLT_SUCCESS == ret_val)
    {
        (void) cy_rtos_get_semaphore(&lp_instance->cmdProcSyncSemaphore,
        CY_RTOS_NEVER_TIMEOUT, is_in_isr());
        /**
         * The result is a volatile variable and it will be updated by the SVC
         * task after processing the set stage request.
         */
        if (CY_RSLT_SUCCESS != lp_instance->lp_app_set_stage_process_result)
        {
            cy_svc_log_err(lp_instance->lp_app_set_stage_process_result,
                    "set stage fail");
        }
        else
        {
            cy_svc_log_info("set stage success");
        }

        ret_val = lp_instance->lp_app_set_stage_process_result;
    }

    CLEAN_RETURN: return ret_val;
}

/**
 * Feed input audio data to staged voice control module in low power domain.
 *
 * @param[in]  data                 Pointer to the input audio data.
 *
 *                                  Application can free/reuse the buffer pointer
 *                                  passed in this API once the API is returned.
 *                                  (i.e., Staged voice module will make a copy
 *                                  of the input data internally in its buffer).
 *                                  Length of the input audio data passed in the
 *                                  buffer is constant should not vary for every
 *                                  feed.(i.e, Application should feed always the
 *                                  complete buffer).
 *
 *                                  If Application needs to feed stereo data,
 *                                  then the data format must be non-interleaved
 *                                  format. example: For stereo data of 10ms
 *                                  worth, first 320bytes must be of channel-1
 *                                  and then next 320bytes must be channel-2.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */

//#define SVC_LP_FEED_START_SIMULUATION 150

cy_rslt_t cy_svc_lp_feed(
CY_SVC_DATA_T *data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    svc_lp_instance_t *lp_instance = NULL;

    lp_instance = svc_lp_get_instance();

    if (false == lp_instance->init_done)
    {
        ret_val = CY_RSLT_SVC_NOT_INITIALIZED;
        cy_svc_log_err_on_no_isr(ret_val, "SVC LP is not yet initialized");
        goto CLEAN_RETURN;
    }

    if(false == lp_instance->api_set_allowed)
    {
        ret_val = CY_RSLT_SVC_INVALID_STATE;
        cy_svc_log_err_on_no_isr(ret_val, "feed not allowed");
        goto CLEAN_RETURN;
    }

    if (NULL == data)
    {
        ret_val = CY_RSLT_SVC_BAD_ARG;
        cy_svc_log_err_on_no_isr(ret_val, "Null argument");
        goto CLEAN_RETURN;
    }

#ifdef SVC_LP_FEED_START_SIMULUATION
    {
        static unsigned int feed_counter = 0;
        if ((++feed_counter) < SVC_LP_FEED_START_SIMULUATION)
            return CY_RSLT_SUCCESS;
    }
#endif

//#ifndef ENABLE_MIC_INPUT_FEED
//#ifdef ENABLE_USB_DBG_OUTPUT
//        usb_send_out_dbg_put(2, data);
//#endif
//#endif

    ret_val = svc_lp_feed_data_to_cbuf_and_notify_task(lp_instance, data);
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err_on_no_isr(ret_val,
                "svc_lp_feed_data_to_cbuf_and_notify_task fail");
    }

    CLEAN_RETURN: return ret_val;
}

/**
 * Deinit the staged voice control module in low power device, deletes the
 * resources created during \ref cy_svc_lp_init.
 *
 * @param none
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_deinit(void)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    svc_lp_instance_t *lp_instance = NULL;

    cy_mem_get_allocated_memory("Start => cy_svc_lp_deinit");

    lp_instance = svc_lp_get_instance();

    lp_instance->api_set_allowed = false;

    if (false == lp_instance->init_done)
    {
        ret_val = CY_RSLT_SVC_NOT_INITIALIZED;
        cy_svc_log_err(ret_val, "cy_svc_lp_init is not initialized");
        goto CLEAN_RETURN;
    }

    ret_val = svc_lp_delete_resource_for_instance(lp_instance);
    if (CY_RSLT_SUCCESS == ret_val)
    {
        memset(lp_instance, 0, sizeof(*lp_instance));
        cy_svc_log_info("cy_svc_lp_deinit success");
    }
    else
    {
        cy_svc_log_err(ret_val, "cy_svc_lp_deinit fail");
    }

    cy_mem_get_allocated_memory("End => cy_svc_lp_deinit");

    CLEAN_RETURN: return ret_val;
}
#endif
