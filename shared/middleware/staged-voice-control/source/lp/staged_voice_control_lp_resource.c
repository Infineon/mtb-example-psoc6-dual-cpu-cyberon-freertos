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
 * @file staged_voice_control_lp_resource.c
 *
 */

#ifdef ENABLE_SVC_LP_MW
#include "staged_voice_control_lp_resource.h"

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

svc_lp_instance_t global_lp_instance = { 0 };

/*******************************************************************************
 *                              Function Declarations
 ******************************************************************************/


svc_lp_instance_t* svc_lp_get_instance(void)
{
    return &global_lp_instance;
}

/**
 * stage voice thread process function
 *
 * @param[in]  thread_input             argument to thread.
 * @param[in]  create_domain        Create low power domain configuration
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
static void svc_lp_thread_func(cy_thread_arg_t thread_input)
{
    svc_lp_instance_t *lp_instance = (svc_lp_instance_t *) thread_input;

    cy_svc_log_info("Entering SVC thread proc");

    if (NULL != lp_instance)
    {
        cy_svc_log_info("proc SVC thread proc started");

        while (false == lp_instance->quit_thread_instance)
        {
            (void) svc_lp_get_data_from_data_queue_and_process(lp_instance);
        }
    }

    cy_svc_log_info("Exiting SVC thread proc");

    cy_rtos_exit_thread();
}

static cy_rslt_t svc_lp_validate_init_params(
        svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *init)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_BAD_ARG;

    if ((CY_SVC_AUDIO_INPUT_TYPE_INVALID >= init->audio_input_type)
            || (CY_SVC_AUDIO_INPUT_TYPE_MAX <= init->audio_input_type))
    {
        cy_svc_log_err(ret_val, "Invalid audio_input_type: %d",
                init->audio_input_type);
        goto CLEAN_RETURN;
    }

    if (CY_SVC_SUPPORTED_FRAME_TIME_MS != init->single_frame_time_ms)
    {
        cy_svc_log_err(ret_val, "Invalid single_frame_time_ms %d",
                init->single_frame_time_ms);
        goto CLEAN_RETURN;
    }

    if ((CY_SVC_SAMPLE_RATE_INVALID >= init->sample_rate)
            || (CY_SVC_SAMPLE_RATE_MAX <= init->sample_rate))
    {
        cy_svc_log_err(ret_val, "Input sample_rate: %d", init->sample_rate);
        goto CLEAN_RETURN;
    }

    if (0 == init->total_circular_buf_size)
    {
        cy_svc_log_err(ret_val, "Invalid total_circular_buf_size: %d",
                init->total_circular_buf_size);
        goto CLEAN_RETURN;
    }

    if (0 == init->stage_config_list)
    {
        cy_svc_log_err(ret_val, "Invalid stage_config_list: %d",
                init->stage_config_list);
        goto CLEAN_RETURN;
    }

    if ((0 == (init->stage_config_list & CY_SVC_ENABLE_SOD))
            && (0 == (init->stage_config_list & CY_SVC_ENABLE_LPWWD)))
    {
        cy_svc_log_err(ret_val, "Invalid stage_config_list: %d, SOD & LPPWD disabled",
                init->stage_config_list);
        goto CLEAN_RETURN;
    }

    if ((init->stage_config_list & CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS)
            || (init->stage_config_list & CY_SVC_ENABLE_ASR_STATE_TRANSITIONS))
    {
        uint32_t total_num_samples_in_cbuf = 0;
        float pre_roll_percentage = 0.0;

        /**
         * All the denominator values are already validated for zero,
         * to avoid div by zero error.
         */
        total_num_samples_in_cbuf = init->total_circular_buf_size
                / (FRAME_DATA_SIZE(init->audio_input_type, SAMPLE_SIZE_IN_BYTES,
                        init->sample_rate*1000, init->single_frame_time_ms));

        if (0 == total_num_samples_in_cbuf)
        {
            cy_svc_log_err(ret_val, "Invalid recommended cbuf size");
            goto CLEAN_RETURN;
        }

        pre_roll_percentage =
                (float) (init->pre_roll_frame_count_from_lpwwd_detect_frame * 100.0)
                        / (float) total_num_samples_in_cbuf;

        cy_svc_log_dbg("Total samples space in cbuf: %d, PreRollPer:%.1f%",
                total_num_samples_in_cbuf, pre_roll_percentage);


        if (pre_roll_percentage > MAX_PRE_ROLL_BUFFER_REQ_PERCENTAGE)
        {
            cy_svc_log_err(ret_val, "PreRoll Buffer is greater than %d percent of total buffers",
                    MAX_PRE_ROLL_BUFFER_REQ_PERCENTAGE);
            goto CLEAN_RETURN;
        }

        if(0 != (init->total_circular_buf_size % total_num_samples_in_cbuf))
        {
            cy_svc_log_err(ret_val, "Invalid Cbuf Size, Pls pass cbuf size "
                    "in multiples of singe frame size");
            goto CLEAN_RETURN;
        }
    }

    if(init->stage_config_list & CY_SVC_ENABLE_SOD)
    {
        if (init->sod_onset_detect_max_late_hit_delay_ms > CY_MAX_SOD_HIT_LATE_DELAY_MS)
        {
            cy_svc_log_err(ret_val, "Invalid SOD hit delay:%d",
                    init->sod_onset_detect_max_late_hit_delay_ms);
            goto CLEAN_RETURN;
        }
    }

    if (init->stage_config_list & CY_SVC_ENABLE_LPWWD)
    {
        if (init->is_lpwwd_external == false)
        {
            if ((NULL == init->wake_word_model_binary_buf)
                    || (NULL == init->wake_word_model_meta_buf))
            {
                cy_svc_log_err(ret_val, "Invalid model:%p %p",
                        init->wake_word_model_binary_buf,
                        init->wake_word_model_meta_buf);
                goto CLEAN_RETURN;
            }
        }
        else
        {
            if (NULL == init->lpwwd_external_data_callback)
            {
                cy_svc_log_err(ret_val, "Invalid data cbk:%p",
                        init->lpwwd_external_data_callback);
                goto CLEAN_RETURN;
            }
        }
    }

    ret_val = CY_RSLT_SUCCESS;
    CLEAN_RETURN: return ret_val;
}

static cy_rslt_t svc_lp_system_components_init(svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *init)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    ret_val = svc_lp_hpf_init(lp_instance);
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err(ret_val, "HPF init fail");
        goto CLEAN_RETURN;
    }

    if (init->stage_config_list & CY_SVC_ENABLE_SOD)
    {
        ret_val = svc_lp_sod_init(lp_instance, init);
        if (CY_RSLT_SUCCESS != ret_val)
        {
            cy_svc_log_err(ret_val, "SOD init fail");
            goto CLEAN_RETURN;
        }
    }

    if (init->stage_config_list & CY_SVC_ENABLE_LPWWD)
    {
        ret_val = svc_lp_lpwwd_init(lp_instance, init);
        if (CY_RSLT_SUCCESS != ret_val)
        {
            cy_svc_log_err(ret_val, "lpwwd init fail");
            goto CLEAN_RETURN;
        }
    }
    CLEAN_RETURN: return ret_val;
}

static cy_rslt_t svc_lp_system_components_deinit(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    ret_val = svc_lp_hpf_deinit(lp_instance);
    if (ret_val != CY_RSLT_SUCCESS)
    {
        cy_svc_log_err(ret_val, "HPF Deinit fail");
    }

    if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
    {
        ret_val = svc_lp_sod_deinit(lp_instance);
        if (ret_val != CY_RSLT_SUCCESS)
        {
            cy_svc_log_err(ret_val, "SOD Deinit fail");
        }
    }

    if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD)
    {
        ret_val = svc_lp_lpwwd_deinit(lp_instance);
        if (ret_val != CY_RSLT_SUCCESS)
        {
            cy_svc_log_err(ret_val, "lpwwd Deinit fail");
        }
    }

    /**
     * TODO: returning right ret_val as there is overwrite in the
     * subsequent deinit calls.
     */
    cy_svc_log_info("system component deinit success");

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
cy_rslt_t svc_lp_create_resource_for_instance(
        svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *init)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    unsigned int data_queue_size = 0;

    cy_svc_log_info("Init:SCL:0x%x,Aud:%d,SR:%d,FrTm:%d,CBf:%d,PreRB:%d",
            init->stage_config_list,
            init->audio_input_type,
            init->sample_rate,
            init->single_frame_time_ms,
            init->total_circular_buf_size,
            init->pre_roll_frame_count_from_lpwwd_detect_frame);

    cy_svc_log_info("Init:LPExt:%d,Cbk:[%p,%p],SOD[%d,%d,%d]",
            init->is_lpwwd_external,
            init->event_callback,
            init->lpwwd_external_data_callback,
            init->sod_sensitivity,
            init->sod_onset_gap_setting_ms,
            init->sod_onset_detect_max_late_hit_delay_ms);

    ret_val = svc_lp_validate_init_params(lp_instance, init);

    cy_mem_get_allocated_memory("Start => svc_lp_create_resource_for_instance");

    if (CY_RSLT_SUCCESS != ret_val)
    {
        goto CLEAN_RETURN;
    }

    ret_val = svc_lp_ipc_init(lp_instance);
    if (ret_val != CY_RSLT_SUCCESS)
    {
        cy_svc_log_err(ret_val, "IPC init fail");
        goto CLEAN_RETURN;
    }

    ret_val = svc_lp_create_circular_buf(lp_instance, init);
    if (ret_val != CY_RSLT_SUCCESS)
    {
        cy_svc_log_err(ret_val, "Create circular buffer fail");
        goto CLEAN_RETURN;
    }

    /**
     * Create cmd event queue
     */
    ret_val = cy_rtos_init_queue(&lp_instance->cmd_queue,
    MAX_CMD_Q_SUPPORTED_SIZE, sizeof(cmd_q_msg_t));
    if (ret_val != CY_RSLT_SUCCESS)
    {
        cy_svc_log_err(ret_val, "Create input cmd queue fail");
        goto CLEAN_RETURN;
    }

    /**
     * Create data received event queue
     */
    data_queue_size = init->pre_roll_frame_count_from_lpwwd_detect_frame +
    MAX_DATA_Q_SUPPORTED_EXTRA_FROM_PREROLL_BUFFER;
    /**
     * Worst case assumption is that, The buffered WWD is processed
     * in real time by the HPWWD, hence the data Q needs to be available to
     * hold the incoming packets at least worth of prerollbuffer count.
     * This value can be optimized depends on the HPWWD size performance to
     * avoid any drop of incoming data packets.
     */

    ret_val = cy_rtos_init_queue(&lp_instance->data_queue,
            data_queue_size, sizeof(data_q_msg_t));
    if (ret_val != CY_RSLT_SUCCESS)
    {
        cy_svc_log_err(ret_val, "Create input data queue fail");
        goto CLEAN_RETURN;
    }

    /* Start SVC data process thread */
    ret_val = cy_rtos_create_thread(&lp_instance->thread_instance,
            svc_lp_thread_func,
            SVC_PROCESS_THREAD_NAME, NULL, SVC_THREAD_STACK_SIZE,
            SVC_THREAD_PRIORITY, (cy_thread_arg_t) lp_instance);
    if (ret_val != CY_RSLT_SUCCESS)
    {
        cy_svc_log_err(ret_val, "Unable to create the SVC thread");
        goto CLEAN_RETURN;
    }

    ret_val = svc_lp_system_components_init(lp_instance, init);
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err(ret_val, "system component init fail");
        goto CLEAN_RETURN;
    }

    ret_val = cy_rtos_init_semaphore( &lp_instance->cmdProcSyncSemaphore, 1, 0 );
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err(ret_val, "Failed to initialize cmd sync semaphore ");
        goto CLEAN_RETURN;
    }

    ret_val = cy_rtos_init_semaphore(&lp_instance->cmdProcIPCSendSemaphore, 1, 0);
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err(ret_val, "Failed to initialize ipc sync semaphore ");
        goto CLEAN_RETURN;
    }

    /* Store the entire create instance params. */
    lp_instance->init_params = *init;

#ifdef ENABLE_AUTO_FRAME_DISC
    lp_instance->enable_auto_detect_frame_feed_discontinuity=true;
    lp_instance->auto_frame_feed_discontinuity_timeout_ms = ENABLE_AUTO_FRAME_DISC;
    cy_svc_log_info("Frame Disc timeout:%d",
            lp_instance->auto_frame_feed_discontinuity_timeout_ms);
#endif

#ifdef ENABLE_AUTO_POST_WWD_ROLL
    lp_instance->post_roll_frame_count_from_lpwwd_detect_frame = ENABLE_AUTO_POST_WWD_ROLL;
    cy_svc_log_info("Post WWD roll:%d",
            lp_instance->post_roll_frame_count_from_lpwwd_detect_frame);
#endif

    ret_val = svc_lp_trigger_state(lp_instance, SVC_TRIGGER_INSTANCE_INIT_DONE);

    cy_mem_get_allocated_memory("End => svc_lp_create_resource_for_instance");

    CLEAN_RETURN:

    if (CY_RSLT_SUCCESS != ret_val)
    {
        (void) svc_lp_delete_resource_for_instance(lp_instance);
        cy_mem_get_allocated_memory("End => svc_lp_create_resource_for_instance");
    }
    return ret_val;
}

/**
 * Delete the resources created for the instance.
 *
 * @param[in]  lp_instance             Staged voice control module instance
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_lp_delete_resource_for_instance(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    cy_mem_get_allocated_memory("Start => svc_lp_delete_resource_for_instance");
    if (NULL != lp_instance->thread_instance)
    {
        /* Delete thread */
        lp_instance->quit_thread_instance = true;

        ret_val = cy_rtos_terminate_thread(&lp_instance->thread_instance);
        if (ret_val != CY_RSLT_SUCCESS)
        {
            cy_svc_log_err(ret_val, "Terminate thread fail");

            /**
             * Note: Intentionally continuing to cleanup the other resources on fail,
             * since it is deallocate path. This is applicable for this function
             */
        }

        ret_val = cy_rtos_join_thread(&lp_instance->thread_instance);
        if (ret_val != CY_RSLT_SUCCESS)
        {
            cy_svc_log_err(ret_val, "Join thread fail");
        }
        else
        {
            cy_svc_log_dbg("Thread cleanup success");
        }
        lp_instance->thread_instance = NULL;
    }
    else
    {
        cy_svc_log_dbg("Thread is not yet created");
    }

    ret_val = svc_lp_system_components_deinit(lp_instance);
    if (ret_val != CY_RSLT_SUCCESS)
    {
        cy_svc_log_err(ret_val, "System component deinit fail");
    }
    else
    {
        cy_svc_log_info("System comp deinit success");
    }

    if (NULL != lp_instance->data_queue)
    {
        ret_val = cy_rtos_deinit_queue(&lp_instance->data_queue);
        if (ret_val != CY_RSLT_SUCCESS)
        {
            cy_svc_log_err(ret_val, "Deinit Data event Q fail");
        }
        else
        {
            cy_svc_log_info("Deinit Data Q success");
        }
        lp_instance->data_queue = NULL;
    }
    else
    {
        cy_svc_log_info("Data Q is not yet created");
    }

    if (NULL != lp_instance->cmd_queue)
    {
        ret_val = cy_rtos_deinit_queue(&lp_instance->cmd_queue);
        if (ret_val != CY_RSLT_SUCCESS)
        {
            cy_svc_log_err(ret_val, "Deinit cmd event Q fail");
        }
        else
        {
            cy_svc_log_info("Deinit CMD Q success");
        }
        lp_instance->cmd_queue = NULL;
    }
    else
    {
        cy_svc_log_info("CMD Q is not created yet");
    }

    (void) svc_lp_delete_circular_buf(lp_instance);

    (void) svc_lp_ipc_deinit(lp_instance);

    if(NULL != lp_instance->cmdProcSyncSemaphore)
    {
        cy_rtos_deinit_semaphore(&lp_instance->cmdProcSyncSemaphore);
        lp_instance->cmdProcSyncSemaphore = NULL;
        cy_svc_log_info("Deinit Sem success");
    }
    else
    {
        cy_svc_log_info("Sem not yet created");
    }

    if(NULL != lp_instance->cmdProcIPCSendSemaphore)
    {
        cy_rtos_deinit_semaphore(&lp_instance->cmdProcIPCSendSemaphore);
        lp_instance->cmdProcIPCSendSemaphore = NULL;
        cy_svc_log_info("Deinit Sem success");
    }
    else
    {
        cy_svc_log_info("Sem not yet created");
    }

    cy_mem_get_allocated_memory("End => svc_lp_delete_resource_for_instance");

    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err(ret_val, "Deinit SVC resource fail");
    }
    return ret_val;
}

#endif
