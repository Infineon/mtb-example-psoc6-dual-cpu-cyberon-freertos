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
 * @file staged_voice_control_lp_data.c
 *
 */
#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_process_data.h"
#include "staged_voice_control_lp_lpwwd.h"

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

cy_rslt_t svc_lp_feed_data_to_cbuf_and_notify_task(svc_lp_instance_t *lp_instance,
                    CY_SVC_DATA_T *input_data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    uint8_t *circular_buffer_wr_pointer = NULL;
    data_q_msg_t q_msg = { 0 };
    bool b_successfully_posted = false;

    ret_val = svc_lp_get_circular_buf_rd_wr_pointer(lp_instance,
            &circular_buffer_wr_pointer, true);
    if ((CY_RSLT_SUCCESS == ret_val) && (NULL != circular_buffer_wr_pointer))
    {
        ret_val = svc_lp_hpf_process(lp_instance, (uint8_t*) input_data,
                circular_buffer_wr_pointer);
        if (CY_RSLT_SUCCESS != ret_val)
        {
            cy_svc_log_err_on_no_isr(ret_val, "HPF process fail");
            goto CLEAN_RETURN;
        }
        else
        {
            q_msg.cmd = SVC_LP_CMD_ID_DATA_RECEIVED;
            q_msg.data_crc_check = svc_lp_create_crc_for_buffer(lp_instance,
                    (char*) input_data);
            q_msg.cbuf_pointer = (char *)circular_buffer_wr_pointer;

            /**
             * Timeout is 0, fail in the put Q gives clue on data
             * Q size of some processing power in some thread is problem.
             * or some thread is not releasing the CPU
             */
            ret_val = cy_rtos_put_queue(&lp_instance->data_queue, &q_msg, 0,
                    is_in_isr());
            if (CY_RSLT_SUCCESS != ret_val)
            {
                cy_svc_log_err_on_no_isr(ret_val, "data input Q put fail");
                goto CLEAN_RETURN;
            }
            else
            {
                b_successfully_posted = true;
            }
        }
    }

    CLEAN_RETURN:

    if (NULL != circular_buffer_wr_pointer)
    {
        if (true == b_successfully_posted)
        {
            (void) svc_lp_circular_buffer_update_rd_wr_offset(lp_instance,
                    true);
        }
        else
        {
            /**
             * Skip the data written to the buffer and update the fail counter
             */
            lp_instance->stats.svc_lp_feed_post_fail_counter_dbg++;
        }
    }

    return ret_val;
}

cy_rslt_t svc_lp_process_data(svc_lp_instance_t *lp_instance, uint8_t *data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_SOD)
    {
        ret_val = svc_lp_sod_process(lp_instance, data);
        if (CY_RSLT_SUCCESS != ret_val)
        {
            cy_svc_log_err(ret_val, "Process SOD fail");
            goto CLEAN_RETURN;
        }
    }

#ifdef ENABLE_SVC_LOW_LATENCY_PROFILE

    if (lp_instance->init_params.stage_config_list & CY_SVC_ENABLE_LPWWD)
    {
        ret_val = svc_lp_process_data_lpwwd(lp_instance, data);
        if (CY_RSLT_SUCCESS != ret_val)
        {
            cy_svc_log_err(ret_val, "Process lpwwd fail");
            goto CLEAN_RETURN;
        }

        if (CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION
                == lp_instance->current_stage)
        {
            lp_instance->post_lpwwd_received_frame_count++;

            if (0 != lp_instance->post_wwd_frame_count_req_by_hp)
            {
                svc_lp_start_circular_buf_update_on_transition_to_hp(
                        lp_instance, NULL,
                        SVC_TRIGGER_SEND_POST_WWD_HPWWD_DET_IN_PROGRESS);
            }
            else if (lp_instance->post_hpwwd_pending_frame_counter_to_hp > 0)
            {
                lp_instance->stats.hpwwd_feed_counter_dbg++;

                svc_lp_send_ipc_event_circular_buffer_update(lp_instance, data,
                        lp_instance->circular_shared_buffer->frame_size_in_bytes,
                        0, 0);

                svc_lp_get_circular_buf_post_shift_buffer_from_any_address(
                        lp_instance, data, 1,
                        &lp_instance->hpwwd_trigger_data_final_address);

                lp_instance->post_hpwwd_pending_frame_counter_to_hp--;

                if(0 == lp_instance->post_hpwwd_pending_frame_counter_to_hp)
                {
                    cy_svc_log_info("Sent post wwd frames");
                }
            }
        }
        else if (CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT
                == lp_instance->current_stage)
        {
            svc_lp_send_ipc_event_circular_buffer_update(lp_instance, data,
                    lp_instance->circular_shared_buffer->frame_size_in_bytes, 0,
                    0);
        }
        else if ((CY_SVC_STAGE_ASR_PROCESSING_QUERY_DETECTED
                == lp_instance->current_stage)
                && (true
                        == lp_instance->stream_requested_on_asr_processing_query_state))
        {
            svc_lp_send_ipc_event_circular_buffer_update(lp_instance, data,
                    lp_instance->circular_shared_buffer->frame_size_in_bytes, 0,
                    0);
        }
    }
#endif

    CLEAN_RETURN:
    return ret_val;
}

#endif
