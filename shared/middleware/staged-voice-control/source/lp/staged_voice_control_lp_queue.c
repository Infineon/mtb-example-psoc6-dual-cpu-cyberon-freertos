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
 * @file staged_voice_control_lp_queue.c
 *
 */
#ifdef ENABLE_SVC_LP_MW
#include "staged_voice_control_lp_private.h"
#include "staged_voice_control_lp_aad.h"
#include "staged_voice_control_lp_queue.h"
#include "staged_voice_control_lp_process_data.h"

/*******************************************************************************
 *                              Macros
 ******************************************************************************/

/*
 * Data Q timeout is infinite. SVC task will always wait for data in data Q.
 * */
#define DATA_QUEUE_GET_DATA_WAIT_TIMEOUT_MS (CY_RTOS_NEVER_TIMEOUT)

#define CMD_QUEUE_GET_DATA_WAIT_TIMEOUT_MS (0)

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

cy_rslt_t svc_lp_post_to_wakeup_msg_to_data_q(
        svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    data_q_msg_t    q_msg = {0};

    q_msg.cmd = SVC_LP_CMD_ID_SET_STATE_FROM_HP;
    ret_val = cy_rtos_put_queue(&lp_instance->data_queue, &q_msg,
    CY_RTOS_NEVER_TIMEOUT, is_in_isr());
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err_on_no_isr(ret_val, "data dummy input Q put fail");

        /*
         * TODO: Adding retry to post dummy msg in case of fail.
         */
    }
    else
    {
        //  cy_svc_log_info("data dummy input Q post success");
     }

    return ret_val;
}


cy_rslt_t svc_lp_post_to_cmd_q_to_set_state_from_hp(
        svc_lp_instance_t *lp_instance,
        cy_svc_set_state_t set_state,
        void *set_state_info)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cmd_q_msg_t q_msg = { 0 };

    q_msg.cmd = SVC_LP_CMD_ID_SET_STATE_FROM_HP;
    q_msg.set_state = set_state;
    memcpy(q_msg.payload_internal, set_state_info, sizeof(q_msg.payload_internal));

    ret_val = cy_rtos_put_queue(&lp_instance->cmd_queue, &q_msg,
    CY_RTOS_NEVER_TIMEOUT, is_in_isr());
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err(ret_val, "cmd input Q put fail");
        goto CLEAN_RETURN;
    }

    (void) svc_lp_post_to_wakeup_msg_to_data_q(lp_instance);

    CLEAN_RETURN: return ret_val;
}


cy_rslt_t svc_lp_post_to_cmd_q_to_set_stage_from_lp_app(
        svc_lp_instance_t *lp_instance,
        cy_svc_stage_t set_stage)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cmd_q_msg_t q_msg = { 0 };

    q_msg.cmd = SVC_LP_CMD_ID_SET_STAGE_FROM_LP_APP;
    q_msg.set_state = set_stage;

    ret_val = cy_rtos_put_queue(&lp_instance->cmd_queue, &q_msg,
    CY_RTOS_NEVER_TIMEOUT, is_in_isr());
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err(ret_val, "cmd input Q put fail");
        goto CLEAN_RETURN;
    }

    (void) svc_lp_post_to_wakeup_msg_to_data_q(lp_instance);

    CLEAN_RETURN: return ret_val;
}



cy_rslt_t svc_lp_get_from_msg_from_cmd_input_queue_and_process(
        svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cmd_q_msg_t    q_msg = {0};

    ret_val = cy_rtos_get_queue(&lp_instance->cmd_queue, &q_msg,
            CMD_QUEUE_GET_DATA_WAIT_TIMEOUT_MS, false);
    if (CY_RSLT_SUCCESS == ret_val)
    {
        if(SVC_LP_CMD_ID_SET_STATE_FROM_HP ==  q_msg.cmd)
        {
//            cy_svc_log_info("Received CMD Q msg: CMD %d,m State:%d", q_msg.cmd,
//                    q_msg.set_state);

            ret_val = svc_lp_trigger_state_from_hp_set_state(lp_instance,
                    q_msg.set_state,
                    q_msg.payload_internal);

#ifdef SIMULATE_SOD_IPC_TRIGGER_TEST
            ret_val = CY_RSLT_SUCCESS;
#endif

            (void) svc_lp_notify_svc_hp_for_state_execution_result(lp_instance,
                    ret_val);

            if (CY_RSLT_SUCCESS != ret_val)
            {
                cy_svc_log_err(ret_val, "set state from hp fail");
                goto CLEAN_RETURN;
            }
        }
        else if (SVC_LP_CMD_ID_SET_STAGE_FROM_LP_APP == q_msg.cmd)
        {
//            cy_svc_log_info("Received.. CMD Q msg: CMD %d,m Stage:%d", q_msg.cmd,
//                    q_msg.set_stage);

            ret_val = svc_lp_trigger_stage_from_lp_app_set_stage(lp_instance,
                    q_msg.set_stage);

            lp_instance->lp_app_set_stage_process_result = ret_val;

            (void) cy_rtos_set_semaphore(&lp_instance->cmdProcSyncSemaphore,
                    is_in_isr());

//            cy_svc_log_info("Signal the cmd completion");
        }
    }

    CLEAN_RETURN:
    return ret_val;
}

cy_rslt_t svc_lp_get_data_from_data_queue_and_process(
        svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    data_q_msg_t q_msg = { 0 };
    uint8_t *circular_buffer_rd_pointer = NULL;

    ret_val = cy_rtos_get_queue(&lp_instance->data_queue, &q_msg,
    DATA_QUEUE_GET_DATA_WAIT_TIMEOUT_MS, false);
    if (CY_RSLT_SUCCESS == ret_val)
    {
        (void) svc_lp_get_from_msg_from_cmd_input_queue_and_process(
                lp_instance);

        if (SVC_LP_CMD_ID_DATA_RECEIVED == q_msg.cmd)
        {
            (void) svc_lp_auto_aad_detection_check(lp_instance);

            lp_instance->stats.frame_counter_received_after_last_aad_dbg++;

            if (lp_instance->stats.svc_lp_feed_post_fail_counter_track_dbg
                    != lp_instance->stats.svc_lp_feed_post_fail_counter_dbg)
            {
                lp_instance->stats.svc_lp_feed_post_fail_counter_dbg++;
                lp_instance->stats.svc_lp_feed_post_fail_counter_track_dbg =
                        lp_instance->stats.svc_lp_feed_post_fail_counter_dbg;

                cy_svc_log_err(ret_val, "SVC feed drop counter:%d, err:0x%x",
                        lp_instance->stats.svc_lp_feed_post_fail_counter_dbg,
                        lp_instance->stats.svc_lp_feed_post_fail_reason_dbg);

                lp_instance->stats.svc_lp_feed_post_fail_reason_dbg = 0;
            }

            ret_val = svc_lp_get_circular_buf_rd_wr_pointer(lp_instance,
                    &circular_buffer_rd_pointer, false);
            if ((CY_RSLT_SUCCESS == ret_val)
                    && (NULL != circular_buffer_rd_pointer))
            {
                ret_val = svc_lp_verify_crc_of_the_buffer(lp_instance,
                        q_msg.cbuf_pointer, q_msg.data_crc_check,
                        (char*) circular_buffer_rd_pointer);
                if (CY_RSLT_SUCCESS != ret_val)
                {
                    cy_svc_log_err(ret_val, "CRC fail-Ignore Buffer");
                    goto CLEAN_RETURN;
                }

#ifdef ENABLE_USB_DBG_OUTPUT
                usb_send_out_dbg_put(2, (short*) circular_buffer_rd_pointer);
#endif
                ret_val = svc_lp_process_data(lp_instance,
                        circular_buffer_rd_pointer);

                lp_instance->last_processed_frame_start_address =
                        (unsigned char*) circular_buffer_rd_pointer;

                if (CY_RSLT_SUCCESS != ret_val)
                {
                    cy_svc_log_err(ret_val, "Process data fail");
                    goto CLEAN_RETURN;
                }
            }
        }
        else
        {
            /**
             * Drop the MSG.
             *
             * Possible other message id could be
             * SVC_LP_CMD_ID_SET_STATE_FROM_HP. This is used to wake up from
             * Data Q and process the command Q.
             */
        }
    }
    else
    {
        cy_svc_log_err(ret_val, "Invalid Get Q return");
    }

    CLEAN_RETURN:

    /**
     * Process buffer completed, Hence updating the read offset
     */
    if (NULL != circular_buffer_rd_pointer)
    {
        (void) svc_lp_circular_buffer_update_rd_wr_offset(lp_instance, false);
    }

    return ret_val;
}

#endif
