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
 * @file staged_voice_control_lp_ipc.c
 *
 */

#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_ipc.h"
#include "staged_voice_control_lp_ipc.h"
#include "staged_voice_control_lp_queue.h"

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

#ifdef ENABLE_PSOC6_IPC_COMMUNICATION
/**
 * Adding global variable to avoid the memory in stack as this API will be
 * called repeatedly from single task.
 */
ipc_msg_t ipc_msg_send_msg = { .client_id = IPC_CM0_TO_CM4_CLIENT_ID,
        .cpu_status = 0, .intr_mask = USER_IPC_PIPE_INTR_MASK, .cmd =
                IPC_CMD_STATUS, .value = 0 };

#define IPC_BUSY_RETRY_COUNT (2)
#define IPC_BUSY_RETRY_SLEEP_INTERNAL_MS (5)

void svc_lp_ipc_send_done_callback(void)
{
    svc_lp_instance_t *lp_instance = svc_lp_get_instance();

    (void) cy_rtos_set_semaphore(&lp_instance->cmdProcIPCSendSemaphore,
            is_in_isr());
    return;
}

void svc_lp_ipc_wait_for_send_complete(void)
{
    svc_lp_instance_t *lp_instance = svc_lp_get_instance();

    (void) cy_rtos_get_semaphore(&lp_instance->cmdProcIPCSendSemaphore,
    CY_RTOS_NEVER_TIMEOUT, is_in_isr());

    return;
}


cy_rslt_t svc_lp_send_ipc_msg_to_hp(
        svc_lp_instance_t *lp_instance,
        uint8_t* data)
{
    cy_en_ipc_pipe_status_t ipc_status = CY_IPC_PIPE_SUCCESS;
    uint32_t retry = IPC_BUSY_RETRY_COUNT;
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    // uint8_t cmd_id = 0;

    memcpy(&ipc_msg_send_msg.msg_pay[0], data,
            sizeof(ipc_msg_send_msg.msg_pay));

    // cmd_id = ipc_msg_send_msg.msg_pay[0];

    do
    {    
        ipc_status = Cy_IPC_Pipe_SendMessage(USER_IPC_PIPE_EP_ADDR_CM4,
        USER_IPC_PIPE_EP_ADDR_CM0, (void *) &ipc_msg_send_msg, svc_lp_ipc_send_done_callback);

        if (CY_IPC_PIPE_SUCCESS != ipc_status)
        {
            retry--;
#if 0
            cy_rtos_delay_milliseconds(IPC_BUSY_RETRY_SLEEP_INTERNAL_MS);
#else
            ;
#endif
        }

        /**
         * Added, Retry to handle the error. 0x8a0207 error in IPC send.
         * Retry in case of busy is working and tested.
         */
    } while ((CY_IPC_PIPE_SUCCESS != ipc_status) && (retry > 0));

    if (CY_IPC_PIPE_SUCCESS != ipc_status)
    {
        cy_svc_log_err(ipc_status, "[IPC] LP SEND msg[%d] fail", ipc_msg_send_msg.msg_pay[0]);
    }
    else
    {
        svc_lp_ipc_wait_for_send_complete();
        cy_svc_log_info("[IPC] LP SEND msg[%d] success", ipc_msg_send_msg.msg_pay[0]);
        ret_val = CY_RSLT_SUCCESS;
    }

    return ret_val;
}
#elif ENABLE_SVC_IPC_STUB_SINGLE_CORE

ipc_msg_t ipc_msg_send_msg_to_hp = { 0 };

cy_rslt_t svc_lp_send_ipc_msg_to_hp(
        svc_lp_instance_t *lp_instance,
        uint8_t* data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    memcpy(&ipc_msg_send_msg_to_hp.msg_pay[0], data,
            sizeof(ipc_msg_send_msg_to_hp.msg_pay));

    ret_val = ipc_stub_post_msg_to_cm4(&ipc_msg_send_msg_to_hp);
    return ret_val;
}
#else
cy_rslt_t svc_lp_send_ipc_msg_to_hp(
        svc_lp_instance_t *lp_instance,
        uint8_t* data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cy_svc_log_info("[IPC] TODO");
    return ret_val;
}
#endif

cy_rslt_t svc_lp_notify_svc_hp_for_state_execution_result(
        svc_lp_instance_t *lp_instance,
        cy_rslt_t set_state_execution_ret_val)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    ipc_cmd_lp_to_hp_set_state_result_t ipc_set_state_rslt = {0};

    ipc_set_state_rslt.cmd_id = IPC_CMD_ID_LP_TO_HP_SET_STATE_RESULT;
    ipc_set_state_rslt.set_state_result = set_state_execution_ret_val;
    ipc_set_state_rslt.cur_stage = lp_instance->current_stage;

    ret_val = svc_lp_send_ipc_msg_to_hp(lp_instance,
            (uint8_t*) &ipc_set_state_rslt);
    if (CY_RSLT_SUCCESS == ret_val)
    {
        cy_svc_log_info("[IPC] cmdid:0x%x ,result:0x%x, stage:%d",
                ipc_set_state_rslt.cmd_id, ipc_set_state_rslt.set_state_result,
                ipc_set_state_rslt.cur_stage);
    }
    return ret_val;
}

cy_rslt_t svc_lp_send_ipc_config_update(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    ipc_cmd_lp_to_hp_cfg_update_t ipc_config_update = {0};

    ipc_config_update.cmd_id = IPC_CMD_ID_LP_TO_HP_CONFIG_UPDATE;

    ipc_config_update.stage_config_list =
            lp_instance->init_params.stage_config_list;

    ret_val = svc_lp_send_ipc_msg_to_hp(lp_instance,
            (uint8_t *) &ipc_config_update);
    if (CY_RSLT_SUCCESS == ret_val)
    {
        lp_instance->config_update_to_hp_done = true;
        cy_svc_log_info("[IPC] config update: 0x%x",
                ipc_config_update.stage_config_list);
    }

    return ret_val;
}

cy_rslt_t svc_lp_send_ipc_event_circular_buffer_update(
        svc_lp_instance_t *lp_instance,
        uint8_t *data_pointer,
        uint32_t data_len,
        uint16_t insuff_preroll_frame_count,
        cy_svc_buffer_info_t buffer_info)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    ipc_cmd_lp_to_hp_data_t ipc_circular_buffer = { 0 };

    if (false == lp_instance->config_update_to_hp_done)
    {
//        cy_svc_log_info("[IPC] performing config update");

        (void) svc_lp_send_ipc_config_update(lp_instance);
    }

    ipc_circular_buffer.cmd_id = IPC_CMD_ID_LP_TO_HP_CIRCULAR_BUF_UPDATE;
    ipc_circular_buffer.current_stage = lp_instance->current_stage;
    ipc_circular_buffer.data_pointer = data_pointer;
    ipc_circular_buffer.frame_count = (uint16_t) (data_len
            / lp_instance->circular_shared_buffer->frame_size_in_bytes);
    ipc_circular_buffer.insuff_preroll_frame_count = insuff_preroll_frame_count;

    /**
     * TODO: Converting uint32_t to uint16_t knowingly. To be changed
     * once the IPC size increased.
     */
    ipc_circular_buffer.data_info_bitmask = (uint16_t) buffer_info;

    ret_val = svc_lp_send_ipc_msg_to_hp(lp_instance,
            (uint8_t *) &ipc_circular_buffer);

    return ret_val;
}

void svc_ipc_lp_msg_receive_callback(uint32_t *msg)
{
    ipc_cmd_hp_to_lp_set_state_t *ipc_set_state = NULL;
    uint8_t cmd_id = 0;
    uint8_t *pay_load = NULL;

    if (msg != NULL)
    {
#ifdef ENABLE_PSOC6_IPC_COMMUNICATION
        cmd_id = ((ipc_msg_t *)msg)->msg_pay[0];
        pay_load = (uint8_t *)((ipc_msg_t *)msg)->msg_pay;
#elif ENABLE_SVC_IPC_STUB_SINGLE_CORE
        cmd_id = ((ipc_msg_t *)msg)->msg_pay[0];
        pay_load = (uint8_t *)((ipc_msg_t *)msg)->msg_pay;
//        cy_svc_log_dbg("Received msg on LP");
#else
        cy_svc_log_dbg("TODO: for Explorer");
#endif

        if (IPC_CMD_ID_HP_TO_LP_SET_STATE == cmd_id)
        {
            ipc_set_state = (ipc_cmd_hp_to_lp_set_state_t *)pay_load;

            (void) svc_lp_post_to_cmd_q_to_set_state_from_hp(svc_lp_get_instance(),
                    ipc_set_state->set_state,
                    ipc_set_state->payload_internal);
        }
    }
}


cy_rslt_t svc_lp_ipc_init(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

#ifdef ENABLE_PSOC6_IPC_COMMUNICATION
    cy_en_ipc_pipe_status_t ipc_status = CY_IPC_PIPE_SUCCESS;
    ipc_status = Cy_IPC_Pipe_RegisterCallback(USER_IPC_PIPE_EP_ADDR,
            svc_ipc_lp_msg_receive_callback,
            IPC_CM4_TO_CM0_CLIENT_ID);
    if (CY_IPC_PIPE_SUCCESS != ipc_status)
    {
        cy_svc_log_err(ipc_status, "[IPC] register fail");
    }
    else
    {
        ret_val = CY_RSLT_SUCCESS;
    }
#elif ENABLE_SVC_IPC_STUB_SINGLE_CORE
    cy_svc_log_dbg("Enabled IPC stub on same core");
    ret_val = CY_RSLT_SUCCESS;
#else

    cy_svc_log_dbg("TODO: IPC init");
#endif

    return ret_val;
}

cy_rslt_t svc_lp_ipc_deinit(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

#ifdef ENABLE_PSOC6_IPC_COMMUNICATION
    cy_en_ipc_pipe_status_t ipc_status = CY_IPC_PIPE_SUCCESS;

    ipc_status = Cy_IPC_Pipe_RegisterCallback(USER_IPC_PIPE_EP_ADDR, NULL,
            IPC_CM4_TO_CM0_CLIENT_ID);
    if (CY_IPC_PIPE_SUCCESS != ipc_status)
    {
        cy_svc_log_err(ipc_status, "[IPC] deregister fail");
    }
    else
    {
        cy_svc_log_info("IPC deinit success");
        ret_val = CY_RSLT_SUCCESS;
    }
#elif ENABLE_SVC_IPC_STUB_SINGLE_CORE
    cy_svc_log_dbg("Deinit IPC stub");
    ret_val = CY_RSLT_SUCCESS;
#else
    cy_svc_log_dbg("TODO IPC deinit");
    ret_val = CY_RSLT_SUCCESS;
#endif
    return ret_val;
}

/** Checks to see if code is currently executing within an interrupt context.
 *
 * @return Boolean indicating whether this was executed from an interrupt context.
 */
bool is_in_isr()
#if 1
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}
#else
{
    #if defined(COMPONENT_CR4) // Can work for any Cortex-A & Cortex-R
    uint32_t mode = __get_mode();
    return (mode == 0x11U /*FIQ*/) || (mode == 0x12U /*IRQ*/) || (mode == 0x13U /*SVC*/) ||
           (mode == 0x17U /*ABT*/) || (mode == 0x1BU /*UND*/);
    #else // Cortex-M
    return (__get_IPSR() != 0);
    #endif
}
#endif

#endif
