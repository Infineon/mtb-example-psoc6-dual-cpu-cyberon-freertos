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
 * @file staged_voice_control_hp_ipc.c
 *
 */

#ifdef ENABLE_SVC_HP_MW
#include "staged_voice_control_ipc.h"
#include "staged_voice_control_hp_ipc.h"
#include "staged_voice_control_hp_thread.h"

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

svc_hp_instance_t global_hp_instance = { 0 };

/**
 * Saves the config information from SVC LP.
 *
 * @param[in]  config_info      config information
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
static cy_rslt_t svc_hp_set_state_result_from_lp(
        svc_hp_instance_t *hp_instance,
        ipc_cmd_lp_to_hp_set_state_result_t *set_state_result)
{
    cy_rslt_t result = CY_RSLT_SVC_GENERIC_ERROR;

    hp_instance->lp_app_set_state_process_result =
            set_state_result->set_state_result;

    if(set_state_result->cur_stage != hp_instance->current_stage)
    {
        hp_instance->current_stage = set_state_result->cur_stage;
    }
    (void) cy_rtos_set_semaphore(&hp_instance->cmdProcSyncSVCLPSemaphore,
            is_in_isr());

    result = CY_RSLT_SUCCESS;
//    cy_svc_log_err_on_no_isr(result,
//            "Config update: 0x%x",hp_instance->stage_config_list_from_lp);
    return result;
}


/**
 * Saves the config information from SVC LP.
 *
 * @param[in]  config_info      config information
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
static cy_rslt_t svc_hp_save_frame_information_from_svc_lpk(
        svc_hp_instance_t *hp_instance,
        ipc_cmd_lp_to_hp_cfg_update_t *config_info)
{
    cy_rslt_t result = CY_RSLT_SVC_GENERIC_ERROR;

    hp_instance->stage_config_list_from_lp = config_info->stage_config_list;

    result = CY_RSLT_SUCCESS;
//    cy_svc_log_err_on_no_isr(result,
//            "Config update: 0x%x",hp_instance->stage_config_list_from_lp);
    return result;
}


#ifdef ENABLE_PSOC6_IPC_COMMUNICATION

#define IPC_BUSY_RETRY_COUNT (2)
#define IPC_BUSY_RETRY_SLEEP_INTERNAL_MS (5)

void svc_hp_ipc_send_done_callback(void)
{
    svc_hp_instance_t *hp_instance = svc_hp_get_instance();

    (void) cy_rtos_set_semaphore(&hp_instance->cmdProcIPCSendSemaphore,
            is_in_isr());
    return;
}

void svc_hp_ipc_wait_for_send_complete(void)
{
    svc_hp_instance_t *hp_instance = svc_hp_get_instance();

    (void) cy_rtos_get_semaphore(&hp_instance->cmdProcIPCSendSemaphore,
    CY_RTOS_NEVER_TIMEOUT, is_in_isr());

    return;
}

/**
 * Generic utility to send IPC message through IPC
 *
 * @param[in]  data      data to be sent through IPC
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_hp_ipc_send_msg_to_lp(uint8_t *data)
{
    cy_en_ipc_pipe_status_t ipc_status = CY_IPC_PIPE_SUCCESS;
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
//    uint8_t cmd_id = 0;
    ipc_msg_t ipc_msg_send = {
        .client_id  = IPC_CM4_TO_CM0_CLIENT_ID,
        .cpu_status = 0,
        .intr_mask  = USER_IPC_PIPE_INTR_MASK,
    };
    unsigned int retry = IPC_BUSY_RETRY_COUNT;

    memcpy(ipc_msg_send.msg_pay, data, sizeof(ipc_msg_send.msg_pay));
//    cmd_id = ipc_msg_send.msg_pay[0];

    do
    {
        ipc_status = Cy_IPC_Pipe_SendMessage(USER_IPC_PIPE_EP_ADDR_CM0,
        USER_IPC_PIPE_EP_ADDR_CM4, (void*) &ipc_msg_send,
                svc_hp_ipc_send_done_callback);

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

#if 0

#else
    if(CY_IPC_PIPE_SUCCESS == ipc_status)
    {
        svc_hp_ipc_wait_for_send_complete();
    }
#endif
    if (CY_IPC_PIPE_SUCCESS != ipc_status)
    {
        ret_val = CY_RSLT_SVC_IPC_SEND_FAIL;
       cy_svc_log_err(ipc_status, "[IPC] Send MSG [%d] fail", ipc_msg_send.msg_pay[0]);
    }
    else
    {
        ret_val = CY_RSLT_SUCCESS;
       cy_svc_log_info("[IPC] Send msg [%d] success", ipc_msg_send.msg_pay[0]);
    }
    return ret_val;
}
#elif ENABLE_SVC_IPC_STUB_SINGLE_CORE
ipc_msg_t ipc_msg_send_msg_to_lp = { 0 };

cy_rslt_t svc_hp_ipc_send_msg_to_lp(
        uint8_t* data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    memcpy(&ipc_msg_send_msg_to_lp.msg_pay[0], data,
            sizeof(ipc_msg_send_msg_to_lp.msg_pay));

    ret_val = ipc_stub_post_msg_to_cm0(&ipc_msg_send_msg_to_lp);
    return ret_val;
}

#else
cy_rslt_t svc_hp_ipc_send_msg_to_lp(uint8_t *data)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cy_svc_log_info("TODO for Explorer");
    return ret_val;
}
#endif

/**
 * Sends the set state command to SVC LP through IPC
 *
 * @param[in]  set_state      state to set
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_hp_ipc_send_command_set_state(
        svc_hp_instance_t *hp_instance,
        cy_svc_set_state_t set_state,
        void *state_info)
{
    ipc_cmd_hp_to_lp_set_state_t ipc_set_state = { 0 };

    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    ipc_set_state.cmd_id = IPC_CMD_ID_HP_TO_LP_SET_STATE;
    ipc_set_state.set_state = set_state;

    if (NULL != state_info)
    {
        memcpy(ipc_set_state.payload_internal, state_info,
                sizeof(ipc_set_state.payload_internal));
    }

    ret_val = svc_hp_ipc_send_msg_to_lp((uint8_t*) &ipc_set_state);
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_svc_log_err(ret_val, "send msg to svc lp fail");
        goto CLEAN_RETURN;
    }

    (void) cy_rtos_get_semaphore(&hp_instance->cmdProcSyncSVCLPSemaphore,
    CY_RTOS_NEVER_TIMEOUT, is_in_isr());

    /**
     * The result is a volatile variable and it will be updated by the SVC
     * task after processing the set state request.
     */
    if (CY_RSLT_SUCCESS != hp_instance->lp_app_set_state_process_result)
    {
        cy_svc_log_err(hp_instance->lp_app_set_state_process_result,
                "set state fail");
    }
    else
    {
        cy_svc_log_info("set state success in core svc-lp");
    }

    ret_val = hp_instance->lp_app_set_state_process_result;

    CLEAN_RETURN: return ret_val;
}


/**
 * Receives the msg callback from IPC
 *
 * @param[in]  msg      Pointer to message. In PSoC6, the length of the
 * message is assume to be of structure size ipc_msg_t. This structure
 * needs to be defined for Explore IPC (recommeded to have size as 12 bytes)
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
void svc_hp_ipc_msg_receive_callback(uint32_t *msg)
{
    ipc_cmd_lp_to_hp_data_t *ipc_circular = NULL;
    ipc_cmd_lp_to_hp_cfg_update_t *config_info = NULL;
    ipc_cmd_lp_to_hp_set_state_result_t *set_state_rslt = NULL;
    svc_hp_instance_t *hp_instance = NULL;
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
#else
        cy_svc_log_dbg("TODO: for Explorer");
#endif
        hp_instance = svc_hp_get_instance();

        if (IPC_CMD_ID_LP_TO_HP_CIRCULAR_BUF_UPDATE == cmd_id)
        {
            ipc_circular = (ipc_cmd_lp_to_hp_data_t *)pay_load;

            hp_instance->current_stage = ipc_circular->current_stage;

#ifdef ENABLE_TASK_FOR_SVC_HP

            if (0 != ipc_circular->insuff_preroll_frame_count)
            {
                (void) svc_hp_push_to_data_queue_from_ipc_cbk(
                        svc_hp_get_instance(),
                        (CY_SVC_DATA_T*) NULL,
                        (uint32_t) ipc_circular->insuff_preroll_frame_count,
                        (cy_svc_buffer_info_t) ipc_circular->data_info_bitmask |
                        CY_SVC_BUF_INFO_PREROLL_INSUFFICIENT_BUF);

                (void) svc_hp_push_to_data_queue_from_ipc_cbk(
                        svc_hp_get_instance(),
                        (CY_SVC_DATA_T*) ipc_circular->data_pointer,
                        (uint32_t) ipc_circular->frame_count,0);
            }
            else
            {
                (void) svc_hp_push_to_data_queue_from_ipc_cbk(
                        svc_hp_get_instance(),
                        (CY_SVC_DATA_T*) ipc_circular->data_pointer,
                        (uint32_t) ipc_circular->frame_count,
                        (cy_svc_buffer_info_t) ipc_circular->data_info_bitmask);
            }


#else
            (void) hp_instance->init_params.data_callback(
                    (CY_SVC_DATA_T *)ipc_circular->data_pointer,
                    ipc_circular->frame_count,
                    (cy_svc_buffer_info_t) ipc_circular->data_info_bitmask,
                    hp_instance->init_params.callback_user_arg);

#endif

        }
        else if (IPC_CMD_ID_LP_TO_HP_SET_STATE_RESULT == cmd_id)
        {
            set_state_rslt = (ipc_cmd_lp_to_hp_set_state_result_t *)pay_load;
            (void) svc_hp_set_state_result_from_lp(
                    svc_hp_get_instance(),
                    set_state_rslt);
        }
        else if (IPC_CMD_ID_LP_TO_HP_CONFIG_UPDATE == cmd_id)
        {
            config_info = (ipc_cmd_lp_to_hp_cfg_update_t *)pay_load;
            (void) svc_hp_save_frame_information_from_svc_lpk(
                    svc_hp_get_instance(),
                    config_info);
        }
#ifdef ENABLE_APP_SIMULATION_FOR_EVENTS
        else if ((IPC_CMD_ID_SIMULATION_START <= cmd_id) &&
                (IPC_CMD_ID_SIMULATION_END >= cmd_id))
        {
            extern cy_rslt_t cy_svc_hp_app_simulation_app_callback_t(
                    uint8_t cmd_id,
                    uint8_t *data);

            (void) cy_svc_hp_app_simulation_app_callback_t(cmd_id,
                    pay_load);
        }
#endif
    }
}

/**
 * Initializes the IPC
 *
 * @param[in]  hp_instance      HP instance
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_hp_ipc_init(svc_hp_instance_t *hp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

#ifdef ENABLE_PSOC6_IPC_COMMUNICATION
    cy_en_ipc_pipe_status_t ipc_status = CY_IPC_PIPE_SUCCESS;

    ipc_status = Cy_IPC_Pipe_RegisterCallback(USER_IPC_PIPE_EP_ADDR,
            svc_hp_ipc_msg_receive_callback,
            IPC_CM0_TO_CM4_CLIENT_ID);
    if (CY_IPC_PIPE_SUCCESS != ipc_status)
    {
        cy_svc_log_err(ipc_status, "[IPC] register fail");
    }
    else
    {
        ret_val = CY_RSLT_SUCCESS;
    }
#elif ENABLE_SVC_IPC_STUB_SINGLE_CORE
    cy_svc_log_dbg("IPC stub on same core");
    ret_val = CY_RSLT_SUCCESS;
#else
    cy_svc_log_dbg("TODO: IPC init");
    ret_val = CY_RSLT_SUCCESS;
#endif

    return ret_val;
}

/**
 * Deinitializes the IPC
 *
 * @param[in]  hp_instance      HP instance
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_hp_ipc_deinit(svc_hp_instance_t *hp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

#ifdef ENABLE_PSOC6_IPC_COMMUNICATION
    cy_en_ipc_pipe_status_t ipc_status = CY_IPC_PIPE_SUCCESS;

    ipc_status = Cy_IPC_Pipe_RegisterCallback(USER_IPC_PIPE_EP_ADDR, NULL,
    IPC_CM0_TO_CM4_CLIENT_ID);
    if (CY_IPC_PIPE_SUCCESS != ipc_status)
    {
        cy_svc_log_err(ipc_status, "[IPC] deregister fail");
    }
    else
    {
        ret_val = CY_RSLT_SUCCESS;
    }
#elif ENABLE_SVC_IPC_STUB_SINGLE_CORE
    cy_svc_log_dbg("IPC deinit stub on same core success");
    ret_val = CY_RSLT_SUCCESS;
#else
    cy_svc_log_dbg("TODO: IPC deinit");
    ret_val = CY_RSLT_SUCCESS;
#endif

    return ret_val;
}

/**
 * Gets the HP instance handle
 *
 * @param none
 *
 * @return    HP instance handle
 */
svc_hp_instance_t* svc_hp_get_instance(void)
{
    return &global_hp_instance;
}

#ifndef ENABLE_SVC_IPC_STUB_SINGLE_CORE
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

#endif
