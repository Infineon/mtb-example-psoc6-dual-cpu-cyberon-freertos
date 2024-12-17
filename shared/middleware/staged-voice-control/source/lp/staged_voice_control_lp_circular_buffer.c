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
 * @file staged_voice_control_lp_circular_buffer.c
 *
 */
#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_circular_buffer.h"
#include "cy_mem_check_utils.h"

/*******************************************************************************
 *                              Macros
 ******************************************************************************/


#ifdef ENABLE_CBUF_GUARD_BYTES

#define MAX_CBUF_GUARD_BYTES (8)

#define SAVE_CBUF_GUARD_BYTES(__X__,__Y__)  *((uint32_t *)(__X__)) = 0x1234; \
                                            *((uint32_t *)((char*)__X__+ 4 +__Y__)) = 0x5678;

#define VERIFY_CBUF_GUARD_BYTES(__X__,__Y__)  if((*((uint32_t *)(__X__)) != 0x1234) \
         || (*((uint32_t*) ((char*) __X__ + 4 + __Y__)) != 0x5678 )) \
         { \
             cy_svc_log_err (CY_RSLT_SVC_CBUF_EXT_CORRUPTION , \
                "Error !!! Cbuf corrupted externally by some task, exiting SVC LP task"); \
                exit(-1); \
         }

#define SIMULATE_GUARD_BYTES_CORRUPTION(__X__) *((uint32_t *)(__X__)) = 0x0;

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

cy_rslt_t svc_lp_create_circular_buf(
        svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *create_domain)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    if (NULL == lp_instance) return ret_val;
    lp_instance->circular_shared_buffer = (void *) calloc(
            sizeof(svc_circular_buffer_header_t), 1);
    if (NULL == lp_instance->circular_shared_buffer)
    {
        ret_val = CY_RSLT_SVC_OUT_OF_MEMORY;
        cy_svc_log_err(ret_val, "Create Circular buffer fail, size:%d",
                sizeof(svc_circular_buffer_header_t));
        goto CLEAN_RETURN;
    }

    CY_MEM_UTIL_PRINT_ADDR("Alloc:svc_circular_shared_buffer",
            lp_instance->circular_shared_buffer,
            sizeof(svc_circular_buffer_header_t));

#ifdef ENABLE_CBUF_GUARD_BYTES
    lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard =
            (void*) calloc(
                    create_domain->total_circular_buf_size
                            + MAX_CBUF_GUARD_BYTES, 1);
    if (NULL
            == lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard)
    {
        ret_val = CY_RSLT_SVC_OUT_OF_MEMORY;
        cy_svc_log_err(ret_val, "Create Circular buffer fail, size:%d",
                create_domain->total_circular_buf_size);
        goto CLEAN_RETURN;
    }

    CY_MEM_UTIL_PRINT_ADDR(
            "Alloc:svc_circular_actual_buffer_start_address_with_guard",
            lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard,
            create_domain->total_circular_buf_size + MAX_CBUF_GUARD_BYTES);
            
    SAVE_CBUF_GUARD_BYTES(lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard,
            create_domain->total_circular_buf_size);

    lp_instance->circular_shared_buffer->buffer_start_address =
            lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard
                    + 4;
#else
    lp_instance->circular_shared_buffer->buffer_start_address =
            (void*) calloc(
                    create_domain->total_circular_buf_size , 1);
    if (NULL
            == lp_instance->circular_shared_buffer->buffer_start_address)
    {
        ret_val = CY_RSLT_SVC_OUT_OF_MEMORY;
        cy_svc_log_err(ret_val, "Create Circular buffer fail, size:%d",
                create_domain->total_circular_buf_size);
        goto CLEAN_RETURN;
    }

    CY_MEM_UTIL_PRINT_ADDR("Alloc:svc_circular_buffer_start_address", lp_instance->circular_shared_buffer->buffer_start_address, create_domain->total_circular_buf_size);
#endif


    lp_instance->circular_shared_buffer->circular_buf_size =
            create_domain->total_circular_buf_size;
    lp_instance->circular_shared_buffer->wr_offset = 0;
    lp_instance->circular_shared_buffer->rd_offset = 0;

    lp_instance->circular_shared_buffer->frame_size_in_bytes =
            FRAME_DATA_SIZE(create_domain->audio_input_type,
                    SAMPLE_SIZE_IN_BYTES, create_domain->sample_rate*1000,
                    create_domain->single_frame_time_ms);
    if (0 == lp_instance->circular_shared_buffer->frame_size_in_bytes)
    {
        ret_val = CY_RSLT_SVC_BAD_ARG;
        cy_svc_log_err(ret_val, "Invalid single frame size");
        goto CLEAN_RETURN;
    }

    lp_instance->circular_shared_buffer->audio_type =
            create_domain->audio_input_type;

    lp_instance->circular_shared_buffer->sameple_size_in_bytes =
    SAMPLE_SIZE_IN_BYTES;

    cy_svc_log_dbg(
            "CBufHDR [Addr:0x%x] [Atype:%d,FrSz:%d,SaSz:%d,Wo:%d,Ro:%d,BufSz:%d,BufAddr:0x%x]",
            lp_instance->circular_shared_buffer,
            lp_instance->circular_shared_buffer->audio_type,
            lp_instance->circular_shared_buffer->frame_size_in_bytes,
            lp_instance->circular_shared_buffer->sameple_size_in_bytes,
            lp_instance->circular_shared_buffer->wr_offset,
            lp_instance->circular_shared_buffer->rd_offset,
            lp_instance->circular_shared_buffer->circular_buf_size,
            lp_instance->circular_shared_buffer->buffer_start_address);

    ret_val = CY_RSLT_SUCCESS;

    CLEAN_RETURN:

    if (CY_RSLT_SUCCESS != ret_val)
    {
        if (NULL != lp_instance)
        {
            if (NULL != lp_instance->circular_shared_buffer)
            {
                if (NULL
                        != lp_instance->circular_shared_buffer->buffer_start_address)
                {
                   CY_MEM_UTIL_PRINT_ADDR("Free:svc_circular_buffer_start_address",
                            lp_instance->circular_shared_buffer->buffer_start_address,
                            0);
                    free(
                            lp_instance->circular_shared_buffer->buffer_start_address);
                    lp_instance->circular_shared_buffer->buffer_start_address =
                            NULL;
                }
                CY_MEM_UTIL_PRINT_ADDR("Free:svc_circular_shared_buffer",
                                     lp_instance->circular_shared_buffer, 0);

                free(lp_instance->circular_shared_buffer);
                lp_instance->circular_shared_buffer = NULL;
            }
        }
    }
    return ret_val;
}

cy_rslt_t svc_lp_get_circular_buf_rd_wr_pointer (
        svc_lp_instance_t *lp_instance,
        uint8_t **buf_ptr,
        bool is_wr_operation)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    unsigned int cur_offset = 0;

    if (true == is_wr_operation)
    {
        cur_offset = lp_instance->circular_shared_buffer->wr_offset;
    }
    else
    {
        cur_offset = lp_instance->circular_shared_buffer->rd_offset;
    }

    *buf_ptr = NULL;

    if (cur_offset
            < lp_instance->circular_shared_buffer->circular_buf_size)
    {
        *buf_ptr =
                lp_instance->circular_shared_buffer->buffer_start_address
                        + cur_offset;
        ret_val = CY_RSLT_SUCCESS;
    }
    else if (cur_offset
            == lp_instance->circular_shared_buffer->circular_buf_size)
    {
        *buf_ptr =
                lp_instance->circular_shared_buffer->buffer_start_address;
        ret_val = CY_RSLT_SUCCESS;
    }
    else
    {
        ret_val = CY_RSLT_SVC_CIRCULAR_BUFFER_INVALID_WR_OFFSET;
        cy_svc_log_err_on_no_isr(ret_val, "Invalid RD(0)/WR(1)=%d buffer offset",
                is_wr_operation);
        goto CLEAN_RETURN;
    }

    CLEAN_RETURN: return ret_val;
}



cy_rslt_t svc_lp_circular_buffer_update_rd_wr_offset (
        svc_lp_instance_t *lp_instance,
        bool is_wr_operation)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    unsigned int *p_cur_offset = 0;

    if (true == is_wr_operation)
    {
        p_cur_offset = &lp_instance->circular_shared_buffer->wr_offset;
    }
    else
    {
        p_cur_offset = &lp_instance->circular_shared_buffer->rd_offset;

#ifdef ENABLE_CBUF_GUARD_BYTES
        VERIFY_CBUF_GUARD_BYTES(
                lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard,
                lp_instance->circular_shared_buffer->circular_buf_size);
#endif

    }

    if (NULL != lp_instance->circular_shared_buffer->buffer_start_address)
    {
        if ((*p_cur_offset
                + lp_instance->circular_shared_buffer->frame_size_in_bytes)
                < lp_instance->circular_shared_buffer->circular_buf_size)
        {
            *p_cur_offset +=
                    lp_instance->circular_shared_buffer->frame_size_in_bytes;
            ret_val = CY_RSLT_SUCCESS;
        }
        else if ((*p_cur_offset
                + lp_instance->circular_shared_buffer->frame_size_in_bytes)
                == lp_instance->circular_shared_buffer->circular_buf_size)
        {
            *p_cur_offset = 0;
            ret_val = CY_RSLT_SUCCESS;
        }
        else
        {
            ret_val = CY_RSLT_SVC_CIRCULAR_BUFFER_INVALID_WR_OFFSET;
            cy_svc_log_err_on_no_isr(ret_val, "Invalid WR buffer offset");
            goto CLEAN_RETURN;
        }
    }

    CLEAN_RETURN: return ret_val;
}

cy_rslt_t svc_lp_delete_circular_buf(
        svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    if(NULL != lp_instance->circular_shared_buffer)
    {
#ifdef ENABLE_CBUF_GUARD_BYTES
        if(NULL != lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard)
        {
            CY_MEM_UTIL_PRINT_ADDR(
                    "Free:svc_circular_actual_buffer_start_address_with_guard",
                    lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard,
                    0);
            free(lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard);
            lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard = NULL;
            lp_instance->circular_shared_buffer->buffer_start_address = NULL;
        }
#else
        if(NULL != lp_instance->circular_shared_buffer->buffer_start_address)
        {
            CY_MEM_UTIL_PRINT_ADDR("Free:svc_circular_buffer_start_address", lp_instance->circular_shared_buffer->buffer_start_address, 0);
            free(lp_instance->circular_shared_buffer->buffer_start_address);
            lp_instance->circular_shared_buffer->buffer_start_address = NULL;
        }
#endif
        CY_MEM_UTIL_PRINT_ADDR("Free:svc_circular_shared_buffer", lp_instance->circular_shared_buffer, 0);
        free(lp_instance->circular_shared_buffer);
        lp_instance->circular_shared_buffer = NULL;

        cy_svc_log_info("Cbuf freed success");
    }
    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}

/**
 * TODO: Check for available data and limit with the max
 */
#define TRIGGER_FRAME_DEF_OFFSET_BY_FRAMES_FOR_SOD_DETECT (10)

static cy_rslt_t svc_lp_identify_start_address_to_send_on_detect (
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger,
        uint8_t *cur_pointer,
        uint8_t **start_notify_pointer,
        uint16_t *insuff_frame_counter)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    uint8_t *pre_shift_buffer = NULL;

    uint32_t number_frames_before_trigger = 0;

    *insuff_frame_counter = 0;

    if(SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED == stage_trigger)
    {
        if (lp_instance->stats.frame_counter_received_after_last_aad_dbg
                >= lp_instance->init_params.pre_roll_frame_count_from_lpwwd_detect_frame)
        {
            number_frames_before_trigger =
                    lp_instance->init_params.pre_roll_frame_count_from_lpwwd_detect_frame
                            - 1;
//            cy_svc_log_info(
//                    "Information !!! Enough frames available: [%d/%d]",
//                    number_frames_before_trigger,
//                    lp_instance->stats.frame_counter_received_after_last_aad_dbg);
        }
        else
        {
            if(lp_instance->stats.frame_counter_received_after_last_aad_dbg > 0)
            {
                number_frames_before_trigger =
                        lp_instance->stats.frame_counter_received_after_last_aad_dbg - 1;

                cy_svc_log_info(
                        "Information !!! NOT PreRoll frames available: [%d/%d/%d]",
                        number_frames_before_trigger,
                        lp_instance->stats.frame_counter_received_after_last_aad_dbg,
                        lp_instance->init_params.pre_roll_frame_count_from_lpwwd_detect_frame);

                *insuff_frame_counter =
                        (uint16_t)(
                                lp_instance->init_params.pre_roll_frame_count_from_lpwwd_detect_frame
                                        - lp_instance->stats.frame_counter_received_after_last_aad_dbg);
            }
            else
            {
                cy_svc_log_info("Error!!!!!! Invalid case !!! ");
            }
        }
    }
    else if (SVC_TRIGGER_SPEECH_ONSET_DETECTED == stage_trigger)
    {
        number_frames_before_trigger =
                TRIGGER_FRAME_DEF_OFFSET_BY_FRAMES_FOR_SOD_DETECT;

        if (lp_instance->init_params.sod_onset_detect_max_late_hit_delay_ms > 0)
        {
            number_frames_before_trigger =
                    lp_instance->init_params.sod_onset_detect_max_late_hit_delay_ms
                            / 10;
        }
    }

    svc_lp_get_circular_buf_pre_shift_buffer_from_any_address(lp_instance,
            cur_pointer, number_frames_before_trigger, &pre_shift_buffer);
    *start_notify_pointer = pre_shift_buffer;

    ret_val = CY_RSLT_SUCCESS;

    return ret_val;
}

bool is_next_stage_goes_direct_to_hp_core(
        svc_lp_instance_t *lp_instance,
        svc_stage_trigger_t stage_trigger)
{
    bool ret = false;
    switch (stage_trigger)
    {
        case SVC_TRIGGER_SPEECH_ONSET_DETECTED:
        {
            /**
             * LPWWD not enabled, HPWWD or ASR is enabled.
             */
            if ((  (!(CY_SVC_ENABLE_LPWWD
                    & lp_instance->init_params.stage_config_list))
                    &&
                    ((CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS
                            & lp_instance->init_params.stage_config_list)
                    || (CY_SVC_ENABLE_ASR_STATE_TRANSITIONS
                            & lp_instance->init_params.stage_config_list))))
            {
                ret = true;
            }
            break;
        }
        case SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED:
        {
            if (CY_SVC_ENABLE_ASR_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
            {
                ret = true;
            }
            break;
        }
        case SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED:
        {
            if (((CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS
                    & lp_instance->init_params.stage_config_list)
                    || (CY_SVC_ENABLE_ASR_STATE_TRANSITIONS
                            & lp_instance->init_params.stage_config_list)))
            {
                ret = true;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    return ret;
}

cy_rslt_t svc_lp_start_circular_buf_update_on_transition_to_hp(
        svc_lp_instance_t *lp_instance,
        uint8_t *cur_pointer,
        svc_stage_trigger_t stage_trigger)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    uint32_t data_to_send_in_bytes = 0;
    uint32_t total_data_sent_in_bytes = 0;
    unsigned char *buffer_end_address = NULL;
    uint8_t *start_notify_pointer = NULL;
    cy_svc_buffer_info_t buffer_info_bitmask = 0;
    uint16_t insuff_frame_counter = 0;

    if (SVC_TRIGGER_SPEECH_ONSET_DETECTED == stage_trigger)
    {
        if (false
                == is_next_stage_goes_direct_to_hp_core(lp_instance,
                        stage_trigger))
        {
            return CY_RSLT_SUCCESS;
        }
        svc_lp_identify_start_address_to_send_on_detect(lp_instance,
                stage_trigger, cur_pointer, &start_notify_pointer,
                &insuff_frame_counter);
    }
    else if (SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED == stage_trigger)
    {
#ifdef ENABLE_CBUF_GUARD_BYTES
       // SIMULATE_GUARD_BYTES_CORRUPTION(lp_instance->circular_shared_buffer->actual_buffer_start_address_with_guard);
#endif
        if (false
                == is_next_stage_goes_direct_to_hp_core(lp_instance,
                        stage_trigger))
        {
            return CY_RSLT_SUCCESS;
        }

        svc_lp_identify_start_address_to_send_on_detect(lp_instance,
                stage_trigger, cur_pointer, &start_notify_pointer,
                &insuff_frame_counter);

        (void) svc_lp_lpwwd_get_wwd_identified(lp_instance,
                &buffer_info_bitmask);
    }
    else if (SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED == stage_trigger)
    {
        if (false
                == is_next_stage_goes_direct_to_hp_core(lp_instance,
                        stage_trigger))
        {
            lp_instance->hpwwd_trigger_data_final_address = NULL;
            lp_instance->post_hpwwd_pending_frame_counter_to_hp = 0;
            return CY_RSLT_SUCCESS;
        }

        if (NULL == lp_instance->hpwwd_trigger_data_final_address)
        {
            cy_svc_log_info(
                    "Ignoring buffer update on HPWWD detected transition as last addr is NULL");
            return CY_RSLT_SUCCESS;
        }
        else
        {
            /**
             * Resume the data from the point last sent.
             */
            start_notify_pointer =
                    lp_instance->hpwwd_trigger_data_final_address;
            cur_pointer = lp_instance->last_processed_frame_start_address;

            if ((cur_pointer == start_notify_pointer)
                    && (lp_instance->post_roll_frame_count_from_lpwwd_detect_frame
                            > 0))
            {
                lp_instance->hpwwd_trigger_data_final_address = NULL;
                return CY_RSLT_SUCCESS;
            }
        }
    }
    else if (SVC_TRIGGER_SEND_POST_WWD_HPWWD_DET_IN_PROGRESS == stage_trigger)
    {
        if ((0 != lp_instance->post_wwd_frame_count_req_by_hp)
                && (0 != lp_instance->post_lpwwd_received_frame_count)
                && (lp_instance->post_wwd_frame_count_req_by_hp
                        <= (lp_instance->post_lpwwd_received_frame_count - 1)))
        {
            /**
             * Resume the data from the point last sent.
             */
            start_notify_pointer =
                    lp_instance->hpwwd_trigger_data_final_address;

            svc_lp_get_circular_buf_post_shift_buffer_from_any_address(
                    lp_instance, lp_instance->hpwwd_trigger_data_final_address,
                    lp_instance->post_wwd_frame_count_req_by_hp - 1,
                    &cur_pointer);
        }
        else
        {
            return CY_RSLT_SUCCESS;
        }
    }
    else
    {
        return CY_RSLT_SUCCESS;
    }

    size_t num_waiting = 0;
    cy_rtos_count_queue(&lp_instance->data_queue, &num_waiting);

    cy_svc_log_info(
            "Trig:%d, SA:%p,BSz:%d,FSz:%d,CPr:%p, Notify:%p, WO:0x%x, RO:0x%x, Q:%d",
            stage_trigger,
            lp_instance->circular_shared_buffer->buffer_start_address,
            lp_instance->circular_shared_buffer->circular_buf_size,
            lp_instance->circular_shared_buffer->frame_size_in_bytes,
            cur_pointer, start_notify_pointer,
            lp_instance->circular_shared_buffer->wr_offset,
            lp_instance->circular_shared_buffer->rd_offset, num_waiting);

    /**
     * Logic to send the frame pointers (single/two IPC msg) to SVC HP depends
     * on the start address and with the current address of the data.
     */
    if (start_notify_pointer < cur_pointer)
    {
        data_to_send_in_bytes = cur_pointer - start_notify_pointer
                + lp_instance->circular_shared_buffer->frame_size_in_bytes;
        total_data_sent_in_bytes += data_to_send_in_bytes;
        svc_lp_send_ipc_event_circular_buffer_update(lp_instance,
                start_notify_pointer, data_to_send_in_bytes,
                insuff_frame_counter, buffer_info_bitmask);
        cy_svc_log_info("Line:%d, StartNotifyPtr:%p Fcount:%d", __LINE__,
                start_notify_pointer, data_to_send_in_bytes/
                lp_instance->circular_shared_buffer->frame_size_in_bytes);
    }
    else
    {
        buffer_end_address =
                lp_instance->circular_shared_buffer->buffer_start_address
                        + lp_instance->circular_shared_buffer->circular_buf_size;
        data_to_send_in_bytes = buffer_end_address - start_notify_pointer;
        total_data_sent_in_bytes += data_to_send_in_bytes;
        svc_lp_send_ipc_event_circular_buffer_update(lp_instance,
                start_notify_pointer, data_to_send_in_bytes,
                insuff_frame_counter, buffer_info_bitmask);
        cy_svc_log_info("Line:%d, StartNotifyPtr:%p Fcount:%d", __LINE__,
                start_notify_pointer,
                data_to_send_in_bytes
                        / lp_instance->circular_shared_buffer->frame_size_in_bytes);

        data_to_send_in_bytes = cur_pointer
                - lp_instance->circular_shared_buffer->buffer_start_address
                + lp_instance->circular_shared_buffer->frame_size_in_bytes;
        total_data_sent_in_bytes += data_to_send_in_bytes;
        svc_lp_send_ipc_event_circular_buffer_update(lp_instance,
                lp_instance->circular_shared_buffer->buffer_start_address,
                data_to_send_in_bytes, 0, 0);
        cy_svc_log_info("Line:%d, StartNotifyPtr:%p Fcount:%d", __LINE__,
                lp_instance->circular_shared_buffer->buffer_start_address,
                data_to_send_in_bytes
                        / lp_instance->circular_shared_buffer->frame_size_in_bytes);
    }

    if (SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED == stage_trigger)
    {
        if (CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS
                & lp_instance->init_params.stage_config_list)
        {
            lp_instance->stats.hpwwd_feed_counter_dbg += total_data_sent_in_bytes
                    / lp_instance->circular_shared_buffer->frame_size_in_bytes;
            /**
             * When the stage trigger is SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED,
             * the data till current_pointer + frame size would be sent to HP.
             *
             * for the next data to be sent from other places, saving the address
             * till where the data is sent. This address will be used to help send the
             * accumulated data till the time HP detects HPWWD.
             */
            svc_lp_get_circular_buf_post_shift_buffer_from_any_address(
                    lp_instance, cur_pointer, 1,
                    &lp_instance->hpwwd_trigger_data_final_address);

            if (lp_instance->post_roll_frame_count_from_lpwwd_detect_frame > 0)
            {
                lp_instance->post_hpwwd_pending_frame_counter_to_hp =
                        lp_instance->post_roll_frame_count_from_lpwwd_detect_frame;

                cy_svc_log_info("Trig to send hpwwd pending live buff cnt:%d",
                        lp_instance->post_hpwwd_pending_frame_counter_to_hp);
            }
        }
    }
    else if (SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED == stage_trigger)
    {
        /**
         * Reset, as the purpose of this address is set by
         * SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED and used
         * by SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED.
         */
        lp_instance->hpwwd_trigger_data_final_address = NULL;

    }
    else if (SVC_TRIGGER_SPEECH_ONSET_DETECTED == stage_trigger)
    {
        lp_instance->hpwwd_trigger_data_final_address = NULL;
    }
    else if (SVC_TRIGGER_SEND_POST_WWD_HPWWD_DET_IN_PROGRESS == stage_trigger)
    {
        if (CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS
                & lp_instance->init_params.stage_config_list)
        {
            lp_instance->stats.hpwwd_feed_counter_dbg += total_data_sent_in_bytes
                    / lp_instance->circular_shared_buffer->frame_size_in_bytes;

            svc_lp_get_circular_buf_post_shift_buffer_from_any_address(
                    lp_instance, lp_instance->hpwwd_trigger_data_final_address,
                    lp_instance->post_wwd_frame_count_req_by_hp,
                    &lp_instance->hpwwd_trigger_data_final_address);

            cy_svc_log_info("LastSentPostWWDAddress:0x%x (next new frame)",
                    lp_instance->hpwwd_trigger_data_final_address);

            lp_instance->post_lpwwd_received_frame_count =
                    lp_instance->post_lpwwd_received_frame_count
                            - lp_instance->post_wwd_frame_count_req_by_hp;
            lp_instance->post_wwd_frame_count_req_by_hp = 0;
        }
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}


cy_rslt_t svc_lp_get_circular_buf_post_shift_buffer_from_any_address(
        svc_lp_instance_t *lp_instance,
        uint8_t *cur_pointer,
        unsigned int number_frames_after_cur_pointer,
        uint8_t **post_buffer)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    uint8_t *end_pointer = NULL;
    uint32_t frames_at_end_of_buffer = 0;

    end_pointer = lp_instance->circular_shared_buffer->buffer_start_address
            + lp_instance->circular_shared_buffer->circular_buf_size;

    *post_buffer  = NULL;

    if (cur_pointer
            + (number_frames_after_cur_pointer
                    * lp_instance->circular_shared_buffer->frame_size_in_bytes)
            >= end_pointer)
    {
        frames_at_end_of_buffer = (end_pointer - cur_pointer)
                / lp_instance->circular_shared_buffer->frame_size_in_bytes;

        *post_buffer =
                lp_instance->circular_shared_buffer->buffer_start_address
                        + ((number_frames_after_cur_pointer
                                - frames_at_end_of_buffer)
                                * lp_instance->circular_shared_buffer->frame_size_in_bytes);
    }
    else
    {
        *post_buffer =
                cur_pointer
                        + (number_frames_after_cur_pointer
                                * lp_instance->circular_shared_buffer->frame_size_in_bytes);
    }

//    cy_svc_log_info("CP:0x%x EP:0x%x NP:0x%x", cur_pointer, end_pointer,
//            *post_buffer);

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}


cy_rslt_t svc_lp_get_circular_buf_pre_shift_buffer_from_any_address(
        svc_lp_instance_t *lp_instance,
        uint8_t *cur_pointer,
        unsigned int number_frames_before_cur_pointer,
        uint8_t **pre_buffer)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    uint8_t *end_pointer = NULL;

    *pre_buffer = NULL;

    if (lp_instance->circular_shared_buffer->buffer_start_address
            <= (cur_pointer
                    - (number_frames_before_cur_pointer
                            * lp_instance->circular_shared_buffer->frame_size_in_bytes)))
    {
//        cy_svc_log_info("Buffer is not wrapped around");

        *pre_buffer =
                cur_pointer
                        - (number_frames_before_cur_pointer
                                * lp_instance->circular_shared_buffer->frame_size_in_bytes);
    }
    else
    {
//        cy_svc_log_info("Buffer Wrapped around");

        number_frames_before_cur_pointer =
                number_frames_before_cur_pointer
                        - (cur_pointer
                                - lp_instance->circular_shared_buffer->buffer_start_address)
                                / lp_instance->circular_shared_buffer->frame_size_in_bytes;

        end_pointer = lp_instance->circular_shared_buffer->buffer_start_address
                + lp_instance->circular_shared_buffer->circular_buf_size;

        *pre_buffer =
                end_pointer
                        - (number_frames_before_cur_pointer
                                * lp_instance->circular_shared_buffer->frame_size_in_bytes);
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}

uint32_t svc_lp_create_crc_for_buffer(svc_lp_instance_t *lp_instance,
        char *data)
{
    uint32_t crc = 0;
    uint32_t *start = NULL;
    uint32_t *end = NULL;

    start = (uint32_t*) data;
    end = (uint32_t*) ((char*) ((char*) data
            + lp_instance->circular_shared_buffer->frame_size_in_bytes
            - sizeof(uint32_t)));

    /**
     * Simple CRC check, done only for start and end of the buffers.
     * Not required to do for the entire buffer size.
     */
    crc = (*start) ^ (*end);

    return crc;
}

cy_rslt_t svc_lp_verify_crc_of_the_buffer(svc_lp_instance_t *lp_instance,
        char *crc_source_pointer,
        uint32_t crc, char *data)
{
    cy_rslt_t ret_val = 0;
    uint32_t crc_computed = 0;

    crc_computed = svc_lp_create_crc_for_buffer(lp_instance, data);

    if(crc != crc_computed)
    {
        size_t num_waiting = 0;
        cy_rtos_count_queue(&lp_instance->data_queue, &num_waiting);

        ret_val = CY_RSLT_SVC_DATA_CORRUPTION;
        cy_svc_log_err(ret_val, "data corruption: [0x%08x] [0x%08x] [%d] [%p %p]",
                crc, crc_computed, num_waiting, crc_source_pointer, data);

        lp_instance->stats.crc_check_fail_counter_dbg++;
        return ret_val;
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}
#endif
