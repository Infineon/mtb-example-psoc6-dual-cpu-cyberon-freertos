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
 * @file staged_voice_control_lp_circular_buffer.h
 *
 */

#ifndef CY_SVC_LP_CIRCULAR_BUFFER_H__
#define CY_SVC_LP_CIRCULAR_BUFFER_H__

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef ENABLE_SVC_LP_MW
#include "staged_voice_control_lp_private.h"
#include "staged_voice_control_lp_resource.h"
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
 *                              Type Definitions
 ******************************************************************************/

/*******************************************************************************
 *                              Structures
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                              Function Declarations
 ******************************************************************************/

cy_rslt_t svc_lp_create_circular_buf(
        svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *create_domain);

cy_rslt_t svc_lp_delete_circular_buf(svc_lp_instance_t *lp_instance);

cy_rslt_t svc_lp_get_circular_buf_rd_wr_pointer(
        svc_lp_instance_t *lp_instance,
        uint8_t **buf_ptr,
        bool is_wr_operation);

cy_rslt_t svc_lp_circular_buffer_update_rd_wr_offset(
        svc_lp_instance_t *lp_instance,
        bool is_wr_operation);

cy_rslt_t svc_lp_start_circular_buf_update_on_transition_to_hp(
        svc_lp_instance_t *lp_instance,
        uint8_t *cur_pointer,
        svc_stage_trigger_t stage_trigger);

cy_rslt_t svc_lp_get_circular_buf_pre_shift_buffer_from_any_address(
        svc_lp_instance_t *lp_instance,
        uint8_t *cur_pointer,
        unsigned int number_frames_before_cur_pointer,
        uint8_t **pre_buffer);

cy_rslt_t svc_lp_get_circular_buf_post_shift_buffer_from_any_address(
        svc_lp_instance_t *lp_instance,
        uint8_t *cur_pointer,
        unsigned int number_frames_after_cur_pointer,
        uint8_t **post_buffer);

uint32_t svc_lp_create_crc_for_buffer(svc_lp_instance_t *lp_instance,
        char *data);

cy_rslt_t svc_lp_verify_crc_of_the_buffer(svc_lp_instance_t *lp_instance,
        char *crc_source_pointer,
        uint32_t crc, char *data);

#endif

#ifdef __cplusplus
}
#endif

#endif /* CY_SVC_LP_CIRCULAR_BUFFER_H__ */
