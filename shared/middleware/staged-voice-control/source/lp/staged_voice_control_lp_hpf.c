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
 * @file staged_voice_control_lp_hpf.c
 *
 */
#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_hpf.h"

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

cy_rslt_t svc_lp_hpf_init(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    if (NULL == lp_instance)
    {
        goto CLEAN_RETURN;
    }

    cy_svc_log_dbg("TODO: HPF init success");

    ret_val = CY_RSLT_SUCCESS;
    CLEAN_RETURN: return ret_val;
}

cy_rslt_t svc_lp_hpf_deinit(svc_lp_instance_t *lp_instance)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    if (NULL == lp_instance)
    {
        goto CLEAN_RETURN;
    }

    cy_svc_log_dbg("TODO HPF deinit success");

    ret_val = CY_RSLT_SUCCESS;
    CLEAN_RETURN: return ret_val;
}

cy_rslt_t svc_lp_hpf_process(
        svc_lp_instance_t *lp_instance,
        uint8_t *input,
        uint8_t *output)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    unsigned int size_to_process = 0;

    if ((NULL == lp_instance) || (NULL == input) || (NULL == output))
    {
        goto CLEAN_RETURN;
    }

    if (CY_SVC_AUDIO_INPUT_TYPE_STEREO
            == lp_instance->init_params.audio_input_type)
    {
        size_to_process =
                lp_instance->circular_shared_buffer->frame_size_in_bytes
                        / 2;
#ifdef ENABLE_HPF_STUB
        /**
         *  Perform simulated (memcopy) operation for HPF
         */
        memcpy(output, input, size_to_process);
        memcpy(output + size_to_process, input + size_to_process,
                size_to_process);
#else
        /**
         *  Perform HPF conversion for each channel separately
         */
#endif
    }
    else
    {
        /**
         * Assumed to be Mono type
         */
        size_to_process =
                lp_instance->circular_shared_buffer->frame_size_in_bytes;
#ifdef ENABLE_HPF_STUB
        /**
         *  Perform simulated (memcopy) operation for HPF
         */
        memcpy(output, input, size_to_process);
#else
        /**
         *  Perform HPF conversion for mono channel separately
         */
#endif
    }

    ret_val = CY_RSLT_SUCCESS;

    CLEAN_RETURN:
    return ret_val;
}
#endif
