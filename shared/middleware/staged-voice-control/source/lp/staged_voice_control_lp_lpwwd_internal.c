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
 * @file staged_voice_control_lp_lpwwd_internal.c
 *
 */
#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_process_data.h"
#include "staged_voice_control_lp_private.h"

#ifdef ENABLE_IFX_LPWWD
#include "staged_voice_control_lp_lpwwd_internal.h"
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

#ifdef ENABLE_IFX_LPWWD
static cy_rslt_t svc_lp_lpwwd_internal_process_detect_result(
        svc_lp_instance_t *lp_instance,
        uint8_t *data,
        cy_lpwwd_postwwd_status_t lpwwd_status)
{
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;

    switch (lpwwd_status)
    {
        case CY_LPWWD_WAKE_WORD_INVALID:
        {
            break;
        }

//        case CY_SVC_LPWWD_STATUS_DETECTION_NOT_STARTED:
//        {
//            break;
//        }

        case CY_LPWWD_WAKE_WORD_DETECTION_IN_PROGRESS:
        {
            break;
        }

        case CY_LPWWD_WAKE_WORD_NOT_DETECTED:
        {
            (void) svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED);

            lp_instance->lpwwd_detected = false;

            cy_svc_log_dbg("LPWWD detection fail");
            break;
        }

        case CY_LPWWD_TIMEOUT:
        {
            (void) svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED);

            lp_instance->lpwwd_detected = false;
//            cy_svc_log_dbg("LPWWD detection timeout");
            break;
        }

        case CY_LPWWD_WAKE_WORD_DETECTED:
        {
            (void) svc_lp_trigger_state(lp_instance,
                    SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED);

            lp_instance->lpwwd_detected = true;

            svc_lp_start_circular_buf_update_on_transition_to_hp(lp_instance,
                    data, SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED);

            cy_svc_log_dbg("LPWWD DETD [Addr:0x%08X,DET:%d:NDET:%d]", data,
                    lp_instance->stats.lpwwd_detect_counter_dbg,
                    lp_instance->stats.lpwwd_not_detect_counter_dbg);
            break;
        }

//        case CY_SVC_LPWWD_STATUS_DETECTION_MAX:
//        {
//            break;
//        }

        default:
        {
            break;
        }
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
}
#endif

cy_rslt_t svc_lp_lpwwd_internal_init(
        svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *init)
{
#ifdef ENABLE_IFX_LPWWD
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cy_lpwwd_config_params_t config_params = {0};

    /* Initialize Pre wake word component */
    config_params.prewwd_config.component_id = CY_FE_COMPONENT_MFCC;
    config_params.prewwd_config.frame_size = (FRAME_SIZE_MS * SAMPLE_RATE / 1000);
    config_params.prewwd_config.frame_shift = (FRAME_SHIFT_MS * SAMPLE_RATE / 1000);
    config_params.prewwd_config.number_of_dct_coefficients = NUM_MFCC_COEFFS;
    config_params.prewwd_config.number_of_filter_banks = NUM_FBANK_BINS;
    config_params.prewwd_config.sampling_rate = SAMPLE_RATE;
    config_params.prewwd_config.audio_input_frame_size = MONO_FRAME_SIZE;

    config_params.postwwd_config.sampling_rate = SAMPLE_RATE;
    config_params.postwwd_config.frame_rate = NN_FRAME_RATE;
    config_params.postwwd_config.stacked_frame_delay = (int16_t)(
            0.5 + NN_STACKED_DELAY_SEC * (1l << Q_LOOKBACK));
    config_params.postwwd_config.hmm_pp_threshold = HMM_PP_THD;

    config_params.ml_config.model_meta_buf = init->wake_word_model_meta_buf;
    config_params.ml_config.model_binary_buf = init->wake_word_model_binary_buf;

    cy_svc_log_info("Init:PrWWD[%d,%d,%d,%d,%d,%d,%d,%p,%p]PoWWD[%d,%d,%d,%f]",
            config_params.prewwd_config.component_id,
            config_params.prewwd_config.frame_size,
            config_params.prewwd_config.frame_shift,
            config_params.prewwd_config.number_of_dct_coefficients,
            config_params.prewwd_config.number_of_filter_banks,
            config_params.prewwd_config.sampling_rate,
            config_params.prewwd_config.audio_input_frame_size,
            config_params.ml_config.model_meta_buf,
            config_params.ml_config.model_binary_buf,
            config_params.postwwd_config.sampling_rate,
            config_params.postwwd_config.frame_rate,
            config_params.postwwd_config.stacked_frame_delay,
            config_params.postwwd_config.hmm_pp_threshold);

    ret_val = cy_lpwwd_init(&config_params, &lp_instance->lpwwd_handle);
    if(ret_val != CY_RSLT_SUCCESS)
    {
        cy_svc_log_err(ret_val, "Failed to initialize LPWWD component");
        return ret_val;
    }

    return ret_val;
#else
    return CY_RSLT_SUCCESS;
#endif
}

cy_rslt_t svc_lp_process_data_lpwwd_internal(
        svc_lp_instance_t *lp_instance,
        uint8_t *data)
{
#ifdef ENABLE_IFX_LPWWD
    cy_rslt_t ret_val = CY_RSLT_SVC_GENERIC_ERROR;
    cy_lpwwd_postwwd_status_t wwd_status = CY_LPWWD_WAKE_WORD_INVALID;

    if ((CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION
            == lp_instance->current_stage)
            && (false == lp_instance->lpwwd_detected))
    {
        lp_instance->stats.lpwwd_feed_counter_dbg++;

#ifdef ENABLE_USB_DBG_OUTPUT
        usb_send_out_dbg_put(3, (short*) data);
#endif

        ret_val = cy_lpwwd_feed(lp_instance->lpwwd_handle, (char *)data, &wwd_status);
        if(CY_RSLT_SUCCESS != ret_val)
        {
            cy_svc_log_err(ret_val, "Failed to process data in LPWWD");
            goto CLEAN_RETURN;
        }

        ret_val = svc_lp_lpwwd_internal_process_detect_result(lp_instance, data,
                wwd_status);
        if (CY_RSLT_SUCCESS != ret_val)
        {
            cy_svc_log_err(ret_val, "postwwd process fail");
            goto CLEAN_RETURN;
        }
    }
    else
    {
        ret_val = CY_RSLT_SUCCESS;
    }


    CLEAN_RETURN: return ret_val;
#else
    return CY_RSLT_SUCCESS;
#endif
}

cy_rslt_t svc_lp_lpwwd_internal_reset(svc_lp_instance_t *lp_instance)
{
#ifdef ENABLE_IFX_LPWWD
    cy_lpwwd_reset(lp_instance->lpwwd_handle);
    return CY_RSLT_SUCCESS;
#else
    return CY_RSLT_SUCCESS;
#endif
}

cy_rslt_t svc_lp_lpwwd_internal_deinit(
        svc_lp_instance_t *lp_instance)
{
#ifdef ENABLE_IFX_LPWWD
    cy_rslt_t result = CY_RSLT_SUCCESS;
    result = cy_lpwwd_deinit(&lp_instance->lpwwd_handle);
    if (CY_RSLT_SUCCESS != result)
    {
        cy_svc_log_err(result, "Failed to de-initialize LPWWD");
        return result;
    }
    lp_instance->lpwwd_handle = NULL;
    return result;
#else
    return CY_RSLT_SUCCESS;
#endif
}
#endif
