/******************************************************************************
* File Name: cy_sod.c
*
* Description: This file contains functions for Speech onset detection.
*              
*
*
*******************************************************************************
* (c) 2021, Infineon Technologies Company. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Infineon Technologies Company (Infineon) or one of its
* subsidiaries and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Infineon hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Infineon's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Infineon.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Infineon
* reserves the right to make changes to the Software without notice. Infineon
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Infineon does
* not authorize its products for use in any products where a malfunction or
* failure of the Infineon product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Infineon's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Infineon against all liability.
*******************************************************************************/

/*******************************************************************************
* Include header file
******************************************************************************/

#include "cy_sod_private.h"

/******************************************************************************
* Defines
*****************************************************************************/

/******************************************************************************
* Constants
*****************************************************************************/

/******************************************************************************
* Functions
*****************************************************************************/

volatile bool sod_init_done = false;

cy_rslt_t cy_sod_init(cy_sod_config_params *config_params, cy_sod_t *handle)
{
    sod_context_t *sod_context = NULL;
    cy_rslt_t ret_val = CY_RSLT_SOD_GENERIC_ERROR;
    int32_t sod_config_prms_local[MAX_SOD_CONFIG_PARAMS_LEN] = {
        0, /* manunally generated configuration file set configuration version to zero */
        16000, /* sampling rate */
        160, /* input frmae size */
        9, /* IP_compnent_id: SOD */
        2, /* number of parameters */
        400, /* gap setting */
        16384 /* sensitivity */
    };

    if (true == sod_init_done)
    {
        ret_val = CY_RSLT_SOD_ALREADY_INITIALIZED;
        cy_sod_app_log_err(ret_val, "Already initialized");
        goto CLEAN_RETURN;
    }

    if ((NULL == config_params) || (NULL == handle))
    {
        ret_val = CY_RSLT_SOD_BAD_ARG;
        cy_sod_app_log_info("Invalid arguments, %p %p", config_params, handle);
        goto CLEAN_RETURN;
    }

    if ((CY_SOD_FRAME_SIZE_MONO_SAMPLES != config_params->input_frame_size)
            || (CY_SOD_SAMPLE_RATE_16000Hz != config_params->sampling_rate))
    {
        ret_val = CY_RSLT_SOD_BAD_ARG;
        cy_sod_app_log_err(ret_val, "Invalid arguments, %d %d",
                config_params->input_frame_size, config_params->sampling_rate);
        goto CLEAN_RETURN;
    }

    if ((0 > config_params->sensitivity)
            || (CY_MAX_SOD_SENSITIVITY < config_params->sensitivity))
    {
        ret_val = CY_RSLT_SOD_BAD_ARG;
        cy_sod_app_log_err(ret_val, "Invalid Sensitivity:%d",
                config_params->sensitivity);
        goto CLEAN_RETURN;
    }

    if (!((CY_SOD_ONSET_GAP_SETTING_1000_MS
            == config_params->onset_gap_setting_ms)
            || (CY_SOD_ONSET_GAP_SETTING_500_MS
                    == config_params->onset_gap_setting_ms)
            || (CY_SOD_ONSET_GAP_SETTING_400_MS
                    == config_params->onset_gap_setting_ms)
            || (CY_SOD_ONSET_GAP_SETTING_300_MS
                    == config_params->onset_gap_setting_ms)
            || (CY_SOD_ONSET_GAP_SETTING_200_MS
                    == config_params->onset_gap_setting_ms)
            || (CY_SOD_ONSET_GAP_SETTING_100_MS
                    == config_params->onset_gap_setting_ms)
            || (CY_SOD_ONSET_GAP_SETTING_0_MS
                    == config_params->onset_gap_setting_ms)))
    {
        ret_val = CY_RSLT_SOD_BAD_ARG;
        cy_sod_app_log_err(ret_val, "Invalid onset gap setting:%d",
                config_params->onset_gap_setting_ms);
        goto CLEAN_RETURN;
    }

    *handle = NULL;
    sod_context = (sod_context_t*) calloc(sizeof(sod_context_t), 1);
    if (NULL == sod_context)
    {
        ret_val = CY_RSLT_SOD_OUT_OF_MEMORY;
        cy_sod_app_log_err(ret_val, "Mem alloc fail");
        goto CLEAN_RETURN;
    }

    CY_MEM_UTIL_PRINT_ADDR("Alloc:sod_handle", sod_context, sizeof(sod_context_t));

    memcpy(sod_context->sod_config_prms, sod_config_prms_local,
            sizeof(sod_config_prms_local));

    /* override the values which are provided by application */
    sod_context->sod_config_prms[1] = config_params->sampling_rate;
    sod_context->sod_config_prms[2] = config_params->input_frame_size;
    sod_context->sod_config_prms[5] = config_params->onset_gap_setting_ms;
    sod_context->sod_config_prms[6] = config_params->sensitivity;

    ret_val = cy_sod_init_parse_and_allot_resource_for_sod(sod_context);
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_sod_app_log_err(ret_val, "allot res fail");
        goto CLEAN_RETURN;
    }

    ret_val = cy_sod_init_sod_system_component(sod_context);
    if (CY_RSLT_SUCCESS != ret_val)
    {
        cy_sod_app_log_err(ret_val, "init sod fail");
        goto CLEAN_RETURN;
    }

    *handle = sod_context;

    cy_sod_app_log_info("SOD Init Success");
    sod_init_done = true;

    ret_val = CY_RSLT_SUCCESS;

    CLEAN_RETURN: return ret_val;
}


cy_rslt_t cy_sod_set_sensitivity(cy_sod_t handle, int sensitivity)
{
    cy_rslt_t ret_val = CY_RSLT_SOD_GENERIC_ERROR;

    if (false == sod_init_done)
    {
        ret_val = CY_RSLT_SOD_NOT_INITIALIZED;
        cy_sod_app_log_err(ret_val, "Not initialized");
        goto CLEAN_RETURN;
    }

    ret_val = CY_RSLT_SOD_FEATURE_NOT_SUPPORTED;

    CLEAN_RETURN:
    return ret_val;
}


/**
 * Reset the SOD MW. This will reset the SOD MW context and make it ready as
 * fresh SOD start detection. This API will be useful when there is
 * discontinuity in the data feed and the application wants
 * to restart of SOD.
 *
 * @param[in]  handle                  Handle to SOD MW
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_sod_reset(cy_sod_t handle)
{
    cy_rslt_t ret_val = CY_RSLT_SOD_GENERIC_ERROR;
    sod_context_t *sod_context = handle;
    uint32_t ErrIdx = 0;

    if (NULL == handle)
    {
        ret_val = CY_RSLT_SOD_BAD_ARG;
        cy_sod_app_log_info("Invalid arguments, %p", handle);
        goto CLEAN_RETURN;
    }

    if (false == sod_init_done)
    {
        ret_val = CY_RSLT_SOD_NOT_INITIALIZED;
        cy_sod_app_log_err(ret_val, "Not initialized");
        goto CLEAN_RETURN;
    }

    ErrIdx |= speech_utils_sod_reset(sod_context->sod_config_prms,
            sod_context->ifx_sod_container);
    if (ErrIdx)
    {
        ret_val = CY_RSLT_SOD_RESET_FAIL;
        cy_sod_app_log_err(ret_val,
                "SOD reset fail! ErrIDx:%x, CIdx=%d, LineNo=%d",
                IFX_SP_ENH_ERR_CODE(ErrIdx),
                IFX_SP_ENH_ERR_COMPONENT_INDEX(ErrIdx),
                IFX_SP_ENH_ERR_LINE_NUMBER(ErrIdx));
        goto CLEAN_RETURN;
    }

    ret_val = CY_RSLT_SUCCESS;
    CLEAN_RETURN:
    return ret_val;
}

cy_rslt_t cy_sod_process(
        cy_sod_t handle,
        bool sod_check_trigger,
        int16_t *data,
        cy_sod_status_t *sod_status)
{
    cy_rslt_t ret_val = CY_RSLT_SOD_GENERIC_ERROR;
    uint32_t ErrIdx = 0;
    bool sod_detect_status = false;
    sod_context_t *sod_context = handle;

    if ((NULL == handle) || (NULL == data) || (NULL == sod_status))
    {
        ret_val = CY_RSLT_SOD_BAD_ARG;
        cy_sod_app_log_info("Invalid arguments, %p %p %p", handle, data,
                sod_detect_status);
        goto CLEAN_RETURN;
    }

    if (false == sod_init_done)
    {
        ret_val = CY_RSLT_SOD_NOT_INITIALIZED;
        cy_sod_app_log_err(ret_val, "Not initialized");
        goto CLEAN_RETURN;
    }

    *sod_status = CY_SOD_STATUS_INVALID;

    ErrIdx |= speech_utils_sod_process(data, sod_context->ifx_sod_container,
            &sod_detect_status);
    if (ErrIdx)
    {
        ret_val = CY_RSLT_SOD_PROCESS_DATA_FAIL;
        cy_sod_app_log_err(ret_val,
                "SOD process fail! ErrIDx:%x, CIdx=%d, LineNo=%d",
                IFX_SP_ENH_ERR_CODE(ErrIdx),
                IFX_SP_ENH_ERR_COMPONENT_INDEX(ErrIdx),
                IFX_SP_ENH_ERR_LINE_NUMBER(ErrIdx));
        goto CLEAN_RETURN;
    }

    if(true == sod_check_trigger)
    {
        if (true == sod_detect_status)
        {
            *sod_status = CY_SOD_STATUS_DETECTED;
        }
        else
        {
            *sod_status = CY_SOD_STATUS_NOT_DETECTED;
        }
    }
    else
    {
        *sod_status = CY_SOD_STATUS_NOT_REQUESTED_DATA_PROCESSED;
    }

    ret_val = CY_RSLT_SUCCESS;

    CLEAN_RETURN: return ret_val;
}

cy_rslt_t cy_sod_deinit(cy_sod_t *handle)
{
    cy_rslt_t ret_val = CY_RSLT_SOD_GENERIC_ERROR;
    ifx_stc_pre_post_process_info_t *sod_info = NULL;
    sod_context_t *sod_context = NULL;

    if (false == sod_init_done)
    {
        ret_val = CY_RSLT_SOD_NOT_INITIALIZED;
        cy_sod_app_log_err(ret_val, "Not initialized");
        goto CLEAN_RETURN;
    }

    if (NULL == handle)
    {
        ret_val = CY_RSLT_SOD_BAD_ARG;
        cy_sod_app_log_info("Invalid arguments, %p", handle);
        goto CLEAN_RETURN;
    }
    sod_context = *handle;

    if(NULL == sod_context)
    {
        ret_val = CY_RSLT_SOD_BAD_ARG;
        cy_sod_app_log_info("Invalid arguments, %p", sod_context);
        goto CLEAN_RETURN;
    }

    sod_info = &sod_context->g_sod_info;

    if(NULL != sod_info->memory.persistent_mem_pt)
    {
        CY_MEM_UTIL_PRINT_ADDR("Free:sod_persistent_mem", sod_context, 0);
        free(sod_info->memory.persistent_mem_pt);
        sod_info->memory.persistent_mem_pt = NULL;
    }
    if(NULL != sod_info->memory.scratch_mem_pt)
    {
        CY_MEM_UTIL_PRINT_ADDR("Free:sod_scratch_mem", sod_context, 0);
        free(sod_info->memory.scratch_mem_pt);
        sod_info->memory.scratch_mem_pt = NULL;
    }

    CY_MEM_UTIL_PRINT_ADDR("Free:sod_handle", sod_context, 0);

    free(sod_context);
    sod_context = NULL;

    *handle = NULL;
    sod_init_done = false;

    cy_sod_app_log_info("SOD DeInit Success");
    ret_val = CY_RSLT_SUCCESS;

    CLEAN_RETURN: return ret_val;
}
