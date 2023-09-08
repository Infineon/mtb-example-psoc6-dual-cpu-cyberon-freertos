/******************************************************************************
* File Name: ifx_pre_post_process.c
*
* Description: This file contains functions to initialize and run frame by frame
*              Infineon pre and post process functions.
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
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "ifx_sp_utils_priv.h"
#include "ifx_sp_utils.h"
#include "supportFunctions_utils.h"
#include "ifx_pre_post_process.h"
#include "ifx_cycle_profiler.h"

#include <math.h>

#ifdef ENABLE_IFX_LPWWD
#include "mel_features_fix.h"
#endif

/******************************************************************************
* Global variables
*****************************************************************************/

/* IP component common fisrt five configuration parameters:
 * configurator version,
 * sampling_rate,
 * frame_size,
 * IP_compnent_id,
 * number of parameters.
 */
int32_t ifx_pre_post_process_parse(int32_t* ip_prms_config, ifx_stc_pre_post_process_info_t* ip_infoPt)
{
    int32_t* int_idx = (int32_t*)ip_prms_config;
    int32_t sampling_rate, frame_size, frame_shift, num_feature, num_coeffs, ip_id, block_size, sz;
    int32_t ret;
    mem_info_t mem_info;

    /* Sanity check of input arguments */
    if (ip_prms_config == NULL)
    {
        return IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
        /* read first five parameters which are always same definition */
    sz = *int_idx++;             /* Configurator version, no checking its validation for now */
    sampling_rate = *int_idx++;  /* sampling rate */
    frame_size = *int_idx++;     /* frame size */
    ip_id = *int_idx++;          /* IP component ID */
    sz = *int_idx++;             /* number of following parameters */
    /* Sanity check of input arguments */
    if (ip_infoPt == NULL)
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
    ip_infoPt->configurator_version = sz;
    ip_infoPt->libsp_version = IFX_SP_LPWWD_VERSION;

    if (sz < 1)
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
    }
    if (sampling_rate != 16000)
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
    }

    ip_infoPt->component_id = ip_id;
    ip_infoPt->sampling_rate = sampling_rate;
    ip_infoPt->input_frame_size = frame_size;

    if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC || ip_id == IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL)
    {
        frame_shift = *int_idx++;    /* frame shift */
        sz--;
        if (sz < 1)
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        else
        {
            num_feature = *int_idx++;    /* number of features */
            sz--;
        }
        if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC)
        {
            if (sz != 1)
            {
                return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
            }
            num_coeffs = *int_idx++;     /* number of DCT coeffs */
            sz = num_coeffs;
        }
        else
        {
            num_coeffs = 0;
            sz = num_feature;
        }
        #ifdef ENABLE_IFX_LPWWD
        if (num_feature < 0 || num_feature > MAX_FBANK_SIZE || num_coeffs > num_feature) /* num_coeffs is less than num_fbank_bins */
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        if (frame_shift > frame_size)
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        #endif

        block_size = (int32_t)pow(2, ceil((log(frame_size) / log(2)))); // Round-up to nearest power of 2.
        if ((block_size < 256) || (block_size > 1024))
        {/* Supported block_size is between 256 to 1024 */
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        ip_infoPt->fft_block_size = block_size;
        ip_infoPt->frame_shift = frame_shift;
        ip_infoPt->output_size = sz;
    }
    else if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_SOD)
    {
        if (frame_size != 160 || sz != 2)
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        /* fill zeros for parameter don't care or not applicable */
        ip_infoPt->frame_shift = 0;
        ip_infoPt->output_size = 0;
        ip_infoPt->fft_block_size = 0;
    }
    else if (ip_id == IFX_POST_PROCESS_IP_COMPONENT_HMMS)
    {
        if (sz != 3)
        {/* Only three parameters (modles) for now */
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        /* fill zeros for parameter don't care or not applicable */
        ip_infoPt->frame_shift = 0;
        ip_infoPt->output_size = 0;
        ip_infoPt->fft_block_size = 0;
    }
    else
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_FEATURE_NOT_SUPPORTED);
    }

    /* Get required memory for model configuration */
    ret = speech_utils_getMem(ip_prms_config, ip_id, &mem_info);
    if (ret != IFX_SP_ENH_SUCCESS)
    {
        return ret;
    }
    else
    {
        ip_infoPt->memory.scratch_mem = mem_info.scratch_mem;
        ip_infoPt->memory.persistent_mem = mem_info.persistent_mem;
    }

    return IFX_SP_ENH_SUCCESS;
}

int32_t ifx_pre_post_process_init(int32_t * ip_prms_config, void **ifx_container, ifx_stc_pre_post_process_info_t* ip_infoPt)
{
    int32_t* int_idx = (int32_t*)ip_prms_config;
    int32_t sampling_rate, ip_id, sz;

    #ifdef ENABLE_IFX_LPWWD
        int32_t block_size, num_coeffs, num_feature, frame_shift, frame_size;
    #endif

    #ifdef ENABLE_IFX_LPWWD
    spectrogram_top_struct *dPt;
    #endif

    /* Sanity check of input arguments */
    if (ip_prms_config == NULL)
    {
        return IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
    /* read first five parameters which are always same definition */
    sz = *int_idx++;                /* Configurator version, ignore for now */
    sampling_rate = *int_idx++;     /* sampling rate */
    #ifdef ENABLE_IFX_LPWWD
        frame_size    = *int_idx++;     /* frame size */
    #endif
    ip_id = *int_idx++;             /* IP component ID */
    sz = *int_idx++;                /* number of following parameters */
    /* Sanity check of input arguments */
    if (ifx_container == NULL || ip_infoPt == NULL)
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
    if ((ip_infoPt->memory.persistent_mem_pt == NULL) || (ip_infoPt->memory.scratch_mem_pt == NULL))
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
    if (sampling_rate != 16000)
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
    }

    if (ip_id == IFX_POST_PROCESS_IP_COMPONENT_HMMS)
    {
#ifdef ENABLE_IFX_LPWWD
        int32_t ret;
        postprocess_top_struct* pp_ptr;

        if (sz != 3)
        {/* Only three parameters (modles) for now */
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        /* Set up post processing data structure in persistent memory */
        pp_ptr = (postprocess_top_struct*)(ip_infoPt->memory.persistent_mem_pt);
        /* Initialize internal post processing structure */
        pp_ptr->fps = *int_idx++;
        pp_ptr->lookback_buffer_length = *int_idx++;
        pp_ptr->stacked_frame_delay = *int_idx++;
#if C_MODELS
        ret = fixed_init_lpwwd_post(&(pp_ptr->ifx_fixed_ppmem), pp_ptr->lookback_buffer_length, pp_ptr->stacked_frame_delay);
#else
        //int16_t *kw_model_ptr, *g_model_ptr;
        //ret = fixed_init_lpwwd_post(&(pp_ptr->ifx_fixed_ppmem), kw_model_ptr, g_model_ptr, NULL);
        //#error Fixed-point Post Process application needs set compilation switch C_MODELS!
        printf("error: Fixed-point Post Process application needs set compilation switch C_MODELS!\n");
        return -1;
#endif
        *ifx_container = pp_ptr;
        return ret;
#endif
    }
    else if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_SOD)
    {
        int32_t ret;

        ret = speech_utils_sod_init(ip_prms_config, ifx_container, &(ip_infoPt->memory));
        return ret;
    }
    else if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC)
    {/* mfcc option */
        if (sz != 3)
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        
        
        #ifdef ENABLE_IFX_LPWWD
            frame_shift = *int_idx++;
            num_coeffs = *int_idx++;
            num_feature = *int_idx++;
        #endif
    }
    else if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL)
    {/* log-mel option */
        if (sz < 2 || sz > 3)
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        #ifdef ENABLE_IFX_LPWWD
            frame_shift = *int_idx++;
            num_coeffs = 0; /* force set num_mfcc_coeffs to zero for log_ml case */
            num_feature = *int_idx++;
        #endif
    }
    else
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_FEATURE_NOT_SUPPORTED);
    }

    if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC || ip_id == IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL)
    {
        #ifdef ENABLE_IFX_LPWWD
        if (frame_shift > frame_size)
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        if (num_feature < 0 || num_feature > MAX_FBANK_SIZE || num_coeffs > num_feature) /* num_coeffs is less than num_fbank_bins */
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }

        /* Set up Spectrogram Transfomation data structure in persistent memory */
        dPt = (spectrogram_top_struct*)(ip_infoPt->memory.persistent_mem_pt);
        sz = sizeof(spectrogram_top_struct);
        memset(dPt, 0, sz);
        block_size = ip_infoPt->fft_block_size;
        if ((block_size < 256) || (block_size > 1024))
        {/* Supported block_size is between 256 to 1024 */
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        dPt->ifx_spectrogram.mel_block_size = block_size;
        dPt->ifx_spectrogram.mel_sample_rate = sampling_rate;
        dPt->ifx_spectrogram.mel_frame_size = frame_size;
        dPt->ifx_spectrogram.mel_frame_shift = frame_shift;
        dPt->ifx_spectrogram.mel_num_fbank_bins = num_feature;
        dPt->ifx_spectrogram.mel_num_mfcc_coeffs = num_coeffs;
        dPt->persistent_size = ip_infoPt->memory.persistent_mem;
        dPt->persistent_pad = ip_infoPt->memory.persistent_mem_pt;
        dPt->scratch.scratch_size = ip_infoPt->memory.scratch_mem;
        dPt->scratch.scratch_pad = ip_infoPt->memory.scratch_mem_pt;

        /* Initialize internal Spectrogram Transfomation structure */
        mel_features_fix_init(dPt, sz);

        *ifx_container = dPt;
        #endif
    }
    else
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_FEATURE_NOT_SUPPORTED);
    }

    return IFX_SP_ENH_SUCCESS;
}


int32_t ifx_spectrogram_transfer(IFX_SP_DATA_TYPE_T* in, void* spectrogram_container, IFX_FE_DATA_TYPE_T* fe_out, int32_t* out_q)
{
    #ifdef ENABLE_IFX_LPWWD
    spectrogram_top_struct* dPt;
    ifx_spectrogram_struc_t* ifxSpectrogramPt;
    ifx_scratch_mem_t* scratchPt;

    /* Sanity check of input arguments */
    if ((spectrogram_container == NULL) || (in == NULL))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_MFCC, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    dPt = (spectrogram_top_struct*)spectrogram_container;
    if (dPt->reset_flag)
    {/* one time input sample tail (zero out) reset */
    #ifdef ENABLE_IFX_LPWWD
        int size = dPt->ifx_spectrogram.mel_block_size - dPt->ifx_spectrogram.mel_frame_size;
        if (size > 0)
        {
            memset(in, 0, size * sizeof(IFX_SP_DATA_TYPE_T));
        }
        dPt->reset_flag = false;
        #endif
    }
    ifxSpectrogramPt = &(dPt->ifx_spectrogram);
    scratchPt = &(dPt->scratch);
#ifdef PROFILER
    ifx_cycle_profile_start(&(dPt->profile));
#endif
    mel_features_fix_compute(ifxSpectrogramPt, scratchPt, in, fe_out, out_q);
#ifdef PROFILER
    ifx_cycle_profile_stop(&(dPt->profile));
#endif
#endif
    return IFX_SP_ENH_SUCCESS;
}

int32_t ifx_pre_post_process_mode_control(void* container, int32_t component_id, bool enable, bool reset)
{
    int32_t ErrIdx = IFX_SP_ENH_SUCCESS;

    /* Sanity check of input arguments */
    if ((container == NULL) || (component_id >= IFX_POST_PROCESS_IP_COMPONENT_MAX_COUNT) || (component_id < IFX_PRE_POST_IP_COMPONENT_START_ID))
    {
        return IFX_SP_ENH_ERROR(component_id, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    if (reset)
    {
        if (component_id == IFX_PRE_PROCESS_IP_COMPONENT_SOD)
        {
            printf("Please use speech_utils_sod_reset() function for SOD reset!\r\n");
            ErrIdx = IFX_SP_ENH_ERROR(component_id, IFX_SP_ENH_ERR_API);
        }
        else if (component_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC || component_id == IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL)
        {/* set one time input sample tail (zero out) reset flag */
            spectrogram_top_struct* dPt;

            dPt = (spectrogram_top_struct*)container;
            dPt->reset_flag = true;
        }
        else if (component_id == IFX_POST_PROCESS_IP_COMPONENT_HMMS)
        {
#ifdef ENABLE_IFX_LPWWD
            postprocess_top_struct* pp_ptr = (postprocess_top_struct*)container;
            fixed_reset_lpwwd_post(&(pp_ptr->ifx_fixed_ppmem));
#endif
        }
        else
        {
            ErrIdx = IFX_SP_ENH_ERROR(component_id, IFX_SP_ENH_FEATURE_NOT_SUPPORTED);
        }
    }

    if (component_id == IFX_PRE_PROCESS_IP_COMPONENT_SOD)
    {/* For now, enable/disable only applicable to SOD */
        sod_top_struct* dPt;

        dPt = (sod_top_struct*)container;
        dPt->enable_flag = enable;
    }

    return ErrIdx;
}

int32_t ifx_pre_post_process_status(void* container, int32_t component_id)
{
    int32_t status = true;

    if (container == NULL)
    {/* Argument is invalid so set to false */
        status = false;
    }
    else if (component_id == IFX_PRE_PROCESS_IP_COMPONENT_SOD)
    {
        sod_top_struct* dPt;

        dPt = (sod_top_struct*)container;
        status = dPt->enable_flag;
    }
    else if (component_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC || component_id == IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL)
    {/* Feature extraction is always enabled */
        status = true;
    }
    else if (component_id == IFX_POST_PROCESS_IP_COMPONENT_HMMS)
    {/* HMMS post process is always enabled */
        status = true;
    }
    else
    {/* un-support componenet reture false */
        status = false;
    }

    return status;
}

int32_t ifx_post_process(IFX_PPINPUT_DATA_TYPE_T* in_probs, void* postprocess_container, int32_t component_id, int32_t* detection)
{
#ifdef ENABLE_IFX_LPWWD
    postprocess_top_struct* dPt;
    struct FIXED_PP_struct* ifxPP_Pt;
    //ifx_scratch_mem_t* scratchPt;

    /* Sanity check of input arguments */
    if ((postprocess_container == NULL) || (in_probs == NULL))
    {
        return IFX_SP_ENH_ERROR(IFX_POST_PROCESS_IP_COMPONENT_HMMS, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    dPt = (postprocess_top_struct*) postprocess_container;
    ifxPP_Pt = &(dPt->ifx_fixed_ppmem);
    if (dPt->reset_flag)
    {/* one time reset */
        fixed_reset_lpwwd_post(ifxPP_Pt);
        dPt->reset_flag = false;
    }
    //scratchPt = &(dPt->scratch);
#ifdef PROFILER
    ifx_cycle_profile_start(&(dPt->profile));
#endif
    *detection = fixed_lpwwd_post(ifxPP_Pt, in_probs);
#ifdef PROFILER
    ifx_cycle_profile_stop(&(dPt->profile));
#endif
#endif
    return IFX_SP_ENH_SUCCESS;

}
