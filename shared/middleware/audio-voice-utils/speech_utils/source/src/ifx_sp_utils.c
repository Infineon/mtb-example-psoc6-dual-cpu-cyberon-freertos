/******************************************************************************
* File Name: ifx_sp_utils.c
*
* Description: This file contains functions to get memory and initialize
*      Infineon LPWWD IP component. Specifically, functions calculate memory
*      need, initializes container and setup working/state memories.
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
#include "ifx_cycle_profiler.h"
#include "supportFunctions_utils.h"
#include "ifx_pre_post_process.h"
//#include "mel_features.h"

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

int32_t speech_utils_getMem(int32_t* ip_prms_buffer, int32_t ip_id, mem_info_t* mem_infoPt)
{
    int32_t sampling_rate, frame_size, sz;
    int32_t *int_idx = (int32_t *)ip_prms_buffer;
    int32_t persistent_sz = 0;
    int32_t scratch_sz = 0;

    /* Sanity check of input arguments */
    if ((ip_id < IFX_PRE_PROCESS_IP_COMPONENT_SOD) || (ip_id > IFX_POST_PROCESS_IP_COMPONENT_DECISION_TREE))
    {
        return IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
    if (ip_prms_buffer == NULL || mem_infoPt == NULL)
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    /* read first five parameters which are always same definition */
    sz = *int_idx++;             /* Configurator version, ignore for now */
    sampling_rate = *int_idx++;  /* sampling rate */
    frame_size = *int_idx++;     /* frame size */
    sz = *int_idx++;             /* IP component ID */
    if (sz != ip_id)
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
    }
    sz = *int_idx++;             /* number of following parameters */

    /* Adjust persistent & scratch memory sizes based on data struct & type */
    if (ip_id == IFX_POST_PROCESS_IP_COMPONENT_HMMS)
    {
#ifdef ENABLE_IFX_LPWWD
        if (sampling_rate != 16000)
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }

        persistent_sz = ALIGN_WORD(sizeof(postprocess_top_struct));
        scratch_sz = ALIGN_WORD(1);     /* Avoid zero */
#endif
    }
    else if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_SOD)
    {
        if ((sampling_rate != 16000) || (frame_size != 160))
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }

        persistent_sz = ALIGN_WORD(sizeof(sod_top_struct));
        scratch_sz = ALIGN_WORD(((frame_size) + D_ORDER) * sizeof(int16_t)) + ALIGN_WORD(((frame_size >> 1) + D_ORDER) * sizeof(int16_t)) + ALIGN_WORD(((frame_size >> 2) + D_ORDER) * sizeof(int16_t));
    }
    else if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC || ip_id == IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL)
    {
        #ifdef ENABLE_IFX_LPWWD
        int32_t num_mel_features, num_mfcc_coeffs, block_size;
        if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC)
        {
            if (sz != 3)
            {
                return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
            }
        }
        else if (sz < 2)
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        int_idx++;  /* skip frame_shift */
        num_mel_features = *int_idx++;
        num_mfcc_coeffs = *int_idx++;
        block_size = (int32_t)pow(2, ceil((log(frame_size) / log(2)))); // Round-up to nearest power of 2.
        if ((sampling_rate != 16000) || (num_mel_features < MIN_FBANK_SIZE) || (num_mel_features > MAX_FBANK_SIZE) || (block_size < 64) || (block_size > 1024))
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        if ((ip_id == IFX_PRE_PROCESS_IP_COMPONENT_MFCC) && (num_mfcc_coeffs > num_mel_features))
        {
            return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_ERR_PARAM);
        }
        if (ip_id == IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL)
        {/* force set num_mfcc_coeffs to zero for log_ml case */
            num_mfcc_coeffs = 0;
        }
        persistent_sz = spectrogram_calculate_persistent_mem_size(sampling_rate, block_size, num_mel_features, num_mfcc_coeffs);
        persistent_sz += ALIGN_WORD(sizeof(spectrogram_top_struct));
        scratch_sz = spectrogram_calculate_scratch_mem_size(block_size, num_mel_features);
        #endif
    }
    else
    {
        return IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_FEATURE_NOT_SUPPORTED);
    }

    memset(mem_infoPt, 0, sizeof(*mem_infoPt));
    mem_infoPt->scratch_mem = scratch_sz;
    mem_infoPt->persistent_mem = persistent_sz;

    return IFX_SP_ENH_SUCCESS;
}

/* SOD internal configuration parameters:
 *     GapSetting (0,100,200,300,400,500,1000)ms, SensLevel (0-32767)]
 * Preprocess internal configuration parameters:
 */
int32_t speech_utils_sod_init(int32_t * sod_prms_buffer, void **sod_container, mem_info_t *sod_mem_infoPt)
{
    int32_t* int_idx = (int32_t*)sod_prms_buffer;
    int32_t sampling_rate, frame_size, gapsetting, senslevel, sz;
    sod_top_struct *dPt;

    /* Sanity check of input arguments */
    if ((sod_container == NULL) || (sod_prms_buffer == NULL) || (sod_mem_infoPt == NULL))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    // read parameters
    sz = *int_idx++;                /* Configurator version, ignore for now */
    sampling_rate = *int_idx++;     /* sampling rate */
    frame_size = *int_idx++;        /* frame size */
    sz = *int_idx++;                /* IP component ID */
    if (sz != IFX_PRE_PROCESS_IP_COMPONENT_SOD)
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }
    sz = *int_idx++;                /* number of following parameters */
    if (sz != 2)
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }
    gapsetting    = *int_idx++;     /* gap setting */
    senslevel     = *int_idx;       /* sensitivity setting */
    if ((sampling_rate != 16000) || (frame_size != 160))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }
    if ((senslevel<0)||(senslevel>32767))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }
    if ((gapsetting!=0)&&(gapsetting!=100)&&(gapsetting!=200)&&(gapsetting!=300)&&(gapsetting!=400)&&(gapsetting!=500)&&(gapsetting!=1000))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }   
    
    if ((sod_mem_infoPt->persistent_mem_pt == NULL) || (sod_mem_infoPt->scratch_mem_pt == NULL))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    if ((sod_mem_infoPt->persistent_mem != ALIGN_WORD(sizeof(sod_top_struct))) ||
        (sod_mem_infoPt->scratch_mem != ALIGN_WORD(((frame_size) + D_ORDER) * sizeof(int16_t)) + ALIGN_WORD(((frame_size >> 1) + D_ORDER) * sizeof(int16_t)) + ALIGN_WORD(((frame_size >> 2) + D_ORDER) * sizeof(int16_t))))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    /* Set up SOD data structure in persistent memory */
    dPt = (sod_top_struct*)(sod_mem_infoPt->persistent_mem_pt);
    memset(dPt, 0, sizeof(sod_top_struct));

    dPt->persistent_size = sod_mem_infoPt->persistent_mem;
    dPt->persistent_pad = sod_mem_infoPt->persistent_mem_pt;
    dPt->scratch.scratch_size = sod_mem_infoPt->scratch_mem;
    dPt->scratch.scratch_pad = sod_mem_infoPt->scratch_mem_pt;
    dPt->enable_flag = true; /* enable SOD detection */

    /* Initialize internal SOD structure */
    initSOD(&(dPt->ifx_sod_comp), (int16_t)gapsetting, (int16_t)senslevel);

    *sod_container = dPt;
    return IFX_SP_ENH_SUCCESS;
}

int32_t speech_utils_sod_reset(int32_t * sod_prms_buffer, void *sod_container)
{
    int32_t* int_idx = (int32_t*)sod_prms_buffer;
    int32_t sampling_rate, frame_size, gapsetting, senslevel, sz;
    sod_top_struct* dPt;
    struct IFX_SOD_STRUCT* ifx_sod_compPt;
    
    /* Sanity check of input arguments */
    if ((sod_container == NULL) || (int_idx == NULL))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
        
     // read parameters
    sz = *int_idx++;                /* Configurator version, ignore for now */
    sampling_rate = *int_idx++;     /* sampling rate */
    frame_size = *int_idx++;        /* frame size */
    sz = *int_idx++;                /* IP component ID */
    if (sz != IFX_PRE_PROCESS_IP_COMPONENT_SOD)
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }
    sz = *int_idx++;                /* number of following parameters */
    if (sz != 2)
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }
    gapsetting = *int_idx++;     /* gap setting */
    senslevel = *int_idx;       /* sensitivity setting */
    if ((sampling_rate != 16000) || (frame_size != 160))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }
    if ((senslevel<0)||(senslevel>32767))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }
    if ((gapsetting!=0)&&(gapsetting!=100)&&(gapsetting!=200)&&(gapsetting!=300)&&(gapsetting!=400)&&(gapsetting!=500)&&(gapsetting!=1000))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_PARAM);
    }       
    
    dPt = (sod_top_struct*)sod_container;
    dPt->enable_flag = true; /* enable SOD detection */
    ifx_sod_compPt = &(dPt->ifx_sod_comp);
  
    /* Initialize internal SOD structure */
    initSOD(ifx_sod_compPt, (int16_t)gapsetting, (int16_t)senslevel);
    
    return IFX_SP_ENH_SUCCESS;      
}


int32_t speech_utils_sod_process(IFX_SP_DATA_TYPE_T *in, void *sod_container, bool *vad)
{
    sod_top_struct* dPt;
    struct IFX_SOD_STRUCT* ifx_sod_compPt;
    ifx_scratch_mem_t* scratchPt;
    bool TrigSet;

    /* Sanity check of input arguments */
    if ((sod_container == NULL) || (in == NULL))
    {
        return IFX_SP_ENH_ERROR(IFX_PRE_PROCESS_IP_COMPONENT_SOD, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    dPt = (sod_top_struct*)sod_container;
    TrigSet = dPt->enable_flag;
    ifx_sod_compPt = &(dPt->ifx_sod_comp);
    scratchPt = &(dPt->scratch);
#ifdef PROFILER
    ifx_cycle_profile_start(&(dPt->profile));
#endif
    *vad = SOD(in, ifx_sod_compPt, scratchPt)&TrigSet;
#ifdef PROFILER
    ifx_cycle_profile_stop(&(dPt->profile));
#endif

    return IFX_SP_ENH_SUCCESS;
}

