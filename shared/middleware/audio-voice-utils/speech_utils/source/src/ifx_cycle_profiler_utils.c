/******************************************************************************
* File Name: ifx_cycle_profiler_utl.c
*
* Description: This file contains Infineon cycle profiler related functions
*
*
*******************************************************************************
* (c) 2021, Infineon Technologies Company. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Infineon Technologies Company or one of its
* subsidiaries ("Infineon") and is protected by and subject to worldwide patent
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
* indemnity Infineon against all liability.
*******************************************************************************/
/******************************************************************************
* Include header file
******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include "ifx_sp_common.h"
#include "ifx_sp_common_priv.h"
#include "ifx_sp_enh_priv.h"
#include "ifx_sp_utils_priv.h"
#include "ifx_pre_post_process.h"
#include "ifx_cycle_profiler.h"

/******************************************************************************
* Global variables
*****************************************************************************/
#ifdef PROFILER
const char * ip_component_string[IFX_POST_PROCESS_IP_COMPONENT_MAX_COUNT] =
{
    "HPF",
    "AEC",
    "BF",
    "ESNS",
    "DEREVERB",
    "NS",
    "ES",
    "ANALYSIS",
    "SYNTHESIS",
    "SOD",
    "MIN_MAX",
    "Z_SCORE",
    "IIR",
    "FIR",
    "LOG_MEL",
    "MFCC",
    "TIME_AVG",
    "POST_PROCESS_HMMS",
    "DECISION_TREE"
};

static inline void profiler_log_print(ifx_profile_log_t *dPt, int32_t type)
{
    dPt->log_buf[PROFILE_MAX_LOG_SIZE] = '\0';
    if (dPt->log_cb)
    {
        dPt->log_cb(dPt->log_arg, dPt->log_buf, type);
    }
    else
    {
        printf("%s\r\n", dPt->log_buf);
    }
}
#endif /* PROFILER */

void cycle_profiler_init(ifx_cycle_profile_t* lPt)
{
#ifdef PROFILER 
        lPt->sum_frames = 0;
        lPt->sum_cycles = 0;
        lPt->peak_frame = 0;
        lPt->peak_cycles = 0;
#endif
}

int32_t ifx_sp_enh_profile_init(void* modelPt, int32_t config, ifx_sp_profile_cb_fun cb_func, void* cb_arg)
{
    sp_enh_struct *dPt = (sp_enh_struct*) modelPt;
    uint8_t app_enable; 
    #ifdef PROFILER
        uint8_t comp_enable;
    #endif
    /* Sanity check of input argument */
    if (modelPt == NULL)
    {
        return  IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    /* Check if configuration setting is supported */
    switch (config)
    {
        case IFX_SP_ENH_PROFILE_DISABLE:
        {
            app_enable = 0;
            #ifdef PROFILER
                comp_enable = 0;
            #endif
            break;
        }
        case IFX_SP_ENH_PROFILE_ENABLE_APP:
        case IFX_SP_ENH_PROFILE_ENABLE_APP_PER_FRAME:
        {
            app_enable = config;
            #ifdef PROFILER
                comp_enable = 0;
            #endif
            break;
        }
        case IFX_SP_ENH_PROFILE_ENABLE_COMPONENT:
        case IFX_SP_ENH_PROFILE_ENABLE_COMPONENT_PER_FRAME:
        {
            app_enable = 0;
            #ifdef PROFILER
                comp_enable = config;
            #endif
            break;
            break;
        }
        default:
        {
             return IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
        }
    }

    dPt->profile.profile_config = app_enable;
#ifdef PROFILER
    profile_log.log_cb = cb_func;
    profile_log.log_arg = cb_arg;
    /* Initialize internal working bufffer */
    cycle_profiler_init(&(dPt->profile));
    for (int i = 0; i < dPt->n_components; i++)
    {
        component_struct_t* lPt;

        lPt = &dPt->l[i];
        lPt->profile.profile_config = comp_enable;
        cycle_profiler_init(&(lPt->profile));
    }
#endif
    return IFX_SP_ENH_SUCCESS;
}

int32_t ifx_pre_post_profile_init(void* modelPt, int32_t ip_id, uint8_t enable, ifx_sp_profile_cb_fun cb_func, void* cb_arg)
{
    /* Sanity check of input argument */
    if (modelPt == NULL)
    {
        return  IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    switch (ip_id)
    {
    case IFX_PRE_PROCESS_IP_COMPONENT_SOD:
    {
        sod_top_struct* dPt = (sod_top_struct*)modelPt;

        dPt->profile.profile_config = enable;
        cycle_profiler_init(&(dPt->profile));
        break;
    }
    case IFX_PRE_PROCESS_IP_COMPONENT_MFCC:
    case IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL:
    {/* MFCC and LOG_MEL has common data structure */
        spectrogram_top_struct* dPt = (spectrogram_top_struct *) modelPt;

        dPt->profile.profile_config = enable;
        cycle_profiler_init(&(dPt->profile));
        break;
    }
#ifdef ENABLE_IFX_LPWWD
    case IFX_POST_PROCESS_IP_COMPONENT_HMMS:
    {
        postprocess_top_struct* dPt = (postprocess_top_struct *) modelPt;

        dPt->profile.profile_config = enable;
        cycle_profiler_init(&(dPt->profile));
        break;
    }
#endif
    default:
    {
        return IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
    }
#ifdef PROFILER
    profile_log.log_cb = cb_func;
    profile_log.log_arg = cb_arg;
#endif
    return IFX_SP_ENH_SUCCESS;
}

int32_t ifx_pre_post_profile_control(void* modelPt, int32_t ip_id, bool enable)
{
    /* Sanity check of input argument */
    if (modelPt == NULL)
    {
        return  IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    switch (ip_id)
    {
    case IFX_PRE_PROCESS_IP_COMPONENT_SOD:
    {
        sod_top_struct* dPt = (sod_top_struct*)modelPt;

        dPt->profile.profile_config = enable;
        cycle_profiler_init(&(dPt->profile));
        break;
    }
    case IFX_PRE_PROCESS_IP_COMPONENT_MFCC:
    case IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL:
    {/* MFCC and LOG_MEL has common data structure */
        spectrogram_top_struct* dPt = (spectrogram_top_struct*)modelPt;

        dPt->profile.profile_config = enable;
        cycle_profiler_init(&(dPt->profile));
        break;
    }
#ifdef ENABLE_IFX_LPWWD
    case IFX_POST_PROCESS_IP_COMPONENT_HMMS:
    {
        postprocess_top_struct* dPt = (postprocess_top_struct*)modelPt;

        dPt->profile.profile_config = enable;
        cycle_profiler_init(&(dPt->profile));
        break;
    }
#endif
    default:
    {
        return IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
    }

    return IFX_SP_ENH_SUCCESS;
}

int32_t ifx_sp_enh_profile_control(void* modelPt, int32_t config)
{
    sp_enh_struct *dPt = (sp_enh_struct*) modelPt;
    uint8_t app_enable, comp_enable;

    /* Sanity check of input argument */
    if (modelPt == NULL)
    {
        return  IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    /* Check if configuration setting is supported */
    switch (config)
    {
        case IFX_SP_ENH_PROFILE_DISABLE:
        {
            app_enable = 0;
            comp_enable = 0;
            break;
        }
        case IFX_SP_ENH_PROFILE_ENABLE_APP:
        case IFX_SP_ENH_PROFILE_ENABLE_APP_PER_FRAME:
        {
            app_enable = config;
            comp_enable = 0;
            break;
        }
        case IFX_SP_ENH_PROFILE_ENABLE_COMPONENT:
        case IFX_SP_ENH_PROFILE_ENABLE_COMPONENT_PER_FRAME:
        {
            app_enable = 0;
            comp_enable = config;
            break;
        }
        default:
        {
             return IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
        }
    }

    dPt->profile.profile_config = app_enable;

    /* Reset/initialize internal working bufffer */
    cycle_profiler_init(&(dPt->profile));
    for (int i = 0; i < dPt->n_components; i++)
    {
        component_struct_t* lPt;

        lPt = &dPt->l[i];
        lPt->profile.profile_config = comp_enable;
        cycle_profiler_init(&(lPt->profile));
    }

    return IFX_SP_ENH_SUCCESS;
}

int32_t ifx_sp_enh_profile_print(void* modelPt)
{
    #ifdef PROFILER
        sp_enh_struct *dPt = (sp_enh_struct *) modelPt;
    #endif

    /* Sanity check of input argument */
    if (modelPt == NULL)
    {
        /* Do nothing */
        return  IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
#ifdef PROFILER
    if (dPt->n_components)
    {
        component_struct_t *lPt = &dPt->l[0];
        float total_avg = 0;

        if (lPt->profile.profile_config)
        {
            for (int j = 0; j < dPt->n_components; j++)
            {
                lPt = &dPt->l[j];
                float avg_cycle = (float)lPt->profile.sum_cycles / lPt->profile.sum_frames;

                total_avg += avg_cycle;

                snprintf(profile_log.log_buf, PROFILE_MAX_LOG_SIZE,
                    "PROFILE_INFO, IP component: %3d - %s profile, avg_cyc=%-10.2f, "
                    "peak_cyc=%-"PRIu32", peak_frame=%-"PRIu32"",
                    j,
                    ip_component_string[lPt->component_typeb],
                    avg_cycle,
                    lPt->profile.peak_cycles,
                    lPt->profile.peak_frame
                );
                profiler_log_print(&profile_log, IFX_SP_ENH_PROFILE_ENABLE_COMPONENT);
            }
            snprintf(profile_log.log_buf, PROFILE_MAX_LOG_SIZE,
                "PROFILE_INFO,  total sum of all layer average cycle per frame=%-10.2f",
                total_avg);
            profiler_log_print(&profile_log, IFX_SP_ENH_PROFILE_ENABLE_COMPONENT);
        }
    }

    if (dPt->profile.profile_config)
    {
        snprintf(profile_log.log_buf, PROFILE_MAX_LOG_SIZE,
                 "PROFILE_INFO, HP SE profile, avg_cyc=%-10.2f, peak_cyc=%-"PRIu32", peak_frame=%-"PRIu32"",
                 (float)dPt->profile.sum_cycles / dPt->profile.sum_frames,
                 dPt->profile.peak_cycles,
                 dPt->profile.peak_frame
                );
        profiler_log_print(&profile_log, IFX_SP_ENH_PROFILE_ENABLE_APP);
    }
#endif  /* ifdef PROFILER */
    return IFX_SP_ENH_SUCCESS;
}

int32_t ifx_pre_post_profile_print(void* Pt, int32_t ip_id)
{
    /* Sanity check of input argument */
    if (Pt == NULL)
    {/* Do nothing */
        return  IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
    if (ip_id >= IFX_POST_PROCESS_IP_COMPONENT_MAX_COUNT || ip_id < IFX_PRE_PROCESS_IP_COMPONENT_SOD)
    {/* Do nothing */
        return  IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }
#ifdef PROFILER
    ifx_cycle_profile_t lPt;

    switch (ip_id)
    {
    case IFX_PRE_PROCESS_IP_COMPONENT_SOD:
    {
        lPt = ((sod_top_struct *) Pt)->profile;
        break;
    }
    case IFX_PRE_PROCESS_IP_COMPONENT_MFCC:
    case IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL:
    {
        lPt = ((spectrogram_top_struct *) Pt)->profile;
        break;
    }
    case IFX_POST_PROCESS_IP_COMPONENT_HMMS:
    {
        lPt = ((postprocess_top_struct*)Pt)->profile;
        break;
    }
    default:
        return  IFX_SP_ENH_ERROR(ip_id, IFX_SP_ENH_FEATURE_NOT_SUPPORTED);
        break;
    }
    if (lPt.profile_config)
    {
        snprintf(profile_log.log_buf, PROFILE_MAX_LOG_SIZE,
            "PROFILE_INFO, IP component: %s, avg_cyc=%-10.2f, peak_cyc=%-"PRIu32", peak_frame=%-"PRIu32"",
            ip_component_string[ip_id],
            (float)lPt.sum_cycles / lPt.sum_frames,
            lPt.peak_cycles,
            lPt.peak_frame
        );
        profiler_log_print(&profile_log, IFX_SP_ENH_PROFILE_ENABLE_COMPONENT);
    }
#endif  /* ifdef PROFILER */
    return IFX_SP_ENH_SUCCESS;
}

#ifdef PROFILER 
void ifx_profile_per_frame_print(ifx_cycle_profile_t dPt)
{
    snprintf(profile_log.log_buf, PROFILE_MAX_LOG_SIZE,
             "Frame %"PRId32" : %"PRIu32" cycles; Total cycles : %10.2f",
             dPt.sum_frames - 1,
             dPt.cycles,
             (float)dPt.sum_cycles
            );
    profiler_log_print(&profile_log, IFX_SP_ENH_PROFILE_FRAME);
}
#endif

