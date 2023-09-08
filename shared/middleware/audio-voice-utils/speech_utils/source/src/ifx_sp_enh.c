/******************************************************************************
* File Name: ifx_sp_enh.c
*
* Description: This file contains HP speech enhancement app
*
* Related Document: See README.md
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ifx_sp_enh.h"
#include "ifx_sp_enh_priv.h"
#include "supportFunctions_utils.h"
#ifdef PROFILER
#include "ifx_cycle_profiler.h"
#endif /* PROFILER */

/******************************************************************************
* Constants and Marco
*****************************************************************************/

/*******************************************************************************
* Function Name: ifx_sp_enh_process
********************************************************************************
* Summary:
*  This function computes complete Infineon high performance speech enhancement
*  algorithms using component-by-component architecture. This function is also
*  responsible for logging, working buffer management.
*
* Parameters:
*  void *modelPt : contains model definition and parameters
*
*******************************************************************************/

int32_t ifx_sp_enh_process(void* modelPt, void* input1, void* input2, void* reference_input, void* output, void* ifx_output)
{
    IFX_SP_DATA_TYPE_T* in1_pt = (IFX_SP_DATA_TYPE_T*)input1;
    IFX_SP_DATA_TYPE_T* in2_pt = (IFX_SP_DATA_TYPE_T*)input2;
    IFX_SP_DATA_TYPE_T* ref_pt = (IFX_SP_DATA_TYPE_T*)reference_input;
    // IFX_SP_DATA_TYPE_T* out = (IFX_SP_DATA_TYPE_T*)output;
    // IFX_SP_DATA_TYPE_T* ifx_out = (IFX_SP_DATA_TYPE_T*)ifx_output;
    IFX_SP_DATA_TYPE_T* ref_analysis_out_pt = NULL;
    IFX_SP_DATA_TYPE_T* analysis_out1_pt = NULL;
    IFX_SP_DATA_TYPE_T* analysis_out2_pt = NULL;
    IFX_SP_DATA_TYPE_T* asa_out_pt = NULL;
    IFX_SP_DATA_TYPE_T* bf_out_pt = NULL;
    IFX_SP_DATA_TYPE_T* esns_out_pt = NULL;

    sp_enh_struct* dPt;
    dPt = (sp_enh_struct*)modelPt;
    component_struct_t* lPt;
    ifx_scratch_mem_t* scratchPt;

    int32_t status, byte_size, scratch_size = 0;

#ifdef PROFILER
    ifx_cycle_profile_start(&(dPt->profile));
#endif

    /* Sanity checking of input arguments */
    if (modelPt == NULL || input1 == NULL || output == NULL || ifx_output == NULL)
    {
        return IFX_SP_ENH_ERROR(IFX_SP_ENH_COMPONENT_ID_INVALID, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
    }

    /* Scratch memory structure pointer */
    scratchPt = &dPt->scratch;
    /* Allocate scratch memory for various imtermediate output buffer */
    byte_size = dPt->fft_size * sizeof(int16_t);
    scratch_size += byte_size;
    analysis_out1_pt = ifx_mem_allocate(scratchPt, byte_size);
    if (input2)
    {
        scratch_size += byte_size;
        analysis_out2_pt = ifx_mem_allocate(scratchPt, byte_size);
    }

    if (reference_input != NULL)
    {/* There is valid eacho reference */
        /* Allocate echo reference output buffer */
        scratch_size += byte_size;
        ref_analysis_out_pt = ifx_mem_allocate(scratchPt, byte_size);
    }

    for (dPt->component = 0; dPt->component < dPt->n_components; dPt->component++)
    {
        lPt = &dPt->l[dPt->component];
#ifdef PROFILER
        ifx_cycle_profile_start(&(lPt->profile));
#endif
        /* Call the process function for each component */
        switch (lPt->component_typeb)
        {
        case IFX_SP_ENH_IP_COMPONENT_HPF:
            {/* HPF may have two mics as input and should be the 1st component */
                if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_HPF))
                {
                    status = hpf_process(dPt, in1_pt, in1_pt); /* output pointer is same as input */
                    if (status != IFX_SP_ENH_SUCCESS)
                    {/* Handle failure from component function */
                        return status;
                    }
                    if (input2)
                    {/* There are two mics */
                        if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_HPF))
                        {
                            status = hpf_process(dPt, in2_pt, in2_pt); /* output pointer is same as input */
                            if (status != IFX_SP_ENH_SUCCESS)
                            {/* Handle failure from component function */
                                return status;
                            }
                        }
                    }
                }
                if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_ANALYSIS))
                {
                    status = analysis_process(modelPt, in1_pt, analysis_out1_pt);
                    if (status != IFX_SP_ENH_SUCCESS)
                    {/* Handle failure from component function */
                        return status;
                    }
                    if (input2)
                    {/* There are two mics */
                        status = analysis_process(modelPt, in2_pt, analysis_out2_pt);
                        if (status != IFX_SP_ENH_SUCCESS)
                        {/* Handle failure from component function */
                            return status;
                        }
                    }
                    else
                    {
                        analysis_out2_pt = NULL;
                    }
                }
                else
                {/* By pass rest of process and rest components should be disabled */
                    output = in1_pt;    /* input2 ignore */
                }
                break;
            }
        case IFX_SP_ENH_IP_COMPONENT_AEC:
            {/* AEC may have two mics */
                if (reference_input)
                {
                    status = analysis_process(modelPt, ref_pt, ref_analysis_out_pt);
                    if (status != IFX_SP_ENH_SUCCESS)
                    {/* Handle failure from component function */
                        return status;
                    }

                }
                else
                {
                    return IFX_SP_ENH_ERROR(IFX_SP_ENH_IP_COMPONENT_AEC, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
                }
                if (COMPONENT_RESET_FLAG(dPt->reset_flag, IFX_SP_ENH_IP_COMPONENT_AEC))
                {
                    // Add code to rest AEC state
                }
                if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_AEC))
                {
                    status = aec_process(dPt, analysis_out1_pt, ref_analysis_out_pt, analysis_out1_pt); /* Output overwrite input */
                    if (status != IFX_SP_ENH_SUCCESS)
                    {/* Handle failure from component function */
                        return status;
                    }
                    if (input2)
                    {/* There are two mics */
                        status = aec_process(dPt, analysis_out2_pt, ref_analysis_out_pt, analysis_out2_pt); /* Output overwrite input */
                        if (status != IFX_SP_ENH_SUCCESS)
                        {/* Handle failure from component function */
                            return status;
                        }
                    }
                }
                break;
            }
        case IFX_SP_ENH_IP_COMPONENT_BF:
            {/* must two mics and two or more beams */
                if (input2 == NULL || dPt->num_beams < 2)
                {
                    return IFX_SP_ENH_ERROR(IFX_SP_ENH_IP_COMPONENT_BF, IFX_SP_ENH_ERR_PARAM);
                }
                if (COMPONENT_RESET_FLAG(dPt->reset_flag, IFX_SP_ENH_IP_COMPONENT_BF))
                {
                    // Add code to rest BF state
                }
                if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_BF))
                {
                    if (asa_out_pt == NULL)
                    {/* Allocate scratch memory if it is not done yet */
                        scratch_size += byte_size; /* Need check if the size is correct! */
                        asa_out_pt = ifx_mem_allocate(scratchPt, byte_size);
                    }
                    if (bf_out_pt == NULL)
                    {/* Allocate scratch memory if it is not done yet */
                        scratch_size += byte_size; /* Need check if the size is correct! */
                        bf_out_pt = ifx_mem_allocate(scratchPt, byte_size);
                    }
                    status = asa_process(dPt, analysis_out1_pt, analysis_out2_pt, asa_out_pt);
                    if (status != IFX_SP_ENH_SUCCESS)
                    {/* Handle failure from component function */    
                        return status;
                    }
                    status = bf_process(dPt, analysis_out1_pt, analysis_out2_pt, asa_out_pt, bf_out_pt);
                    if (status != IFX_SP_ENH_SUCCESS)
                    {/* Handle failure from component function */
                        return status;
                    }
                    if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_DEREVERB))
                    {
                        status = dereverb_process(dPt, bf_out_pt, asa_out_pt, bf_out_pt); /* Output overwrite input */
                        if (status != IFX_SP_ENH_SUCCESS)
                        {/* Handle failure from component function */
                            return status;
                        }
                    }
                }
                else
                {/* BF disabled and only take input1 */
                    bf_out_pt = analysis_out1_pt;
                    asa_out_pt = NULL;
                }
                break;
            }
        case IFX_SP_ENH_IP_COMPONENT_ESNS:
            {
                if (lPt->component_typeb == IFX_SP_ENH_IP_COMPONENT_ES && reference_input == NULL)
                {
                    return IFX_SP_ENH_ERROR(IFX_SP_ENH_IP_COMPONENT_AEC, IFX_SP_ENH_ERR_INVALID_ARGUMENT);
                }
                if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_ESNS))
                {
                    esns_out_pt = bf_out_pt; /* Output overwrite input */
                    status = esns_process(dPt, bf_out_pt, asa_out_pt, ref_analysis_out_pt, esns_out_pt);
                    if (status != IFX_SP_ENH_SUCCESS)
                    {/* Handle failure from component function */
                        return status;
                    }
                    ifx_output = esns_out_pt;
                    if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_SYNTHESIS))
                    {
                        synthesis_process(dPt, esns_out_pt, output);
                    }
                    else
                    {
                        output = esns_out_pt;
                    }
                }
                else
                {
                    ifx_output = bf_out_pt;
                    if (COMPONENT_ENABLE_FLAG(dPt->enable_flag, IFX_SP_ENH_IP_COMPONENT_SYNTHESIS))
                    {
                        synthesis_process(dPt, bf_out_pt, output);
                    }
                    else
                    {
                        output = bf_out_pt;
                    }
                }
                break;
            }
        default:
            return IFX_SP_ENH_ERROR(lPt->component_typeb, IFX_SP_ENH_FEATURE_NOT_SUPPORTED);
            break;
        }
#ifdef PROFILER
        ifx_cycle_profile_stop(&(lPt->profile));
        if (dPt->profile.profile_config & IFX_SP_ENH_PROFILE_FRAME)
        {
            ifx_profile_per_frame_print(lPt->profile);
        }
#endif
    }
    
    /* Free scratch memory */
    ifx_mem_free(scratchPt, scratch_size);
    /* check if all scratch memory is freed */
    assert(ifx_mem_counter(scratchPt) == 0);
    ifx_mem_reset(scratchPt);
#ifdef PROFILER
    ifx_cycle_profile_stop(&(dPt->profile));
    if (dPt->profile.profile_config & IFX_SP_ENH_PROFILE_FRAME)
    {
        ifx_profile_per_frame_print(dPt->profile);
    }
#endif

    return IFX_SP_ENH_SUCCESS;
}

/*******************************************************************************
* Function Name: ifx_sp_enh_mode_control
********************************************************************************
* Summary:
*  This function controls Infineon high performance speech enhancement IP 
*  component including update its parameter.
*******************************************************************************/

int32_t ifx_sp_enh_mode_control(void* modelPt, ifx_sp_enh_ip_component_config_t component_id, bool enable, bool reset, bool update, int32_t value)
{
    sp_enh_struct* dPt;
    dPt = (sp_enh_struct*)modelPt;
    component_struct_t* lPt;
    ifx_sp_enh_ip_component_config_t block_id = IFX_SP_ENH_IP_COMPONENT_HPF;

    if (enable)
    {
        dPt->enable_flag |= (1 << component_id);
    }
    else
    {
        dPt->enable_flag &= ~(1 << component_id);
    }

    if (reset)
    {
        dPt->reset_flag |= (1 << component_id);
    }
    else
    {
        dPt->reset_flag &= ~(1 << component_id);
    }

    if (update)
    {
        if (component_id == IFX_SP_ENH_IP_COMPONENT_DEREVERB)
        {
            block_id = IFX_SP_ENH_IP_COMPONENT_BF;
        }
        if (component_id == IFX_SP_ENH_IP_COMPONENT_ES || component_id == IFX_SP_ENH_IP_COMPONENT_NS)
        {
            block_id = IFX_SP_ENH_IP_COMPONENT_ESNS;
        }
        for (int i = 0; i < dPt->n_components; i++)
        {
            lPt = &dPt->l[i];
            if (lPt->component_typeb == block_id)
            {
                switch (component_id)
                {
                case IFX_SP_ENH_IP_COMPONENT_HPF:
                {
                    hpf_struct *hpf_pt = (hpf_struct *)lPt->xxComponent;
                    hpf_pt->gain = value;
                }
                break;
                case IFX_SP_ENH_IP_COMPONENT_AEC:
                {
                    aec_struct *aec_pt = (aec_struct *)lPt->xxComponent;
                    aec_pt->bulk_delay = value;
                }
                break;
                case IFX_SP_ENH_IP_COMPONENT_BF:
                {
                    bf_struct *bf_pt = (bf_struct *)lPt->xxComponent;
                    bf_pt->agressiveness = value;
                }
                break;
                case IFX_SP_ENH_IP_COMPONENT_DEREVERB:
                {
                    // dereverb_struct *der_pt = (dereverb_struct *)lPt->xxComponent;

                }
                break;
                case IFX_SP_ENH_IP_COMPONENT_ES:
                {

                }
                break;
                case IFX_SP_ENH_IP_COMPONENT_NS:
                {

                }
                break;
                default: /* Other cases do nothing */
                break;
                }
                break;   
            }
        }
    }
    return IFX_SP_ENH_SUCCESS;
}

/* Temporary empty function to make it build */
int32_t ifx_sp_enh_model_parse(char* fn_prms, ifx_stc_sp_enh_info_t* mdl_infoPt)
{
    mdl_infoPt->libsp_version = IFX_SP_ENH_VERSION;
    mdl_infoPt->configurator_version = 0;  /* temporary to set random value */
    mdl_infoPt->memory.persistent_mem = sizeof(sp_enh_struct) + 9 * sizeof(component_struct_t) + 4; /* temporary to set random value */
    mdl_infoPt->memory.scratch_mem = 4;  /* temporary to set random value */
    mdl_infoPt->sampling_rate = 16000;
    mdl_infoPt->num_of_mic = 2;
    mdl_infoPt->num_of_beam = 2;
    mdl_infoPt->aec_config = 0;
    mdl_infoPt->num_of_components = 0; /* temporary to set random value */
    mdl_infoPt->input_frame_size = 160; /* temporary to set random value */
    mdl_infoPt->output_size = 4; /* temporary to set random value */
    mdl_infoPt->ifx_output_size = 4; /* temporary to set random value */
    return 0;
}

int32_t ifx_sp_enh_init(void** dPt_container, char* fn_prms
    , char* persistent_mem, char* scratch_mem, ifx_stc_sp_enh_info_t* mdl_infoPt)
{
    sp_enh_struct *dPt;

    /* Set up data structure in persistent memory */
    dPt = (sp_enh_struct*) persistent_mem;
    /* temporary to set random value */
    dPt->n_components = mdl_infoPt->num_of_components;
    dPt->num_beams = mdl_infoPt->num_of_beam;

    *dPt_container = dPt;
    return 0;
}

int hpf_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* out)
{
    return 0;
}

int analysis_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* out)
{
    return 0;
}

int aec_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* ref_in, IFX_SP_DATA_TYPE_T* out)
{
    return 0;
}

int asa_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in1, IFX_SP_DATA_TYPE_T* in2, IFX_SP_DATA_TYPE_T* out)
{
    return 0;
}

int bf_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in1, IFX_SP_DATA_TYPE_T* in2, IFX_SP_DATA_TYPE_T* in_asa, IFX_SP_DATA_TYPE_T* out)
{
    return 0;
}

int dereverb_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* in_asa, IFX_SP_DATA_TYPE_T* out)
{
    return 0;
}

int esns_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* asa_in, IFX_SP_DATA_TYPE_T* ref_in, IFX_SP_DATA_TYPE_T* out)
{
    return 0;
}

int synthesis_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* out)
{
    return 0;
}
