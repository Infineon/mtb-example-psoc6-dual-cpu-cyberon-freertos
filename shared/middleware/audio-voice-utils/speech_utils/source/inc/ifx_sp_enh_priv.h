/******************************************************************************
* File Name: ifx_sp_enh_priv.h
*
* Description: This file contains private interface for HP speech enhancement app
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
* Include guard
*******************************************************************************/
#ifndef __IFX_SP_ENH_PRIV_H
#define __IFX_SP_ENH_PRIV_H

/*******************************************************************************
* Include header file
*******************************************************************************/
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "ifx_sp_common.h"
#include "ifx_sp_common_priv.h"
/*******************************************************************************
* Compile-time flags
*******************************************************************************/

/*******************************************************************************
* Macros
*******************************************************************************/

/*******************************************************************************
* extern variables
******************************************************************************/

/*******************************************************************************
* Structures and enumerations
******************************************************************************/
/* all IP componet structures defined here and IP owner has reposibilty to define his IP structure */
typedef struct
{
    int32_t in_size;        /* input size */
    int32_t gain;           /* High pass filet gain */
    int32_t hpf_order;      /* High pass filter order */
} hpf_struct;

typedef struct
{
    int32_t shift_size;
    int32_t block_size;
    int32_t tail;
    int32_t bulk_delay;
} analysis_struct;

typedef struct
{
    int32_t tail;
    int32_t bulk_delay;
} aec_struct;

typedef struct
{
    int32_t mic_distance;
    int32_t angle_range;
    int8_t agressiveness;
} bf_struct;

typedef struct
{
    int8_t agressiveness;
} dereverb_struct;

typedef struct
{
    int8_t agressiveness;
} es_struct;

typedef struct
{
    int8_t agressiveness;
} ns_struct;

typedef struct
{
    IFX_SP_DATA_TYPE_COEFF_T* coeff;
    IFX_SP_DATA_TYPE_STATE_T* state_out;
    void* xxComponent;              /* point to each component structure */
    int32_t out_size;               /* output size of each component */
    ifx_sp_enh_ip_component_config_t component_typeb;
#if IFX_SP_FIXED_POINT
    int8_t sQ;                      /* state Q value */
    int8_t coeffQ;                  /* coefficient Q value */
#endif
    ifx_cycle_profile_t profile;
} component_struct_t;

typedef struct sp_enh_struct_t
{
    int32_t sampling_rate;
    int32_t frame_size;
    int32_t fft_size;
    int32_t num_beams;          /* Number of audio input channels which should be equal to 2 */
    int32_t n_components;

    ifx_scratch_mem_t scratch; /* Scratch memory structure */

    char* persistent_pad;       /* pointer to allocated persistent memory */
    int32_t persistent_size;    /* allocated persistent memory size */

    uint32_t enable_flag;       /* each bit control enable/disable one component with bit 0 is control component 0. when enabled component state will be rest */
    uint32_t reset_flag;        /* each bit control reset one component with bit 0 is control component 0. when enabled component state will be rest */

    int8_t data_q;              /* q factor of current component's input data */
    uint8_t component;
    component_struct_t* l;

    ifx_cycle_profile_t profile;             /* cycle profile structure */
}sp_enh_struct;

/*******************************************************************************
* Function prototypes of all IP components
******************************************************************************/
int hpf_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* out);

int analysis_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* out);

int aec_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* ref_in, IFX_SP_DATA_TYPE_T* out);

int asa_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in1, IFX_SP_DATA_TYPE_T* in2, IFX_SP_DATA_TYPE_T* out);

int bf_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in1, IFX_SP_DATA_TYPE_T* in2, IFX_SP_DATA_TYPE_T* in_asa, IFX_SP_DATA_TYPE_T* out);

int dereverb_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* in_asa, IFX_SP_DATA_TYPE_T* out);

int esns_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* asa_in, IFX_SP_DATA_TYPE_T* ref_in, IFX_SP_DATA_TYPE_T* out);

int synthesis_process(sp_enh_struct* dPt
    , IFX_SP_DATA_TYPE_T* in, IFX_SP_DATA_TYPE_T* out);

#endif /*__IFX_SP_ENH_PRIV_H */

/* [] END OF FILE */
