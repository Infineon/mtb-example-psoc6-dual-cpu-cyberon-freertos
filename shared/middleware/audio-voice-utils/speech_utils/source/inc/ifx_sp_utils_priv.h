/******************************************************************************
* File Name: ifx_sp_utils_priv.h
*
* Description: This file contains private interface for speech utilities
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
* Include guard
*******************************************************************************/
#ifndef __IFX_SP_UTILS_PRIV_H
#define __IFX_SP_UTILS_PRIV_H

/*******************************************************************************
* Include header file
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "ifx_sp_common.h"
#include "ifx_sp_common_priv.h"
#include "SOD.h"
#ifdef ENABLE_IFX_LPWWD
#include "mel_features_fix.h"
#endif

#ifdef ENABLE_IFX_LPWWD
#include "lpwwd_post.h"
#endif

/*******************************************************************************
* Compile-time flags
*******************************************************************************/

/*******************************************************************************
* Speech utilities data type & defines
*******************************************************************************/


/*******************************************************************************
* Structures and enumerations
*******************************************************************************/
/* Internally defined structures and IP owner has reposibilty to define his structure */
typedef struct sod_top_struct_t
{
    /*@{*/
    struct IFX_SOD_STRUCT ifx_sod_comp; /**<: Infineon internal SOD structure */
    ifx_scratch_mem_t scratch;          /**<: Point to scratch memory structure */
    char* persistent_pad;               /* pointer to allocated persistent memory */
    int32_t persistent_size;            /* allocated persistent memory size */
    bool enable_flag;                   /* 1: detect SOD (i.e. normal operation); 0: don�t detect, just update statistics */
    ifx_cycle_profile_t profile;        /* cycle profile structure */
    /*@}*/
} sod_top_struct;

typedef struct spectrogram_top_struct_t
{
    /*@{*/
    #ifdef ENABLE_IFX_LPWWD
    ifx_spectrogram_struc_t ifx_spectrogram; /**<: Infineon internal spectrogram tranformation structure */
    #endif
    
    ifx_scratch_mem_t scratch;               /**<: Point to scratch memory structure */
    char* persistent_pad;                    /* pointer to allocated persistent memory */
    int32_t persistent_size;                 /* allocated persistent memory size */
    bool reset_flag;                         /* one time input sample tail (zero out) reset flag */
    ifx_cycle_profile_t profile;             /* cycle profile structure */
    /*@}*/
} spectrogram_top_struct;

#ifdef ENABLE_IFX_LPWWD
typedef struct postprocess_top_struc_t
{
    /*@{*/
    struct FIXED_PP_struct ifx_fixed_ppmem;  /**<: Infineon internal HMMS post processing structure */
    int32_t fps;                             /**<: frame per second rate should be on the order of 20-50Hz */
    int32_t lookback_buffer_length;          /**<: lookback buffer length in seconds, Q12 */
    int32_t stacked_frame_delay;             /**<: stacked frame delay in second, Q12 */
    ifx_scratch_mem_t scratch;               /**<: Point to scratch memory structure */
    char* persistent_pad;                    /* pointer to allocated persistent memory */
    int32_t persistent_size;                 /* allocated persistent memory size */
    bool reset_flag;                         /* one time input sample tail (zero out) reset flag */
    ifx_cycle_profile_t profile;             /* cycle profile structure */
    /*@}*/
} postprocess_top_struct;
#endif

#endif /*__IFX_SP_UTILS_PRIV_H */

/* [] END OF FILE */
