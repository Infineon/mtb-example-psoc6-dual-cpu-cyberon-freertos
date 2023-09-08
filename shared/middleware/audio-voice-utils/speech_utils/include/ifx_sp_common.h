/******************************************************************************
* File Name: ifx_sp_common.h
*
* Description: This file contains public interface for Infineon speech common header
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
#ifndef __IFX_SP_COMMON_H
#define __IFX_SP_COMMON_H

/*******************************************************************************
* Include header file
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
/*******************************************************************************
* Compile-time flags
*******************************************************************************/

/*******************************************************************************
* Infineon speech common data type & defines
*******************************************************************************/
#define IFX_SP_ENH_SUCCESS                  (0)

#define IFX_SP_ENH_LINE_SHIFT               (16u)
#define IFX_SP_ENH_LINE_MASK                (0xFFFF0000)

#define IFX_SP_ENH_COMPONENT_ID_SHIFT       (8u)
#define IFX_SP_ENH_COMPONENT_ID_MASK        (0x0000FF00)
#define IFX_SP_ENH_COMPONENT_ID_INVALID     (255u)

#define IFX_SP_ENH_ERR_CODE_MASK            (0x000000FF)
#define IFX_SP_ENH_ERR_CONFIGURATOR_VERSION (0x01)
#define IFX_SP_ENH_ERR_OVER_MAX_SCRATCH_MEM (0x02)
#define IFX_SP_ENH_ERR_INVALID_ARGUMENT     (0x03)
#define IFX_SP_ENH_FEATURE_NOT_SUPPORTED    (0x04)
#define IFX_SP_ENH_ERR_PARAM                (0x05)
#define IFX_SP_ENH_MISMATCH_PARM_CHECKSUM   (0x06)
#define IFX_SP_ENH_ERR_API                  (0x07)

#define IFX_SP_ENH_ERROR(x, y)              ((__LINE__ << IFX_SP_ENH_LINE_SHIFT) | \
                                            (((x) << IFX_SP_ENH_COMPONENT_ID_SHIFT) & IFX_SP_ENH_COMPONENT_ID_MASK) | \
                                            ((y) & IFX_SP_ENH_ERR_CODE_MASK))
#define IFX_SP_ENH_ERR_CODE(x)              (uint8_t)((x) & IFX_SP_ENH_ERR_CODE_MASK)
#define IFX_SP_ENH_ERR_COMPONENT_INDEX(x)   (uint8_t)(((x) & IFX_SP_ENH_COMPONENT_ID_MASK) >> IFX_SP_ENH_COMPONENT_ID_SHIFT)
#define IFX_SP_ENH_ERR_LINE_NUMBER(x)       (uint16_t)(((x) & IFX_SP_ENH_LINE_MASK) >> IFX_SP_ENH_LINE_SHIFT)


#define FEATURE_EXTRACTION_OUTPUT_8BIT      (1) /* Enable or disable 8bit feature extraction output */

#if FEATURE_EXTRACTION_OUTPUT_8BIT
typedef int8_t  IFX_FE_DATA_TYPE_T;
#define FEATURE_EXTRACTION_OUTPUT_8BIT_DYNAMIC  (0)     /* Enable or disable dynamic 8bit output */
#if !FEATURE_EXTRACTION_OUTPUT_8BIT_DYNAMIC
#define MEL_LOG_8BIT_OUTPUT_Q        (2)        /* CAUTION: Value subjucts change inside library so this is information example for fixed scaling Q value */
#define MFCC_8BIT_OUTPUT_Q           (0)        /* CAUTION: Value subjucts change inside library so this is information example for fixed scaling Q value */
#endif
#else
typedef int16_t  IFX_FE_DATA_TYPE_T;
#define MEL_LOG_16BIT_OUTPUT_Q       (9)        /* CAUTION: Value subjucts change inside library so this is information example for fixed scaling Q value */
#define MFCC_16BIT_OUTPUT_Q          (6)        /* CAUTION: Value subjucts change inside library so this is information example for fixed scaling Q value */
#endif

/** Defines ifx speech enhancement process IP components */
typedef enum
{
    IFX_SP_ENH_IP_COMPONENT_HPF = 0,
    IFX_SP_ENH_IP_COMPONENT_AEC,
    IFX_SP_ENH_IP_COMPONENT_BF,
    IFX_SP_ENH_IP_COMPONENT_ESNS,
    IFX_SP_ENH_IP_COMPONENT_DEREVERB,       /* Used to update DEREVERB (part of BF) agressiveness and eanable/disable */
    IFX_SP_ENH_IP_COMPONENT_NS,             /* Used to update NS (part of NSES) suppression gain and eanable/disable */
    IFX_SP_ENH_IP_COMPONENT_ES,             /* Used to update ES (part of NSES) suppression agressiveness and eanable/disable */
    IFX_SP_ENH_IP_COMPONENT_ANALYSIS,       /* always configured and can't disabled */
    IFX_SP_ENH_IP_COMPONENT_SYNTHESIS,      /* always configured and can't disabled */
    IFX_PRE_POST_IP_COMPONENT_START_ID      /* Infineon LPWWD pre and post process component IP starting index */
} ifx_sp_enh_ip_component_config_t;

typedef struct
{
    /*@{*/
    int32_t scratch_mem;        /**< scratch memory byte size required for post process */
    int32_t persistent_mem;     /**< persistent memory byte size required for post process */
    void* scratch_mem_pt;       /**< scratch memory pointer */
    void* persistent_mem_pt;    /**< persistent memory pointer */
    /*@}*/
} mem_info_t;

/** Defines profile settings */
#define IFX_SP_ENH_PROFILE_FRAME            (0x01)
#define IFX_SP_ENH_PROFILE_COMPONENT        (0x02)
#define IFX_SP_ENH_PROFILE_APP              (0x04)

typedef enum
{
    IFX_SP_ENH_PROFILE_DISABLE                     = 0,
    IFX_SP_ENH_PROFILE_ENABLE_APP                  = IFX_SP_ENH_PROFILE_APP,
    IFX_SP_ENH_PROFILE_ENABLE_COMPONENT            = IFX_SP_ENH_PROFILE_COMPONENT,
    IFX_SP_ENH_PROFILE_ENABLE_APP_PER_FRAME        = (IFX_SP_ENH_PROFILE_APP | IFX_SP_ENH_PROFILE_FRAME),
    IFX_SP_ENH_PROFILE_ENABLE_COMPONENT_PER_FRAME  = (IFX_SP_ENH_PROFILE_COMPONENT | IFX_SP_ENH_PROFILE_FRAME),
} ifx_sp_enh_profile_config_t;

typedef int16_t IFX_SP_DATA_TYPE_T;

typedef int16_t  IFX_PPINPUT_DATA_TYPE_T;

typedef int (*ifx_sp_profile_cb_fun) (void* arg, char* buf, int32_t type);


#endif /*__IFX_SP_COMMON_H */

/* [] END OF FILE */