/******************************************************************************
* File Name: ifx_sp_common_priv.h
*
* Description: This file contains private interface for Infineon speech common header
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
#ifndef __IFX_SP_COMMON_PRIV_H
#define __IFX_SP_COMMON_PRIV_H

/*******************************************************************************
* Include header file
*******************************************************************************/
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "ifx_sp_common.h"

#if EMBEDDED_DEV
#include "arm_math.h"
#else
#undef ARM_MATH_DSP
#undef ARM_MATH_LOOPUNROLL
#endif /* EMBEDDED_DEV */

#define FIXED16MAX ((1 << 15) - 1)
#define FIXED16MIN (-1 - FIXED16MAX)
#define FIXED8MAX ((1 << 7) - 1)
#define FIXED8MIN (-1 - FIXED8MAX)

#define ROUND(shift) ( (((int32_t)0x1) << (shift)) >> 1 )
#define OUTPUT_ROUND_RIGHT_SHIFT(a, b) (((a) + ROUND(b)) >> (b))

/*******************************************************************************
* Macros
*******************************************************************************/
/* Maximum log buffer size */
#define PROFILE_MAX_LOG_SIZE   127

/* Scratch memory structure */
typedef struct
{
    char* scratch_pad;          /* pointer to allocated scratch memory */
    int32_t scratch_cnt;        /* keep track scratch memory used size */
    int32_t scratch_size;       /* allocated scrtach memory size */
} ifx_scratch_mem_t;

/* Cycle profile structure */
typedef struct
{
    uint8_t profile_config;
#if PROFILER
    uint32_t cycles;
    uint32_t sum_frames;
    uint64_t sum_cycles;
    uint32_t peak_frame;
    uint32_t peak_cycles;
#endif
} ifx_cycle_profile_t;

#if PROFILER
typedef struct
{
    ifx_sp_profile_cb_fun log_cb;
    void* log_arg;
    char log_buf[PROFILE_MAX_LOG_SIZE + 1];
} ifx_profile_log_t;

ifx_profile_log_t profile_log;  /* global profile log data structure when PROFILER is defined */
#endif

typedef enum
{
    IFX_SP_DATA_UNKNOWN            = 0u,      /**< Unknown data type */
    IFX_SP_DATA_INT8               = 1u,      /**< 8-bit fixed-point */
    IFX_SP_DATA_INT16              = 2u,      /**< 16-bit fixed-point */
    IFX_SP_DATA_FLOAT              = 3u,      /**< 32-bit float-point */
} ifx_sp_data_type_t;


// #define COMPONENT_ENABLE_FLAG(enable_flag, component_id)       ((enable_flag) && (1 << (component_id)))
// #define COMPONENT_RESET_FLAG(reset_flag, component_id)         ((reset_flag) && (1 << (component_id)))

#define COMPONENT_ENABLE_FLAG(enable_flag, component_id)       ((enable_flag) & (1 << (component_id)))
#define COMPONENT_RESET_FLAG(reset_flag, component_id)         ((reset_flag) & (1 << (component_id)))

typedef int16_t IFX_SP_DATA_TYPE_COEFF_T;
typedef int16_t IFX_SP_DATA_TYPE_STATE_T;  /* Not sure it should be 16bit or 32bit */
typedef int32_t IFX_SP_DATA_TYPE_ACCUM_T;

#endif /*__IFX_SP_COMMON_PRIV_H */

/* [] END OF FILE */