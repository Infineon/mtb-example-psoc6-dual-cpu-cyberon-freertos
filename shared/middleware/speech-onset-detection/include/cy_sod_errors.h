/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/** @file cy_sod_errors.h
 *
 * @brief This file is the header file for SOD library
 * error codes defines
 *
 * Abbreviations used in the header file
 *
 * SOD      -   Speech Onset Detection
 */

#ifndef __CY_SOD_ERRORS_H__
#define __CY_SOD_ERRORS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cy_log.h"
#include "cy_errors.h"
#include "cy_result.h"
#include "cy_result_mw.h"


/**
 * \defgroup sod_results speech onset detection (SOD) results/error codes
 * @ingroup group_sod_macros
 *
 * SOD middleware APIs return results of type cy_rslt_t and consist of three parts:
 * - module base
 * - type
 * - error code
 *
 * \par Result Format
 *
   \verbatim
              Module base                            Type     Library-specific error code
      +--------------------------------------------+--------+------------------------------+
      |CY_RSLT_MODULE_CY_SVC_BASE     | 0x2 |           Error Code         |
      +--------------------------------------------+--------+------------------------------+
                14 bits               2 bits            16 bits

   See the macro section of this document for library-specific error codes.
   \endverbatim
 *
 * The data structure cy_rslt_t is part of cy_result.h located in <core_lib/include>
 *
 * Module base: This base is derived from CY_RSLT_MODULE_MIDDLEWARE_BASE (defined in cy_result.h) and is an offset of the CY_RSLT_MODULE_MIDDLEWARE_BASE.
 *              The details of the offset and the middleware base are defined in cy_result_mw.h, which is part of the [GitHub connectivity-utilities] (https://github.com/Infineon/connectivity-utilities) repo.
 *              For example, the buffer pool library uses CY_RSLT_MODULE_CY_SVC_BASE as the module base.
 *
 * Type: This type is defined in cy_result.h and can be one of CY_RSLT_TYPE_FATAL, CY_RSLT_TYPE_ERROR, CY_RSLT_TYPE_WARNING, or CY_RSLT_TYPE_INFO. AWS library error codes are of type CY_RSLT_TYPE_ERROR.
 *
 * Library-specific error code: These error codes are library-specific and defined in the macro section.
 *
 * Helper macros used for creating the library-specific result are provided as part of cy_result.h.
 * \{
 */

/** SOD error code base. */
#define CY_RSLT_SOD_ERR_BASE                        CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_SOD_BASE, 0)

#define CY_RSLT_SOD_OUT_OF_MEMORY                   ( CY_RSLT_SOD_ERR_BASE + 1 )
#define CY_RSLT_SOD_GENERIC_ERROR                   ( CY_RSLT_SOD_ERR_BASE + 2 )
#define CY_RSLT_SOD_BAD_ARG                         ( CY_RSLT_SOD_ERR_BASE + 3 )
#define CY_RSLT_SOD_NOT_INITIALIZED                 ( CY_RSLT_SOD_ERR_BASE + 4 )
#define CY_RSLT_SOD_ALREADY_INITIALIZED             ( CY_RSLT_SOD_ERR_BASE + 5 )
#define CY_RSLT_SOD_INVALID_STATE                   ( CY_RSLT_SOD_ERR_BASE + 6 )
#define CY_RSLT_SOD_PROCESS_PARSE_FAIL              ( CY_RSLT_SOD_ERR_BASE + 7 )
#define CY_RSLT_SOD_PROCESS_INIT_FAIL               ( CY_RSLT_SOD_ERR_BASE + 8 )
#define CY_RSLT_SOD_PROCESS_DATA_FAIL               ( CY_RSLT_SOD_ERR_BASE + 9 )
#define CY_RSLT_SOD_FEATURE_NOT_SUPPORTED           ( CY_RSLT_SOD_ERR_BASE + 10)
#define CY_RSLT_SOD_DEINIT_FAIL                     ( CY_RSLT_SOD_ERR_BASE + 11)
#define CY_RSLT_SOD_INIT_FAIL                       ( CY_RSLT_SOD_ERR_BASE + 12)
#define CY_RSLT_SOD_RESET_FAIL                      ( CY_RSLT_SOD_ERR_BASE + 13)


/** \} group_sod_macros */
#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef __CY_SOD_ERRORS_H__ */
