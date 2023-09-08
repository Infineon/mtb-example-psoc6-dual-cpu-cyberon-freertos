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

/**
 * @file staged_voice_control_lp_lpwwd_internal.h
 *
 */

#ifndef __CY_SVC_LP_LPWWD_INTERNAL_H_
#define __CY_SVC_LP_LPWWD_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef ENABLE_SVC_LP_MW

#include "staged_voice_control_lp_private.h"
#include "staged_voice_control_lp_resource.h"
#include "staged_voice_control_lp_queue.h"
#include "staged_voice_control_lp_circular_buffer.h"
#include "staged_voice_control_lp_hpf.h"

#include "cy_sod.h"
#ifdef ENABLE_IFX_LPWWD
#include "cy_lpwwd.h"
#include "cy_lpwwd_defines.h"
#endif

/*******************************************************************************
 *                              Macros
 ******************************************************************************/

/*******************************************************************************
 *                              Constants
 ******************************************************************************/

/*******************************************************************************
 *                              Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                              Type Definitions
 ******************************************************************************/

/*******************************************************************************
 *                              Structures
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                              Function Declarations
 ******************************************************************************/\

cy_rslt_t svc_lp_lpwwd_internal_init(
        svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *init);

cy_rslt_t svc_lp_process_data_lpwwd_internal(
        svc_lp_instance_t *lp_instance,
        uint8_t *data);

cy_rslt_t svc_lp_lpwwd_internal_reset(svc_lp_instance_t *lp_instance);

cy_rslt_t svc_lp_lpwwd_internal_deinit(svc_lp_instance_t *lp_instance);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __CY_SVC_LP_LPWWD_INTERNAL_H_ */
