/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 * @file staged_voice_control_lp_resource.h
 *
 */

#ifndef CY_SVC_LP_RESOURCE_H__
#define CY_SVC_LP_RESOURCE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef ENABLE_SVC_LP_MW
#include "staged_voice_control_lp_private.h"
#include "staged_voice_control_lp_queue.h"
#include "staged_voice_control_lp_stages.h"
#include "staged_voice_control_lp_circular_buffer.h"
#include "staged_voice_control_lp_sod.h"
#include "staged_voice_control_lp_hpf.h"
#include "staged_voice_control_lp_ipc.h"
#include "staged_voice_control_ipc.h"
#include "staged_voice_control_lp_lpwwd.h"
#include "cy_mem_check_utils.h"

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
 ******************************************************************************/
/**
 * Create the required resource for the instance.
 *
 * @param[in]  lp_instance             Staged voice control module instance
 * @param[in]  create_domain        Create low power domain configuration
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_lp_create_resource_for_instance(
        svc_lp_instance_t *lp_instance,
        cy_svc_lp_config_t *create_domain);

/**
 * Delete the resources created for the instance.
 *
 * @param[in]  lp_instance             Staged voice control module instance
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t svc_lp_delete_resource_for_instance(svc_lp_instance_t *lp_instance);

svc_lp_instance_t* svc_lp_get_instance(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* CY_SVC_LP_RESOURCE_H__ */
