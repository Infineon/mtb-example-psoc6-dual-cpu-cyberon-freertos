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
 * @file staged_voice_control_hp_ipc.h
 *
 */

#ifndef __CY_SVC_HP_IPC_H__
#define __CY_SVC_HP_IPC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef ENABLE_SVC_HP_MW
#include "staged_voice_control_hp_private.h"

#ifdef ENABLE_PSOC6_IPC_COMMUNICATION
#include "ipc_communication.h"
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
 ******************************************************************************/

cy_rslt_t svc_hp_ipc_init(svc_hp_instance_t *hp_instance);

cy_rslt_t svc_hp_ipc_deinit(svc_hp_instance_t *hp_instance);

cy_rslt_t svc_hp_ipc_send_command_set_state(
        svc_hp_instance_t *hp_instance,
        cy_svc_set_state_t set_state,
        void *state_info);

svc_hp_instance_t* svc_hp_get_instance(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __CY_SVC_HP_IPC_H__ */
