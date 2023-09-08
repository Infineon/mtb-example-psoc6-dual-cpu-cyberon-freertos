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
 * @file staged_voice_control_ipc.h
 *
 */


#ifndef __CY_SVC_IPC_H__
#define __CY_SVC_IPC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "cy_staged_voice_control_common.h"
/*******************************************************************************
 *                              Macros
 ******************************************************************************/

#ifdef ENABLE_PSOC6_IPC_COMMUNICATION
#include "ipc_communication.h"
#elif ENABLE_SVC_IPC_STUB_SINGLE_CORE
#include "ipc_communication.h"
#endif

#ifdef ENABLE_APP_SIMULATION_FOR_EVENTS
/* *
 * Refer svc_simnulation.h for more detailed simulation commands.
 */
#define IPC_CMD_ID_SIMULATION_START                    ((uint8_t)0x50)
#define IPC_CMD_ID_SIMULATION_END                      ((uint8_t)0x60)
#endif

#define IPC_CMD_ID_HP_TO_LP_SET_STATE                   ((uint8_t)0x88)
#define IPC_CMD_ID_LP_TO_HP_CONFIG_UPDATE               ((uint8_t)0x89)
#define IPC_CMD_ID_LP_TO_HP_CIRCULAR_BUF_UPDATE         ((uint8_t)0x90)
#define IPC_CMD_ID_LP_TO_HP_SET_STATE_RESULT            ((uint8_t)0x91)

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
 * IPC message for config update from SVC LP. Size of this message
 * should be equivalent to size of (ipc_msg_t.msg_pay) = 12 Bytes
 */
typedef struct __attribute__((packed, aligned(4)))
{
    uint8_t     cmd_id;                     /* Byte 1       - Command identifier    */
    uint8_t     dbg_message_counter;        /* Byte 2       - message counter       */
    uint8_t     reserved1[2];               /* Byte 3-4     - Reserved              */

    cy_svc_set_state_t     set_state;       /* Byte 5-8     - Set state             */

    uint8_t     payload_internal[MAX_SET_STATE_INFO_SIZE_IN_BYTES]; /* Byte 9-12    - Reserved              */

} ipc_cmd_hp_to_lp_set_state_t ;




/**
 * IPC message for config update from SVC LP. Size of this message
 * should be equivalent to size of (ipc_msg_t.msg_pay) = 12 Bytes
 */
typedef struct __attribute__((packed, aligned(4)))
{
    uint8_t     cmd_id;                     /* Byte 1       - Command identifier    */
    uint8_t     dbg_message_counter;        /* Byte 2       - message counter       */
    uint8_t     reserved1[6];               /* Byte 3-8     - Reserved              */

    cy_svc_stage_config_t stage_config_list; /* Byte  9-12  - LP config list        */

} ipc_cmd_lp_to_hp_cfg_update_t ;


/**
 * IPC message for set state execution result from SVC LP to SVC HP.
 * Size of this message should be equivalent to size of (ipc_msg_t.msg_pay)
 * = 12 Bytes
 */
typedef struct __attribute__((packed, aligned(4)))
{
    uint8_t     cmd_id;                     /* Byte 1       - Command identifier    */
    uint8_t     dbg_message_counter;        /* Byte 2       - message counter       */
    uint8_t     reserved1[2];               /* Byte 3-4     - Reserved              */

    cy_svc_stage_t cur_stage;               /* Byte 5-8     - Current stage         */
    cy_rslt_t   set_state_result;           /* Byte  9-12   - set state result list */

} ipc_cmd_lp_to_hp_set_state_result_t ;

typedef struct __attribute__((packed, aligned(4)))
{
    uint8_t     cmd_id;                     /* Byte 1       - Command identifier    */
    uint16_t    data_info_bitmask;          /* Byte 2-3     - Buffer info           */
    uint8_t     current_stage;              /* Byte 4       - Current Stage         */
    uint8_t      *data_pointer;             /* Byte 5 - 8   - Data pointer          */
    uint16_t     frame_count;               /* Byte 9 - 10  - Frame count           */
    uint16_t     insuff_preroll_frame_count;/* Byte 11 - 12 - In sufficient pre-roll Frame count */

} ipc_cmd_lp_to_hp_data_t ;


#ifdef __cplusplus
}
#endif

#endif /* __CY_SVC_IPC_H__ */
