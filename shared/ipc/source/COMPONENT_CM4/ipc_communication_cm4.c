/******************************************************************************
 * File Name:   ipc_communication_cm4.c
 *
 * Description: This file contains function definitions for setting up system
 *              IPC communication and user IPC pipe.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
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
 *******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "ipc_communication.h"
#include "system_psoc6.h"

/*******************************************************************************
 * Function definitions
 ******************************************************************************/

/*******************************************************************************
 * Function Name: setup_ipc_communication_cm4
 ********************************************************************************
 * Summary:
 *        - Initializes IPC Semaphore.
 *        - Configures CY_PIPE used for System calls, Flash, and BLE.
 *        - Configures USER_PIPE used in the example for sending CapSense touch
 *          detection data.
 *        - Initializes both CY_PIPE and USER_PIPE.
 *        - Initializes flash after CY_PIPE is initialized.
 *
 *******************************************************************************/
void setup_ipc_communication_cm4(void)
{
#ifdef __CM0P_PRESENT
#if (__CM0P_PRESENT == 0)
    /* Allocate and initialize semaphores for the system operations. */
    static uint32_t ipcSemaArray[CY_IPC_SEMA_COUNT / CY_IPC_SEMA_PER_WORD];
    (void)Cy_IPC_Sema_Init(CY_IPC_CHAN_SEMA, CY_IPC_SEMA_COUNT, ipcSemaArray);
#else
    (void)Cy_IPC_Sema_Init(CY_IPC_CHAN_SEMA, 0ul, NULL);
#endif /* (__CM0P_PRESENT) */
#else
    (void)Cy_IPC_Sema_Init(CY_IPC_CHAN_SEMA, 0ul, NULL);
#endif /* __CM0P_PRESENT */

    /* Create an array of endpoint structures */
    static cy_stc_ipc_pipe_ep_t systemIpcPipeEpArray[CY_IPC_MAX_ENDPOINTS];

    Cy_IPC_Pipe_Config(systemIpcPipeEpArray);

    static cy_ipc_pipe_callback_ptr_t systemIpcPipeSysCbArray[CY_SYS_CYPIPE_CLIENT_CNT];

    static const cy_stc_ipc_pipe_config_t systemIpcPipeConfigCm4 =
        {
            /* .ep0ConfigData */
            {
                CY_IPC_INTR_CYPIPE_EP0,       /* .ipcNotifierNumber    */
                CY_SYS_INTR_CYPIPE_PRIOR_EP0, /* .ipcNotifierPriority  */
                CY_SYS_INTR_CYPIPE_MUX_EP0,   /* .ipcNotifierMuxNumber */
                CY_IPC_EP_CYPIPE_CM0_ADDR,    /* .epAddress            */
                CY_SYS_CYPIPE_CONFIG_EP0      /* .epConfig             */
            },
            /* .ep1ConfigData */
            {
                CY_IPC_INTR_CYPIPE_EP1,       /* .ipcNotifierNumber    */
                CY_SYS_INTR_CYPIPE_PRIOR_EP1, /* .ipcNotifierPriority  */
                0u,                           /* .ipcNotifierMuxNumber */
                CY_IPC_EP_CYPIPE_CM4_ADDR,    /* .epAddress            */
                CY_SYS_CYPIPE_CONFIG_EP1      /* .epConfig             */
            },
            CY_SYS_CYPIPE_CLIENT_CNT, /* .endpointClientsCount     */
            systemIpcPipeSysCbArray,  /* .endpointsCallbacksArray  */
            &Cy_SysIpcPipeIsrCm4      /* .userPipeIsrHandler       */
        };

    static cy_ipc_pipe_callback_ptr_t user_ipc_pipe_cb_array[USER_IPC_PIPE_CLIENT_CNT];

    static const cy_stc_ipc_pipe_config_t user_ipc_pipe_config_cm4 =
        {
            /* .ep2ConfigData */
            {
                USER_IPC_PIPE_INTR_EP0,     /* .ipcNotifierNumber       */
                1UL,                        /* .ipcNotifierPriority     */
                USER_IPC_PIPE_INTR_MUX_EP0, /* .ipcNotifierMuxNumber    */
                USER_IPC_PIPE_EP_ADDR_CM0,  /* .epAddress               */
                USER_IPC_PIPE_EP0_CONFIG    /* .epConfig                */
            },
            /* .ep3ConfigData */
            {
                USER_IPC_PIPE_INTR_EP1,    /* .ipcNotifierNumber       */
                1UL,                       /* .ipcNotifierPriority     */
                0UL,                       /* .ipcNotifierMuxNumber    */
                USER_IPC_PIPE_EP_ADDR_CM4, /* .epAddress               */
                USER_IPC_PIPE_EP1_CONFIG   /* .epConfig                */
            },
            USER_IPC_PIPE_CLIENT_CNT, /* .endpointClientsCount     */
            user_ipc_pipe_cb_array,   /* .endpointsCallbacksArray  */
            &user_ipc_pipe_isr_cm4    /* .userPipeIsrHandler       */
        };

    Cy_IPC_Pipe_Init(&systemIpcPipeConfigCm4);

#if defined(CY_DEVICE_PSOC6ABLE2)
    Cy_Flash_Init();
#endif /* defined(CY_DEVICE_PSOC6ABLE2) */

    Cy_IPC_Pipe_Init(&user_ipc_pipe_config_cm4);
}

/*******************************************************************************
 * Function Name: Cy_SysIpcPipeIsrCm4
 ****************************************************************************/
/**
 *
 * This is the interrupt service routine for the system pipe.
 *
 *******************************************************************************/
void Cy_SysIpcPipeIsrCm4(void)
{
    Cy_IPC_Pipe_ExecuteCallback(CY_IPC_EP_CYPIPE_CM4_ADDR);
}

/*******************************************************************************
 * Function Name: user_ipc_pipe_isr_cm4
 ********************************************************************************
 * Summary: User IRQ handler function that is called when IPC receives data from
 *          CM0+ to CM4 through USER_PIPE.
 *
 *******************************************************************************/
void user_ipc_pipe_isr_cm4(void)
{
    Cy_IPC_Pipe_ExecuteCallback(USER_IPC_PIPE_EP_ADDR);
}
