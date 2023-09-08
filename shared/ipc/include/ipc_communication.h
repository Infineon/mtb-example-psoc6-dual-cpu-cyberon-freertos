/******************************************************************************
* File Name:   ipc_communication.h
*
* Description: This file contains definitions of constants and structures for
*              setting up user pipe and function prototypes for configuring
*              system and user IPC pipe.
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

#ifndef SOURCE_IPC_COMMUNICATION_H
#define SOURCE_IPC_COMMUNICATION_H

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* Endpoint indexes in the pipe array */
#define USER_IPC_PIPE_EP_ADDR_CM0       (2UL)
#define USER_IPC_PIPE_EP_ADDR_CM4       (3UL)

#if (CY_CPU_CORTEX_M0P)
    #define USER_IPC_PIPE_EP_ADDR       USER_IPC_PIPE_EP_ADDR_CM0
#else
    #define USER_IPC_PIPE_EP_ADDR       USER_IPC_PIPE_EP_ADDR_CM4
#endif  /* (CY_CPU_CORTEX_M0P) */

/* Number of clients on each endpoint */
#define USER_IPC_PIPE_CLIENT_CNT        (4UL)

/* IPC data channel for User PIPE EP0 */
#define USER_IPC_PIPE_CHAN_EP0          (8UL)

/* IPC data channel for User PIPE EP1 */
#define USER_IPC_PIPE_CHAN_EP1          (9UL)

/* Notifier EP0 */
#define USER_IPC_PIPE_INTR_EP0          (8UL)

/* Notifier EP1 */
#define USER_IPC_PIPE_INTR_EP1          (9UL)

/* CM0+ NVIC MUX for IPC */
#define USER_IPC_PIPE_INTR_MUX_EP0      (2UL)

#define USER_IPC_PIPE_INTR_MASK         (uint32_t)((1UL << USER_IPC_PIPE_CHAN_EP0) |\
                                                 (1UL << USER_IPC_PIPE_CHAN_EP1))
#define USER_IPC_PIPE_EP0_CONFIG        (_VAL2FLD(CY_IPC_PIPE_CFG_IMASK, USER_IPC_PIPE_INTR_MASK) |\
                                        _VAL2FLD(CY_IPC_PIPE_CFG_INTR,  USER_IPC_PIPE_INTR_EP0) |\
                                        _VAL2FLD(CY_IPC_PIPE_CFG_CHAN,  USER_IPC_PIPE_CHAN_EP0))
#define USER_IPC_PIPE_EP1_CONFIG        (_VAL2FLD(CY_IPC_PIPE_CFG_IMASK, USER_IPC_PIPE_INTR_MASK) |\
                                        _VAL2FLD(CY_IPC_PIPE_CFG_INTR,  USER_IPC_PIPE_INTR_EP1) |\
                                        _VAL2FLD(CY_IPC_PIPE_CFG_CHAN,  USER_IPC_PIPE_CHAN_EP1))

#define IPC_CM0_TO_CM4_CLIENT_ID        (1U)
#define IPC_CM4_TO_CM0_CLIENT_ID        (2U)

#define IPC_CMD_INIT                    0x81
#define IPC_CMD_START                   0x82
#define IPC_CMD_STOP                    0x83
#define IPC_CMD_STATUS                  0x41

#define SEND_IPC_MSG(i,x)   i.cmd = x; \
                            Cy_IPC_Pipe_SendMessage(USER_IPC_PIPE_EP_ADDR_CM0, \
                                                    USER_IPC_PIPE_EP_ADDR_CM4, \
                                                    (void *) &i, 0);   

/* Semaphore number to be used in this example. Semaphores 0-15 are reserved
       for system use. */
#define IPC_SEMA_NUM                16u

/*******************************************************************************
* Enumerations
*******************************************************************************/


#define MAX_MSG_PAY_SIZE_IN_BYTES (12)

typedef struct __attribute__((packed, aligned(4)))
{
    uint8_t     client_id;
    uint8_t     cpu_status;
    uint16_t    intr_mask;

    uint8_t     msg_pay[MAX_MSG_PAY_SIZE_IN_BYTES];

#if 1
    uint8_t     cmd;
    uint32_t    value;      // Number of samples in the content
    void        *content;
#endif
} ipc_msg_t ;


/*******************************************************************************
* Function prototypes
*******************************************************************************/
void setup_ipc_communication_cm0(void);
void user_ipc_pipe_isr_cm0(void);
void setup_ipc_communication_cm4(void);
void user_ipc_pipe_isr_cm4(void);


#endif /* SOURCE_IPC_COMMUNICATION_H */

/* [] END OF FILE */
