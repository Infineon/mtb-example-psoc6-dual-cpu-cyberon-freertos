/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the CM0p core source code of the Dual CPU Cyberon 
 *              Code Example for ModusToolbox.
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "cyabs_rtos.h"
#include "ipc_communication.h"
#include "svc_lp.h"
#include "custom_one_stage_asr.h"

/****************************************************************************
 * Macros
 *****************************************************************************/
/* Define how many samples in a frame */
#define FRAME_SIZE                      (160U)

/* Application thread stack size */
#define APPLICATION_THREAD_STACK_SIZE   (10U * configMINIMAL_STACK_SIZE)

/* Application thread priority */
#define APPLICATION_THREAD_PRIORITY     (4U)

/* Audio data queue length */
#define AUDIO_DATA_QUEUE_LENGTH         (2U)

/* Audio data queue itemsize */
#define AUDIO_DATA_QUEUE_ITEM_SIZE      (2U * FRAME_SIZE)


/****************************************************************************
 * Functions Prototypes
 *****************************************************************************/
static void application_thread(void *arg);
static void app_pdm_mic_handler(void);
static void app_log_cm0_init(void);
static int app_log_cm0_log_handler(CY_LOG_FACILITY_T facility, 
                                   CY_LOG_LEVEL_T level, char *logmsg);

/****************************************************************************
 * Global variables
 *****************************************************************************/
/*FreeRTOS queue to communicate data between PDM-PCM ISR and application thread*/
static cy_queue_t audio_data_queue_handle;

/****************************************************************************
 * Constants
 *****************************************************************************/
const cy_stc_sysint_t pdm_pcm_int_cfg = 
{
    .intrSrc = (IRQn_Type)NvicMux6_IRQn,
    .cm0pSrc = CYBSP_PDM_PCM_IRQ,
    .intrPriority = 2
};

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function for CM0p CPU.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    BaseType_t retval;
    cy_rslt_t result;

    /* Init the IPC communication for CM0+ */
    setup_ipc_communication_cm0();

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Lock the sempahore to wait for CM4 to be init */
    Cy_IPC_Sema_Set(IPC_SEMA_NUM, false);
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);

    /* Wait till CM4 unlocks the semaphore*/
    do
    {
        __WFE();
    } 
    while (Cy_IPC_Sema_Status(IPC_SEMA_NUM) == CY_IPC_SEMA_STATUS_LOCKED);

    /* Update the syatem clocks of the CM0p */
    SystemCoreClockUpdate();

    /* Initialize the PDM/PCM interrupt and enable it */
    Cy_SysInt_Init(&pdm_pcm_int_cfg, app_pdm_mic_handler);
    NVIC_EnableIRQ(pdm_pcm_int_cfg.intrSrc);

    /* Init logs */
    app_log_cm0_init();

    /* Init SVC LP*/
    svc_init();

    /* One stage Cyberon init*/
    if (!cyberon_asr_init(asr_callback))
    {
        CY_ASSERT(0);
    }

    /* Create CM0 application thread */
    retval = xTaskCreate(application_thread, "application_thread", 
                         APPLICATION_THREAD_STACK_SIZE, NULL, 
                         APPLICATION_THREAD_PRIORITY, NULL);
    if (pdPASS != retval)
    {
        CY_ASSERT(0);
    }

    /* Init queue for data transfer from ISR to the application thread */
    if (CY_RSLT_SUCCESS != cy_rtos_init_queue(&audio_data_queue_handle, 
                                              AUDIO_DATA_QUEUE_LENGTH, 
                                              AUDIO_DATA_QUEUE_ITEM_SIZE))
    {
        CY_ASSERT(0);
    }

    /* Initialize the PDM/PCM block */
    Cy_PDM_PCM_Init(CYBSP_PDM_PCM_HW, &CYBSP_PDM_PCM_config);
    Cy_PDM_PCM_Enable(CYBSP_PDM_PCM_HW);

    /* Start Scheduler */
    vTaskStartScheduler();

    for (;;)
    {
        /* vTaskStartScheduler never returns */
    }
}

/*******************************************************************************
 * Function Name: application_thread
 ********************************************************************************
 * Summary:
 *  The application task unblocks whenever there is data in queue.
 *  The 10 msec data is fed into CM0p SVC everytime application task is unblocked.
 *
 * Parameters:
 *  arg: not used
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void application_thread(void *arg)
{
    (void)arg;

    for (;;)
    {
        int16_t pcm_data[FRAME_SIZE] = {0};
        cy_rtos_get_queue(&audio_data_queue_handle, pcm_data, 
                          CY_RTOS_NEVER_TIMEOUT, false);

        if (CY_RSLT_SUCCESS != cy_svc_lp_feed(pcm_data))
        {
            CY_ASSERT(0);
        }
    }
}

/*******************************************************************************
 * Function Name: app_pdm_mic_handler
 ********************************************************************************
 * Summary:
 *  PDM PCM converter ISR handler.
 *  Populates audio_data[] buffer every 10msec.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_pdm_mic_handler(void)
{
    /* Audio buffer to hold 10 msec pcm data */
    int16_t audio_data[FRAME_SIZE] = {0};
    uint16_t count = 0;
    
    do
    {
        audio_data[count] = (int16_t)Cy_PDM_PCM_ReadFifo(CYBSP_PDM_PCM_HW);
        count++;
    } 
    while (count < FRAME_SIZE);

    /* Send data to application task over queue */
    cy_rtos_put_queue(&audio_data_queue_handle, &audio_data[0], 
                      CY_RTOS_NEVER_TIMEOUT, true);

    /* Clear the PCM interrupt */
    Cy_PDM_PCM_ClearInterrupt(CYBSP_PDM_PCM_HW, PDM_INTR_MASK_RX_TRIGGER_Msk);
}

/*******************************************************************************
 * Function Name: app_log_cm0_log_handler
 ********************************************************************************
 * Summary:
 *  Prints the logmsg.
 *
 * Parameters:
 *  facility: Run-time log levels for output
 *  level: Logging level
 *  logmsg: Message to be printed
 *
 * Return:
 *  int
 *
 *******************************************************************************/
static int app_log_cm0_log_handler(CY_LOG_FACILITY_T facility, CY_LOG_LEVEL_T level, 
                            char *logmsg)
{
    (void)facility;
    (void)level;

    Cy_SCB_UART_PutString(CYBSP_UART_HW, logmsg);

    return 0;
}

/*******************************************************************************
 * Function Name: app_log_cm0_init
 ********************************************************************************
 * Summary:
 *  Initialize the logging subsystem.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_log_cm0_init(void)
{
    cy_log_init(CY_LOG_DEBUG4, app_log_cm0_log_handler, NULL);
}

/* [] END OF FILE */
