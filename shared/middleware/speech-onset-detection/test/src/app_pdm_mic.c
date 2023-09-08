/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM0+ in the the Dual CPU IPC Pipes 
*              Application for ModusToolbox.
*
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

#ifdef RUN_SOD_ONLY
#include "cy_pdl.h"
#include "cycfg.h"
#include "cy_pdl.h"
#include "cy_log.h"
//#include "app_cm0_control.h"

/****************************************************************************
* Constants
*****************************************************************************/
#define MAX_AUDIO_SAMPLES  36


/****************************************************************************
* Global variables
*****************************************************************************/
const cy_stc_sysint_t pdm_pcm_int_cfg = {
#if (CY_CPU_CORTEX_M0P)
    /*.intrSrc =*/ NvicMux7_IRQn,               /* CM0+ interrupt is NVIC #7 */
    /*.cm0pSrc =*/ ioss_interrupts_gpio_0_IRQn, /* Source of NVIC #7 is GPIO port 0 interrupt */
    /*.intrPriority =*/ 2UL                     /* Interrupt priority is 2 */
#else
    /*.intrSrc =*/ ioss_interrupts_gpio_0_IRQn, /* Interrupt source is GPIO port 0 interrupt */
    /*.intrPriority =*/ 4UL                     /* Interrupt priority is 4 */
#endif
    };

int16_t audio_data[MAX_AUDIO_SAMPLES] = {0};

/****************************************************************************
* Functions Prototypes
*****************************************************************************/
void app_pdm_mic_handler(void);

#define MAX_SIZE_FOR_1_FRAME (640)
#define MAX_SIZE_FOR_1_FRAME_FOR_MONO (MAX_SIZE_FOR_1_FRAME/2)

char input_pdm_mic_data_frame1[MAX_SIZE_FOR_1_FRAME] = {0};
char input_pdm_mic_data_frame2[MAX_SIZE_FOR_1_FRAME] = {0};
#if 0
char input_pdm_mic_data_mono_frame[MAX_SIZE_FOR_1_FRAME_FOR_MONO] = {0};
#endif
char cur_frame = 1;
unsigned int pdm_mic_callback_counter = 0;
volatile unsigned int pdm_mic_callback_counter_frame_complete = 0;
unsigned int filled_mic_data_so_far = 0;

void convert_stereo_interleaved_10ms_frame_to_stereo_non_interleaved(
        char *stereo,
        char *mono)
{

}

void convert_stereo_interleaved_10ms_frame_to_mono(uint16_t *stereo, uint16_t *mono)
{
    int i =0;

    #define MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA (160)
    for (i = 0; i < MAX_AUD_SAMPLES_PER_CHANNEL_FOR_10MS_DATA; i++)
    {
        *mono = *stereo;
        stereo += 2;
        mono += 1;
    }
}

cy_rslt_t app_pdm_mic_accumulate_10ms_frame_and_feed(char *data,
        unsigned int len)
{
#if 0
    cy_rslt_t ret_val;
    unsigned int size_to_copy_in_cur_buffer = 0;
    unsigned int buffer_to_send = 0;

    pdm_mic_callback_counter++;

    if (MAX_SIZE_FOR_1_FRAME - filled_mic_data_so_far == len)
    {
        size_to_copy_in_cur_buffer = len;
        if (1 == cur_frame)
        {
            memcpy(input_pdm_mic_data_frame1 + filled_mic_data_so_far, data,
                    size_to_copy_in_cur_buffer);
            filled_mic_data_so_far += size_to_copy_in_cur_buffer;
        }
        else
        {
            memcpy(input_pdm_mic_data_frame2 + filled_mic_data_so_far, data,
                    size_to_copy_in_cur_buffer);
            filled_mic_data_so_far += size_to_copy_in_cur_buffer;
        }
        buffer_to_send = cur_frame;
        pdm_mic_callback_counter_frame_complete = pdm_mic_callback_counter;
        filled_mic_data_so_far = 0;
    }
    else if (MAX_SIZE_FOR_1_FRAME - filled_mic_data_so_far > len)
    {
        size_to_copy_in_cur_buffer = len;
        if (1 == cur_frame)
        {
            memcpy(input_pdm_mic_data_frame1 + filled_mic_data_so_far, data,
                    size_to_copy_in_cur_buffer);
            filled_mic_data_so_far += size_to_copy_in_cur_buffer;
        }
        else
        {
            memcpy(input_pdm_mic_data_frame2 + filled_mic_data_so_far, data,
                    size_to_copy_in_cur_buffer);
            filled_mic_data_so_far += size_to_copy_in_cur_buffer;
        }

    }
    else
    {
        size_to_copy_in_cur_buffer = MAX_SIZE_FOR_1_FRAME
                - filled_mic_data_so_far;
        if (1 == cur_frame)
        {
            memcpy(input_pdm_mic_data_frame1 + filled_mic_data_so_far, data,
                    size_to_copy_in_cur_buffer);
            filled_mic_data_so_far += size_to_copy_in_cur_buffer;
        }
        else
        {
            memcpy(input_pdm_mic_data_frame2 + filled_mic_data_so_far, data,
                    size_to_copy_in_cur_buffer);
            filled_mic_data_so_far += size_to_copy_in_cur_buffer;
        }

        buffer_to_send = cur_frame;
        filled_mic_data_so_far = 0;
        pdm_mic_callback_counter_frame_complete = pdm_mic_callback_counter;

        size_to_copy_in_cur_buffer = len - size_to_copy_in_cur_buffer;
        if (1 == cur_frame)
        {
            memcpy(input_pdm_mic_data_frame2 + filled_mic_data_so_far, data,
                    size_to_copy_in_cur_buffer);
            cur_frame = 2;
            filled_mic_data_so_far += size_to_copy_in_cur_buffer;
        }
        else
        {
            memcpy(input_pdm_mic_data_frame1 + filled_mic_data_so_far, data,
                    size_to_copy_in_cur_buffer);
            cur_frame = 1;
            filled_mic_data_so_far += size_to_copy_in_cur_buffer;
        }
    }

    if (1 == buffer_to_send)
    {
        convert_stereo_interleaved_10ms_frame_to_mono(input_pdm_mic_data_frame1,
                input_pdm_mic_data_mono_frame);
        app_sod_feed_data(input_pdm_mic_data_mono_frame);
    }
    else if (2 == buffer_to_send)
    {
        convert_stereo_interleaved_10ms_frame_to_mono(input_pdm_mic_data_frame2,
                input_pdm_mic_data_mono_frame);
        app_sod_feed_data(input_pdm_mic_data_mono_frame);
    }

    ret_val = CY_RSLT_SUCCESS;
    return ret_val;
#endif
}

#if 0
/*******************************************************************************
* Function Name: app_pdm_mic_handler
****************************************************************************//**
*
* PDM/PCM Interrupt Handler Implementation. 
*  
*******************************************************************************/
void app_pdm_mic_handler(void)
{   
    uint32_t count = 0;
    do
    {
        audio_data[count] = (int16_t) Cy_PDM_PCM_ReadFifo(CYBSP_PDM_PCM_HW);
        count++;
    } 
    while ((Cy_PDM_PCM_GetNumInFifo(CYBSP_PDM_PCM_HW)) != 0 && (count < MAX_AUDIO_SAMPLES));


    app_pdm_mic_accumulate_10ms_frame_and_feed((char *)&audio_data[0], count * sizeof(audio_data[0]));

    /* Clear the PCM interrupt */
    Cy_PDM_PCM_ClearInterrupt(CYBSP_PDM_PCM_HW, PDM_INTR_MASK_RX_TRIGGER_Msk);  
}

int app_pdm_mic_init(void)
{
    /* Initialize the PDM/PCM interrupt and enable it */
    Cy_SysInt_Init(&pdm_pcm_int_cfg, app_pdm_mic_handler);
    NVIC_EnableIRQ(pdm_pcm_int_cfg.intrSrc);

    /* Initialize the PDM/PCM block */
    Cy_PDM_PCM_Init(CYBSP_PDM_PCM_HW, &CYBSP_PDM_PCM_config);
    Cy_PDM_PCM_Enable(CYBSP_PDM_PCM_HW);
}
#endif
#endif

/* [] END OF FILE */
