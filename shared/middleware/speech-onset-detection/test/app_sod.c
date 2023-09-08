/******************************************************************************
* File Name: cy_sod.c
*
* Description: This file contains functions for Speech onset detection.
*              
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
* Include header file
******************************************************************************/

#ifdef RUN_SOD_ONLY
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "cy_sod.h"

void *app_sod_handle = NULL;
cy_thread_t sod_print_thread;
volatile unsigned int sod_feed_counter = 0;
volatile unsigned int sod_feed_total_process_time_ms = 0;



unsigned int process_start_time = 0;
unsigned int process_end_time = 0;

/******************************************************************************
* Defines
*****************************************************************************/
#define SOD_STATUS_CHECK_DELAY_MS (100)

#define cy_sod_log_info(format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[SOD] "format" \r\n",##__VA_ARGS__);
#define cy_sod_log_err(ret_val,format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_ERR,"[SOD] [Err:0x%lx] "format" \r\n",ret_val,##__VA_ARGS__);
#define cy_sod_log_dbg(format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_DEBUG"[SOD] "format" \r\n",##__VA_ARGS__);

/******************************************************************************
* Constants
*****************************************************************************/

/******************************************************************************
* Variables
*****************************************************************************/

bool start_feed = false;
volatile unsigned int global_speech_detected_counter = 0;

/******************************************************************************
* Functions
*****************************************************************************/

int32_t app_sod_config_prms[] = {
  0,     /* manunally generated configuration file set configuration version to zero */
  16000, /* sampling rate */
  160,   /* input frmae size */
  9,     /* IP_compnent_id: SOD */
  2,     /* number of parameters */
  400,   /* gap setting */
  16384  /* sensitivity */
};

char mono_5_sec[320*100*5];

static void sod_status_print_thread(cy_thread_arg_t thread_input)
{
    unsigned int local_speech_detected_counter = 0;
    unsigned int ignore_feed = 0;

    unsigned int local_total_process_time = 0;
    unsigned int local_sod_feed_counter = 0;

    for(;;)
    {
        cy_rtos_delay_milliseconds (SOD_STATUS_CHECK_DELAY_MS);

        if((start_feed == false) &&  ((ignore_feed++) > 100))
        {
            start_feed = true;
            cy_sod_log_info("Started SOD feed");
        }

        if (global_speech_detected_counter != local_speech_detected_counter)
        {
            local_speech_detected_counter = global_speech_detected_counter;
            local_total_process_time = sod_feed_total_process_time_ms;
            local_sod_feed_counter = sod_feed_counter;

            cy_sod_log_info("SOD detected Counter: %d, FrameCounter:%d, TotTime:%d, PerFrameTime:%d, %d %d",
                    global_speech_detected_counter,
                    sod_feed_counter,
                    local_total_process_time,
                    local_total_process_time/sod_feed_counter,
                    process_start_time,
                    process_end_time)
        }
    }
    cy_rtos_exit_thread();
}


void app_sod_create_print_status_thread(void)
{
    cy_rslt_t ret_val;

    /* Start SVC data process thread */
    ret_val = cy_rtos_create_thread(&sod_print_thread,
            sod_status_print_thread,
            "sod_status_print_thread", NULL, 2*1280,
            CY_RTOS_PRIORITY_ABOVENORMAL, NULL);

    if (ret_val != CY_RSLT_SUCCESS)
    {
        cy_sod_log_err(ret_val, "Unable to create the UT thread func");
    }
    else
    {
        cy_sod_log_info("create the UT thread success");
    }
    return;
}

cy_rslt_t app_sod_init(void)
{
    cy_rslt_t ret_val;

    app_sod_create_print_status_thread();

    ret_val = cy_sod_init(app_sod_config_prms, &app_sod_handle);
    if (CY_RSLT_SUCCESS == ret_val)
    {
        cy_sod_log_info("cy_sod_init init done");
    }
    else
    {
        cy_sod_log_err(ret_val, "cy_sod_init init fail");
    }
    return ret_val;
}


cy_rslt_t app_sod_feed_data(char *data)
{
    cy_rslt_t ret_val;
    bool b_speech_detected = false;

    if(start_feed == false) return ret_val;

    cy_rtos_get_time(&process_start_time);
    ret_val = cy_sod_process(app_sod_handle, (int16_t*)data, &b_speech_detected);
    cy_rtos_get_time(&process_end_time);

    sod_feed_counter++;
    sod_feed_total_process_time_ms += process_end_time - process_start_time;

    if ((CY_RSLT_SUCCESS == ret_val) && (b_speech_detected == true))
    {
        global_speech_detected_counter++;
    }
    else
    {
//        cy_app_cm0_log_info("SOD (cy_sod_process) is not detected");
    }

    return ret_val;
}

cy_rslt_t app_sod_deinit(void)
{
    cy_rslt_t ret_val;

    ret_val = cy_sod_deinit(app_sod_handle);

    app_sod_handle = NULL;

    return ret_val;
}
#endif
