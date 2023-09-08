/******************************************************************************
* File Name: cy_sod.h
*
* Description: Speech onset detection module header file with APIs and data structures.
*
* Related Document: See README.md
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
* Include guard
*******************************************************************************/
#ifndef __CY_SOD_PRIVATE_H__
#define __CY_SOD_PRIVATE_H__

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cy_mem_check_utils.h"
#include "stdint.h"
#include "ifx_sp_utils.h"
#include "ifx_pre_post_process.h"

#include "cy_errors.h"
#include "cy_log.h"
#include "cy_sod_errors.h"
#include "cy_sod.h"

/* This is the total length of sod_config_prms array */
#define MAX_SOD_CONFIG_PARAMS_LEN (7)

#if ENABLE_SOD_LOGS == 2
#define cy_sod_app_log_info(format,...)  printf ("[SOD] "format" \r\n",##__VA_ARGS__);
#define cy_sod_app_log_err(ret_val,format,...)  printf ("[SOD] [Err:0x%lx] "format" \r\n",ret_val,##__VA_ARGS__);
#define cy_sod_app_log_dbg(format,...)  printf ("[SOD] "format" \r\n",##__VA_ARGS__);
#elif ENABLE_SOD_LOGS
#define cy_sod_app_log_info(format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[SOD] "format" \r\n",##__VA_ARGS__);
#define cy_sod_app_log_err(ret_val,format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_ERR,"[SOD] [Err:0x%lx] "format" \r\n",ret_val,##__VA_ARGS__);
#define cy_sod_app_log_dbg(format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_DEBUG"[SOD] "format" \r\n",##__VA_ARGS__);
#else
#define cy_sod_app_log_info(format,...)
#define cy_sod_app_log_err(ret_val,format,...)
#define cy_sod_app_log_dbg(format,...)
#endif

typedef struct
{
    ifx_stc_pre_post_process_info_t g_sod_info;
    int32_t sod_config_prms[MAX_SOD_CONFIG_PARAMS_LEN];
    void *ifx_sod_container;
} sod_context_t;


cy_rslt_t cy_sod_init_sod_system_component(sod_context_t *sod_context);
cy_rslt_t cy_sod_init_parse_and_allot_resource_for_sod(sod_context_t *sod_context);


#endif /*__CY_SOD_PRIVATE_H__ */

/* [] END OF FILE */
