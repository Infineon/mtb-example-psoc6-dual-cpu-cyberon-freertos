
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
 * @file cy_mem_check_utils.c
 *
 */

#include "cy_mem_check_utils.h"

#if !defined(__ARMCC_VERSION)
#if defined(__GNUC__)
#include <malloc.h>
#define MALLINFO mallinfo
#elif defined(__IAR_SYSTEMS_ICC__)
#include <iar_dlmalloc.h>
#define MALLINFO __iar_dlmallinfo
#endif
#endif

long malloc_info_command( void );

#ifdef ENABLE_MALLOC_INFO_STATS
#define cy_mem_log_allocated_memory(format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[MALLINFO] "format" \r\n",##__VA_ARGS__);
#define CY_MEM_GET_ALLOCATED_MEMORY(__X__) cy_mem_log_allocated_memory("%s, mem_allocated: %ld",  __X__, malloc_info_command());
#else
#define CY_MEM_GET_ALLOCATED_MEMORY(__X__)
#endif

void cy_mem_get_allocated_memory(char* str)
{
    CY_MEM_GET_ALLOCATED_MEMORY(str);
}

#if !defined(__ARMCC_VERSION)
long malloc_info_command( void )
{
    volatile struct mallinfo mi = MALLINFO();
    long used_memory;
    used_memory = mi.usmblks + mi.uordblks;
    return used_memory;
}
#endif
