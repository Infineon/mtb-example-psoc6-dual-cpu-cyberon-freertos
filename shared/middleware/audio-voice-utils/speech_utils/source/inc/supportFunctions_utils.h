/******************************************************************************
* File Name: supportFunctions.h
*
* Description: This file contains interface for support file/buffer management
*.. functions
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
******************************************************************************/
#ifndef __SUPPORT_FUNCTIONS_UTILS_H__
#define __SUPPORT_FUNCTIONS_UTILS_H__
#include "ifx_sp_common_priv.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>

#define ALIGN_SIZE(x, y) (((x) + (y) - 1) & ~((y) - 1))
#define ALIGN_WORD(x) ALIGN_SIZE(x, 4)


#if !EMBEDDED_DEV
int printToFile(char *fn, FILE *fid_o, void *pt, int len, int q, ifx_sp_data_type_t type);
#endif

int printSTDout(void *pt, int len, int q, ifx_sp_data_type_t type);
uint32_t reverseBits(uint32_t ulNum);
uint32_t CRC32Value(int32_t lNum, uint32_t ulPolynomial);
bool checkCRC32Value(char* ucBuffer, uint32_t ulSize, uint32_t ulPolynomial);
int calculate_pad4size(int flt_w, int *pad4flt_w);

static inline void* ifx_mem_allocate(ifx_scratch_mem_t* dPt, uint32_t byte_size)
{
    void* addr;
    int32_t cnt;

    addr = dPt->scratch_pad + dPt->scratch_cnt;
    cnt = dPt->scratch_cnt + byte_size;
    if (cnt <= dPt->scratch_size)
    {
        dPt->scratch_cnt = cnt;
        return addr;
    }
    else
    {/* Over limit error and return NULL */
        return NULL;
    }
}

static inline void ifx_mem_free(ifx_scratch_mem_t* dPt, uint32_t byte_size)
{
    dPt->scratch_cnt -= byte_size;
    assert(dPt->scratch_cnt >= 0);
}

static inline void ifx_mem_reset(ifx_scratch_mem_t* dPt)
{/* Effectively free all memory */
    dPt->scratch_cnt = 0;
}

static inline int32_t ifx_mem_counter(ifx_scratch_mem_t* dPt)
{
    return dPt->scratch_cnt;
}

#endif /*__SUPPORT_FUNCTIONS_UTILS_H__ */

/* [] END OF FILE */
