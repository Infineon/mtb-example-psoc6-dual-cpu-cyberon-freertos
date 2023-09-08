/******************************************************************************
* File Name: supportFunctions.c
*
* Description: This file contains general purpose file/buffer management
*  functions
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
* Include header file
******************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ifx_sp_common_priv.h"

#ifndef ENABLE_IFX_LPWWD
#if EMBEDDED_DEV
/*
Definition: Prints Data stdout
fn: if length of "fn" > 0, it will open a file, otherwise use FILE
fid_o: opened file
pt: data-pointer
len: length of data
q:  (optional) Q value, set < 0 for no scaling
type: 0: float32, 1: int32, 2: int16, 3: double, 4: int8
*/
int printSTDout(void *pt, int len, int q, ifx_sp_data_type_t type)
{
    switch (type)
    {
        case IFX_SP_DATA_FLOAT:
        {
            float *pt_t;
            pt_t = (float *)pt;
            for (int j = 0; j < len; j++)
            {
                printf("%12.6f ", *pt_t++);
            }
            printf("\r\n");
        }
        break;
        case IFX_SP_DATA_INT16:
        {
            int16_t *pt_t;
            pt_t = (int16_t *)pt;
            float norm_fac = 1.0f;
            if (q > 0)
            {
                norm_fac = (float)(1.0f / powf(2., (float)q));
            }
            else
            {
                norm_fac = (float)(1 << (-q));
            }

            for (int j = 0; j < len; j++)
            {
                printf("%12.6f ", ((float)(*pt_t++))*norm_fac);
            }
            printf("\r\n");
        }
        break;
        case IFX_SP_DATA_INT8:
        {
            int8_t *pt_t;
            pt_t = (int8_t *)pt;
            float norm_fac = 1.0f;
            if (q > 0)
            {
                norm_fac = (float)(1.0f / powf(2., (float)q));
            }
            else
            {
                norm_fac = (float)(1 << (-q));
            }
            for (int j = 0; j < len; j++)
            {
                printf("%12.6f ", ((float)(*pt_t++))*norm_fac);
            }
            printf("\r\n");
        }
        break;
        default:
        break;
    }
    return 0;
}

#else
/*
Definition: Prints Data to file
fn: if length of "fn" > 0, it will open a file, otherwise use FILE
fid_o: opened file
pt: data-pointer
len: length of data
q:  (optional) Q value, set < 0 for no scaling
type: 0: float32, 1: int32, 2: int16, 3: double, 4: int8
*/
int printToFile(char *fn, FILE *fid_o, void *pt, int len, int q, int16_t zero_offset, ifx_sp_data_type_t type)
{
    FILE *fid;
    if (strlen(fn) > 0)
    {
        if ((fid = fopen(fn, "w")) == NULL)
        {
            fprintf(stderr, "can't open %s, skipping!\n", fn);
            return -1;
        }
    }
    else
    {
        // use already openfile
        fid = fid_o;
        if (fid == NULL)
        {
            fprintf(stderr, "File pointer is NULL, skipping!\n");
            return -1;
        }
    }
    switch (type)
    {
        case IFX_SP_DATA_FLOAT:
        {
            float *pt_t;
            pt_t = (float *)pt;
            for (int j = 0; j < len; j++)
            {
                fprintf(fid, "%12.6f ", *pt_t++);
#if ML_DEBUG
                if ((j+1) % 10 == 0) fprintf(fid, "\n");
#endif
            }
            fprintf(fid, "\n");
        }
        break;
        case IFX_SP_DATA_INT16:
        {
            int16_t *pt_t;
            pt_t = (int16_t *)pt;
            float norm_fac = 1.0f;
            if (q > 0)
            {
                norm_fac = (float)(1.0f / powf(2., (float)q));
            }
            else
            {
                norm_fac = (float)(1 << (-q));
            }

            for (int j = 0; j < len; j++)
            {
                fprintf(fid, "%12.6f ", ((float)(*pt_t++)) * norm_fac);
#if ML_DEBUG
                if ((j+1) % 10 == 0) fprintf(fid, "\n");
#endif
            }
            fprintf(fid, "\n");
        }
        break;
        case IFX_SP_DATA_INT8:
        {
            int8_t *pt_t;
            pt_t = (int8_t *)pt;
            float norm_fac = 1.0f;
            if (q > 0)
            {
                norm_fac = (float)(1.0f / powf(2., (float)q));
            }
            else
            {
                norm_fac = (float)(1 << (-q));
            }
            for (int j = 0; j < len; j++)
            {
                fprintf(fid, "%12.6f ", ((float)((int32_t)(*pt_t++) + zero_offset)) * norm_fac);
#if ML_DEBUG
                if ((j+1) % 10 == 0) fprintf(fid, "\n");
#endif
            }
            fprintf(fid, "\n");
        }
        break;
        default:
        break;
    }
    if (strlen(fn) > 0)
        fclose(fid);

    return 0;
}

#endif /* EMBEDDED_DEV */

uint32_t reverseBits(uint32_t ulNum)
{
    uint32_t uiNumOfBits = sizeof(ulNum) * 8;
    uint32_t ulReverseNum = 0;
    uint32_t i;

    for (i = 0; i < uiNumOfBits; i++)
    {
        if ((ulNum & (1 << i)))
        {
            ulReverseNum |= 1 << ((uiNumOfBits - 1) - i);
        }
    }

    return ulReverseNum;
}

uint32_t CRC32Value(int32_t lNum, uint32_t ulPolynomial)
{
    uint32_t ulCRC = lNum;
    uint32_t ulPolynomialReverse = reverseBits(ulPolynomial);
    uint32_t i;

    for (i = 8; i > 0; i--)
    {
        if (ulCRC & 1)
        {
            ulCRC = (ulCRC >> 1) ^ ulPolynomialReverse;
        }
        else
        {
            ulCRC >>= 1;
        }
    }

    return ulCRC;
}

bool checkCRC32Value(char* ucBuffer, uint32_t ulSize, uint32_t ulPolynomial)
{
    uint32_t ulTemp1;
    uint32_t ulTemp2;
    uint32_t ulCRCsum = 0;
    uint32_t ulCount = ulSize;
    uint32_t ulCRCref = *((uint32_t*)(ucBuffer + ulSize));

    while (ulCount-- != 0)
    {
        ulTemp1 = (ulCRCsum >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int32_t)ulCRCsum ^ *ucBuffer++) & 0xff, ulPolynomial);
        ulCRCsum = ulTemp1 ^ ulTemp2;
    }

    //printf("0x%08x 0x%08x\n", ulCRCsum, ulCRCref);

    if (ulCRCsum > 0 && ulCRCref > 0 && ulCRCsum == ulCRCref)
    {
        return false;
    }
    else
    {
        return true;
    }
}
#endif
