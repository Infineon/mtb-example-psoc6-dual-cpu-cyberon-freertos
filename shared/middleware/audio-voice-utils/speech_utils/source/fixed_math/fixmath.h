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

/** @file
 *
 */

/*
 * $ Copyright Broadcom Corporation $
 */
/* $Id: fixmath.h 1.1 2004/10/14 21:55:32 rzopf Exp $ */

/*****************************************************************************
 * Copyright 2004 Broadcom Corp.  All Rights Reserved.
 *
 * $Log: fixmath.h $
 * Revision 1.1  2004/10/14 21:55:32  rzopf
 * Initial Version.
 *
 *
 ******************************************************************************/
#ifndef FIXMATH_H
#define FIXMATH_H

#ifdef __cplusplus
extern "C" {
#endif



int16_t Bdiv( int16_t num16, int16_t den16 );
int32_t Bdiv32( int32_t L_num, int32_t denom );
int32_t pow2(int16_t exp, int16_t frac);
//double expm1(double x);
void expm1_fx(int32_t x_32, int16_t Q_x_32, int32_t *res_32, int16_t *Q_res_32);
void Log2(
          int32_t L_x,       /* (i) Q0 : input value                                 */
          int16_t *exponent, /* (o) Q0 : Integer part of Log2.   (range: 0<=val<=30) */
          int16_t *fraction  /* (o) Q15: Fractional  part of Log2. (range: 0<=val<1) */
          );
int16_t sqrts(int16_t x);
int16_t sqrtL(int32_t xl);

#ifdef __cplusplus
}
#endif

#endif
