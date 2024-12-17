/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
/* $Id: fixmath.c 1.1 2004/10/14 21:55:32 rzopf Exp $ */

/*****************************************************************************
 * Copyright 2004 Broadcom Corp.  All Rights Reserved.
 *
 * $Log: fixmath.c $
 * Revision 1.1  2004/10/14 21:55:32  rzopf
 * Initial Version.
 *
 *
 ******************************************************************************/

#include <assert.h>
#include <stdint.h>
#include "CYopslib.h"
#include "fixmath.h"

int16_t pow2table[33] = {
 16384, 16743, 17109, 17484, 17867, 18258, 18658, 19066, 19484, 19911,
 20347, 20792, 21247, 21713, 22188, 22674, 23170, 23678, 24196, 24726,
 25268, 25821, 26386, 26964, 27554, 28158, 28774, 29405, 30048, 30706,
 31379, 32066, 32767 };

/*-----------------------------------------------------*
 | Table for routine Log2().                           |
 | -----------------------------------------------------*/

const int16_t tablog[ 33 ] =
{
    0,      1455,  2866,  4236,  5568,  6863,  8124,  9352, 10549, 11716,
    12855, 13967, 15054, 16117, 17156, 18172, 19167, 20142, 21097, 22033,
    22951, 23852, 24735, 25603, 26455, 27291, 28113, 28922, 29716, 30497,
    31266, 32023, 32767
};

 /* sqrt(x), Q15 - generated in Matlab with :            */
/* N=65;                                                */
/* Q=15;                                                */
/* x=linspace(0, 1, N);                                 */
/* sqrttab=max(min(round((2^Q)*sqrt(x)),32767),-32768); */

const int16_t tabsqrt[65] = {
        0,   4096,   5793,   7094,   8192,   9159,  10033,  
    10837,  11585,  12288,  12953,  13585,  14189,  14768,  
    15326,  15864,  16384,  16888,  17378,  17854,  18318,  
    18770,  19212,  19644,  20066,  20480,  20886,  21283,  
    21674,  22058,  22435,  22806,  23170,  23530,  23884,  
    24232,  24576,  24915,  25249,  25580,  25905,  26227,  
    26545,  26859,  27170,  27477,  27780,  28081,  28378,  
    28672,  28963,  29251,  29537,  29819,  30099,  30377,  
    30652,  30924,  31194,  31462,  31727,  31991,  32252,  
    32511,  32767};

int16_t Bdiv(int16_t num16, int16_t den16)
{
    int16_t out16;
    int32_t num32;
    int32_t den32;
    int16_t sign;

    if (((num16 > 0) && (den16 > 0)) || ((num16 < 0) && (den16 < 0)))
    {
        sign = 1;
    }
    else
    {
        sign = -1;
    }

    num16 = absolute(num16);
    den16 = absolute(den16);

    out16 = 0;
    if (num16 > den16)
    {
        out16 = 0;
    }
    else if (num16 == den16)
    {
        out16 = MAX_16T;
    }
    else
    {
        int i;
        num32 = num16;
        den32 = den16;

        for (i = 0; i < 15; i++)
        {
            out16 <<= 1;
            num32 <<= 1;
            if (num32 >= den32)
            {
                num32 = L_sub(num32, den32);
                out16 = add(out16, 1);
            }
        }
    }
    out16 *= sign;
    return out16;
}

/* denom must be normalized, and num < denom) */
int32_t Bdiv32(int32_t L_num, int32_t denom)
{
    int16_t approx, hi, lo, n_hi, n_lo;
    int32_t L_32;

    approx = Bdiv((int16_t)0x3fff, (int16_t)(denom >> 16)); /* result in Q14 */
    /* Note: 3fff = 0.5 in Q15 */

    /* 1/L_denom = approx * (2.0 - L_denom * approx) */

    L_32 = mult32_16s(denom, approx);         /* result in Q30 */
    L_32 = L_addc(L_32, L_32);
    L_32 = L_sub((int32_t)0x7fffffffL, L_32); /* result in Q30 */

    L_32 = mult32_16s(L_32, approx);          /* = 1/L_denom in Q29 */
    L_32 = L_addc(L_32, L_32);

    /* L_num * (1/L_denom) */

    gethilo(L_32, &hi, &lo);
    gethilo(L_num, &n_hi, &n_lo);
    L_32 = (int32_t)(LL_mult0(L_32, L_num) >> 32);
    L_32 = L_shlpc(L_32, 3); /* From Q28 to Q31 */

    return L_32;
}

/*___________________________________________________________________________
|                                                                           |
|   Function Name : Log2()                                                  |
|                                                                           |
|       Compute log2(L_x).                                                  |
|       L_x is positive.                                                    |
|                                                                           |
|       assert if L_x is negative or zero.                                  |
|---------------------------------------------------------------------------|
|  Algorithm:                                                               |
|                                                                           |
|   The function Log2(L_x) is approximated by a table and linear            |
|   interpolation.                                                          |
|                                                                           |
|   1- Normalization of L_x.                                                |
|   2- exponent = 30-exponent                                               |
|   3- i = bit25-b31 of L_x,    32 <= i <= 63  ->because of normalization.  |
|   4- a = bit10-b24                                                        |
|   5- i -=32                                                               |
|   6- fraction = tablog[i]<<16 - (tablog[i] - tablog[i+1]) * a * 2            |
|___________________________________________________________________________|
*/

void Log2(
          int32_t L_x,       /* (i) Q0 : input value                                 */
          int16_t *exponent, /* (o) Q0 : Integer part of Log2.   (range: 0<=val<=30) */
          int16_t *fraction  /* (o) Q15: Fractional  part of Log2. (range: 0<=val<1) */
          )
{
   int16_t exp, i, a, tmp;
   int32_t L_y;

#if EMBEDDED_DEV
#ifdef __ICCARM__
   exp = __iar_builtin_CLZ(L_x);
#else
   exp = __builtin_clz(L_x);
#endif
   exp--;
#else
   assert(L_x > 0);
   exp = L_norm(L_x);
#endif
   L_x = L_shlp(L_x, exp);               /* L_x is normalized */

   *exponent = sub(30, exp);

   L_x = L_shrp(L_x, 9);
   i   = extract_h(L_x);                 /* Extract b25-b31 */
   L_x = L_shrp(L_x, 1);
   a   = L_x & 0x7fff;                   /* Extract b10-b24 of fraction */

   i   = sub(i, 32);

   L_y = L_deposit_h(tablog[i]);    /* tablog[i] << 16        */
   tmp = sub(tablog[i], tablog[i+1]);  /* tablog[i] - tablog[i+1] */
   L_y = L_msu(L_y, tmp, a);             /* L_y -= tmp*a*2        */

   *fraction = extract_h( L_y);

   return;
}

/*******************************************/
/* y = sqrt(x)                             */
/* table look-up with linear interpolation */
/*******************************************/

int16_t sqrts(int16_t x)
{
   int16_t xb, y, exp, idx, sub_frac, sub_tab;
   int32_t a0;

   if(x <= 0){
      y = 0;
   }
   else{
#if EMBEDDED_DEV
#ifdef __ICCARM__
       exp = __iar_builtin_CLZ(x);
#else
       exp = __builtin_clz(x);
#endif
       exp -= 17;
#else
       exp = norm(x);
#endif
      /* use 65-entry table */
      xb = shlp(x, exp);                            // normalization of x
      idx = shrp(xb, 9);                            // for 65 entry table

      a0 = L_deposit_h(tabsqrt[idx]);              // Q31 table look-up value
      sub_frac = shlp((int16_t)(xb & 0x01FF), 6);    // Q15 sub-fraction
      sub_tab = sub(tabsqrt[idx+1], tabsqrt[idx]); // Q15 table interval for interpolation
      a0 = L_mac(a0, sub_frac, sub_tab);           // Q31 linear interpolation between table entries
      if(exp & 0x0001){
         exp = shrp(add(exp, 1), 1);                // normalization of sqrt()
         a0 = L_shrp(a0, exp);
         y = rnd(a0);                            // Q15
         a0 = L_mac(a0, 13573, y);                 // Q31 incorporate the missing "/sqrt(2)"
      }
      else{
         exp = shrp(exp, 1);                        // normalization of sqrt()
         a0 = L_shrp(a0, exp);                      // Q31
      }
      y = rnd(a0);                               // Q15
   }

   return y;
};

int16_t sqrtL(int32_t xl)
{
    int16_t shft;
    int16_t res;

#if EMBEDDED_DEV
#ifdef __ICCARM__
    shft = __iar_builtin_CLZ(xl);
#else
    shft = __builtin_clz(xl);
#endif
    shft--;
#else
    shft = L_norm(xl);
#endif
    if ((shft & 1) == 1)
        shft--;
    xl = L_shlp(xl, shft);
    res = sqrts(extract_h(xl));
    xl = L_mult(res, 181);
    shft = (shft >> 1) + 8;
    res = ((xl >> (shft - 1)) + 1) >> 1;
    return(res);
}

/****************************************************************************
 res_32 = 2^(exp.frac) in Q0
 ****************************************************************************/
int32_t pow2(int16_t exp, int16_t frac)
{
    int i;
    int16_t a;
    int32_t res_32;
    int16_t t_16;

    i = frac >> 10;
    a = frac & 1023;
    res_32 = L_shlp(pow2table[i], 16);
    t_16 = sub(pow2table[i], pow2table[i + 1]);
    res_32 = L_msu(res_32, t_16, a);
    res_32 = L_shrp_r(res_32, sub(30, exp));
    return(res_32);
}
