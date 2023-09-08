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

/**************************************************************************************
 * Bopslib.c
 *
 * This file contains basic operators for 16/32/64 bit fixed point math
 *
 * Author: Robert Zopf
 * Cypress Semiconductor.
 * 03/24/2017
 **************************************************************************************/

//#include "typedef.h"
#include <stdint.h>
#include "CYopslib.h"

#if OF_CHECK
int16_t OFflag = 0;
#endif

#if !INLINE_OPS
void CYopsliberr()
{
   int16_t  a;
   a=0; /* set a break point here during development/debugging */
}
#endif

/* 64-bit MAC with 32-bit inputs */
#if !INLINE_OPS
int64_t LL_mac( int64_t acc, int32_t x, int32_t y )
{
    return acc+(((int64_t)x) * y);
}
#endif

/* 64-bit norm */
int16_t LL_norm( int64_t in64 )
{
    int shift;

    if ( in64 == 0 )
    {
        shift = 0;
    }
    else if ( in64 == MIN64_T )
    {
        shift = 0;
    }
    else
    {
        if ( in64 < 0 )
        {
            in64 = -in64;
        }
        for ( shift = 0; in64 < (int64_t) 0x4000000000000000; shift++ )
        {
            in64 <<= 1;
        }
    }
    return shift;
}


int16_t norm(int16_t in16)
{
   int16_t shift=0;

   if (in16>0)
   {
      while (in16<16384)
      {
         shift++;
         in16<<=1;
      }
   }
   if (in16<0)
   {
      while (in16>-16384)
      {
         shift++;
         in16<<=1;
      }
   }
   return(shift);
}

/* 32-bit norm */
int16_t L_norm( int32_t in32 )
{
    int shift;

    if ( in32 == 0 )
    {
        shift = 0;
    }
    else if ( in32 == MIN_32T )
    {
        shift = 0;
    }
    else
    {
        if ( in32 < 0 )
        {
            in32 = -in32;
        }
        for ( shift = 0; in32 < (int32_t) 0x40000000; shift++ )
        {
            in32 <<= 1;
        }
    }
    return shift;
}


/* 32x16 bit mult with 15-bit downshift returning 32 bit result */
#if !INLINE_OPS
int32_t mult32_16( int32_t in32, int16_t in16 )
{
    return (int32_t) ( ( (int64_t) ( in32 ) * (int64_t) ( in16 ) ) >> 15 );
}
#endif

/* 32x16 bit mult with 16-bit downshift returning 32 bit result */
#if !INLINE_OPS
int32_t mult32_16s( int32_t in32, int16_t in16 )
{
    return (int32_t) ( ( (int64_t) ( in32 ) * (int64_t) ( in16 ) ) >> 16 );
}
#endif

/* 32x32 bit multiply returning 64 bits with no left shift */
#if !INLINE_OPS
int64_t LL_mult0( int32_t x32, int32_t y32 )
{
    return ( (int64_t) x32 ) * y32;
}
#endif

/* 32x32 bit multiply >> 31 return 32-bit */
#if !INLINE_OPS
int64_t LL_mult(int32_t x32, int32_t y32)
{
    return  ((int32_t)((((int64_t)(x32)) * (y32)) >> 31));
}
#endif



/* 32-bit add with clipping */
int32_t L_addc( int32_t x32, int32_t y32 )
{
    int32_t res32;

    res32 = x32 + y32;
    if ( ( x32 < 0 ) && ( y32 < 0 ) && ( res32 > 0 ) )
    {
        res32 = MIN_32T;
    }
    else if ( ( x32 > 0 ) && ( y32 > 0 ) && ( res32 < 0 ) )
    {
        res32 = MAX_32T;
    }
    return res32;
}


/* 32-bit add with NO clipping */
#if !INLINE_OPS
int32_t L_add( int32_t x32, int32_t y32 )
{
    int32_t res32;

    res32 = x32 + y32;
    #if OF_CHECK
    if ( ( ( x32 > 0 ) && ( y32 > 0 ) && ( res32 < 0 ) ) || ( ( x32 < 0 ) && ( y32 < 0 ) && ( res32 > 0 ) ) )
    {
        OFflag = 1;
    }
    #endif
    return res32;
}
#endif

/* 32-bit abs */
int32_t L_absolute( int32_t in32 )
{
    int32_t out32;

    out32 = in32;
    if ( in32 == MIN_32T )
    {
        out32 = MAX_32T;
    }
    else if ( in32 < 0 )
    {
        out32 = -in32;
    }
    return out32;
}


/* 16-bit abs */
int16_t absolute( int16_t in16 )
{
    int16_t out16;

    out16 = in16;
    if ( in16 == MIN_16T )
    {
        out16 = MAX_16T;
    }
    else if ( in16 < 0 )
    {
        out16 = -in16;
    }
    return out16;
}


/* 32-bit sub with NO clipping */
#if !INLINE_OPS
int32_t L_sub( int32_t x32, int32_t y32 )
{
    int32_t res32;

    res32 = x32 - y32;
    #if OF_CHECK
    if ( ( ( x32 > 0 ) && ( y32 < 0 ) && ( res32 < 0 ) ) || ( ( x32 < 0 ) && ( y32 > 0 ) && ( res32 > 0 ) ) )
    {
        OFflag = 1;
    }
    #endif
    return res32;
}
#endif

/* 32-bit sub with clipping */
int32_t L_subc( int32_t x32, int32_t y32 )
{
    int32_t res32;

    res32 = x32 - y32;

    if ( ( x32 > 0 ) && ( y32 < 0 ) && ( res32 < 0 ) )
    {
        res32 = MAX_32T;
    }
    else if ( ( x32 < 0 ) && ( y32 > 0 ) && ( res32 > 0 ) )
    {
        res32 = MIN_32T;
    }

    return res32;
}


/* 16-bit add */
#if !INLINE_OPS
int16_t add( int16_t x16, int16_t y16 )
{
    int16_t res16;

    res16 = x16 + y16;
    #if OF_CHECK
    if ( ( ( x16 > 0 ) && ( y16 > 0 ) && ( res16 < 0 ) ) || ( ( x16 < 0 ) && ( y16 < 0 ) && ( res16 > 0 ) ) )
    {
        OFflag = 1;
    }
    #endif
    return res16;
}
#endif

/* 16-bit sub */
#if !INLINE_OPS
int16_t sub( int16_t x16, int16_t y16 )
{
    int16_t res16;

    res16 = x16 - y16;
    #if OF_CHECK
    if ( ( ( x16 < 0 ) && ( y16 > 0 ) && ( res16 > 0 ) ) || ( ( x16 > 0 ) && ( y16 < 0 ) && ( res16 < 0 ) ) )
    {
        OFflag = 1;
    }
    #endif
    return res16;
}
#endif

/* 16-bit add with clipping */
int16_t addc( int16_t x16, int16_t y16 )
{
    int16_t res16;

    res16 = x16 + y16;
    if ( ( x16 > 0 ) && ( y16 > 0 ) && ( res16 < 0 ) )
    {
        res16 = MAX_16T;
    }
    else if ( ( x16 < 0 ) && ( y16 < 0 ) && ( res16 > 0 ) )
    {
        res16 = MIN_16T;
    }
    return res16;
}
#if !INLINE_OPS
/* 16-bit shift leff positive */
int16_t shlp(int16_t in16, int16_t shft)
{
   int16_t out16;

    #if OF_CHECK
    if ( shft < 0 )
    {
        OFflag = 1;
    }
    #endif
    if ( shft < 0 )
    {
        shft = 0;
    }
    out16 = in16;
    for ( ; shft > 0; shft-- )
    {
        out16 = add( out16, out16 );
    }
    return out16;
}
#endif

/* 32-bit shift left with clipping */
int32_t L_shlc( int32_t in32, int16_t shft )
{
    int32_t out32;

    if ( shft >= 0 )
    {
        out32 = L_shlpc( in32, shft );
    }
    else
    {
        out32 = L_shlpc( in32, -shft );
    }
    return out32;
}


/* 32-bit shift right with clipping */
int32_t L_shrc( int32_t in32, int16_t shft )
{
    int32_t out32;

    if ( shft >= 0 )
    {
        out32 = L_shrp( in32, shft );
    }
    else
    {
        out32 = L_shlpc( in32, -shft );
    }
    return out32;
}


/* 32-bit shift left by positive amount */
#if !INLINE_OPS
int32_t L_shlp( int32_t in32, int16_t shft /* + only */ )
{
    int32_t out32;

    #if OF_CHECK
    if ( shft < 0 )
    {
        OFflag = 1;
    }
    #endif
    if ( shft < 0 )
    {
        shft = 0;
    }
    out32 = in32;
    for ( ; shft > 0; shft-- )
    {
        out32 = L_add( out32, out32 );
    }
    return out32;
}
#endif

/* 32-bit shift left by positive amount with clipping */
int32_t L_shlpc( int32_t in32, int16_t shft /* + only */ )
{
    int32_t out32;

#if OF_CHECK
    if ( shft < 0 )
    {
        OFflag = 1;
    }
#endif
    if ( shft < 0 )
    {
        shft = 0;
    }
    out32 = in32;
    for ( ; shft > 0; shft-- )
    {
        out32 = L_addc( out32, out32 );
    }
    return out32;
}

/* 32-bit shift left by positive amount with clipping */
int16_t shlpc( int16_t in16, int16_t shft /* + only */ )
{
    int16_t out16;

#if OF_CHECK
    if ( shft < 0 )
    {
        OFflag = 1;
    }
#endif
    if ( shft < 0 )
    {
        shft = 0;
    }
    out16 = in16;
    for ( ; shft > 0; shft-- )
    {
        out16 = addc( out16, out16 );
    }
    return out16;
}


/* 32-bit shift right by positive amount */
#if !INLINE_OPS
int32_t L_shrp( int32_t in32, int16_t shft /* + only */ )
{
    int32_t out32;

    #if OF_CHECK
    if ( shft < 0 )
    {
        OFflag = 1;
    }
    #endif
    if ( shft < 0 )
    {
        shft = 0;
    }
    out32 = in32 >> shft;

    return out32;
}
#endif

/* 16x16 mult with 1 bit left-shift with clipping */
int32_t L_multc( int16_t x16, int16_t y16 )
{
    int32_t res32;

    res32 = ( (int32_t) x16 ) * y16;
    res32 = L_addc( res32, res32 );
    return res32;
}


/* 16x16 mult with 1 bit left-shift */
#if !INLINE_OPS
int32_t L_mult( int16_t x16, int16_t y16 )
{
    int32_t res32;

    res32 = ( (int32_t) x16 ) * y16;
    res32 = L_add( res32, res32 );
    return res32;
}
#endif

/* multiply, shift left 1, subtract (negative mac basically), with clipping */
int32_t L_msuc( int32_t mac32, int16_t x16, int16_t y16 )
{
    int32_t res32;

    res32 = L_multc( x16, y16 );
    mac32 = L_subc( mac32, res32 );
    return mac32;
}


/* multiply, shift left1, subtract (negative mac basically) */
#if !INLINE_OPS
int32_t L_msu( int32_t mac32, int16_t x16, int16_t y16 )
{
    int32_t res32;

    res32 = L_mult( x16, y16 );
    mac32 = L_sub( mac32, res32 );
    return mac32;
}
#endif

/* extract hi and lo parts of 32-bit */
void gethilo( int32_t in32, int16_t* hi16, int16_t* lo16 )
{
    int32_t res32;

    *hi16  = (int16_t) L_shrp( in32, 16 );
    res32  = L_shrp( in32, 1 );
    res32  = L_msuc( res32, *hi16, 16384 );
    *lo16  = (int16_t) res32;
}


/* 32-bit negate */
#if !INLINE_OPS
int32_t L_negate( int32_t in32 )
{
    int32_t res32;

    res32 = ( in32 == MIN_32T ) ? MAX_32T : -in32;
    return res32;
}
#endif

/* 32-bit round and return the upper 15 bits + sign*/
int16_t rnd( int32_t in32 )
{
    return (int16_t) ( L_addc( in32, (int32_t) 0x00008000L ) >> 16 );
}


/* 16x16 multiply into 32-bits with NO left shift*/
#if !INLINE_OPS
int32_t L_mult0( int16_t x16, int16_t y16 )
{
    return ( (int32_t) x16 ) * y16;
}
#endif

/* 16x16 mult and return upper 16-bits with leftshift 1 with rounding */
#if !INLINE_OPS
int16_t mult_r( int16_t x16, int16_t y16 )
{
    int32_t res32;

    res32 = L_mult0( x16, y16 );
    res32 = L_add( res32, 16384 );
    res32 = L_shrp( res32, 15 );
    return (int16_t) res32;
}
#endif

/* 16x16 mult and return upper 16-bits with leftshift 1*/
#if !INLINE_OPS
int16_t mult( int16_t x16, int16_t y16 )
{
    return (int16_t) ( ( ( (int32_t) x16 ) * y16 ) >> 15 );
}
#endif

/* 16x16 mac into 32 bits with no leftshift */
#if !INLINE_OPS
int32_t L_mac0( int32_t acc32, int16_t x16, int16_t y16 )
{
    int32_t res32;

    res32 = L_mult0( x16, y16 );
    acc32 = L_add( acc32, res32 );
    return acc32;
}
#endif

/* 16x16 multiply subtract into 32 bits with no leftshift */
#if !INLINE_OPS
int32_t L_msu0( int32_t acc32, int16_t x16, int16_t y16 )
{
    int32_t res32;

    res32 = L_mult0( x16, y16 );
    acc32 = L_sub( acc32, res32 );
    return acc32;
}
#endif

int32_t L_shrp_r( int32_t in_32, int16_t shft /* + only */ )
{
   int32_t res_32;

   res_32 = L_shrp(in_32, shft);
   if (shft>0)
   {
      if (in_32 > 0)
      {
         if (L_sub(in_32, L_shlp(res_32, shft)) >= (L_shlp(1, shft-1)))
            res_32 = L_add(res_32, 1);
      }
      else
      {
         if (L_sub(in_32, L_shlp(res_32, shft)) > (L_shlp(1, shft-1)))
            res_32 = L_add(res_32, 1);
      }
   }
   return(res_32);
}

int16_t shrp_r (int16_t in_16, int16_t shft)
{
    int16_t res_16;

#if !INLINE_OPS
    if (shft < 0)
       CYopsliberr();
#endif
    if (shft > 15)
    {
        res_16 = 0;
    }
    else
    {
        res_16 = shrp (in_16, shft);
        if (shft > 0)
        {
            if ((in_16 & ((int16_t) 1 << (shft - 1))) != 0)
            {
                res_16++;
            }
        }
    }
    return (res_16);
}

#if !INLINE_OPS
int16_t shrp (int16_t in_16, int16_t shft)
{
   int16_t res_16;
#if !INLINE_OPS
    if (shft < 0)
       CYopsliberr();
#endif
    res_16 = in_16>>shft;
    return(res_16);
}
#endif

#if !INLINE_OPS
int16_t extract_h (int32_t in_32)
{
    int16_t res_16;

    res_16 = (int16_t) (in_32 >> 16);
    return (res_16);
}
#endif

#if !INLINE_OPS
int16_t extract_l (int32_t in_32)
{
    int16_t res_16;

    res_16 = (int16_t) in_32;
    return (res_16);
}
#endif

#if !INLINE_OPS
int32_t L_deposit_h (int16_t in_16)
{
    int32_t res_32;

    res_32 = (int32_t) (in_16 << 16);
    return (res_32);
}
#endif

#if !INLINE_OPS
int32_t L_mac(int32_t res_32, int16_t in1_16, int16_t in2_16)
{
   res_32 = L_add(res_32, L_mult(in1_16, in2_16));
   return(res_32);
}
#endif
