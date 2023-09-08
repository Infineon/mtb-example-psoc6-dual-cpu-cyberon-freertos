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
 * @file cy_staged_voice_control.h
 *
 * @brief This file is the header file for Staged Voice Control (SVC) library.
 * Depends on the CORE selected, the staged voice control module operations
 * would change. Hence appropriate internal header files would be selected
 * based on the CORE selected.
 *
 * For Example:
 *
 * 1) For Low Power device (M33), the internal header file
 * cy_staged_voice_control_lp.h would be selected. The application developer
 * need to use the API interfaces defined under cy_staged_voice_control_lp.h to
 * develop application required for low power core.
 *
 * 2) For High Performance device (M55), the internal header file
 * cy_staged_voice_control_hp.h would be selected. The application developer
 * need to use the API interfaces defined under cy_staged_voice_control_hp.h
 * to develop application required for high performance core.
 */

#ifndef __CY_STAGED_VOICE_CONTROL_H__
#define __CY_STAGED_VOICE_CONTROL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(ENABLE_SVC_LP_MW)
/**
 * Low Power device (M33) is selected, So including the corresponding header
 * file for M33
 */
#include "cy_staged_voice_control_lp.h"
#endif

#if defined(ENABLE_SVC_HP_MW)
/**
 * High Performance device (M55) is selected, So including the corresponding
 * header file for M55
 */
#include "cy_staged_voice_control_hp.h"
#else
/**
 * Invalid core has been selected. Throwing compile time error.
 */
//#error   "Invalid Core selected"
#endif


#ifdef __cplusplus
}
#endif

#endif /* __CY_STAGED_VOICE_CONTROL_H__ */
