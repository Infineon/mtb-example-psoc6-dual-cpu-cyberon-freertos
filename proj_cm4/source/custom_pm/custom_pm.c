/******************************************************************************
 * File Name:   CUSTOM_PM.c
 *
 * Description: This file contains custom settings of power management in system deep sleep.
 *
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "custom_pm.h"

/****************************************************************************
 * Constants
 *****************************************************************************/

/****************************************************************************
 * Global variables
 *****************************************************************************/

/****************************************************************************
 * Functions Definitions
 *****************************************************************************/
/* Registers a custom power management callback that prepares the clock system 
 for entering deep sleep mode
 and restore the clocks upon wakeup from deep sleep.
 NOTE: This is called automatically as part of \ref cybsp_init */

cy_rslt_t cybsp_register_custom_sysclk_pm_callback(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    static cy_stc_syspm_callback_params_t cybsp_sysclk_pm_callback_param = {NULL, NULL};
    static cy_stc_syspm_callback_t cybsp_sysclk_pm_callback =
        {
            .callback = &Cy_SysClk_Custom_DeepSleepCallback,
            .type = CY_SYSPM_DEEPSLEEP,
            .callbackParams = &cybsp_sysclk_pm_callback_param,
            .order = CYBSP_SYSCLK_PM_CALLBACK_ORDER};

    if (!Cy_SysPm_RegisterCallback(&cybsp_sysclk_pm_callback))
    {
        result = CYBSP_RSLT_ERR_SYSCLK_PM_CALLBACK;
    }
    return result;
}

/** \cond INTERNAL */
/* Timeout count for use in function Cy_SysClk_DeepSleepCallback() is sufficiently large for ~1 second */
#define TIMEOUT (1000000UL)
/** \endcond */

cy_en_syspm_status_t Cy_SysClk_Custom_DeepSleepCallback(cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{

    /* Bitmapped paths with enabled FLL/PLL sourced by ECO */
    static uint16_t changedSourcePaths;
    static uint16_t pllAutoModes;

    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;

    (void)callbackParams; /* Suppress "not used" warning */
    (void)changedSourcePaths;
    (void)pllAutoModes;

    switch (mode)
    {
    case CY_SYSPM_CHECK_READY:
        /* Don't allow entry into Deep Sleep mode if currently measuring a frequency */
        break;

    case CY_SYSPM_CHECK_FAIL:
        /* Cancellation of going into Deep Sleep, therefore allow a new clock measurement */
        retVal = CY_SYSPM_SUCCESS;
        break;

    case CY_SYSPM_BEFORE_TRANSITION:
    {
        retVal = CY_SYSPM_SUCCESS;
    }
    break;

    case CY_SYSPM_AFTER_TRANSITION:
    {
        /* Allow clock measurement */
        retVal = CY_SYSPM_SUCCESS;
    }
    break;

    default: /* Unsupported mode, return CY_SYSPM_FAIL */
        break;
    }

    return (retVal);
}