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

#include "cy_sod_private.h"

/******************************************************************************
* Defines
*****************************************************************************/

/******************************************************************************
* Constants
*****************************************************************************/

/******************************************************************************
* Variables
*****************************************************************************/

/******************************************************************************
* Functions
*****************************************************************************/

cy_rslt_t cy_sod_init_sod_system_component(sod_context_t *sod_context)
{
    uint32_t ErrIdx = 0;
    cy_rslt_t ret_val = CY_RSLT_SOD_GENERIC_ERROR;

    /* Initialize SOD and get SOD Container/object */
      ErrIdx |= ifx_pre_post_process_init(sod_context->sod_config_prms,
              &sod_context->ifx_sod_container, &sod_context->g_sod_info);
      if (ErrIdx)
      {
          ret_val = CY_RSLT_SOD_PROCESS_INIT_FAIL;
          cy_sod_app_log_err(ret_val,
                  "SOD initialization fail, ErrIdx=%x,CIdx=%d, LineNo=%d",
                  IFX_SP_ENH_ERR_CODE(ErrIdx),
                  IFX_SP_ENH_ERR_COMPONENT_INDEX(ErrIdx),
                  IFX_SP_ENH_ERR_LINE_NUMBER(ErrIdx));
          goto CLEAN_RETURN;
      }

      /* Setup profile configuration */
      ErrIdx |= ifx_pre_post_profile_init(sod_context->ifx_sod_container,
              IFX_PRE_PROCESS_IP_COMPONENT_SOD, false, NULL, NULL);
      if (ErrIdx)
      {
          ret_val = CY_RSLT_SOD_PROCESS_INIT_FAIL;
          cy_sod_app_log_err(ret_val,
                  "SOD initialization fail, ErrIdx=%x,CIdx=%d, LineNo=%d",
                  IFX_SP_ENH_ERR_CODE(ErrIdx),
                  IFX_SP_ENH_ERR_COMPONENT_INDEX(ErrIdx),
                  IFX_SP_ENH_ERR_LINE_NUMBER(ErrIdx));
          goto CLEAN_RETURN;
      }

      ret_val = CY_RSLT_SUCCESS;
      CLEAN_RETURN:
      return ret_val;
}

cy_rslt_t cy_sod_init_parse_and_allot_resource_for_sod(sod_context_t *sod_context)
{
    cy_rslt_t ret_val = CY_RSLT_SOD_GENERIC_ERROR;
    int ret = 0;
    ifx_stc_pre_post_process_info_t *sod_info = NULL;

    sod_info = &sod_context->g_sod_info;

    /* Step 1: Parse and get required memory for SOD configuration */
    ret = ifx_pre_post_process_parse(sod_context->sod_config_prms, sod_info);

    /* Step 2: Allocate memory */
    if (0 == ret) /*Model Parsing successful*/
    {
        sod_info->memory.persistent_mem_pt = (char*) malloc(
                sod_info->memory.persistent_mem);
        if (sod_info->memory.persistent_mem_pt == NULL)
        {
            ret_val = CY_RSLT_SOD_OUT_OF_MEMORY;
            cy_sod_app_log_err(ret_val, "mem alloc fail");
            return ret_val;
        }

        CY_MEM_UTIL_PRINT_ADDR("Alloc:sod_persistent_mem", sod_info->memory.persistent_mem_pt, sod_info->memory.persistent_mem);

        sod_info->memory.scratch_mem_pt = (char*) malloc(
                sod_info->memory.scratch_mem);
        if (sod_info->memory.scratch_mem_pt == NULL)
        {
            ret_val = CY_RSLT_SOD_OUT_OF_MEMORY;
            cy_sod_app_log_err(ret_val, "mem alloc fail");
            return ret_val;
        }

        CY_MEM_UTIL_PRINT_ADDR("Alloc:sod_scratch_mem", sod_info->memory.scratch_mem_pt, sod_info->memory.scratch_mem);

        cy_sod_app_log_info("Mem allocated: scratch:%d, persistent:%d",
                sod_info->memory.scratch_mem,
                sod_info->memory.persistent_mem);
    }
    else
    {
        ret_val =  CY_RSLT_SOD_PROCESS_PARSE_FAIL;
        cy_sod_app_log_err(ret_val, "Get memory failed! Error code=%x, "
                "Component index=%d, Line number=%d",
                IFX_SP_ENH_ERR_CODE(ret), IFX_SP_ENH_ERR_COMPONENT_INDEX(ret),
                IFX_SP_ENH_ERR_LINE_NUMBER(ret));
        goto CLEAN_RETURN;
    }

    ret_val = CY_RSLT_SUCCESS;

    CLEAN_RETURN:
    return ret_val;
}
