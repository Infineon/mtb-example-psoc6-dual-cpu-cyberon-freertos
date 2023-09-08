/******************************************************************************
* File Name: ifx_sp_enh.h
*
* Description: This file contains public interface for Infineon HP speech enhancement app
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
#ifndef __IFX_SP_ENH_H
#define __IFX_SP_ENH_H

/*******************************************************************************
* Include header file
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "ifx_sp_common.h"
/*******************************************************************************
* Compile-time flags
*******************************************************************************/

/*******************************************************************************
* High performance speech enhancement data type & defines
*******************************************************************************/
#define IFX_SP_ENH_VERSION_MAJOR            1
#define IFX_SP_ENH_VERSION_MINOR            0
#define IFX_SP_ENH_VERSION_PATCH            0
#define IFX_SP_ENH_VERSION                  100

/*******************************************************************************
* Structures and enumerations
*******************************************************************************/
/** Defines BF, ES, NS aggressiveness level */
typedef enum
{
    IFX_SP_ENH_AGGRESIVE_LOW = 0,
    IFX_SP_ENH_AGGRESIVE_MEDIUM,
    IFX_SP_ENH_AGGRESIVE_HIGH
} ifx_aggressiveness_config_t;

/**
 * Shared control/model structure
 */
typedef struct
{
    /*@{*/
        int sampling_rate;      /**< audio and voice data sampling rate */
        int num_of_mic;         /**< number of mics */
        int num_of_beam;        /**< number of beams */
        int aec_config;         /**< echo cancellation configured or not */
        int input_frame_size;   /**< input data frame size in samples */
        int output_size;        /**< spreech enhancement application output size in its data type */
        int ifx_output_size;    /**< spreech enhancement application Infineon internal output size in its data type */
        int num_of_components;  /**< number of IP components in spreech enhancement application */
       int scratch_mem;        /**< scratch memory size required for inference */
        int persistent_mem;     /**< persistent memory size required for inference */
        mem_info_t memory;      /**< memory structure contains info required for process */
        int libsp_version;          /**< version number of speech enhancement IP library */
        int configurator_version;   /**< Audio Front-End configurator version number */
    /*@}*/
} ifx_stc_sp_enh_info_t;

/******************************************************************************
* Function prototype
******************************************************************************/
/**
 * \addtogroup API
 * @{
 */

/**
 * \brief : ifx_sp_enh_process() is the API function to perform audio & voice enhancement process.
 *
 * Ifx audio & voice enhancement proces API function targeted for ifx MCU embedded application.
 * 
 * \param[in]      modelPt         : Pointer to ifx parsed audio & voice enhancement process model data container
 * \param[in, out] input1          : Data pointer to first audio (e.g. first microphone) input and output of HPF
 * \param[in, out] input2          : Data pointer to second audio (e.g. second microphone) input and output of HPF; NULL if there is no second audio/mic input
 * \param[in]      reference_input : AEC reference input data pointer; NULL if there is no AEC reference 
 * \param[out]     output          : Ifx audio & voice enhancement proces output data pointer
 * \param[out]     ifx_output      : Output data pointer for Infineon internal useage
 *
 * \return                  : Return 0 when success, otherwise return following error code
 *                            IFX_SP_ENH_ERR_INVALID_ARGUMENT if input or output argument is invalid
 *                            or error code from specific ifx audio & voice enhancement proces module.
 *                            Please note error code is 8bit LSB, line number where the error happened in
 *                            code is in 16bit MSB, and its IP component index if applicable will be at
 *                            bit 8 to 15 in the combined 32bit return value.
 */
int32_t ifx_sp_enh_process(void *modelPt, void *input1, void* input2, void *reference_input, void *output, void *ifx_output);

/**
 * \brief : ifx_sp_enh_model_parse() is the API function to parse audio & voice enhancement process model parameters to get basic info.
 *
 * It will parse info from the audio & voice enhancement process model parameter buffer to get required basic info such as
 * persistent and scratch memory sizes, input data size, output classification size and stored
 * them in to ifx_stc_sp_enh_info_t structure.
 *
 * \param[in]   fn_prms     : Ifx audio & voice enhancement process model parameter buffer pointer
 * \param[out]  mdl_infoPt  : Pointer to ifx_stc_sp_enh_info_t structure
 *
 * \return                  : Return 0 when success, otherwise return following error code
 *                                IFX_SP_ENH_ERR_INVALID_ARGUMENT if input or output argument is invalid
 *                                Please note error code is 8bit LSB, line number where the error happened in
 *                                code is in 16bit MSB, and its IP component index if applicable will be at
 *                                bit 8 to 15 in the combined 32bit return value.
 */

int32_t ifx_sp_enh_model_parse(char *fn_prms , ifx_stc_sp_enh_info_t *mdl_infoPt);

/**
 * \brief : ifx_sp_enh_init() is the API function to parse and initialize ifx audio & voice enhancement process data container.
 *
 * From the API inputs, it parses audio & voice enhancement process model and initializes its data container.
 *
 * \param[out]  dPt_container   : Pointer to ifx audio & voice enhancement process model data container pointer
 * \param[in]   fn_prms         : ifx audio & voice enhancement process model parameter buffer pointer
 * \param[in]   persistent_mem  : Pointer to allocated persistent memory
 * \param[in]   scratch_mem     : Pointer to allocated scratch memory
 * \param[in]   mdl_infoPt      : Pointer to ifx_stc_sp_enh_info_t structure
 *
 * \return                      : Return 0 when success, otherwise return error code
 *                                Return IFX_SP_ENH_ERR_INVALID_ARGUMENT if input or output argument is invalid,
 *                                Otherwise return other errors:
 *                                e.g. IFX_SP_ENH_ERR_PARAM if the audio & voice enhancement component IP parameter is invalid.
 *                                Please note error code is 8bit LSB, line number where the error happened in
 *                                code is in 16bit MSB, and its IP component index if applicable will be at bit 8 to 15
 *                                in the combined 32bit return value.
*/

int32_t ifx_sp_enh_init(void **dPt_container, char *fn_prms
           , char *persistent_mem, char* scratch_mem, ifx_stc_sp_enh_info_t *mdl_infoPt);

/**
 * \brief : ifx_sp_enh_mode_control() is the API function to control ifx audio & voice enhancement process compenent's mode.
 *
 * This API is used to control ifx audio & voice enhancement process IP component's mode. This API can be called after initilization.
 * If the ifx audio & voice enhancement process IP component is not configured as valid. This API will do nothing.
 *
 * \param[out]  modelPt         : Pointer to ifx audio & voice enhancement process model data container pointer
 * \param[in]   component_id    : ifx audio & voice enhancement process component ID
 * \param[in]   enable          : Enable or disable Component_id
 * \param[in]   reset           : If value = true, reset component_id's state. If value = false, no change on component's state.
 * \param[in]   update          : If value = true, update component_id's parameter. If value = false, no change on component's parameter.
 * \param[in]   new_value       : If update = true, this is new parameter. If update = false, this is dummy and ignored by this function.
 *
 * \return                      : Return 0 when success, otherwise return error code
 *                                Return IFX_SP_ENH_ERR_INVALID_ARGUMENT if input argument is invalid.
*/

int32_t ifx_sp_enh_mode_control(void* modelPt, ifx_sp_enh_ip_component_config_t component_id, bool enable, bool reset, bool update, int32_t value);

/**
 * \brief : ifx_sp_enh_status() is the API function to query ifx audio & voice enhancement process compenent's status.
 *
 * This API can be called after initilization.
 * If the ifx audio & voice enhancement process IP component is not configured as valid. This API will return status as disabled.
 *
 * \param[out]  modelPt         : Pointer to ifx audio & voice enhancement process model data container pointer
 * \param[in]   component_id    : ifx audio & voice enhancement process component ID
 *
 * \return                      : Return 0 means disabled
 *                                Return 1 means enabled
*/

int32_t ifx_sp_enh_status(void* modelPt, ifx_sp_enh_ip_component_config_t component_id);

/**
 * \brief : my_profile_get_tsc() is an API function to read time stamp counter (TSC) .
 *
 * Platform specific function to read HW time stamp counter or OS tick timer counter for profiling.
 * The application program developer should provide this function if profiling is enabled.
 *
 * \param[out]   val        : Pointer to time stamp counter return value
 *
 * \return                  : Return 0 when success, otherwise return error code
 */
int32_t platform_profile_get_tsc(uint32_t *val);

/**
 * \brief : Initialize profile configuration.
 *
 * This API is used to setup profile configuration and specify the callback function to handle the
 * profile log. If no callback function is specified, the profile log will be printed out on console.
 *
 * \param[in]  modelPt         : Pointer to ifx audio & voice enhancement process data container pointer
 * \param[in]  config          : Profile setting
 * \param[in]  cb_func         : Callback function to handle the profile result.
 * \param[in]  cb_arg          : Callback function argument
 *
 * \return                     : Return 0 when success, otherwise return error code
 *                               Return IFX_SP_ENH_ERR_INVALID_ARGUMENT if input parameter is invalid.
*/
int32_t ifx_sp_enh_profile_init(void *modelPt, int32_t config, ifx_sp_profile_cb_fun cb_func, void *cb_arg);

/**
 * \brief : Update  profile configuration.
 *
 * This API is used to update profile configuration.
 *
 * \param[in]  modelPt         : Pointer to ifx audio & voice enhancement process data container pointer
 * \param[in]  config          : Profile setting
 *
 * \return                     : Return 0 when success, otherwise return error code
 *                               Return IFX_SP_ENH_ERR_INVALID_ARGUMENT if input parameter is invalid.
*/
int32_t ifx_sp_enh_profile_control(void *modelPt, int32_t config);

/**
 * \brief : Print profile log.
 *
 *
 * \param[in]  modelPt         : Pointer to ifx audio & voice enhancement process data container pointer
 * \return                     : Return 0 when success, otherwise return error code
 *                               Return IFX_SP_ENH_ERR_INVALID_ARGUMENT if input parameter is invalid.
*/
int32_t ifx_sp_enh_profile_print(void *modelPt);

/**
 * @} end of API group
 */

#endif /*__IFX_SP_ENH_H */

/* [] END OF FILE */
