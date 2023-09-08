/******************************************************************************
* File Name: ifx_pre_post_process.h
*
* Description: This file contains public interface for Infineon pre and
*              post process
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
#ifndef __IFX_PRE_POST_PROCESS_H
#define __IFX_PRE_POST_PROCESS_H

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
* Infineon pre and post process utilities data type & defines
*******************************************************************************/
#define IFX_PRE_POST_PROCESS_VERSION_MAJOR            1
#define IFX_PRE_POST_PROCESS_VERSION_MINOR            0
#define IFX_PRE_POST_PROCESS_VERSION_PATCH            0
#define IFX_PRE_POST_PROCESS_VERSION                  100

/*******************************************************************************
* Structures and enumerations
*******************************************************************************/
#define MEL_LOG_OUTPUT_Q       (9)
#define MFCC_OUTPUT_Q          (6)

/** Defines ifx pre and post process IP components */
typedef enum
{
    IFX_PRE_PROCESS_IP_COMPONENT_SOD = IFX_PRE_POST_IP_COMPONENT_START_ID,  /* Infineon LPWWD SOD IP component */
    IFX_PRE_PROCESS_IP_COMPONENT_MIN_MAX,
    IFX_PRE_PROCESS_IP_COMPONENT_Z_SCORE,
    IFX_PRE_PROCESS_IP_COMPONENT_IIR,
    IFX_PRE_PROCESS_IP_COMPONENT_FIR,
    IFX_PRE_PROCESS_IP_COMPONENT_LOG_MEL,
    IFX_PRE_PROCESS_IP_COMPONENT_MFCC,
    IFX_POST_PROCESS_IP_COMPONENT_TIME_AVG,
    IFX_POST_PROCESS_IP_COMPONENT_HMMS,
    IFX_POST_PROCESS_IP_COMPONENT_DECISION_TREE,
    IFX_POST_PROCESS_IP_COMPONENT_MAX_COUNT
} ifx_pre_post_ip_component_config_t;

/**
 * Shared control/model structure
 */
typedef struct
{
    /*@{*/
    int component_id;           /**< pre or post process compnent ID */
    int sampling_rate;          /**< audio and voice data sampling rate in Hz */
    int input_frame_size;       /**< input data frame size in samples */
    int frame_shift;            /**< input data frame shift in samples */
    int fft_block_size;         /**< fft block size in samples */
    int output_size;            /**< spreech enhancement application output size in its data type */
    mem_info_t memory;          /**< memory structure contains info required for process */
    int libsp_version;          /**< version number of Infineon pre and post process IP library */
    int configurator_version;   /**< Configurator version number */
    /*@}*/
} ifx_stc_pre_post_process_info_t;


typedef enum
{
    WUP_PROCESSING = 0,         /* Monitoring for WUP */
    WUP_REJECTED,               /* WUP rejected */
    WUP_DETECTED,               /* WUP detected */
} ifx_wup_detection_config_t;

/******************************************************************************
* Function prototype
******************************************************************************/
/**
 * \addtogroup API
 * @{
 */

/**
 * \brief : ifx_pre_post_process_parse() is the API function to parse basic info and estimate memory
 *          needed by ip_id component block. This only needed for parse confiurator generated parameter file.
 *
 *
 * \param[in]  ip_prms_buffer   : Configuration parameter buffer which has component ID and related parameters
 *                              : For SOD, it contains SOD configuration parameter: [configuration version, sampling_rate(Hz),
 *                              : input frame_size (samples), SOD component ID, number of following parameters,
 *                              : GapSetting (0,100,200,300,400,500,1000)ms, SensLevel (0-32767)]
 *                              : For preprocess, it contains preprocess configuration parameter: [configuration version,
 *                              : sampling_rate(Hz), input frame size (samples), compenent ID, number of following parameters,
 *                              : frame shift (samples), mel_banks, dct_coefs (set to 0 for LOG MEL)]
 * \param[out] ip_infoPt        : Pointer to ifx_stc_pre_post_process_info_t structure
 * \return                      : Return 0 when success, otherwise return error code
 *                                INVALID_ARGUMENT if input or output argument is invalid
 *                                or error code from specific infineon pre and post process component.
 *                                Please note error code is 8bit LSB, line number where the error happened in
 *                                code is in 16bit MSB, and its IP component index if applicable will be at
 *                                bit 8 to 15 in the combined 32bit return value.
*/
int32_t ifx_pre_post_process_parse(int32_t * ip_prms_buffer, ifx_stc_pre_post_process_info_t *ip_infoPt);

/**
 * \brief : ifx_pre_post_process_init() is the API function to initilize component container based on component ID.
 *
 *
 * \param[out]  container       : Pointer to component ID's container that contains state memory and parameters
 * \param[out]  ip_infoPt       : Pointer to component ID's ifx_stc_pre_post_process_info_t
 * \param[in]   prms_buffer     : Manually generated parameter buffer which has component id and related parameters
 *                              : For SOD, it contains SOD configuration parameter: [configuration version, sampling_rate(Hz),
 *                              : input frame_size (samples), SOD component ID, number of following parameters,
 *                              : GapSetting (0,100,200,300,400,500,1000)ms, SensLevel (0-32767)]
 *                              : For preprocess, it contains preprocess configuration parameter: [configuration version,
 *                              : sampling_rate(Hz), input frame size (samples), compenent ID, number of following parameters,
 *                              : frame shift (samples), mel_banks, dct_coefs (set to 0 for LOG MEL)]
 * \return                      : Return 0 when success, otherwise return error code
 *                                INVALID_ARGUMENT if input or output argument is invalid
 *                                or error code from specific infineon pre and post process component.
 *                                Please note error code is 8bit LSB, line number where the error happened in
 *                                code is in 16bit MSB, and its IP component index if applicable will be at
 *                                bit 8 to 15 in the combined 32bit return value.
*/
int32_t ifx_pre_post_process_init(int32_t * prms_buffer, void **container, ifx_stc_pre_post_process_info_t *ip_infoPt);

/**
 * \brief : ifx_time_pre_process() is the API function to pre process in time domain given user's selected method.
 *
 *
 * \param[out] output               : pointer to buffer where preprocessed time domain output of one frame is kept
 * \param[in]  preprocess_container : Pointer to preprocess container that contains state memory and parameters
 * \param[in]  component_id         : Infineon pre and post process component ID
 * \param[in]  input                : Pointer to input samples
 * \return                          : Return 0 when success, otherwise return error code
 *                                    INVALID_ARGUMENT if input or output argument is invalid
 *                                    or error code from specific infineon preprocess component.
 *                                    Please note error code is 8bit LSB, line number where the error happened in
 *                                    code is in 16bit MSB, and its IP component index if applicable will be at
 *                                    bit 8 to 15 in the combined 32bit return value.
*/
int32_t ifx_time_pre_process(IFX_SP_DATA_TYPE_T *input, void *preprocess_container, int32_t component_id, IFX_SP_DATA_TYPE_T *output);

/**
 * \brief : ifx_spectrogram_transfer() is the API function to do spectrogram transformation given user's method.
 *
 *
 * \param[out] fe_out               : pointer to buffer where feature for one frame is kept
 * \param[out] out_q                : pointer to output fixed-point Q value
* \param[in]  preprocess_container : Pointer to preprocess container that contains state memory and parameters
 * \param[in]  input                : Pointer to input samples
 * \return                          : Return 0 when success, otherwise return error code
 *                                    INVALID_ARGUMENT if input or output argument is invalid
 *                                    or error code from specific infineon preprocess component.
 *                                    Please note error code is 8bit LSB, line number where the error happened in
 *                                    code is in 16bit MSB, and its IP component index if applicable will be at
 *                                    bit 8 to 15 in the combined 32bit return value.
*/
int32_t ifx_spectrogram_transfer(IFX_SP_DATA_TYPE_T *input, void *spectrogram_container, IFX_FE_DATA_TYPE_T *fe_out, int32_t *out_q);

/**
 * \brief : ifx_post_process() is the API function to use output of the Neural Network and use post process to declare WWD. 
 *
 *
 * \param[in]  postprocess_container    : Pointer to postprocess container that contains state memory and parameters
 * \param[in]  component_id             : Infineon pre and post process component ID
 * \param[in]  in_probs                 : pointer for the buffer containing output probabilities from classifier in 
 *                                        the order of garbage, noise, token 1, token 2.
 * \param[out] detection                : WUP detection output; 0 means monitoring, 1 means rejected, 2 means detected.
 * \return                              : Return 0 when success, otherwise return error code
 *                                        INVALID_ARGUMENT if input or output argument is invalid
 *                                        or error code from specific infineon post process component.
 *                                        Please note error code is 8bit LSB, line number where the error happened in
 *                                        code is in 16bit MSB, and its IP component index if applicable will be at
 *                                        bit 8 to 15 in the combined 32bit return value.
*/
int32_t ifx_post_process(IFX_SP_DATA_TYPE_T *in_probs, void *postprocess_container, int32_t component_id, int32_t *detection);

/**
 * \brief : ifx_pre_post_process_mode_control() is the API function to control Infineon pre and post process compenent's mode.
 * This API is used to control Infineon pre and post process IP component's mode. This API can be called after initilization.
 * If the Infineon pre and post process IP component is not configured as valid. This API will do nothing.
 *
 * \param[out]  container       : Pointer to Infineon pre and post process model data container pointer
 * \param[in]   component_id    : Infineon pre and post process component ID
 * \param[in]   enable          : Enable or disable component specified by component_id
 * \param[in]   reset           : If value = true, reset component_id's state. If value = false, no change on component's state.
 *
 * \return                      : Return 0 when success, otherwise return error code
 *                                Return INVALID_ARGUMENT if input argument is invalid.
 *                                or error code from specific infineon post process component.
 *                                Please note error code is 8bit LSB, line number where the error happened in
 *                                code is in 16bit MSB, and its IP component index if applicable will be at
 *                                bit 8 to 15 in the combined 32bit return value.
*/
int32_t ifx_pre_post_process_mode_control(void* container, int32_t component_id, bool enable, bool reset);

/**
 * \brief : ifx_pre_post_process_status() is the API function to query Infineon pre and post process compenent's status.
 * This API can be called after initilization.
 * If the Infineon pre and post process IP component is not configured as valid. This API will return status as disabled.
 *
 * \param[out]  container       : Pointer to Infineon pre and post process model data container pointer
 * \param[in]   component_id    : ifx audio & voice enhancement process component ID
 *
 * \return                      : Return 0 means disabled
 *                                Return 1 means enabled
*/
int32_t ifx_pre_post_process_status(void* container, int32_t component_id);

/**
 * \brief : Initialize pre/post process profile configuration.
 *
 * This API is used to setup pre/post process profile configuration and specify the callback function to handle
 * the profile log. If no callback function is specified, the profile log will be printed out on console.
 *
 * \param[in]  modelPt         : Pointer to ifx pre/post process data container pointer
 * \param[in]  ip_id           : Pre/post process component ID number.
 * \param[in]  enable          : Profile setting enable (true) or disable (false).
 * \param[in]  cb_func         : Callback function to handle the profile result.
 * \param[in]  cb_arg          : Callback function argument
 *
 * \return                     : Return 0 when success, otherwise return error code
 *                               Return IFX_SP_ENH_ERR_INVALID_ARGUMENT if input parameter is invalid.
*/
int32_t ifx_pre_post_profile_init(void* modelPt, int32_t ip_id, uint8_t enable, ifx_sp_profile_cb_fun cb_func, void* cb_arg);

/**
 * \brief : Update pre/post process profile configuration.
 *
 * This API is used to update pre/post process profile configuration.
 *
 * \param[in]  modelPt         : Pointer to ifx pre/post process data container pointer
 * \param[in]  ip_id           : Pre/post process component ID number.
 * \param[in]  enable          : Profile setting enable (true) or disable (false).
 *
 * \return                     : Return 0 when success, otherwise return error code
 *                               Return IFX_SP_ENH_ERR_INVALID_ARGUMENT if input parameter is invalid.
*/
int32_t ifx_pre_post_profile_control(void* modelPt, int32_t ip_id, bool enable);

/**
 * \brief : Print pre/post process profile log.
 *
 *
 * \param[in]  lPt             : Pointer to profile data container pointer
 * \param[in]  ip_id           : Pre/post process component ID number.
 * \return                     : Return 0 when success, otherwise return error code
 *                               Return IFX_SP_ENH_ERR_INVALID_ARGUMENT if input parameter is invalid.
*/
int32_t ifx_pre_post_profile_print(void* lPt, int32_t ip_id);

#endif /*__IFX_PRE_POST_PROCESS_H */

/* [] END OF FILE */