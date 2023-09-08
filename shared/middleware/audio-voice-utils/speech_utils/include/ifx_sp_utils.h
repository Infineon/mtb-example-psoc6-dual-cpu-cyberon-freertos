/******************************************************************************
* File Name: ifx_sp_utils.h
*
* Description: This file contains public interface for Infineon speech utilities
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
#ifndef __IFX_SP_UTILS_H
#define __IFX_SP_UTILS_H

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
* Speech utilities data type & defines
*******************************************************************************/
#define IFX_SP_LPWWD_VERSION_MAJOR            1
#define IFX_SP_LPWWD_VERSION_MINOR            0
#define IFX_SP_LPWWD_VERSION_PATCH            0
#define IFX_SP_LPWWD_VERSION                  100

/*******************************************************************************
* Structures and enumerations
*******************************************************************************/

/******************************************************************************
* Function prototype
******************************************************************************/
/**
 * \addtogroup API
 * @{
 */

/**
 * \brief : speech_utils_getMem() is the API function to estimate memory needed by ip_id component block.
 *
 *
 * \param[in]  ip_prms_buffer   : Configuration parameter buffer of the given ip_id component block
 *                              : For SOD, it contains SOD configuration parameter: [configuration version, sampling_rate(Hz),
 *                              : input frame_size (samples), SOD component ID, number of following parameters,
 *                              : GapSetting (0,100,200,300,400,500,1000)ms, SensLevel (0-32767)]
 *                              : For preprocess, it contains preprocess configuration parameter: [configuration version,
 *                              : sampling_rate(Hz), input frame size (samples), compenent ID, number of following parameters,
 *                              : frame shift (samples), mel_banks, dct_coefs (set to 0 for LOG MEL)]
 *                              : For postprocess, its contains to be defined
 * \param[in]  ip_id            : The given IP component block's id number
 * \param[out] mem_infoPt       : Pointer to mem_info_t structure, output scratch and persistent memory sizes when success
 * \return                      : Return 0 when success, otherwise return error code
 *                                IFX_SP_ENH_ERR_INVALID_ARGUMENT if input or output argument is invalid
 *                                or error code from specific ifx audio & voice enhancement proces module.
 *                                Please note error code is 8bit LSB, line number where the error happened in
 *                                code is in 16bit MSB, and its IP component index if applicable will be at
 *                                bit 8 to 15 in the combined 32bit return value.
*/
int32_t speech_utils_getMem(int32_t * ip_prms_buffer, int32_t ip_id, mem_info_t *mem_infoPt);

/**
 * \brief : speech_utils_sod_init() is the API function to initilize SOD given user options.
 *
 *
 * \param[out]  sod_container       : Pointer to SOD container that contains state memory
 * \param[in]   sod_mem_infoPt      : Pointer to SOD's mem_info_t struct
 * \param[in]   sod_prms_buffer     : Configuration parameter buffer of the given ip_id component block
 *                                  : For SOD, it contains SOD configuration parameter: [configuration version, sampling_rate(Hz),
 *                                  : frame_size (samples), SOD component ID, number of following parameters,
 *                                  : GapSetting (0,100,200,300,400,500,1000)ms, SensLevel (0-32767)]
 *                                  : Note - at this time, frame_size=160, sampling_rate=16000 only are valid
 * \return                          : Return 0 when success, otherwise return error code
 *                                    IFX_SP_ENH_ERR_INVALID_ARGUMENT if input or output argument is invalid
 *                                    or error code from specific ifx audio & voice enhancement proces module.
 *                                    Please note error code is 8bit LSB, line number where the error happened in
 *                                    code is in 16bit MSB, and its IP component index if applicable will be at
 *                                    bit 8 to 15 in the combined 32bit return value.
*/
int32_t speech_utils_sod_init(int32_t * sod_prms_buffer, void **sod_container, mem_info_t *sod_mem_infoPt);

/**
 * \brief : speech_utils_sod_reset() is the API function to reset SOD to initial memory settings.
 *
 *
 * \param[in]  sod_container        : Pointer to SOD container that contains state memory and its params
 * \param[in]  sod_prms_buffer      : Configuration parameter buffer of the given ip_id component block
 *                                  : For SOD, it contains SOD configuration parameter: [frame_size (samples), sampling_rate(Hz),
 *                                  : GapSetting (0,100,200,300,400,500,1000)ms (400 suggested), SensLevel (0-32767) (16384 suggested)]
 *                                  : Note: Only GapSetting and SensLevel can be reconfigured during reset.
 * \return                          : Return 0 when success, otherwise return error code
 *                                    IFX_SP_ENH_ERR_INVALID_ARGUMENT if input or output argument is invalid
 *                                    or error code from specific ifx audio & voice enhancement proces module.
 *                                    Please note error code is 8bit LSB, line number where the error happened in
 *                                    code is in 16bit MSB, and its IP component index if applicable will be at
 *                                    bit 8 to 15 in the combined 32bit return value.
*/
int32_t speech_utils_sod_reset(int32_t * sod_prms_buffer, void *sod_container);

/**
 * \brief : speech_utils_sod_process() is the API function to do SOD process.
 *
 *
 * \param[out] vad               : pointer to SOD detection is written, 1=SOD detected, 0=NO SOD detected
 * \param[in]  sod_container     : Pointer to SOD container that contains state memory and its params
 * \param[in]  in                : Pointer to PCM samples input
 * \return                       : Return 0 when success, otherwise return error code
 *                                 IFX_SP_ENH_ERR_INVALID_ARGUMENT if input or output argument is invalid
 *                                 or error code from specific ifx audio & voice enhancement proces module.
 *                                 Please note error code is 8bit LSB, line number where the error happened in
 *                                 code is in 16bit MSB, and its IP component index if applicable will be at
 *                                 bit 8 to 15 in the combined 32bit return value.
*/
int32_t speech_utils_sod_process(IFX_SP_DATA_TYPE_T *in, void *sod_container, bool *vad);

#endif /*__IFX_SP_UTILS_H */

/* [] END OF FILE */