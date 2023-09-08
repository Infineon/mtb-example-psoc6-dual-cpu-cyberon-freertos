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
 * @file cy_staged_voice_control_lp.h
 *
 * @brief This file is the header file for Staged voice control library
 * running on low power domain (M33).
 *
 * Abbreviations used in the header file
 *
 * SVC      -   Stage Voice Control
 * SVC-LP   -   Staged Voice Control MW running in low power domain (M33)
 * SVC-HP   -   Staged Voice Control MW running in high performance domain
 */

#ifndef __CY_STAGED_VOICE_CONTROL_LP_H__
#define __CY_STAGED_VOICE_CONTROL_LP_H__

#include "cy_staged_voice_control_common.h"
#include "cy_sod.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                          Macros
 ******************************************************************************/

/**
 * Maximum pre-roll buffer configuration. It is safety max check to request
 * for PreRoll Buffer. For actual usecase, the application should choice the
 * right pre-roll buffer count.
 */
#define MAX_PRE_ROLL_BUFFER_REQ_PERCENTAGE (90.0)

/*******************************************************************************
 *                          Constants
 ******************************************************************************/

/*******************************************************************************
 *                          Enumerations
 ******************************************************************************/

/**
 * External Low performance WWD detection state.
 */
typedef enum
{
    /**
     * External LPWWD State is invalid
     */
    CY_SVC_EXTERNAL_LPWWD_WWD_STATE_INVALID,

    /**
     * WWD detection is in progress with External LPWWD
     */
    CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_IN_PROGRESS,

    /**
     * WWD detection is detected with External LPWWD
     */
    CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_SUCCESS,

    /**
     * WWD detection failed with External LPWWD
     */
    CY_SVC_EXTERNAL_LPWWD_WWD_DETECION_FAILED,

    /**
     * WWD detection max - invalid enum
     */
    CY_SVC_EXTERNAL_LPWWD_WWD_STATE_MAX

} cy_svc_lp_external_lpwwd_state_t;

/*******************************************************************************
 *                          Type Definitions
 ******************************************************************************/
/**
 * Callback to get notification on different events from staged voice control
 * library. Event callback needs to be registered using the API
 * \ref cy_svc_lp_init.
 *
 * Application should handle the event from staged voice control without
 * blocking the event callback.
 *
 * @param[in] event                 Event type which occurred
 * @param[in] callback_user_arg     User argument passed in \ref cy_svc_lp_init
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
typedef cy_rslt_t (*cy_svc_lp_event_callback_t)(
        cy_svc_event_t event,
        void *callback_user_arg);

/**
 * Callback to provide the data to SVC's application for any
 * external low power wake word detection.
 *
 * Application should return the buffer as soon as the callback ends. Application
 * should not use this buffer after callback returns). Application may decide
 * to copy the buffer or process in the callback itself and it depends the
 * external application implementation. Holding this callback for longer time
 * may results in overwrite of the internal buffer due to data feed through
 * cy_svc_lp_feed()
 *
 * @param[in] frame_buffer          frame buffer
 * @param[in] frame_count           frame count
 * @param[in] callback_user_arg     application passed user argument
 * @param[out] lpwwd_state          application returns the lpwwd stage to SVC.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
typedef cy_rslt_t (*cy_svc_lp_data_callback_t)(
        CY_SVC_DATA_T *frame_buffer,
        uint32_t frame_count,
        void *callback_user_arg,
        cy_svc_lp_external_lpwwd_state_t *lpwwd_state);

/*******************************************************************************
 *                          Structures
 ******************************************************************************/

/**
 * SVC configuration structure for low power domain.
 */
typedef struct
{
    /**
     * List of the features enabled/disabled depends on the application's choice.
     * It is a bit-mask, Application can enable one or more multiple features.
     */
    cy_svc_stage_config_t stage_config_list;

    /**
     * User argument for callback.
     */
    void *callback_user_arg;

    /**
     * Callback functions for event from SVC module to application. Registering
     * this event callback is recommended. Alternatively, the application may
     * chose not to register and application can use the API
     * \ref cy_svc_lp_get_current_stage to get the state.
     *
     * \note Application should not call any SVC LP apis from this callback. Calling
     * SVC apis may results in blocking the SVC internal thread.
     */
     cy_svc_lp_event_callback_t event_callback;

    /**
     * Audio feed Data Info: Audio Data Type
     */
    cy_svc_audio_input_type_t audio_input_type;

    /**
     * Audio feed Data Info: Sample rate of the input data.
     * Currently only 16KHz is supported.
     */
    cy_svc_sample_rate_t sample_rate;

    /**
     * Audio feed Data Info: Single audio frame is assumed to be of 10ms worth
     * of data. Currently only 10ms is supported. Hence application needs to
     * submit audio frames for every 10ms worth of data
     * \ref CY_SVC_LP_SUPPORTED_FRAME_TIME_MS
     */
    uint32_t single_frame_time_ms;

    /**
     * Size of the buffer total buffer size that staged voice control module
     * would maintain the input data feed and it will help processing the input
     * data. This parameter is configurable, so that application may fine tune
     * the buffer size depends on the memory requirement/use case.
     *
     * Max check for the total circular buf size will not be validated.
     * Application has to choose the right value. Total circular buffer size
     * must be the multiple of number of audio frames.
     *
     * Example: For Mono, FrameSize:320Bytes, the circular buffer size must be
     * multiple of 320Bytes.
     */
    uint32_t total_circular_buf_size;

    /**
     * Setting this flag to "true" indicate that, IFX LPWWD will not be used
     * instead, application will use third party LPWWD.
     */
    bool is_lpwwd_external;

    /**
     * Callback function needs to register to get audio data. This callback
     * registration is mandatory only if "is_lpwwd_external" is set to true.
     * Application can feed the data to the third party LPWWD processing.
     */
    cy_svc_lp_data_callback_t   lpwwd_external_data_callback;

    /**
     * Once the LPWWD is detected, the pre-roll frame count will be used to
     * send the saved buffer to the high performance domain.
     *
     *      |--------------------------------|
     *      |       Buffer of X sec          |
     *      |--------------------------------|
     *      t0 t1 ..                        tn
     *                      ^
     *                      |
     *                      LWWD detect frame pointer
     *
     *         |<--PreRoll->|
     *
     *  Error will be thrown if the PreRollBuffer occupies more than
     *  MAX_PRE_ROLL_BUFFER_REQ_PERCENTAGE % of of total frame
     *  buffers (in total_circular_buf_size)
     */
    uint32_t pre_roll_frame_count_from_lpwwd_detect_frame;

    /**
     * Sensitivity, The detection sensitivity (0- 32767) \ref: cy_sod.h
     * 0 = least sensitive
     * 32767 (MAX_SOD_SENSITIVITY) = most sensitive
     * 16384 = nominal sensitive
     */
    int sod_sensitivity;

    /**
     * Onset gap settings. Generally, a talker will pause momentarily before
     * addressing a person, or in this case, a device.  The pause is
     * approximately in the range of 200-500ms in conversational speech.
     * For more details \ref cy_sod.h
     *
     * Allowed gap settings are:
     * CY_SOD_ONSET_GAP_SETTING_1000_MS,
     * CY_SOD_ONSET_GAP_SETTING_500_MS,
     * CY_SOD_ONSET_GAP_SETTING_400_MS,
     * CY_SOD_ONSET_GAP_SETTING_300_MS
     * CY_SOD_ONSET_GAP_SETTING_200_MS
     * CY_SOD_ONSET_GAP_SETTING_100_MS
     * CY_SOD_ONSET_GAP_SETTING_0_MS
     *
     * For wake-word detection, better behavior is seen with a
     * CY_SOD_ONSET_GAP_SETTING_400_MS gap setting.
     */
    int sod_onset_gap_setting_ms;

    /**
     * SOD onset detect late hit delay. After SOD detect, the
     * buffer would be look backed with this delay, to get the
     * correct/approximate close onset of the speech.
     */
    unsigned int sod_onset_detect_max_late_hit_delay_ms;

    /**
     * Neural network model parameter buffer pointer. This model
     * is applicable only when the IFX LPWWD is used. (In other
     * words if the flag "is_lpwwd_external" set to true, then
     * this variable will be unused.)
     */
    const char *wake_word_model_meta_buf;

    /**
     * Neural network model weights & biases buffer pointer. This model
     * is applicable only when the IFX LPWWD is used. (In other
     * words if the flag "is_lpwwd_external" set to true, then
     * this variable will be unused.)
     */
    const char *wake_word_model_binary_buf;

} cy_svc_lp_config_t;


/*******************************************************************************
 *                          Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
/**
 * Initializes the staged voice control module in low power device, creates the
 * required resources based on the input configuration, the resource could be
 * buffer, internal task, IPC communication, etc.,
 *
 * @param[in]  init                 Staged voice control module init
 *                                  configuration parameter in low power domain.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_init(cy_svc_lp_config_t *init);

/**
 * Get the current stage of staged voice control module. Get stage API
 * returns immediately with the current stage of SVC LP MW.
 *
 * By any chance, If there is any stage transition is happening in SVC,
 * this API will return the current stage and it will not return the new
 * stage which is being applied. In that case application has to call
 * this API again.
 *
 * Additionally the cy_svc_lp_set_stage API is synchronous. Application
 * can call the  cy_svc_lp_get_current_stage api after the
 * cy_svc_lp_set_stage API from single thread/task.
 *
 * @param[out]  stage                staged voice control module's current stage
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_get_current_stage(cy_svc_stage_t *stage);

/**
 * Feed input audio data to staged voice control module in low power domain.
 *
 * @param[in]  data                 Pointer to the input audio data.
 *
 *                                  Application can free/reuse the buffer pointer
 *                                  passed in this API once the API is returned.
 *                                  (i.e., Staged voice module will make a copy
 *                                  of the input data internally in its buffer).
 *                                  Length of the input audio data passed in the
 *                                  buffer is constant should not vary for every
 *                                  feed.(i.e, Application should feed always the
 *                                  complete buffer).
 *
 *                                  If Application needs to feed stereo data,
 *                                  then the data format must be non-interleaved
 *                                  format. example: For stereo data of 10ms
 *                                  worth, first 320bytes must be of channel-1
 *                                  and then next 320bytes must be channel-2.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_feed(CY_SVC_DATA_T *data);

/**
 * SVC-LP application can force the stage in SVC module.
 *
 * Set stage API is synchronous, It successfully applies the stage requested
 * internally in the SVC LP (internal thread) MW and then this API will be
 * returned to the caller with the appropriate return results.
 *
 * @param[in] stage                 Required stage to set.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_set_stage(cy_svc_stage_t stage);

/**
 * Deinit the staged voice control module in low power device, deletes the
 * resources created during \ref cy_svc_lp_init.
 *
 * Application should take care of not calling any of SVC MW LP APIs (cy_svc_lp_feed
 * ,cy_svc_lp_get_current_stage and cy_svc_lp_set_stage APIs) when the
 * cy_svc_lp_deinit is in progress. During deinit if the application calls any
 * of the API, it may results in crash. Adding protection internally between
 * these apis and cy_svc_lp_deinit results in unwanted synchronous mechanism and adds
 * load to the system.
 * The usecase of cy_svc_lp_deinit is very rare. Hence application should take
 * care of this condition.
 *
 * @param none
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_lp_deinit(void);


#ifdef __cplusplus
}
#endif

#endif /* __CY_STAGED_VOICE_CONTROL_LP_H__ */
