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
 * @file cy_staged_voice_control_hp.h
 *
 * @brief This file is the header file for Staged voice control (SVC) library
 * running on high performance domain (M55).
 *
 * Abbreviations used in the header file
 *
 * SVC      -   Stage Voice Control
 * SVC-LP   -   Staged Voice Control MW running in low power domain (M33)
 * SVC-HP   -   Staged Voice Control MW running in high performance domain
 */

#ifndef __CY_STAGED_VOICE_CONTROL_HP_H__
#define __CY_STAGED_VOICE_CONTROL_HP_H__

#include "cy_staged_voice_control_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Constants
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      Type Definitions
 ******************************************************************************/
/**
 * SVC-HP domain would receive the data from SVC-LP domain. Once SVC-HP domain
 * receives the data, the received data would be sent to SVC-HP's application
 * through this callback.
 *
 * SVC-HP application may use the data pointer and process the memory even after
 * the callback is returned. SVC MW will not wait for any acknowledgment of the
 * data consumption from the SVC-HP's application. SVC module will continue to
 * reuse the pointer if the SVC-HP application is not fast enough to process
 * the data sent.
 *
 * Application must register this callback using the API \ref cy_svc_hp_init
 *
 * @param[in] frame_buffer          1. Valid buffer pointer pointing to audio frame.
 *                                  2. frame_buffer can be set to NULL when the
 *                                  buf_info points to
 *                                  CY_SVC_BUF_INFO_PREROLL_INSUFFICIENT_BUF
 *                                  3. In case of stereo, the buffers will be
 *                                  non-interleaved.
 *                                  In other words, first 320 bytes refers channel1
 *                                  and next 320 bytes refers be channel2.
 * @param[in] frame_count           Number of audio frames
 * @param[in] buf_info              Buffer information. For more details refer
 *                                  \ref cy_svc_buffer_info_t
 * @param[in] callback_user_arg     User argument passed in \ref cy_svc_hp_init
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
typedef cy_rslt_t (*cy_svc_hp_data_callback_t)(
        CY_SVC_DATA_T *frame_buffer,
        uint32_t frame_count,
        cy_svc_buffer_info_t buf_info,
        void *callback_user_arg);

/*******************************************************************************
 *                      Structures
 ******************************************************************************/
/**
 * SVC configuration structure for high performance domain.
 */
typedef struct
{
    /**
     * User argument for callback.
     */
    void *callback_user_arg;

    /**
     * Callback function needs to register to get audio data. This callback
     * registration is mandatory.
     *
     * Application needs to consume the data pointer as fast as possible and
     * return this API without blocking.
     *
     * In the other side, SVC LP MW will not wait for any acknowledgment
     * for the data processing status by SVC HP MW. SVC LP MW will continue to
     * consume the circular buffer in real time. the SVC HP APP needs to consume
     * the buffers (preferably at non real time - faster than real time).
     *
     * Note: Application should not call any SVC HP APIs from this callback.
     * Calling SVC APIs may results in blocking the SVC internal thread.
     */
    cy_svc_hp_data_callback_t   data_callback;

} cy_svc_hp_config_t;

/*******************************************************************************
 *                      Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                      Function Declarations
 ******************************************************************************/
/**
 * Initializes the Staged Voice Control module on high performance domain.
 * It creates the resources required to communicate with staged voice control
 * running on low power (m33). Resources could be IPC, internal task, etc.,
 *
 * @param[in]  init                 Staged voice control module init
 *                                  configuration parameter in high performance
 *                                  domain.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_hp_init(cy_svc_hp_config_t *init);

/**
 * Set state to SVC-HP, This api works in synchronous mode. This api interacts
 * with SVC LP and sets the state and then it returns to application.
 *
 * In other words, Once SVC-HP application process the input data and get the
 * response of data processing (example from HPWWD or ASR). the response of
 * data processing needs to be sent to SVC through this API. This API would
 * take care of sending the message internally through IPC to the SVC-LP module.
 *
 * @param[in] state                 State of the processed data
 * @param[in] state_info            Information requested by the SVC HP application
 *
 *                                  example:
 *                                  For state - CY_SVC_SET_STATE_HIGH_PERFORMANCE_DETECT_IN_PROGRESS:
 *                                  state-info cy_svc_hp_set_state_hpwwd_detection_in_prog_info_t
 *
 *                                  For state - CY_SVC_SET_STATE_ASR_DETECTED
 *                                  state-info - cy_svc_hp_set_state_asr_detected_info_t
 *
 *                                  For other states: state_info must be NULL.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_hp_set_state(cy_svc_set_state_t state, void *state_info);

/**
 * Get the current stage of staged voice control module.
 *
 * Get current stage will return immediately with the latest available status
 * from SVC-HP module, whereas cy_svc_hp_set_state() is synchronous, the
 * application needs to ensure the cy_svc_hp_set_state() is executed first
 * and then it has to call cy_svc_hp_get_stage for any transition stage. In
 * other words, the cy_svc_hp_set_state and cy_svc_hp_get_stage() needs to be
 * called from the same thread.
 *
 *
 * @param[out]  stage                staged voice control module's current stage
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_hp_get_current_stage(cy_svc_stage_t *stage);

/**
 * De-initialize the Staged Voice Control module in high performance domain and
 * deletes the resources created during \ref cy_svc_hp_init
 *
 * Application should take care of not calling any of SVC MW LP APIs (cy_svc_hp_set_state
 * cy_svc_hp_get_current_stage ) when the cy_svc_hp_deinit is in progress.
 * During deinit if the application calls any of the API, it may results in crash.
 * Adding protection internally between these apis and cy_svc_lp_deinit results
 * in unwanted synchronous mechanism and adds load to the system.
 * The usecase of cy_svc_lp_deinit is very rare. Hence application should take
 * care of this condition.
 *
 * @param None
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_svc_hp_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __CY_STAGED_VOICE_CONTROL_HP_H__ */
