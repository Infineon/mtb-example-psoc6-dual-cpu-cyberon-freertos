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
 * @file cy_staged_voice_control_common.h
 *
 * @brief This file is the common header file for Staged voice control library
 * running on low power domain (SVC-LP) and Staged voice control running
 * on high performance domain (SVC-HP).
 *
 * Abbreviations used in the header files
 *
 * SVC      -   Stage Voice Control
 * SVC-LP   -   Staged Voice Control Middleware running in low power domain (M33)
 * SVC-HP   -   Staged Voice Control Middleware running in high performance domain
 * WWD      -   Wake-up Word Detection
 * LPWWD    -   Low Power Wake-up Word Detection (in M33)
 * HPWWD    -   High Performance Wake-up Word Detection (in M55)
 * ASR      -   Automatic Speech Recognition
 */

#ifndef __CY_STAGED_VOICE_CONTROL_COMMON_H__
#define __CY_STAGED_VOICE_CONTROL_COMMON_H__

#include "cy_log.h"
#include "cy_result.h"
#include "cy_staged_voice_control_errors.h"
#include "stdbool.h"
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/**
 * Audio Sample data type
 */
#ifndef CY_SVC_DATA_T
/**
 * Audio Sample data bit-width is 16bit.
 */
#define CY_SVC_DATA_T int16_t
#endif
/**
 * Supported Input data audio buffer feed in millisecond. Every buffer fed
 * through the API \ref cy_svc_lp_feed should be of this frame time
 */
#define CY_SVC_SUPPORTED_FRAME_TIME_MS            (10)

/*******************************************************************************
 *                      Constants
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/


/**
 * Staged voice control audio input types. This enumeration will be used
 * to inform that staged voice control module on the input data type feed through
 * the API \ref cy_svc_lp_feed_data
 */
typedef enum
{
    /**
     *  Invalid audio input type minimum value.
     */
    CY_SVC_AUDIO_INPUT_TYPE_INVALID,

    /**
     * Audio input type Mono
     */
    CY_SVC_AUDIO_INPUT_TYPE_MONO,

    /**
     * Audio input type Stereo
     */
    CY_SVC_AUDIO_INPUT_TYPE_STEREO,

    /**
     * Invalid audio input type maximum value.
     */
    CY_SVC_AUDIO_INPUT_TYPE_MAX

} cy_svc_audio_input_type_t;

/**
 * Staged voice control stage configuration
 */
typedef enum
{
    /**
     * Enable SOD transitions in SVC LP
     */
    CY_SVC_ENABLE_SOD = (1<<0),

    /**
     * Enable Low Power Wake-up Word (LPWWD) detection in low power domain.
     */
    CY_SVC_ENABLE_LPWWD = (1 << 1),

    /**
     * Enable Staged Voice Control (SVC) to wait for High Performance Wake-up
     * Word Detection (HPWWD).
     *
     * If this option is enabled, then HPWWD detection state will be used by the
     * SVC for its internal stage transitions and actions, otherwise SVC will
     * not consider/wait for HPWWD state. Application needs to set the HPWWD
     * state through the API \ref cy_svc_hp_set_state to SVC.
     *
     * \note Enabling this option, will not enable HPWWD detection in high
     * performance domain. The application running on high performance domain
     * needs to take care of the enabling HPWWD module.
     */
    CY_SVC_ENABLE_HPWWD_STATE_TRANSITIONS = (1 << 2),

    /**
     * Enable Staged Voice Control (SVC) to wait for Automatic Speech
     * Recognition (ASR) detection.
     *
     * If this option is enabled, then ASR detection state will be used by the
     * SVC for its internal stage transitions and actions, otherwise SVC will not
     * consider/wait for ASR state response. Application needs to set the ASR
     * state through the API \ref cy_svc_hp_set_state to SVC.
     *
     * \note Enabling this option, will not enable ASR in high performance
     * domain. The application running on high performance needs to take care of
     * enabling ASR module.
     */
    CY_SVC_ENABLE_ASR_STATE_TRANSITIONS = (1 << 3)

} cy_svc_stage_config_t;

/**
 * Staged voice control sample rate
 */
typedef enum
{
    /**
     * Invalid audio sample rate minimum value.
     */
    CY_SVC_SAMPLE_RATE_INVALID,

    /**
     * Sample rate 16KHz
     */
    CY_SVC_SAMPLE_RATE_16KHZ = 16,

    /**
     * Invalid audio sample rate maximum value.
     */
    CY_SVC_SAMPLE_RATE_MAX

} cy_svc_sample_rate_t;

/**
 * Staged voice control various stages. Staged voice control module will be
 * operating at any one of the stages at any time.
 */
typedef enum
{
    /**
     *  Invalid stage minimum value.
     */
    CY_SVC_STAGE_INVALID,

    /**
     *  Stage details are not known.
     */
    CY_SVC_STAGE_UNKNOWN,

    /**
     *  Waiting for acoustic activity detection.
     */
    CY_SVC_STAGE_WAITING_FOR_ACOUSTIC_ACTIVITY,

    /**
     *  Waiting for Speech Onset Detection (SOD) in audio frame
     */
    CY_SVC_STAGE_WAITING_FOR_SPEECH_ONSET_DETECTION,

    /**
     *  Waiting for Low Power Wakeup Word Detection
     */
    CY_SVC_STAGE_WAITING_FOR_LOW_POWER_WAKEUP_WORD_DETECTION,

    /**
     *  Waiting for High Performance Wakeup Word Detection
     */
    CY_SVC_STAGE_WAITING_FOR_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTION,

    /**
     *  Waiting for ASR (Automatic Speech Recognition) to detect speech
     */
    CY_SVC_STAGE_WAITING_FOR_ASR_REQUEST_DETECT,

    /**
     *  Processing the ASR query command
     */
    CY_SVC_STAGE_ASR_PROCESSING_QUERY_DETECTED,

    /**
     *  Invalid stage maximum value
     */
    CY_SVC_STAGE_MAX

} cy_svc_stage_t;

/**
 * Staged voice control various event types. Staged voice control module
 * would notify staged voice control application on this event using the
 * event callback registered \ref cy_svc_lp_event_callback_t
 */
typedef enum
{
    /**
     *  Invalid event minimum value.
     */
    CY_SVC_EVENT_INVALID,

    /**
     * Acoustic activity detected event
     */
    CY_SVC_EVENT_ACOUSTIC_ACTIVITY_DETECTED,

    /**
     * Low noise detected event
     */
    CY_SVC_EVENT_LOW_NOISE_DETECTED,

    /**
     * Speech onset detected event
     */
    CY_SVC_EVENT_SPEECH_ONSET_DETECTED,

    /**
     * Low power wake-up word detected event
     */
    CY_SVC_EVENT_LOW_POWER_WAKEUP_WORD_DETECTED,

    /**
     * Low power wake-up word not detected event
     */
    CY_SVC_EVENT_LOW_POWER_WAKEUP_WORD_NOT_DETECTED,

    /**
     * High performance wake-up word detected event.
     */
    CY_SVC_EVENT_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTED,

    /**
     * High performance wake-up word not detected event.
     */
    CY_SVC_EVENT_HIGH_PERFORMANCE_WAKEUP_WORD_NOT_DETECTED,

    /**
     * Automatic Speech Recognition (ASR) detected event
     */
    CY_SVC_EVENT_ASR_DETECTED,

    /**
     * Automatic Speech Recognition (ASR) not detected event
     */
    CY_SVC_EVENT_ASR_NOT_DETECTED,

    /**
     * Automatic Speech Recognition (ASR) processing completed event
     */
    CY_SVC_EVENT_ASR_PROCESSING_COMPLETED,

    /**
     *  Invalid event maximum value.
     */
    CY_SVC_EVENT_MAX

} cy_svc_event_t;

/**
 * States set by SVC-HP to SVC-LP through an API \ref cy_svc_hp_set_state.
 */
typedef enum
{
    /*
     *  Invalid set state minimum value.
     */
    CY_SVC_SET_STATE_INVALID,

    /**
     * High performance wake-up word detection in progress
     */
    CY_SVC_SET_STATE_HIGH_PERFORMANCE_DETECT_IN_PROGRESS,

    /**
     * High performance wake-up word detected state
     */
    CY_SVC_SET_STATE_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTED,

    /**
     * High performance wake-up word not detected state
     */
    CY_SVC_SET_STATE_HIGH_PERFORMANCE_WAKEUP_WORD_NOT_DETECTED,

    /**
     * ASR (Automatic speech recognition) detected state
     */
    CY_SVC_SET_STATE_ASR_DETECTED,

    /**
     * ASR (Automatic speech recognition) not detected state
     */
    CY_SVC_SET_STATE_ASR_NOT_DETECTED,

    /**
     * ASR (Automatic speech recognition) detected speech has been processed
     * completely.
     */
    CY_SVC_SET_STATE_ASR_PROCESSING_COMPLETED,

    /*
     *  Invalid set state maximum value.
     */
    CY_SVC_SET_STATE_MAX

} cy_svc_set_state_t;


/**
 * Staged voice control buffer information
 */
typedef enum
{
    /**
     * Data Callback set with this bitmask, indicate there is lesser number
     * of preRollWWD buffer. frame count corresponds to the insufficient buffer
     * count.
     */
    CY_SVC_BUF_INFO_PREROLL_INSUFFICIENT_BUF = (1<<0),

    /**
     * "OK infineon" WWD detected from LPWWD. Callback set with this bitmask
     * indicate that, OK Infineon keyword is detected in LPWWD.
     */
    CY_SVC_BUF_INFO_OK_INFINEON_WWD = (1<<1),

    /**
     * "Alexa" detected from LPWWD. Callback set with this bitmask
     * indicate that, Alexa keyword is detected in LPWWD.
     */
    CY_SVC_BUF_INFO_ALEXA_WWD = (1 << 2),

    /**
     * "OK Google" detected from LPWWD.  Callback set with this bitmask
     * indicate that, "OK Google" keyword is detected in LPWWD.
     */
    CY_SVC_BUF_INFO_OK_GOOGLE_WWD = (1 << 3)

} cy_svc_buffer_info_t;


/**
 * HP set state api action
 */
typedef enum
{
    CY_SVC_STREAM_DATA_REQUEST = (1<<0),

    CY_SVC_SEND_POST_WWD_BUFFER = (1<<1)

} cy_svc_hp_set_state_action;

#define MAX_SET_STATE_INFO_SIZE_IN_BYTES (4)

typedef struct
{
    /**
     * [Byte1 to Byte2]
     * Bit-mask of cy_svc_hp_set_state_action
     * Applicable value: CY_SVC_STREAM_DATA_REQUEST
     */
    uint16_t action;

    uint8_t reserved[MAX_SET_STATE_INFO_SIZE_IN_BYTES-2];

} cy_svc_hp_set_state_asr_detected_info_t;


typedef struct
{
    /**
     * [Byte1 to Byte2]
     * Bit-mask of cy_svc_hp_set_state_action,
     * Applicable value: CY_SVC_SEND_POST_WWD_BUFFER
     */
    uint16_t action;

    /*
     * [Byte3 and Byte4] post wwd frame count. This frames will be sent
     * to SVC HP application on request.
     */
    uint16_t post_wwd_frame_count;

    //uint8_t reserved[0]; // MAX_SET_STATE_INFO_SIZE_IN_BYTES-4];

} cy_svc_hp_set_state_hpwwd_det_in_prog_info_t;

/*******************************************************************************
 *                      Type Definitions
 ******************************************************************************/

/*******************************************************************************
 *                      Structures
 ******************************************************************************/

/*******************************************************************************
 *                      Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                      Function Declarations
 ******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* __CY_STAGED_VOICE_CONTROL_COMMON_H__ */
