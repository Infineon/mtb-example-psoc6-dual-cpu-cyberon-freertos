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
 * @file staged_voice_control_lp_private.h
 *
 */

#ifndef __SVC_LP_PRIVATE_H__
#define __SVC_LP_PRIVATE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef ENABLE_SVC_LP_MW
#include "cy_log.h"
#include "cy_result.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
#include "stddef.h"
#include "stdio.h"
#include "stdlib.h"
//#include "cy_buffer_pool.h"
#include "cyabs_rtos.h"
#include "cy_staged_voice_control.h"
#include "cy_staged_voice_control_common.h"

#ifdef ENABLE_TIMELINE_MARKER
#include "audio_timeline_marker.h"
#endif

#ifdef ENABLE_USB_DBG_OUTPUT
#include "audio_usb_send_utils.h"
#endif

#ifdef ENABLE_IFX_LPWWD
#include "cy_lpwwd.h"
#endif

#include "cy_sod.h"

/*******************************************************************************
 *                              Macros
 ******************************************************************************/

#define ENABLE_SVC_LOW_LATENCY_PROFILE
#define ENABLE_HPF_STUB
#define READABLE_SVC_LOG
#define ENABLE_CBUF_GUARD_BYTES

#if ENABLE_SVC_LP_LOGS == 2
#define cy_svc_log_info(format,...)  printf ("[SLP] "format" \r\n",##__VA_ARGS__);
#define cy_svc_log_err(ret_val,format,...)  printf ("[SLP] [Err:0x%lx, %s:%d] "format" \r\n",ret_val,__FUNCTION__,__LINE__,##__VA_ARGS__);
#define cy_svc_log_err_on_no_isr(ret_val,format,...)  if(false == is_in_isr()) printf ("[SLP] [Err:0x%lx, %s:%d] "format" \r\n",ret_val,__FUNCTION__,__LINE__,##__VA_ARGS__);
#define cy_svc_log_dbg(format,...)  printf ("[SLP] "format" \r\n",##__VA_ARGS__);
#elif ENABLE_SVC_LP_LOGS
#define cy_svc_log_info(format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[SLP] "format" \r\n",##__VA_ARGS__);
#define cy_svc_log_err(ret_val,format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[SLP] [Err:0x%lx, %s:%d] "format" \r\n",ret_val,__FUNCTION__,__LINE__,##__VA_ARGS__);
#define cy_svc_log_err_on_no_isr(ret_val,format,...)  if(false == is_in_isr()) cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[SLP] [Err:0x%lx, %s:%d] "format" \r\n",ret_val,__FUNCTION__,__LINE__,##__VA_ARGS__);
#define cy_svc_log_dbg(format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[SLP] "format" \r\n",##__VA_ARGS__);
#else
#define cy_svc_log_info(format,...)
#define cy_svc_log_err(format,...)
#define cy_svc_log_err_on_no_isr(format,...)
#define cy_svc_log_dbg(format,...)
#endif

#define SVC_PROCESS_THREAD_NAME              "svc_lp_thread"
#define SVC_THREAD_PRIORITY                  (CY_RTOS_PRIORITY_ABOVENORMAL)

/**
 * TODO: Estimate the correct Stack size and modify the right value
 */
#define SVC_THREAD_STACK_SIZE                (2*1280)

#define SAMPLE_SIZE_IN_BYTES (2)

#define FRAME_DATA_SIZE(NO_OF_CHANNELS,BYTES_PER_SAMPLE,SAMPLE_RATE,SINGLE_FRAME_MS) \
       (NO_OF_CHANNELS * BYTES_PER_SAMPLE * SAMPLE_RATE * SINGLE_FRAME_MS / 1000 )

#define MAX_CMD_Q_SUPPORTED_SIZE (10)

//TODO: Remove later, once the completed functionality is verified.
//Right now fixed to the PreRollBufferCount
#define MAX_DATA_Q_SUPPORTED_EXTRA_FROM_PREROLL_BUFFER (50)

#define FIND_MIN(a,b) (((a)<(b))?(a):(b))

extern bool is_in_isr();

/*******************************************************************************
 *                              Constants
 ******************************************************************************/

/*******************************************************************************
 *                              Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                              Type Definitions
 ******************************************************************************/

/*******************************************************************************
 *                              Structures
 ******************************************************************************/

/**
 * Staged voice control event types
 */
typedef enum
{
    //Invalid Event with value 0.
    SVC_TRIGGER_INVALID,

    SVC_TRIGGER_INSTANCE_INIT_DONE,

    SVC_TRIGGER_PUT_TO_DEEP_SLEEP,

    // Force the stage to acoustic activity detection
    SVC_TRIGGER_ACOUSTIC_ACTIVITY_FORCEFULLY,

    // Trigger the wait for Speech onset detection forcefully.
    SVC_TRIGGER_WAIT_FOR_SPEECH_DETECTION_FORCEFULLY,

    // Trigger to wait for LPWWD */.
    SVC_TRIGGER_WAIT_FOR_LOW_POWER_WAKEWORD_DETECTION_FORCEFULLY,

    SVC_TRIGGER_WAIT_FOR_HIGH_POWER_WAKEWORD_DETECTION_FORCEFULLY,

    // Trigger ASR Detection
    SVC_TRIGGER_ASR_DETECTION_FORCEFULLY,

    // Acoustic activity detected event or wake up from Deep sleep
    SVC_TRIGGER_ACOUSTIC_ACTIVITY_DETECTED,

    // Speech detected event
    SVC_TRIGGER_SPEECH_ONSET_DETECTED,

    // Low power wakeup word detected event
    SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_DETECTED,

    // Low power wakeup word not detected event
    SVC_TRIGGER_LOW_POWER_WAKEUP_WORD_NOT_DETECTED,

    // High power wakeup word detected event.
    // Actual source of this event is from High power domain.
    SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_DETECTED,

    // High power wakeup word not detected event
    // Actual source of this event is from High power domain.
    SVC_TRIGGER_HIGH_POWER_WAKEUP_WORD_NOT_DETECTED,

    // ASR detetced event
    // Actual source of this event is from High power domain.
    SVC_TRIGGER_ASR_DETECTED,

    // ASR not detected event
    // Actual source of this event is from High power domain.
    SVC_TRIGGER_ASR_NOT_DETECTED,

    // ASR processing completed event
    // Actual source of this event is from High power domain.
    SVC_TRIGGER_ASR_PROCESSING_COMPLETED,

    SVC_TRIGGER_PRINT_STATISTICS,

    SVC_TRIGGER_SEND_POST_WWD_HPWWD_DET_IN_PROGRESS,

    //Max Event supported
    SVC_TRIGGER_MAX

} svc_stage_trigger_t;

typedef struct
{
    /**
     * Buffer data information
     */
    unsigned int audio_type; /* Mono/stereo */
    unsigned int frame_size_in_bytes;
    unsigned int sameple_size_in_bytes;

    unsigned int wr_offset;
    unsigned int rd_offset;

    unsigned int circular_buf_size;
    unsigned char *buffer_start_address;
    unsigned char *actual_buffer_start_address_with_guard;

} svc_circular_buffer_header_t;

/* List of events */
#define SVC_LP_CMD_ID_DATA_RECEIVED (1)
#define SVC_LP_CMD_ID_SET_STATE_FROM_HP     (2)
#define SVC_LP_CMD_ID_SET_STAGE_FROM_LP_APP     (3)


typedef struct
{
    union {
        cy_svc_set_state_t     set_state;
        cy_svc_stage_t         set_stage;
    };
    uint8_t cmd;
    uint8_t payload_internal[MAX_SET_STATE_INFO_SIZE_IN_BYTES];

} cmd_q_msg_t;


typedef struct
{
    uint8_t cmd;

    uint32_t data_crc_check;

    char *cbuf_pointer;

} data_q_msg_t;


typedef struct
{
    /**
     * Statistics at various levels
     */
    unsigned int frame_counter_received_after_last_aad_dbg;

    volatile unsigned int svc_lp_feed_post_fail_counter_dbg;
    volatile unsigned int svc_lp_feed_post_fail_counter_track_dbg;
    unsigned int crc_check_fail_counter_dbg;

    unsigned int sod_detect_counter_dbg;

    unsigned int lpwwd_detect_counter_dbg;
    unsigned int lpwwd_not_detect_counter_dbg;
    unsigned int lpwwd_feed_counter_dbg;

    unsigned int hpwwd_detect_counter_dbg;
    unsigned int hpwwd_not_detect_counter_dbg;
    unsigned int hpwwd_feed_counter_dbg;

    unsigned int asr_detect_counter_dbg;
    unsigned int asr_not_detect_counter_dbg;
    unsigned int asr_process_completed_counter_dbg;

    cy_time_t last_frame_feed_time_ms;

    /**
     * Remove below params
     */
    volatile unsigned int svc_lp_feed_post_fail_reason_dbg;

} cy_svc_lp_stats_t;

typedef struct
{
    /*
     * Status to maintain the SVC module init done or not
     */
    volatile bool init_done;

    volatile bool api_set_allowed;

    /**
     * Instance create parameters (sent by the application)
     */
    cy_svc_lp_config_t init_params;

    /**
     * SVC thread instance
     */
    cy_thread_t thread_instance;

    /**
     * Quit the thread instance. Required to exit the thread and perform the
     * cleanup required.
     */
    volatile bool quit_thread_instance;

    /**
     * Data Queue handle
     */
    cy_queue_t data_queue;

    /**
     * CMD Queue handle
     */
    cy_queue_t cmd_queue;

    /**
     * Stage on which the SVC module is operating.
     */
    volatile cy_svc_stage_t current_stage;

    /**
     * cmd process semaphore, helps to synchronous command processing
     */
    cy_semaphore_t cmdProcSyncSemaphore;

    cy_semaphore_t cmdProcIPCSendSemaphore;

    volatile cy_rslt_t lp_app_set_stage_process_result;

    /**
     * SOD informations
     */
    cy_sod_t sod_handle;

    bool sod_detected;

    /**
     * Circular buffer information
     */
    svc_circular_buffer_header_t *circular_shared_buffer;

    unsigned char *last_processed_frame_start_address;

    bool lpwwd_detected;
    bool config_update_to_hp_done;

    unsigned char *hpwwd_trigger_data_final_address;

    cy_svc_lp_stats_t stats;

#ifdef ENABLE_IFX_LPWWD
    /* LPWWD handle */
    cy_lpwwd_handle_t lpwwd_handle;
#endif

    bool stream_requested_on_asr_processing_query_state;

    uint16_t post_wwd_frame_count_req_by_hp;
    uint16_t post_lpwwd_received_frame_count;

    /**
     * Deprecated parameter from init, if init, it has to be added later.
     * This will help to provide automatic post wwd buffer.
     */
    uint32_t post_roll_frame_count_from_lpwwd_detect_frame;
    unsigned int post_hpwwd_pending_frame_counter_to_hp;

    /**
     * Enable automatic detection of frame feed discontinuity. In case if the
     * LP core goes to <DeepSleep> and <wake up>. There will be discontinuity in the
     * data feed time. Discontinuity in the data can be detected using the time
     * of the current audio data feed and the last audio data feed.
     */
    bool enable_auto_detect_frame_feed_discontinuity;


    /**
     * Automatic frame feed discontinuity detect timeout. if application
     * sets 1000, then feed discontinuity will be detected if the last frame
     * and the current feed differs by 1sec. If there is a discontinuity, then
     * SVC MW will send an event #CY_SVC_EVENT_ACOUSTIC_ACTIVITY_DETECTED.
     *
     * Acoustic activity is assumed as there is first frame feed after deep sleep.
     *
     * Detection of frame feed discontinuity helps to internal reset of SOD and
     * LPWWD.
     */
    unsigned int auto_frame_feed_discontinuity_timeout_ms;

} svc_lp_instance_t;

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                              Function Declarations
 ******************************************************************************/

#endif

#ifdef __cplusplus
}
#endif

#endif /* __SVC_LP_PRIVATE_H__ */
