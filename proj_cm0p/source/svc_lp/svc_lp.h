/******************************************************************************
 * File Name:   svc_lp.h
 *
 * Description: This file contains the low power staged voice control(lp svc) 
 * declarations and constants.
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
#ifndef SVC_LP_H
#define SVC_LP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include "cy_pdl.h"
#include "cy_log.h"
#include "cy_staged_voice_control_lp.h"
#include "custom_one_stage_asr.h"


/****************************************************************************
 * Macros
 *****************************************************************************/
/* Circular buffer size in seconds */
#define CIRCULAR_BUFFER_SIZE_IN_SEC                (10U)

/* Pre-roll size in seconds. Whenever CM0p wakeword detected 
   PRE_ROLL_SIZE_IN_SEC seconds pre-roll audio data is communicated to CM4 */
#define PRE_ROLL_SIZE_IN_SEC                       (3U)

/* Frame size fed into svc */
#define FRAME_SIZE                                 (160U)

/* Board specific SCB for logging the CM0p mesages onto the terminal */
#define CYBSP_UART_HW                              (SCB0)

/* The detection sensitivity (0- 32767) */
#define SOD_SENSITIVITY                            (16384U)

/* After SOD detect, the buffer would be look backed with this delay, to get 
   the correct/approximate close onset of the speech. */
#define SOD_MAX_LATE_HIT_DELAY_IN_MSEC             (100U)

/* Number of frames multiplier calculated for a second cosidering a 
   frame of 10msec */
#define NUMBER_OF_FRAMES_MULTIPLIER                (1000U / 10U)

/*******************************************************************************
 * Enumerations
 ******************************************************************************/
typedef enum 
{
    /* CM0p wake word detection is in progress */
    LPWW_DETECTION_IN_PROGRESS = 1U, 

    /* CM0p wake word detection is succesful */
    LPWW_DETECTION_SUCCESFUL,

    /* CM0p wake word detection is failed */
    LPWW_DETECTION_FAILED

} ww_detection_status_t;

/****************************************************************************
 * Functions Prototypes
 *****************************************************************************/
cy_rslt_t svc_init();
cy_rslt_t svc_lp_cyb_app_cbk(cy_svc_event_t, void *);
cy_rslt_t cy_svc_lp_app_data_callback_t_lpwwd_external(CY_SVC_DATA_T *, 
                                                       uint32_t, void *, 
                                                       cy_svc_lp_external_lpwwd_state_t *);

#ifdef __cplusplus
}
#endif

#endif /* SVC_LP_H */
