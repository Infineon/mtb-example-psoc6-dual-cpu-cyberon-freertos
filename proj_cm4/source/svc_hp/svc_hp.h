/******************************************************************************
* File Name:   svc_hp.h
*
* Description: This file contains the high performance staged voice 
               control(lp svc) declarations and constants.
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
#ifndef SVC_HP_H
#define SVC_HP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cy_staged_voice_control.h"
#include "cy_staged_voice_control_common.h"
#include "cy_staged_voice_control_hp.h"
#include "cyabs_rtos.h"
#include "custom_two_stage_asr.h"

/****************************************************************************
* Macros
*****************************************************************************/
/* Task will be created for SVC HP if this macro is defined */
#define ENABLE_TASK_FOR_SVC_HP

/* Audio frame size */
#define FRAME_SIZE                                      (160U)

/* Audio frame size depends on mic configuration*/
#define CYBERON_FRAME_SIZE                              (160U)

/* Pre-roll size in seconds */
#define PRE_ROLL_SIZE_IN_SEC                            (3U)

/* Post-roll size in seconds */
#define POST_ROLL_SIZE_IN_SEC                           (0.35)

/* Number of frames multiplier calculated for a second cosidering a frame of 10msec*/
#define NUMBER_OF_FRAMES_MULTIPLIER                     (1000U / 10U)

/*******************************************************************************
 *                          Enumerations
 ******************************************************************************/
typedef enum 
{
    /* ASR commands */
    WW_CMD   = 101U, /*101U*/
    ASR_CMD1 = 201U, /*201U*/
    ASR_CMD2,
    ASR_CMD3,
    ASR_CMD4
} asr_cmd_t;

typedef enum 
{
    /* LED states */
    BLINK_QUICKLY = 1U, 
    BLINK_SLOWLY,
    STOP_BLINKING,
} led_blink_state_t;

typedef enum {
    CYB_STATE1 = 1U,
    CYB_STATE2
} two_stage_cyb_state_t;

/****************************************************************************
* Global variables
*****************************************************************************/
extern cy_queue_t audio_data_queue_handle;
extern uint16_t cyb_frame_count;
extern led_blink_state_t led_state;

typedef struct svc_hp_audio_data_t
{
    int16_t *data_ptr;
    uint32_t frame_count;

} svc_hp_audio_data_t;

/****************************************************************************
* Functions Prototypes
*****************************************************************************/
cy_rslt_t svc_init();

#ifdef __cplusplus
}
#endif

#endif /* SVC_HP_H */
