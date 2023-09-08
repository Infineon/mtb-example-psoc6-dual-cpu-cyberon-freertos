/*******************************************************************************
 * File Name:   cyberon_asr.c
 *
 * Description: This file contains the custom two stage cyberon asr process 
 *              and callback code.
 *
 * Related Document: See README.md
 *
 *
 ********************************************************************************
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
 ********************************************************************************/

/*******************************************************************************
 Note: The Dspotter library is Cyberon product for voice recognition.
 For more details refer 
    Cyberon webpage: https://www.cyberon.com.tw/index.php?lang=en
    Cyberon CEs: https://github.com/CyberonEBU
 ********************************************************************************/


#include "cy_pdl.h"
#include "cybsp.h"
#include "custom_two_stage_asr.h"
#include "cy_staged_voice_control_hp.h"
#include "svc_hp.h"
#include "DSpotterSDKApi.h"
#include "CybModelInfor.h"

#define MAX_TIME             (700U)
#define TIMEOUT              (900U)
#define UPPER_LIMIT          (50U)
#define POST_WW_FRAME_COUNT  (35U)
#define CHAR_BUFFER_SIZE     (16U)
#define COMMAND_SIZE         (64U)
#define DECIMAL_BASE         (10U)

extern const char __start_command_data;
extern const char __start_license_data;

extern cyberon_asr_callback g_lpfnCallback;

extern BYTE **g_lppbyGroup;
extern INT g_nGroup;
extern BYTE *g_lpbyMemPool;
extern INT g_nMemPool;
extern HANDLE g_hDSpotter;
extern BOOL bEnableTimeout;
extern HANDLE g_hCybModel;
BYTE **g_lppbyMapID  = NULL;
two_stage_cyb_state_t cyb_state = CYB_STATE1;

/* Timer object used for blinking the LED */
extern cyhal_timer_t led_blink_timer;

/* Counter to measure the delay of the LED */
extern volatile uint8_t hw_timer_count;

cy_svc_hp_set_state_asr_detected_info_t asr_detected_state_info = {0};
cy_svc_hp_set_state_hpwwd_det_in_prog_info_t hpwwd_in_progress = {0};

/*******************************************************************************
 * Function Name: asr_callback
 ********************************************************************************
 * Summary:
 *  The callback of automatic speech recognition.
 *
 *  Parameters:
 *  function: function name
 *  message: status message
 *  parameter: status code
 *
 *******************************************************************************/
void asr_callback(const char *function, char *message, int parameter)
{
    printf("[%s]%s(%s)\r\n", function, message, (char *)parameter);
}

/*******************************************************************************
 * Function Name: custom_two_stage_asr_process
 ********************************************************************************
 * Summary:
 *  The custom cyberon asr process for two stage cyberon models
 *
 *  Parameters:
 *  lpsSample: Pointer to the frame
 *  nNumSample: Number of samples in the frame
 *  
 *  Return:
 *  void
 *
 *******************************************************************************/
void custom_two_stage_asr_process(short *lpsSample, int nNumSample)
{
    INT nErr;
    static UINT frame_count = 0;
    static UINT post_ww_frame_count = 0;
    static INT cyb_result = 0;
    static INT nRecognitionTimes = 0;
    const UINT post_ww_frame_count_req = POST_WW_FRAME_COUNT;
    char pchBuf[CHAR_BUFFER_SIZE];
    char pchCommand[COMMAND_SIZE];
    INT nMapID;

    if (!g_hDSpotter)
    {
        return;
    }

    frame_count++;
    
    if ((nErr = DSpotter_AddSample(g_hDSpotter, lpsSample, nNumSample)) 
         == DSPOTTER_SUCCESS)
    {
        cyb_result = DSpotter_GetResult(g_hDSpotter);

        if (cyb_state == CYB_STATE1) 
        {
            CybModelGetCommandInfo(g_hCybModel, 0, cyb_result, pchCommand, 
                                   COMMAND_SIZE, &nMapID, NULL);
        }else if (cyb_state == CYB_STATE2) 
        {
            CybModelGetCommandInfo(g_hCybModel, 1, cyb_result, pchCommand, 
                                   COMMAND_SIZE, &nMapID, NULL);
        }
        else 
        {
            /* DO NOTHING */
        }
        cyb_result = nMapID;

        g_lpfnCallback(__func__, "Result id", itoa(cyb_result, pchBuf, 
                       DECIMAL_BASE));
       DSpotter_Reset(g_hDSpotter);

        if ((nRecognitionTimes += 1) > UPPER_LIMIT)
            g_lpfnCallback(__func__, "Upper limit of recognition is reached", 
                            (char *)UPPER_LIMIT);
    }
    /* Process all the frames in the context of CYB_SATE1 and check for WW*/
    if (cyb_state == CYB_STATE1)
    {
        /* Check for WW in PRE_ROLL_SIZE_IN_SEC + POST_ROLL_SIZE_IN_SEC seconds of time */
        if (frame_count <= (NUMBER_OF_FRAMES_MULTIPLIER * (PRE_ROLL_SIZE_IN_SEC
                            + POST_ROLL_SIZE_IN_SEC)))
        {
            /* Request post ww frames if ww is not detected within pre roll */
            if ((frame_count >= (NUMBER_OF_FRAMES_MULTIPLIER * 
                PRE_ROLL_SIZE_IN_SEC)) && (post_ww_frame_count < 
                (NUMBER_OF_FRAMES_MULTIPLIER * POST_ROLL_SIZE_IN_SEC)) && 
                (WW_CMD != cyb_result))
            {
                hpwwd_in_progress.action = CY_SVC_SEND_POST_WWD_BUFFER;
                if (((NUMBER_OF_FRAMES_MULTIPLIER * POST_ROLL_SIZE_IN_SEC) - 
                    post_ww_frame_count) > post_ww_frame_count_req)
                {
                    hpwwd_in_progress.post_wwd_frame_count = post_ww_frame_count_req;
                    post_ww_frame_count += post_ww_frame_count_req;
                }
                else
                {
                    hpwwd_in_progress.post_wwd_frame_count = 
                    ((NUMBER_OF_FRAMES_MULTIPLIER * POST_ROLL_SIZE_IN_SEC) 
                      - post_ww_frame_count);
                    post_ww_frame_count = (NUMBER_OF_FRAMES_MULTIPLIER * 
                                           POST_ROLL_SIZE_IN_SEC);
                }
                cy_svc_hp_set_state(CY_SVC_SET_STATE_HIGH_PERFORMANCE_DETECT_IN_PROGRESS, 
                                    &hpwwd_in_progress);
            }
            /* WWD */
            if (WW_CMD == cyb_result)
            {
                frame_count = 0;
                cyb_result = 0;
                post_ww_frame_count = 0;
                /* Switch to cyberon state 2 */
                cyb_state = CYB_STATE2;

                /* Turn ON LED to indicate the detection of WW */
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

                /* Enable gropu2 of Cyberon */
                g_hDSpotter = DSpotter_Init_Multi((BYTE *)CybModelGetBase(g_hCybModel), 
                             (BYTE **)&g_lppbyGroup[1], 1,
                             MAX_TIME, g_lpbyMemPool, g_nMemPool, NULL, 0, 
                             &nErr, (BYTE *)&__start_license_data);
                DSpotter_SetResultMapID_Multi(g_hDSpotter, 
                                              (BYTE **)&g_lppbyMapID[1], 
                                              g_nGroup - 3);

                /* set SVC HP stage to WWD */
                cy_svc_hp_set_state(CY_SVC_SET_STATE_HIGH_PERFORMANCE_WAKEUP_WORD_DETECTED, NULL);
            }
            else if (frame_count == (NUMBER_OF_FRAMES_MULTIPLIER * 
                     (PRE_ROLL_SIZE_IN_SEC + POST_ROLL_SIZE_IN_SEC)))
            {
                /* WWND within pre roll + post roll frames */
                frame_count = 0;
                post_ww_frame_count = 0;
                /* set SVC HP stage to WW Not Detected */
                cy_svc_hp_set_state(CY_SVC_SET_STATE_HIGH_PERFORMANCE_WAKEUP_WORD_NOT_DETECTED, NULL);
            }
        }
        else
        {
            /* You should not land here: 
              CYB_STATE1 for frame_count>(pre_roll + post_roll) 
              should not happen */
              printf("Frame_count>(pre_roll + post_roll) not expected in CYB_STATE1");
        }
    }
    /* Process all the frames in the context of CYB_SATE2 and check for ASR*/
    else if (cyb_state == CYB_STATE2)
    {
        if (TIMEOUT < frame_count)
        {
            frame_count = 0;
            /* Switch to cyberon state 2 */
            cyb_state = CYB_STATE1;

            /* Turn OFF LED to indicate the TIMEOUT */
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

            /* Enable gropu1 of Cyberon */
            g_lpfnCallback(__func__, "Timeout", itoa(TIMEOUT, pchBuf, DECIMAL_BASE));
            g_hDSpotter = DSpotter_Init_Multi((BYTE *)CybModelGetBase(g_hCybModel), 
                                              (BYTE **)&g_lppbyGroup[0], 1,
                                              MAX_TIME, g_lpbyMemPool, 
                                              g_nMemPool, NULL, 0, &nErr, 
                                              (BYTE *)&__start_license_data);
            DSpotter_SetResultMapID_Multi(g_hDSpotter, g_lppbyMapID, 1);
            cy_svc_hp_set_state(CY_SVC_SET_STATE_ASR_NOT_DETECTED, NULL);
        }
        else
        {
            if ((ASR_CMD1 <= cyb_result) && (ASR_CMD4 >= cyb_result))
            {
                /* set local frame count to 0 */
                frame_count = 0;
                /* Switch to cyberon state 1 */
                cyb_state = CYB_STATE1;
                /* Enable gropu1 of Cyberon */
                g_hDSpotter = DSpotter_Init_Multi((BYTE *)CybModelGetBase(g_hCybModel), 
                                                  (BYTE **)&g_lppbyGroup[0], 1, 
                                                  MAX_TIME, g_lpbyMemPool,
                                                  g_nMemPool, NULL, 0, &nErr, 
                                                  (BYTE *)&__start_license_data);
                DSpotter_SetResultMapID_Multi(g_hDSpotter, g_lppbyMapID, 1);

                cy_svc_hp_set_state(CY_SVC_SET_STATE_ASR_DETECTED, 
                                    &asr_detected_state_info);
            }
            
            switch (cyb_result)
            {
                case ASR_CMD1:
                    /* Start Die temperature conversion for SAR ADC */
                    cyb_result = 0;
                    /* Initiate continuous conversions. */
                    Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS);
                    cy_svc_hp_set_state(CY_SVC_SET_STATE_ASR_PROCESSING_COMPLETED, NULL);
                    /* Turn OFF LED to indicate the TIMEOUT */
                    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
                    break;
                    
                case ASR_CMD2:
                    /* Start the timer which will blink LED at 10Hz */
                    cyb_result = 0;
                    led_state = BLINK_QUICKLY;
                    hw_timer_count = 0;
                    /* Start LED blinking by starting the timer */
                    cyhal_timer_start(&led_blink_timer);
                    cy_svc_hp_set_state(CY_SVC_SET_STATE_ASR_PROCESSING_COMPLETED, NULL);
                    break;
                    
                case ASR_CMD3:
                    /* Start the timer which will blink LED at 10Hz */
                    cyb_result = 0;
                    led_state = BLINK_SLOWLY;
                    hw_timer_count = 0;
                    /* Start LED blinking by starting the timer */
                    cyhal_timer_start(&led_blink_timer);
                    cy_svc_hp_set_state(CY_SVC_SET_STATE_ASR_PROCESSING_COMPLETED, NULL);
                    break;
                    
                case ASR_CMD4:
                    /* Stop LED blinking */
                    cyb_result = 0;
                    led_state = STOP_BLINKING;
                    /* Stop LED blinking by stopping the timer */
                    cyhal_timer_stop(&led_blink_timer);
                    /* Turn OFF LED */
                    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
                    cy_svc_hp_set_state(CY_SVC_SET_STATE_ASR_PROCESSING_COMPLETED, NULL);
                    break;

                default:
                    break;
            }
        }
    }
}
