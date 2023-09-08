/*******************************************************************************
* File Name:   cyberon_asr.c
*
* Description: This file contains the one stage asr process and callback code.
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

#include "cyberon_asr.h"
#include "CybModelInfor.h"
#include "svc_lp.h"
#include "custom_one_stage_asr.h"

#define MAX_TIME                      (500U)
#define NOT_SHOW_MULTI_PRONUNCIATION  (1U)
#define ENABLE_AGC                    (1U)
/* Max length of array used for logging messages */
#define MAX_LEN                       (128U)
#define CHAR_BUFFER_SIZE              (16U)
#define COMMAND_SIZE                  (64U)
#define DECIMAL_BASE                  (10U)

extern const char __start_command_data;
extern const char __start_license_data;

extern cyberon_asr_callback g_lpfnCallback;

extern BYTE **g_lppbyGroup;
extern INT g_nGroup;
extern BYTE *g_lpbyMemPool;
extern INT g_nMemPool;
extern HANDLE g_hDSpotter;
extern HANDLE g_hCybModel;

/*******************************************************************************
 * Function Name: asr_callback
 ********************************************************************************
 * Summary:
 *  Callback to print the messages from cyberon functions.
 *
 * Parameters:
 *  function: Function name
 *  message: Message to be printed
 *  parameter: Parameter value to be printed
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void asr_callback(const char *function, char *message, char *parameter)
{
    char print_message[MAX_LEN] = "";
    snprintf(print_message, sizeof(print_message), "[%s]%s(%s)\n\r", 
             function, message, parameter);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, print_message);
}

void custom_one_stage_asr_process(short *lpsSample, int nNumSample, int *cyb_wwd_status)
{
    INT nErr;
    char pchBuf[CHAR_BUFFER_SIZE];
    char pchCommand[COMMAND_SIZE];
    INT nCommandID;
    INT nMapID;
    INT nConfidenceScore;

    if (!g_hDSpotter)
    {
        return;
    }

    if (DSPOTTER_SUCCESS == (nErr = DSpotter_AddSample(g_hDSpotter, lpsSample, 
                                                       nNumSample)))
    {
        nCommandID = DSpotter_GetResult(g_hDSpotter);

        DSpotter_GetResultScore(g_hDSpotter, &nConfidenceScore, NULL, NULL);
        CybModelGetCommandInfo(g_hCybModel, 0, nCommandID, pchCommand, COMMAND_SIZE, 
                               &nMapID, NULL);
        *cyb_wwd_status = LPWW_DETECTION_SUCCESFUL; 
        DSpotter_Reset(g_hDSpotter);

#if NOT_SHOW_MULTI_PRONUNCIATION
        if (strstr(pchCommand, " ^"))
            strstr(pchCommand, " ^")[0] = '\0';
#endif

    }
    else if (DSPOTTER_ERR_Expired == nErr)
    {
        g_lpfnCallback(__func__, "Upper limit of recognition times is reached", 
                       itoa(DSPOTTER_ERR_Expired, pchBuf, DECIMAL_BASE));
    }
}
