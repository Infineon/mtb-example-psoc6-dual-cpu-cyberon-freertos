/*******************************************************************************
* File Name:   cyberon_asr.h
*
* Description: This file contains the one stage asr process and callback 
*              declaration.
*
* Related Document: See README.md
*
*
********************************************************************************
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
********************************************************************************/

/*******************************************************************************
 Note: The Dspotter library is Cyberon product for voice recognition.
 For more details refer 
    Cyberon webpage: https://www.cyberon.com.tw/index.php?lang=en
    Cyberon CEs: https://github.com/CyberonEBU
 ********************************************************************************/

#ifndef CUSTOM_CYBERON_ASR_H_
#define CUSTOM_CYBERON_ASR_H_

#include "DSpotterSDKApi.h"
#include "cyberon_asr.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef void (*cyberon_asr_callback)(const char *lpchFunction, 
              char *lpchMessage, char *lpchParameter);

void asr_callback(const char *function, char *message, char *parameter);
/** Do automatic speech recognition
 *
 * @param[in] lpsSample   The pointer of voice data buffer
 * @param[in] nNumSample  The number of voice data (a unit is a short, we prefer to add 480 samples per call)
 * @param[in] nNumSample  The
 */
void custom_one_stage_asr_process(short *lpsSample, int nNumSample, int *cyb_wwd_status);


#if defined(__cplusplus)
}
#endif

#endif /* CUSTOM_CYBERON_ASR_H_ */
