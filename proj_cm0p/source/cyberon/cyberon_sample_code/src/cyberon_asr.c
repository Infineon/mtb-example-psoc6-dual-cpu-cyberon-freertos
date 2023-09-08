/*
PLEASE READ THE CYBERON END USER LICENSE AGREEMENT ("LICENSE(Cyberon)") CAREFULLY BEFORE DOWNLOADING, INSTALLING, COPYING, OR USING THIS SOFTWARE AND ACCOMPANYING DOCUMENTATION.
BY DOWNLOADING, INSTALLING, COPYING OR USING THE SOFTWARE, YOU ARE AGREEING TO BE BOUND BY THE AGREEMENT.
IF YOU DO NOT AGREE TO ALL OF THE TERMS OF THE AGREEMENT, PROMPTLY RETURN AND DO NOT USE THE SOFTWARE.
*/

#include "cyberon_asr.h"
#include "CybModelInfor.h"

#define MAX_TIME (500)
#define NOT_SHOW_MULTI_PRONUNCIATION (1)
#define ENABLE_AGC (1)

extern const char __start_command_data;
extern const char __start_license_data;

cyberon_asr_callback g_lpfnCallback;

BYTE **g_lppbyGroup = NULL;
INT g_nGroup = 0;
BYTE *g_lpbyMemPool = NULL;
INT g_nMemPool = 0;
HANDLE g_hDSpotter = NULL;
HANDLE g_hCybModel = NULL;

BOOL cyberon_asr_init(cyberon_asr_callback lpfnCallback)
{
    INT nErr;
    char pchBuf[16];

    if(!lpfnCallback)
        return FALSE;

    g_lpfnCallback = lpfnCallback;

    if(g_hDSpotter)
        cyberon_asr_release();

    do
    {
        g_hCybModel = CybModelInit((const BYTE *)&__start_command_data, NULL, 0, &nErr);
        if(!g_hCybModel)
        {
            g_lpfnCallback(__func__, "Fail to initialize CybModel", itoa(nErr, pchBuf, 10));
            break;
        }

        g_nGroup = CybModelGetGroupCount(g_hCybModel);
        g_lpfnCallback(__func__, "Number of groups", itoa(g_nGroup, pchBuf, 10));

        g_lppbyGroup = (BYTE **)malloc(sizeof(BYTE *) * g_nGroup);
        if(!g_lppbyGroup)
        {
            g_lpfnCallback(__func__, "No available memory", itoa(sizeof(BYTE *) * g_nGroup, pchBuf, 10));
            break;
        }

        for(int i = 0; i < g_nGroup; i++)
            g_lppbyGroup[i] = (BYTE *)CybModelGetGroup(g_hCybModel, i);

        g_nMemPool = DSpotter_GetMemoryUsage_Multi((BYTE *)CybModelGetBase(g_hCybModel), (BYTE **)&g_lppbyGroup[0], 1, MAX_TIME);
        g_lpfnCallback(__func__, "Memory usage", itoa(g_nMemPool, pchBuf, 10));

        g_lpbyMemPool = (BYTE *)malloc(g_nMemPool);
        if(!g_lpbyMemPool)
        {
            g_lpfnCallback(__func__, "No available memory", itoa(g_nMemPool, pchBuf, 10));
            break;
        }

        g_hDSpotter = DSpotter_Init_Multi((BYTE *)CybModelGetBase(g_hCybModel), (BYTE **)&g_lppbyGroup[0], 1, MAX_TIME, g_lpbyMemPool, g_nMemPool, NULL, 0, &nErr, (BYTE *)&__start_license_data);
        if(!g_hDSpotter)
        {
            g_lpfnCallback(__func__, "Fail to initialize SDK", itoa(nErr, pchBuf, 10));
            break;
        }
#if ENABLE_AGC
        nErr = DSpotterAGC_Enable(g_hDSpotter);
        if(nErr != DSPOTTER_SUCCESS)
        {
            g_lpfnCallback(__func__, "Fail to enable AGC", itoa(nErr, pchBuf, 10));
            break;
        }
#endif
        return TRUE;
    }while(0);

    cyberon_asr_release();

    return FALSE;
}

void cyberon_asr_process(short *lpsSample, int nNumSample)
{
    INT nErr;
    char pchBuf[16];
    char pchCommand[64];
    INT nCommandID;
    INT nMapID;
    INT nConfidenceScore;
    INT nVolumeEnergy;

    if(!g_hDSpotter)
        return;

    if((nErr = DSpotter_AddSample(g_hDSpotter, lpsSample, nNumSample)) == DSPOTTER_SUCCESS)
    {
        nCommandID = DSpotter_GetResult(g_hDSpotter);
        DSpotter_GetResultScore(g_hDSpotter, &nConfidenceScore, NULL, NULL);
        nVolumeEnergy = DSpotter_GetCmdEnergy(g_hDSpotter);
        CybModelGetCommandInfo(g_hCybModel, 0, nCommandID, pchCommand, 64, &nMapID, NULL);
        DSpotter_Continue(g_hDSpotter);

#if NOT_SHOW_MULTI_PRONUNCIATION
        if(strstr(pchCommand, " ^"))
            strstr(pchCommand, " ^")[0] = '\0';
#endif
        g_lpfnCallback(__func__, "**********Result**********", "++");
        g_lpfnCallback(__func__, "Command", pchCommand);
        g_lpfnCallback(__func__, "Command ID", itoa(nCommandID, pchBuf, 10));
        g_lpfnCallback(__func__, "Map ID", itoa(nMapID, pchBuf, 10));
        g_lpfnCallback(__func__, "Confidence Score", itoa(nConfidenceScore, pchBuf, 10));
        g_lpfnCallback(__func__, "Volume Energy", itoa(nVolumeEnergy, pchBuf, 10));
        g_lpfnCallback(__func__, "**************************", "--");
    }
    else if(nErr == DSPOTTER_ERR_Expired)
    {
        g_lpfnCallback(__func__, "Upper limit of recognition times is reached", itoa(DSPOTTER_ERR_Expired, pchBuf, 10));
    }
}

void cyberon_asr_release()
{
    if(g_hDSpotter)
    {
        DSpotter_Release(g_hDSpotter);
        g_hDSpotter = NULL;
    }

    if(g_hCybModel)
    {
        CybModelRelease(g_hCybModel);
        g_hCybModel = NULL;
    }

    SAFE_FREE(g_lppbyGroup);
    SAFE_FREE(g_lpbyMemPool);
}
