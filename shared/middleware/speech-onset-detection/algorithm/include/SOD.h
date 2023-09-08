/**
 * SOD.h                                                                         
 *                                                                               
 * Low Complexity Speech Onset Detection                                         
 *                                                                               
 * Robert Zopf                                                                   
 * Infineon Technologies                                                         
 * 11/29/2021                                                                    
 */
#ifndef SOD_H
#define SOD_H

#include "ifx_sp_common.h"
#include "supportFunctions_utils.h"
#include "ifx_sp_common_priv.h"

#define SENS            0.5  // sensitivity setting - original = 0.5, 1.0 = high hit rate, high false = low thresholds
#define S_R             0.7

#define ONSET_GAP       400 //ms, 0, 100, 200, 300, ,400, 500, 1000

/**
 * Frame size 160 samples.
 */
#define FRAME_SIZE_16K 160 // 10ms
#define D_ORDER (16)

/**
 * Enumerate the 5 different bands
 */
enum band {_0_8, _0_4, _0_2, _4_8, _2_4};

/**
 * Structure containing the final settings to be used by the SOD.  These settings may be derived based on user 
 * init settings of onset gap and sensitivity
 */
struct IFX_SOD_SETTINGS_STRUCT
{
    int16_t  a_init;
    int16_t  maxnthl;
    int16_t  q_maxnthl;
    int16_t  aventhl;
    int16_t  q_aventhl;
    int16_t  maxnthh;
    int16_t  q_maxnthh;
    int16_t  aventhh;
    int16_t  q_aventhh;
    int16_t  maxnthhl;
    int16_t  q_maxnthhl;
    int16_t  aventhhl;
    int16_t  q_aventhhl;
    int16_t  ravec;         // Running Average Coefficient
    int16_t  maxdecay;      // 0.995; % Max tracking decay
    int16_t  smaxdecayl;
    int16_t  smaxdecayh;
    int16_t  maxsthl;
    int16_t  q_maxsthl;
    int16_t  maxsthh;
    int16_t  q_maxsthh;
    int16_t  minth;
    int16_t  ravecth;
    int16_t  q_ravecth;
    int16_t  sodcnt;        //     % how many frames to verify a trigger
    int16_t  spth;
    int16_t  q_spth;
    int16_t  snth;
    int16_t  q_snth;
    int16_t  nmaxave;
    int16_t  q_nmaxave;
    int16_t  th_update;
};

/**
 * Contains compile time configurations
 */
struct IFX_SOD_CONFIG_STRUCT
{
    int16_t  ravec;         // Running Average Coefficient
    int16_t  maxdecay;      // 0.995; % Max tracking decay
    int16_t  smaxdecayl;    
    int16_t  smaxdecayh;
    int16_t  maxsthl;
    int16_t  q_maxsthl;
    int16_t  maxsthh;
    int16_t  q_maxsthh;
    int16_t  minth;
    int16_t  ravecth;
    int16_t  q_ravecth;
    int16_t  sodcnt;        //     % how many frames to verify a trigger
    int16_t  spth;
    int16_t  q_spth;
    int16_t  snth;
    int16_t  q_snth;
    int16_t  nmaxave;
    int16_t  q_nmaxave;
    int16_t  th_update;
    int16_t  sens_range;  
};

/**
 * Settings dependent on the onset gap setting.
 */
struct IFX_SOD_GAP_CONFIG_STRUCT
{
    int16_t  a_init;
    int16_t  maxnthl;
    int16_t  q_maxnthl;
    int16_t  aventhl;
    int16_t  q_aventhl;
    int16_t  maxnthh;
    int16_t  q_maxnthh;
    int16_t  aventhh;
    int16_t  q_aventhh;
    int16_t  maxnthhl;
    int16_t  q_maxnthhl;
    int16_t  aventhhl;
    int16_t  q_aventhhl;
};

/**
 * Static memory for frame based classification 
 */
struct IFX_SOD_CLASS_STRUCT         // 180
{
    int32_t nRA[5];                 // 20
    int16_t th[5];                  // 10
    int16_t nth[5];                 // 10
    int16_t sth[5];                 // 10
    int16_t snth[1];                // 2
    int32_t nMAX[5];                // 20
    int32_t sMAX[5];                // 20
    int32_t shadow_nMAX[2][5];      // 40
    int32_t shadow_nRA[2][5];       // 40
    int16_t ActiveSpeech;           // 2
    int16_t sodcnt;                 // 2
    int16_t init;                   // 2
    int16_t thcnt;                  // 2
};

/**
 * Static memory for feature extraction, and class and settings memory included
 */
struct IFX_SOD_STRUCT               // 80
{
    int16_t DCxmem;                 // 2
    int16_t DCymem;                 // 2
    int16_t mem16k[D_ORDER];        // 32
    int16_t mem8k[D_ORDER];         // 32
    uint32_t E0_8;                  // 4
    uint32_t E0_4;                  // 4
    uint32_t E0_2;                  // 4
    struct IFX_SOD_CLASS_STRUCT SODCLASSmem;
    struct IFX_SOD_SETTINGS_STRUCT SODSETTINGSmem;
};

/**
 * void initSOD(struct IFX_SOD_STRUCT *SODmem, int16_t onsetgap, int16_t sensitivity)
 *
 * Purpose - initialize the SOD
 *
 * Input-
 *    *SODmem     - pointer to allocated memory block
 *    onsetgap    - the minimum non-speech time before onset (0,100,200,300,400,500,1000) milliseconds), 400 = nominal
 *    sensitivity - the detection sensitivity (0- 32767) 0 = least sensitive, 32767 = most sensitive, 16384 = nominal
 * Output-
 *    Initialized memory
 *
 */
void initSOD(struct IFX_SOD_STRUCT *SODmem, int16_t onsetgap, int16_t sensitivity);

/**
 * int16_t SOD(int16_t *in, struct IFX_SOD_STRUCT *SODmem)
 *
 * Purpose - Speech Onset Detection
 *
 * Input-
 *    *in   - pointer to input buffer of length FRAME_SIZE_16K+MAX_FIR_ORDER, with *in containing MAX_FIR_ORDER preceding.  Hence in[-MAX_FIR_ORDER] is valid.
 *            The SOD will overwrite the samples in this buffer.
 *    *SODmem - pointer to initialized SOD memory
 *    *scratchPt - scratch memory pointer
 * 
 * Output-
 *    none
 *
 * Return - 0 = no SOD, 1 = SOD
 */
int16_t SOD(int16_t *in, struct IFX_SOD_STRUCT *SODmem, ifx_scratch_mem_t* scratchPt);

#endif