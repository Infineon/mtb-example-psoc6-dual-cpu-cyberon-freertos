/******************************************************************************
* File Name: app_defines.h
*
*******************************************************************************/

#ifdef RUN_SOD_ONLY
/*******************************************************************************
* Macros
******************************************************************************/
#define STEREO_FRAME_SIZE           (320)
#define MONO_FRAME_SIZE             (160)

/* Define how many samples in a frame */
#define FRAME_SIZE                  (320)

/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz */
#define SAMPLE_RATE_HZ              16000u

/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ          24576000u

/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             64u

/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

/* Number of buffers to store the input audio data */
#define NO_OF_BUFFERS              50

/* Size of each buffer to store the input audio data */
#define SIZE_OF_BUFFER             640

#define MAX_SIZE_FOR_1_FRAME_FOR_MONO (SIZE_OF_BUFFER/2)

/* Queue length to store the number of audio frames to process by audio thread */
#define DATA_BUFFER_QUEUE_LENGTH (20)

/* Approx wave header length.
 * TODO : Currently it is approximately set for current file. Need to calculate header length */
#define WAV_HEADER_LENGTH        (100)

//#define FILE_INPUT
#define PRINT_OUTPUT_DECISION
//#define PRINT_OUTPUT_SCORE
//#define PRINT_FEATURE_FRAMES
#define PRINT_SOD_STATUS
#endif
