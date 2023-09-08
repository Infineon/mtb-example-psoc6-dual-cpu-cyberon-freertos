/******************************************************************************
* File Name:   main.c
*
* Description: This file has example for SOD detection on PSOC6.
*
* Related Document: See Readme.md
*******************************************************************************/
#ifdef RUN_SOD_ONLY
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "stdlib.h"
#include "cy_log.h"
#include "cy_buffer_pool.h"
#include "app_defines.h"
#include "cy_sod.h"

#define FILE_INPUT

#ifdef FILE_INPUT
#include "Ok_Infineon_5_talkers.h"
#endif

/* Enable below flag to feed the audio data */
#define USE_THREAD

//#define DUMP_INPUT_AUDIO_DATA
/*******************************************************************************
* Macros
********************************************************************************/
char input_pdm_mic_data_mono_frame[MAX_SIZE_FOR_1_FRAME_FOR_MONO] = {0};

#define app_sod_log_info(format,...)         cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[App] "format" \r\n",##__VA_ARGS__);
#define app_sod_log_err(ret_val,format,...)  cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_ERR,"[App] [Err:0x%lx] "format" \r\n",ret_val,##__VA_ARGS__);
#define app_sod_log_debug(format,...)        cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_DEBUG,"[App] "format" \r\n",##__VA_ARGS__);

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void clock_init(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg =
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_STEREO,
    .word_length     = 16,  /* bits */
    .left_gain       = 0,   /* dB */
    .right_gain      = 0,   /* dB */
};

int16_t audio_app_buffer_ping[FRAME_SIZE];
int16_t audio_app_buffer_pong[FRAME_SIZE];

cy_buffer_t audio_input_buffer = NULL;

cy_thread_t thread;
void *handle = NULL;
/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
*  Initialize the clocks in the system.
*
*******************************************************************************/
void clock_init(void)
{
    /* Initialize the PLL */
    cyhal_clock_get(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_init(&pll_clock);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (CLK_HF[1])
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    cyhal_clock_get(&audio_clock, &CYHAL_CLOCK_HF[1]);
    cyhal_clock_init(&audio_clock);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}

/*******************************************************************************
* Function Name: pdm_pcm_isr_handler
********************************************************************************
* Summary:
*  PDM/PCM ISR handler. Invoked when number of bytes received
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void) arg;
    (void) event;
    static bool ping_pong = false;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_afe_app_get_free_input_buffer_from_queue(&audio_input_buffer, true);
    if(CY_RSLT_SUCCESS != result)
    {
        return;
    }

    if (ping_pong)
    {
        cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

        /* Copy data to buffer from the pool */
        memcpy(audio_input_buffer, audio_app_buffer_pong, sizeof(audio_app_buffer_pong));

        /* Push audio data to thread for further processing */
        result = cy_afe_app_push_output_buffer_to_process_queue(audio_input_buffer, true);
        if(CY_RSLT_SUCCESS != result)
        {
            return;
        }

        cyhal_pdm_pcm_read_async(&pdm_pcm, audio_app_buffer_ping, FRAME_SIZE);
    }
    else
    {
        cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_ON);

        /* Copy data to buffer from the pool */
        memcpy(audio_input_buffer, audio_app_buffer_ping, sizeof(audio_app_buffer_ping));

        /* Push audio data to thread for further processing */
        result = cy_afe_app_push_output_buffer_to_process_queue(audio_input_buffer, true);
        if(CY_RSLT_SUCCESS != result)
        {
            return;
        }

        cyhal_pdm_pcm_read_async(&pdm_pcm, audio_app_buffer_pong, FRAME_SIZE);
    }

    ping_pong = !ping_pong;
}

cy_rslt_t cy_afe_app_setup_input_buffer()
{
    cy_buffer_t buffer;
    cy_buffer_pool_params_t buffer_pool_params;
    cy_buffer_pool_handle pool_buf_handle;
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int i = 0;

    memset(&buffer_pool_params, 0, sizeof(cy_buffer_pool_params_t));

    /* Currently, Buffer pool manager middleware implementation uses mutex to protect the critical section for
     * cy_buffer_pool_get and cy_buffer_pool_free APIs and cannot be called from ISR. Hence, get all the buffers
     * from the buffer pool and push it to the queue then use it whenever data received from PDM mic. */

    /* Initialize queue to store all the free buffers from buffer pool */
    cy_afe_app_initialize_free_input_buffer_queue(NO_OF_BUFFERS, sizeof(cy_buffer_t));

    buffer_pool_params.no_of_buffers = NO_OF_BUFFERS;
    buffer_pool_params.pool_name = "INPUT_AUDIO_DATA_POOL";
    buffer_pool_params.sizeof_buffer = SIZE_OF_BUFFER;

    result = cy_buffer_pool_create(&buffer_pool_params, &pool_buf_handle);
    if(result != CY_RSLT_SUCCESS)
    {
        app_sod_log_err(result, "Failed to create buffer pool");
        return result;
    }

    app_sod_log_debug("Get all the buffers from pool and push to queue");

    for(i = 0; i < NO_OF_BUFFERS; i++)
    {
        /* Get the buffer from pool */
        result = cy_buffer_pool_get(pool_buf_handle, &buffer);
        if(result != CY_RSLT_SUCCESS)
        {
            app_sod_log_err(result, "Failed to get the buffer from pool");
            return result;
        }

        app_sod_log_debug("Buffer from pool : [%p]", buffer);

        /* Push free buffer to queue */
        cy_afe_app_push_free_input_buffer_to_queue(buffer, false);
    }

    app_sod_log_info("cy_buffer_pool_create success, handle : [%0x] \r\n", pool_buf_handle);
}

#ifdef USE_THREAD
static void audio_processing_thread(cy_thread_arg_t thread_input)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_buffer_t afe_out_buf = NULL;
    int i = 0;
    unsigned int processed_audio_data_size = 0;

    while(1)
    {
#ifdef FILE_INPUT
        processed_audio_data_size = WAV_HEADER_LENGTH;
        /* Loop through audio data from file */
        while(processed_audio_data_size < hex_array_size)
        {

            memcpy(input_pdm_mic_data_mono_frame, &hex_array[processed_audio_data_size], (MONO_FRAME_SIZE * sizeof(int16_t)));

#ifdef DUMP_INPUT_AUDIO_DATA
            printf("Input audio data: \n");
            for(i= 0;i<(MONO_FRAME_SIZE * sizeof(int16_t));i++)
            {
                printf("0x%02x ", (unsigned char) input_pdm_mic_data_mono_frame[i]);

                if ((i % 16 == 0) && i)
                    printf("\n");
            }
            printf("\r\n");
#endif

            app_sod_feed_data(handle, input_pdm_mic_data_mono_frame);

            processed_audio_data_size += (MONO_FRAME_SIZE * sizeof(int16_t));
        }
#endif

#ifndef FILE_INPUT
        result = cy_afe_app_get_output_buffer_from_process_queue(&afe_out_buf, false);
        if(result != CY_RSLT_SUCCESS)
        {
            app_sod_log_err(result, "failed to get the audio data from process queue \r\n");
            return;
        }

#ifdef DUMP_INPUT_AUDIO_DATA
        printf("Input audio data: \r\n");
        char* temp = afe_out_buf;
        for(i = 0; i<(STEREO_FRAME_SIZE * sizeof(uint16_t));i++)
        {
            printf("%0x ", temp[i]);
        }

        printf("\r\n");
#endif

        convert_stereo_interleaved_10ms_frame_to_mono(afe_out_buf, input_pdm_mic_data_mono_frame);

#ifdef DUMP_INPUT_AUDIO_DATA
        printf("Mono audio data: \r\n");
        for(i = 0; i<320;i++)
        {
            printf("%0x ", input_pdm_mic_data_mono_frame[i]);
        }

        printf("\r\n");
#endif

        app_sod_feed_data(handle, input_pdm_mic_data_mono_frame);

        result = cy_afe_app_push_free_input_buffer_to_queue(afe_out_buf, false);
        if(CY_RSLT_SUCCESS != result)
        {
            printf("cy_afe_app_get_free_buffer_from_queue failed \r\n");
            return;
        }
#endif
    }
}
#endif

int main(void)
{
    cy_rslt_t result;
    cy_sod_config_params sod_config;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Init the clocks */
    clock_init();

    /* Initialize retarget-io to use the debug UART port */
    // cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    app_sod_log_info("\x1b[2J\x1b[;H");

    app_sod_log_info
    ("****************** SOD test app ****************** \r\n\n");

    cy_log_init(CY_LOG_DEBUG4, NULL, NULL);

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    cy_afe_app_setup_input_buffer();
    app_sod_log_info("Input buffers are created to store incoming audio");

    cy_afe_app_initialize_data_buffer_queue(DATA_BUFFER_QUEUE_LENGTH, sizeof(cy_buffer_t));
    app_sod_log_info("Initialized queue for processing the incoming audio");

    sod_config.sampling_rate = SAMPLE_RATE_HZ;
    sod_config.input_frame_size = MONO_FRAME_SIZE;
    result = cy_sod_init(&sod_config, &handle);
    if (CY_RSLT_SUCCESS == result)
    {
        app_sod_log_info("SOD Initialized successfully");
    }
    else
    {
        app_sod_log_info("cy_sod_init init fail");
        return -1;
    }

#ifndef FILE_INPUT
    /* Initialize the PDM/PCM block */
    cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);

    app_sod_log_info("Ready to receive audio data from Mic....");

    /* Setup to read the next frame */
    cyhal_pdm_pcm_read_async(&pdm_pcm, audio_app_buffer_ping, FRAME_SIZE);

#else
    app_sod_log_info("Ready to receive audio data from file....");
#endif


#ifdef USE_THREAD
    result = cy_rtos_create_thread(&thread,
            audio_processing_thread,
                "audio_processing_thread", NULL, 1280,
                CY_RTOS_PRIORITY_ABOVENORMAL, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        app_sod_log_err(result, "Failed to create thread for AFE tests");
        return;
    }
#endif

    vTaskStartScheduler();
}
#endif
/* [] END OF FILE */
