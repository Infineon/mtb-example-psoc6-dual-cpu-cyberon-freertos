/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the CM4 core source code of the Cyberon code Example
 *              for ModusToolbox.
 *
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
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
 *******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "ipc_communication.h"
#include "svc_hp.h"
#include "custom_two_stage_asr.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
/* Number of samples in an audio frame */
#define FRAME_SIZE                       (160U)

/* Number of frames multiplier calculated for a second cosidering a 
   frame of 10 msec */
#define NUMBER_OF_FRAMES_MULTIPLIER      (1000U / 10U)

/* Audio data buffer size */
#define BUFFER_SIZE_IN_BYTES     (FRAME_SIZE * NUMBER_OF_FRAMES_MULTIPLIER * \
                                  sizeof(int16_t) * PRE_ROLL_SIZE_IN_SEC)

/* Audio data queue length */
#define AUDIO_DATA_QUEUE_LENGTH          (5U)

/* Application thread stack size */
#define APPLICATION_THREAD_STACK_SIZE    (10U * configMINIMAL_STACK_SIZE)

/* Application thread priority */
#define APPLICATION_THREAD_PRIORITY      (4U)

/* The ADC clock is an integer divider of the PeriClk.
   The maximum supported clock frequency for the SAR is 18 MHz.
   With a PeriClk of 50 MHz, the minimum target clock divider is 3,
   SAR Clock = 16.67 MHz */
#define SAR_TARGET_CLK_DIVIDER           (3U)

/* Set the priority for the SAR interrupt */
#define SAR_IRQ_PRIORITY                 (7U)

/* Enable channel 0 */
#define CONFIG1_CHAN_EN                  (1U)

/* Channel 0 is single ended.
   Aperture time is set by Sample Time 0.
   The DieTemp sensor is connected on the SARMUX_VIRT port at PIN_ADDR 0.
   The sensor is enabled once connection to the SAR ADC is made.
   Averaging is enabled. */
#define CONFIG1_CHAN0_CONFIG        (CY_SAR_CHAN_SINGLE_ENDED \
                                    | CY_SAR_CHAN_SAMPLE_TIME_0 \
                                    | CY_SAR_POS_PORT_ADDR_SARMUX_VIRT \
                                    | CY_SAR_CHAN_POS_PIN_ADDR_0 \
                                    | CY_SAR_CHAN_AVG_ENABLE)

/* Single ended channels are signed.
   Averaging mode is set to sequential fixed with 32 samples of averaging. */
#define CONFIG1_SAMPLE_CTRL         (CY_SAR_SINGLE_ENDED_SIGNED \
                                    | CY_SAR_AVG_CNT_32 \
                                    | CY_SAR_AVG_MODE_SEQUENTIAL_FIXED)

/* Channels 1 through 15 are unconfigured. */
#define CONFIG1_CHAN_CONFIG         {(uint32_t)CONFIG1_CHAN0_CONFIG \
                                    , 0UL, 0UL,0UL,0UL,0UL,0UL,0UL,0UL,0UL, \
                                    0UL,0UL,0UL,0UL,0UL,0UL}

/* Enable the End of Scan interrupt only. */
#define CONFIG1_INTR_MASK           (CY_SAR_INTR_EOS_MASK)

/* Use the internal 1.2 V bandgap reference for the SAR reference source.
   Enable the bypass capacitor connection.
   Connect the negative terminal for single ended channels to VSSA. */
#define CONFIG1_VREF_MV_VALUE       (1200UL)
#define CONFIG1_CTRL                (CY_SAR_VREF_SEL_BGR \
                                    | CY_SAR_BYPASS_CAP_ENABLE \
                                    | CY_SAR_NEG_SEL_VSSA_KELVIN)

/* Config1 will operate in single shot mode.
   Set the sample time to meet the DieTemp settling time requirement of 1 us.
   With a 16.67 MHz SAR clock, 17 cycles (or a value of 18 in the register)
   gives an aperture time of 1.02 us. */
#define CONFIG1_SAMPLE_TIME01       ((18 << CY_SAR_SAMPLE_TIME0_SHIFT) \
                                    | (4 << CY_SAR_SAMPLE_TIME1_SHIFT))

/* Set the sample times for 2 and 3 to be 4 clock cycles.
   Note that these two sample times are not used by any channels and 
   only shown for reference. */
#define CONFIG1_SAMPLE_TIME23       ((4 << CY_SAR_SAMPLE_TIME2_SHIFT) \
                                    | (4 << CY_SAR_SAMPLE_TIME3_SHIFT))

/* Set the initial state of switches.
   Close the switch between the DieTemp sensor and the positive terminal 
   of the SAR (TEMP_VPLUS). Close the switch between VSSA and the negative 
   terminal of the SAR (VSSA_VMINUS). */
#define CONFIG1_MUX_SWITCH0         (CY_SAR_MUX_FW_VSSA_VMINUS \
                                    | CY_SAR_MUX_FW_TEMP_VPLUS)

/* Enable sequencer control for the VSSA and TEMP switches.
   While unnecessary in this design because there is only one channel in 
   Config1, the code is provided for reference for designs with multiple 
   channels. */
#define CONFIG1_MUX_SWITCH_SQ_CTRL  (CY_SAR_MUX_SQ_CTRL_VSSA \
                                    | CY_SAR_MUX_SQ_CTRL_TEMP)


/* Constants used to convert ADC counts to degrees Celsius
   for the DieTemp sensor. */
#define DieTemp_SAR_TEMP_OFFSET_SHIFT      (10U)
#define DieTemp_SAR_TEMP_OFFSET_MULT       (0x400)
#define DieTemp_SAR_TEMP_OFFSET_DIVIDER    (0x10000)
#define DieTemp_SAR_TEMP_SHIFT             (16U)
#define DieTemp_SAR_TEMP_DIVIDER           (0x10000)
#define DieTemp_SCALE_ADJUSTMENT_DIVIDER   (16U)
#define DieTemp_HALF_OF_ONE                ((int32)1U << (DieTemp_SAR_TEMP_SHIFT - 1U))

/* (effectively 0.5 << 4u) 0.5 in Q28.4 format */
#define DieTemp_SCALE_ADJUSTMENT           (8U)

/* 15 in Q16.16 format */
#define DieTemp_DUAL_SLOPE_CORRECTION      (0xF0000)

/* 100 in Q16.16 format */
#define DieTemp_HIGH_TEMPERATURE           (0x640000)

/* 40 in Q16.16 format */
#define DieTemp_LOW_TEMPERATURE            (0x280000)

/* Die temperature thread priority */
#define DIE_TEMP_THREAD_PRIORITY           (3U)

/* LED blink timer clock value in Hz  */
#define LED_BLINK_TIMER_CLOCK_HZ           (10000U)

/* LED blink timer period value. 
   Period in sec. =  (LED_BLINK_TIMER_PERIOD + 1)/LED_BLINK_TIMER_CLOCK_HZ */
#define LED_BLINK_TIMER_PERIOD             (999U)

/* Set lowest priority for the SAR interrupt. */
#define TIMER_IRQ_PRIORITY                 (4U)

/* Blink LED quickly delay */
#define BLINK_QUICKLY_DELAY                (1U)

/* Blink LED slowly delay  */
#define BLINK_SLOWLY_DELAY                 (5U)

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void application_thread(void *arg);

static void sar_adc_isr(void);
static int32_t die_temperature_counts_to_celsius(int16_t adcCounts);
static void die_temp_thread(void *arg);

static void timer_init(void);
static void timer_isr(void *callback_arg, cyhal_timer_event_t event);

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
/* FreeRTOS queue to receive data from svc thread */
cy_queue_t audio_data_queue_handle;

/* Counter to count frames processed by Dspotter */
uint16_t cyb_frame_count = 0;

/* Define the initialization structure for Config1 */
const cy_stc_sar_config_t config1 =
{
    .ctrl               = (uint32_t)CONFIG1_CTRL,
    .sampleCtrl         = (uint32_t)CONFIG1_SAMPLE_CTRL,
    .sampleTime01       = CONFIG1_SAMPLE_TIME01,
    .sampleTime23       = CONFIG1_SAMPLE_TIME23,
    .rangeThres         = CY_SAR_DEINIT,
    .rangeCond          = CY_SAR_RANGE_COND_BELOW,
    .chanEn             = CONFIG1_CHAN_EN,
    .chanConfig         = CONFIG1_CHAN_CONFIG,
    .intrMask           = CONFIG1_INTR_MASK,
    .satIntrMask        = CY_SAR_DEINIT, /* Disable the saturation interrupt */
    .rangeIntrMask      = CY_SAR_DEINIT, /* Disable the range interrupt */
    .muxSwitch          = CONFIG1_MUX_SWITCH0,
    .muxSwitchSqCtrl    = CONFIG1_MUX_SWITCH_SQ_CTRL,
    .configRouting      = true,
    .vrefMvValue        = CONFIG1_VREF_MV_VALUE,
};

/* Configuration structure for the SAR interrupt. */
const cy_stc_sysint_t SAR_IRQ_cfg = 
{
    .intrSrc            = pass_interrupt_sar_IRQn,
    .intrPriority       = SAR_IRQ_PRIORITY
};

/* Die temperature value */
static int16_t die_temperature = 0;

/* Flag to indicate die temperature is available */
volatile bool is_die_temp_available = 0;

/* Counter to measure the delay of the LED */
volatile uint8_t hw_timer_count = 0;

/* Timer object */
cyhal_timer_t led_blink_timer;

/* LED blinking state */
led_blink_state_t led_state = STOP_BLINKING;

/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 * This is the main function for CM4 CPU.
 * 
 * Parameters:
 *  none
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    BaseType_t retval;
    cy_rslt_t result;

    /* Init the IPC communication for CM4 */
    setup_ipc_communication_cm4();

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Enable global interrupts */
    __enable_irq();


    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

        /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("***************************************************\r\n");
    printf("***** PSoC 6 MCU: Dual-CPU audio wake word and command detection using Cyberon *******\r\n");
    printf("***************************************************\r\n\r\n");

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable analog reference block needed by the SAR. */
    Cy_SysAnalog_Init(&Cy_SysAnalog_Fast_Local);
    Cy_SysAnalog_Enable();

    /* Configure the clock for the SAR for a 16.67 MHz clock frequency. */
    Cy_SysClk_PeriphAssignDivider(PCLK_PASS_CLOCK_SAR, CY_SYSCLK_DIV_8_BIT, 1u);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_8_BIT, 1u, SAR_TARGET_CLK_DIVIDER - 1u);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_8_BIT, 1u);

    /* Configure and enable the SAR interrupt. */
    (void)Cy_SysInt_Init(&SAR_IRQ_cfg, sar_adc_isr);
    NVIC_EnableIRQ(SAR_IRQ_cfg.intrSrc);

    /* Initialize and enable the SAR to Config0. */
    Cy_SAR_Init(SAR, &config1);
    Cy_SAR_Enable(SAR);

    /* Init logs */
    cy_log_init(CY_LOG_OFF, NULL, NULL);

    /* Unlock the semaphore and wake-up the CM0+ */
    Cy_IPC_Sema_Clear(IPC_SEMA_NUM, false);
    __SEV();

    /* Init SVC HP */
    svc_init();

    /* Two stage cyberon init */
    uint64_t uid;
    uid = Cy_SysLib_GetUniqueId();
    printf("\n\r uniqueIdHi: 0x%08lX, uniqueIdLo: 0x%08lX\r\n", 
            (uint32_t)(uid >> 32), (uint32_t)(uid << 32 >> 32));

    if (!cyberon_asr_init((cyberon_asr_callback)asr_callback))
    {
        CY_ASSERT(0);
    }
    
    /* Create CM4 application thread */
    retval = xTaskCreate(application_thread, "application_thread", 
                         APPLICATION_THREAD_STACK_SIZE, NULL, 
                         APPLICATION_THREAD_PRIORITY, NULL);
    if (pdPASS != retval)
    {
        CY_ASSERT(0);
    }

    /* Create Die temperature thread */
    retval = xTaskCreate(die_temp_thread, "die_temp_thread", 
                         configMINIMAL_STACK_SIZE, NULL, 
                         DIE_TEMP_THREAD_PRIORITY, NULL);
    if (pdPASS != retval)
    {
        CY_ASSERT(0);
    }

    /* Initialize timer to toggle the LED */
    timer_init();

    /* Start Scheduler */
    vTaskStartScheduler();

    for (;;)
    {
        /* vTaskStartScheduler never returns */
    }
}

/*******************************************************************************
 * Function Name: application_thread
 *******************************************************************************
 * Summary:
 *  The application task unblocks whenever there is data in queue.
 *  The 10 msec data frame is copied into golbal buffer to be processed by two 
 *  stage cyberon.
 *
 * Parameters:
 *  arg: not used
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void application_thread(void *arg)
{
    (void)arg;
    svc_hp_audio_data_t audio_data_n_frame_count = {0};
    int16_t *audio_data_ptr;

    if (CY_RSLT_SUCCESS != cy_rtos_init_queue(&audio_data_queue_handle, 
        AUDIO_DATA_QUEUE_LENGTH, sizeof(svc_hp_audio_data_t)))
    {
        CY_ASSERT(0);
    }
    
    for (;;)
    {
        if (CY_RSLT_SUCCESS == cy_rtos_get_queue(&audio_data_queue_handle, 
            &audio_data_n_frame_count, CY_RTOS_NEVER_TIMEOUT, is_in_isr()))
        {
            audio_data_ptr = (int16_t *)audio_data_n_frame_count.data_ptr;
            cyb_frame_count = audio_data_n_frame_count.frame_count;
            do
            {
                custom_two_stage_asr_process(audio_data_ptr, CYBERON_FRAME_SIZE);
                audio_data_ptr += CYBERON_FRAME_SIZE;
                cyb_frame_count -= CYBERON_FRAME_SIZE / FRAME_SIZE;
            } 
            while (cyb_frame_count > 0);
        }
    }
}

/*******************************************************************************
* Function Name: die_temp_thread
********************************************************************************
* Summary:
*  Prints Die temperature whenever it is available.
*
* Parameters:
*  arg: not used
*
* Return:
*  void
*
**********************************************************************************/
static void die_temp_thread(void *arg) 
{
    for (;;)
    {
        if (is_die_temp_available) 
        {
            is_die_temp_available = 0;
            
            /* Print die temperature */
            printf("Current die temperature is = %d degC\r\n\r\n", die_temperature);
        }
        vTaskDelay(10);
    }
}

/*******************************************************************************
* Function Name: sar_adc_isr
********************************************************************************
* Summary:
*  When an End of Scan (EOS) interrupt occurs, die temperature value is read.
*  The results is stored in global variable and flag is set to indicate die 
*  temperature is available.
*
* Parameters:
*  void
*
* Return:
*  void
*
**********************************************************************************/
static void sar_adc_isr(void)
{
    /* Read interrupt status register. */
    uint32_t intrStatus = 0u;
    intrStatus = Cy_SAR_GetInterruptStatus(SAR);

    /* Check for the EOS interrupt. */
    if ((intrStatus & CY_SAR_INTR_EOS_MASK) == CY_SAR_INTR_EOS_MASK)
    {
        /* Clear handled interrupt. */
        Cy_SAR_ClearInterrupt(SAR, intrStatus);

        /* Get the ADC result for the DieTemp sensor in Config1.
           Convert the ADC results to degrees Celsius. */
        die_temperature = (int16_t) die_temperature_counts_to_celsius
                                    (Cy_SAR_GetResult16(SAR, 0));
        is_die_temp_available = 1;

        /* Initiate continuous conversions. */
        Cy_SAR_StopConvert(SAR);
        Cy_SAR_DeInit(SAR, true);
        Cy_SAR_Init(SAR, &config1);
        Cy_SAR_Enable(SAR);
    }
}

/*******************************************************************************
* Function Name: die_temperature_counts_to_celsius
********************************************************************************
* Summary:
*  Function to convert ADC counts to degrees Celsius. For details on operation
*  please see the Die Temperature (DieTemp) Component datasheet.
*
* Parameters:
*  int16_t adcCounts - ADC counts for DieTemp scan from ADC.
*
* Return:
*  int32_t - Temperature in whole degrees Celsius.
*
**********************************************************************************/
static int32_t die_temperature_counts_to_celsius(int16_t adcCounts)
{
    int32_t tempCelsius;
    int32_t tInitial;
    int32_t tAdjust;
    int32_t offsetReg;
    int32_t multReg;

    offsetReg = (int16_t)SFLASH->SAR_TEMP_OFFSET;
    multReg   = (int16_t)SFLASH->SAR_TEMP_MULTIPLIER;

    /* Calculate tInitial in Q16.16 */
    tInitial = (adcCounts * multReg) + (offsetReg * DieTemp_SAR_TEMP_OFFSET_MULT);

    if (tInitial >= DieTemp_DUAL_SLOPE_CORRECTION)
    {
        /* Shift (100 - tInitial) by 4 bits to prevent scale-adjustment 
           from overflowing. */
        /* Then divide by the integer bits of (100 - cutoff) to end up with 
           a Q16.16 tAdjust */
        tAdjust = (DieTemp_SCALE_ADJUSTMENT * (((int32)DieTemp_HIGH_TEMPERATURE - tInitial)
            / (int32)DieTemp_SCALE_ADJUSTMENT_DIVIDER)) /
            (((int32)DieTemp_HIGH_TEMPERATURE - (int32)DieTemp_DUAL_SLOPE_CORRECTION) /
            DieTemp_SAR_TEMP_DIVIDER);
    }
    else
    {
        /* Shift (40 + tInitial) by 4 bits to prevent scale-adjustment 
           from overflowing. */
        /* Then divide by the integer bits of (40 + cutoff) to end up with 
           a Q16.16 tAdjust */
        tAdjust = ((int32)DieTemp_SCALE_ADJUSTMENT * (((int32)DieTemp_LOW_TEMPERATURE + tInitial)
           / (int32)DieTemp_SCALE_ADJUSTMENT_DIVIDER)) /
            (((int32)DieTemp_LOW_TEMPERATURE + (int32)DieTemp_DUAL_SLOPE_CORRECTION) /
            (int32)DieTemp_SAR_TEMP_DIVIDER);
    }

    /* Add tInitial + tAdjust + 0.5 to round to nearest int. 
       Shift off frac bits, and return. */
    tempCelsius = tInitial + tAdjust + DieTemp_HALF_OF_ONE;

    return (tempCelsius / DieTemp_SAR_TEMP_OFFSET_DIVIDER);
}

/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
*  This function creates and configures a Timer object. The timer ticks 
*  continuously produces a periodic interrupt on every terminal count 
*  event.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void timer_init(void)
 {
    cy_rslt_t result;

    const cyhal_timer_cfg_t led_blink_timer_cfg = 
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = LED_BLINK_TIMER_PERIOD,   /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
       does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&led_blink_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction, 
       duration */
    cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&led_blink_timer, timer_isr, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                              TIMER_IRQ_PRIORITY, true);
 }

/*******************************************************************************
* Function Name: timer_isr
********************************************************************************
* Summary:
*  This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*  callback_arg     Arguments passed to the interrupt callback
*  event            Timer/counter interrupt triggers
*
* Return:
*  void
*
*******************************************************************************/
static void timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    hw_timer_count++;

    if (led_state != STOP_BLINKING)
    {
        if (led_state == BLINK_QUICKLY)
        {
            if ((BLINK_QUICKLY_DELAY) == hw_timer_count) 
            {
                hw_timer_count = 0;
                /* Invert the USER LED state */
                cyhal_gpio_toggle(CYBSP_USER_LED);
            }
        }
        else
        {
            if ((BLINK_SLOWLY_DELAY) == hw_timer_count) 
            {
                hw_timer_count = 0;
                /* Invert the USER LED state */
                cyhal_gpio_toggle(CYBSP_USER_LED);
            }
        }
    }
}
/* [] END OF FILE */
