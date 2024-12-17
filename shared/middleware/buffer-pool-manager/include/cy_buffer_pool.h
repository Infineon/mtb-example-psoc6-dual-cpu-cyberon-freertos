/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 */

/** @file cy_buffer_pool.h
 *  Defines the Buffer Pool Manager Interface.
 *
 *  The following functionalities are provided by buffer pool manager middleware:
 *  - Allows application to create the buffer pool of fixed size.
 *  - Allows application to get the buffer from created pool.
 *  - Allows application to free the buffer back to pool.
 *  - Allows application to delete the created pool.
 *  - Allows application to register callback to know when buffer is available to allocate.
 *  - Allows application to know the statistics of created pool (buffer allocated currently, used, free etc.)
 *
 */

/**
 * \defgroup group_buffer_pool Buffer Pool Manager API
 * \brief The buffer pool manager library provides APIs to create pool, get the buffer from pool, free buffer back to pool and
 *        delete the pool.
 * \addtogroup group_buffer_pool
 * \{
 * \defgroup group_buffer_pool_macros Macros
 * \defgroup group_buffer_pool_typedefs Typedefs
 * \defgroup group_buffer_pool_enums Enumerated types
 * \defgroup group_buffer_pool_structures Structures
 * \defgroup group_buffer_pool_functions Functions
 */

#ifndef BUFFER_POOL_MANAGER_H__
#define BUFFER_POOL_MANAGER_H__

#include "cy_result.h"
#include "cyabs_rtos.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
/**
 * \addtogroup group_buffer_pool_typedefs
 * \{
 */

/** Buffer Pool handle */
typedef void* cy_buffer_pool_handle;

/** Buffer created from the pool */
typedef void* cy_buffer_t;

typedef void (*cy_buffer_pool_notification_callback_t)(
        cy_buffer_pool_handle handle,
        void* user_args);

/** \} group_buffer_pool_typedefs */
/******************************************************
 *                    Structures
 ******************************************************/
/**
 * \addtogroup group_buffer_pool_structures
 * \{
 */

typedef struct
{
    uint8_t *pool_name; /** Pool name */
    uint16_t no_of_buffers; /** No of buffers in the pool to be created */
    uint16_t sizeof_buffer; /** Sizeof each buffer in the pool */
} cy_buffer_pool_params_t;

typedef struct
{
    uint16_t num_buf_created; /** Total number of buffers created */
    uint16_t num_buf_in_use; /** Number of buffers currently allocated */
    /**
     * Highest number of buffers ever allocated in the pool
     *
     * buf_high_mark will be set initially to 0 and will keep updating the max peak heap usage.
     * It will only reset on the deletion of the pool instance. This will be useful to know the
     * max peak buffer usage which will help to finetune the buffer requirements. */
    uint16_t peak_num_buf_used;
} cy_buffer_pool_stats_t;

/** \} group_buffer_pool_structures */

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * \addtogroup group_buffer_pool_functions
 * \{
 */

/**
 * Creates a buffer pool
 *
 * Buffer pool consist of chunks of fixed size of buffers. Application can configure the size of the each buffer
 * and total number of buffers through the buffer pool parameters.
 *
 * @param[in] buffer_pool_params    Pointer to buffer pool parameter structure
 * @param[out] handle               Pool handle
 *
 * @return On success, \ref cy_buffer_pool_handle returned. returns cy_rslt_t error codes
 */
cy_rslt_t cy_buffer_pool_create(
        cy_buffer_pool_params_t *buffer_pool_params,
        cy_buffer_pool_handle *handle);

/**
 * Get the free buffer from created pool
 *
 * If free buffer is not available, then \ref cy_buffer_pool_get API would return CY_RSLT_BUFFER_POOL_BUFFER_NOT_AVAILABLE
 * error code.
 *
 * @param[in] handle     Pointer to the buffer pool manager
 * @param[out] buffer    Pointer to the buffer
 *
 * @return On sucess, returns \ref cy_buffer_t, On error, returns cy_rslt_t error codes
 */
cy_rslt_t cy_buffer_pool_get(
        cy_buffer_pool_handle handle,
        cy_buffer_t *buffer);

/**
 * When there are no free buffers available in the buffer pool for particular pool instance then \ref cy_buffer_pool_get
 * API would return error code CY_RSLT_BUFFER_POOL_BUFFER_NOT_AVAILABLE. Application can register callback to know when
 * buffer becomes available.
 *
 * \note Only one callback can be registered. If multiple callbacks are registered then only latest callback registration
 * would be honoured.
 *
 * @param[in] handle          Pointer to the buffer pool manager
 * @param[in] callback        Callback to be registered for notification
 * @param[in] user_args       User argument
 *
 * @return On sucess, returns CY_RSLT_SUCCESS, On error, returns cy_rslt_t error codes
 */
cy_rslt_t cy_buffer_pool_register_callback(
        cy_buffer_pool_handle handle,
        cy_buffer_pool_notification_callback_t callback,
        void *user_args);

/**
 * Free the allocated buffer through \ref cy_buffer_pool_get API back to the pool
 *
 * @param[in] buffer         Pointer to buffer structure
 *
 * @return On sucess, returns CY_RSLT_SUCCESS, On error, returns cy_rslt_t error codes
 */
cy_rslt_t cy_buffer_pool_free(cy_buffer_t buffer);

/**
 * Delete created pool
 *
 * @param[in, out] handle     Pointer to the buffer pool which was created in \ref cy_buffer_pool_create.
 *                            handle will be set to NULL on return once the buffer pool is deleted.
 *
 * @return On success, returns CY_RSLT_SUCCESS, On error, returns cy_rslt_t error codes
 */
cy_rslt_t cy_buffer_pool_delete(cy_buffer_pool_handle *handle);

/**
 * Get the statistics of the pool
 *
 * Application can get the statistics of buffer pool instance (number of buffers created, number of buffers in use,
 * peak usage of number of buffers etc.). This can help application to finetune the buffer usage.
 *
 * @param[in] handle              Pointer to the buffer pool which was created in \ref cy_buffer_pool_create
 * @param[in] buffer_pool_stats   Pointer to buffer pool statistics
 *
 * @return On sucess, returns CY_RSLT_SUCCESS, On error, returns cy_rslt_t error codes
 */
cy_rslt_t cy_buffer_pool_get_stats(
        cy_buffer_pool_handle handle,
        cy_buffer_pool_stats_t *buffer_pool_stats);

/** \} group_buffer_pool_functions */

#ifdef __cplusplus
}
#endif

#endif /* BUFFER_POOL_MANAGER_H__ */
