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

/**
 * @file cy_buffer_pool.c
 * @brief Implementation of buffer pool manager APIs
 *
 */

#include "cy_buffer_pool.h"
#include "cy_buffer_pool_errors.h"
#include "cy_log.h"
#include "stdlib.h"
/******************************************************
 *                     Macros
 ******************************************************/

#define BUFFER_POOL_MAGIC_HEADER         0xbaefdcbd
#define BUFFER_POOL_MAGIC_FOOTER         0xefbcabfe


#if ENABLE_BUFFER_POOL_LOGS == 2
#include "stdio.h"
#define cy_buffer_pool_log_info(format,...) printf ("[BPM] "format" \r\n", ##__VA_ARGS__);
#define cy_buffer_pool_log_err(ret_val,format,...) printf ("[BPM] [Err:0x%lx] "format" \r\n", ret_val, ##__VA_ARGS__);
#define cy_buffer_pool_log_dbg(format,...) printf ("[BPM] "format" \r\n", ##__VA_ARGS__);
#elif ENABLE_BUFFER_POOL_LOGS
#define cy_buffer_pool_log_info(format,...) cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_INFO,"[BPM] "format" \r\n", ##__VA_ARGS__);
#define cy_buffer_pool_log_err(ret_val,format,...) cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_ERR,"[BPM] [Err:0x%lx] "format" \r\n", ret_val, ##__VA_ARGS__);
#define cy_buffer_pool_log_dbg(format,...) cy_log_msg (CYLF_MIDDLEWARE,CY_LOG_DEBUG,"[BPM] "format" \r\n", ##__VA_ARGS__);
#else
#define cy_buffer_pool_log_info(format,...)
#define cy_buffer_pool_log_err(ret_val,format,...)
#define cy_buffer_pool_log_dbg(format,...)
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct list_node_t {
    void *pool_handle; /* Stores the pool handle */
    struct list_node_t *next; /* Points to the next buffer in the list */
    uint8_t *buffer_ptr; /* Pointer to the actual buffer created */
} list_node_t;

typedef struct {
    uint32_t magic_header; /* Magic header to validate the pool handle */
    uint8_t *pool_name; /* Pool name */
    uint16_t total_num_buf_created; /* Total number of buffers created */
    uint16_t sizeof_buffer; /* Size of each created buffer */
    uint8_t *data_buffer; /* Base address of data buffer */
    list_node_t *head; /* Pointing to the head node in the buffer list */

    cy_mutex_t lock; /* lock to protect the resource */

    cy_buffer_pool_notification_callback_t callback; /* Notification callback */
    void *user_args; /* User argument */

    /* Members used for stats */
    uint16_t num_buf_allocted; /* Number of buffers currently allocated */
    uint16_t buf_high_mark; /* Highest number of buffers ever allocated in the pool */

    uint32_t magic_footer; /* Magic footer to validate the pool handle */
} cy_internal_buffer_pool_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Functions
 ******************************************************/
static cy_rslt_t cy_buffer_pool_sanity_check(cy_internal_buffer_pool_t *handle);

/*******************************************************************************
 * Function Name: cy_buffer_pool_create
 ********************************************************************************
 * Summary:
 *   Allocates memory for the buffer pool handle, allocates memory for the header + data
 *   for the buffers and builds chain of buffers.
 *
 * Parameters:
 *   buffer_pool_params (in)  : Buffer parameters/configuration from application
 *   pool_handle (out)        : Buffer pool handle
 *******************************************************************************/
cy_rslt_t cy_buffer_pool_create(cy_buffer_pool_params_t *buffer_pool_params,
        cy_buffer_pool_handle *pool_handle) {
    uint16_t header_overhead = 0;
    uint8_t *data_buffer = NULL;
    int count = 1;
    list_node_t *current_node = NULL;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if (NULL == buffer_pool_params) {
        result = CY_RSLT_BUFFER_POOL_BAD_ARG;
        cy_buffer_pool_log_err(result, "Buffer pool parameters passed NULL");
        return result;
    }

    cy_buffer_pool_log_info(
            "name:[%s], BufCnt:[%d], Size:[%d]",
            buffer_pool_params->pool_name, buffer_pool_params->no_of_buffers, buffer_pool_params->sizeof_buffer);

    /* Allocate memory for the pool handle */
    cy_internal_buffer_pool_t *pool = calloc(1,
            sizeof(cy_internal_buffer_pool_t));
    if (NULL == pool) {
        result = CY_RSLT_BUFFER_POOL_OUT_OF_MEMORY;
        cy_buffer_pool_log_err(result, "Failed to allocate memory for buffer pool manager handle");
        return result;
    }

//  cy_buffer_pool_log_dbg("Buf pool handle : [%0x]", pool);

    /* Calculate total number of bytes overhead required to store the buffer headers */
    header_overhead = sizeof(list_node_t);

    /* Allocate memory for actual buffer */
    data_buffer = calloc(1,
            (buffer_pool_params->no_of_buffers
                    * buffer_pool_params->sizeof_buffer)
                    + (header_overhead * buffer_pool_params->no_of_buffers));
    if (NULL == data_buffer) {
        result = CY_RSLT_BUFFER_POOL_OUT_OF_MEMORY;
        cy_buffer_pool_log_err(result, "Failed to allocate memory for actual data");
        free(pool);
        return result;
    }

//  cy_buffer_pool_log_dbg("Base address of allocated buffer : [%0x]",
//          data_buffer);

    /* Initialize pool members */
    pool->pool_name = buffer_pool_params->pool_name;
    pool->total_num_buf_created = buffer_pool_params->no_of_buffers;
    pool->sizeof_buffer = buffer_pool_params->sizeof_buffer;
    pool->data_buffer = data_buffer;
    pool->magic_header = BUFFER_POOL_MAGIC_HEADER;
    pool->magic_footer = BUFFER_POOL_MAGIC_FOOTER;

    result = cy_rtos_init_mutex(&pool->lock);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Failed to initialize mutex");
        free(pool);
        free(data_buffer);
        return CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
    }

//  cy_buffer_pool_log_dbg("Start building up the chain of buffers....");

    /* Iterate over and build up the chain of buffers */
    while (count <= buffer_pool_params->no_of_buffers) {
        list_node_t *node = (list_node_t*) data_buffer;

//      cy_buffer_pool_log_dbg("Creating buffer no [%d]....", count);

        node->pool_handle = pool;
        node->next = NULL;
        node->buffer_ptr = data_buffer + header_overhead;

//      cy_buffer_pool_log_dbg(
//              "Buffer node addr : [%0x], actual buffer addr : [%0x]", node,
//              node->buffer_ptr);

        /* If first node to be added to the list, change the head pointer */
        if (pool->head == NULL) {
            pool->head = node;
            current_node = node;
        } else {
            current_node->next = node;
            current_node = node;
        }

        data_buffer += buffer_pool_params->sizeof_buffer + header_overhead;

        count++;
    }

    *pool_handle = pool;

    cy_buffer_pool_log_info("cy_buffer_pool_create success");

    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
 * Function Name: cy_buffer_pool_get
 ********************************************************************************
 * Summary:
 *   Check if there are any buffers available from the allocated list. if available,
 *   then return the buffer address excluding header data. if not available, then
 *   return error CY_RSLT_BUFFER_POOL_BUFFER_NOT_AVAILABLE.
 *
 * Parameters:
 *   pool_handle (in)  : Buffer pool handle
 *   buffer (out)      : Buffer pointer for application to fill the data
 *******************************************************************************/
cy_rslt_t cy_buffer_pool_get(cy_buffer_pool_handle pool_handle,
        cy_buffer_t *buffer) {
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_internal_buffer_pool_t *handle = (cy_internal_buffer_pool_t*) pool_handle;
    list_node_t *list_node = NULL;

    if (NULL == buffer) {
        result = CY_RSLT_BUFFER_POOL_BAD_ARG;
        cy_buffer_pool_log_err(result, "Invalid buffer passed");
        return result;
    }

    result = cy_buffer_pool_sanity_check(handle);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Basic sanity check failed");
        return result;
    }

    /* Acquire lock */
    result = cy_rtos_get_mutex(&handle->lock, CY_RTOS_NEVER_TIMEOUT);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Acquiring mutex failed");
        result = CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
        return result;
    }

//  cy_buffer_pool_log_dbg("Available buffers in the pool : [%d]",
//          (handle->total_num_buf_created - handle->num_buf_allocted));

    /* If there are no free available buffer in the list, then return error */
    if (handle->head == NULL) {
        result = CY_RSLT_BUFFER_POOL_BUFFER_NOT_AVAILABLE;
        cy_buffer_pool_log_err(result, "No available buffers");
        cy_rtos_set_mutex(&handle->lock);
        return result;
    } else {

        list_node = handle->head;

        /* Make the free available node as head */
        handle->head = list_node->next;
    }

    *buffer = list_node->buffer_ptr;

    /* Increment count for buffer allocated */
    handle->num_buf_allocted = handle->num_buf_allocted + 1;

    if (handle->num_buf_allocted > handle->buf_high_mark) {
        handle->buf_high_mark = handle->num_buf_allocted;
    }

    /* Release lock */
    result = cy_rtos_set_mutex(&handle->lock);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result,"Unlocking mutex failed");
        result = CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
        return result;
    }

//  cy_buffer_pool_log_dbg("Mutex unlocked, Node addr : %0x, buffer addr : %0x", list_node, *buffer);

    return result;
}

/*******************************************************************************
 * Function Name: cy_buffer_pool_free
 ********************************************************************************
 * Summary:
 *   Access the header from the passed buffer pointer and validate if it is the valid
 *   buffer and node has MAGIC header and footer.
 *
 * Parameters:
 *   buffer (in)      : Buffer to be freed back to the pool
 *******************************************************************************/
cy_rslt_t cy_buffer_pool_free(cy_buffer_t buffer) {
    cy_rslt_t result = CY_RSLT_SUCCESS;
    list_node_t *node = NULL;
    cy_internal_buffer_pool_t *pool_handle = NULL;
    list_node_t *list_node = NULL;

    if (NULL == buffer) {
        result = CY_RSLT_BUFFER_POOL_BAD_ARG;
        cy_buffer_pool_log_err(result, "Invalid buffer passed");
        return result;
    }

//  cy_buffer_pool_log_dbg("buffer address : [%0x]", buffer);
    
    node = (list_node_t*) ((buffer) - sizeof(list_node_t));

    // if (NULL == node) {
    //     result = CY_RSLT_BUFFER_POOL_BAD_ARG;
    //     cy_buffer_pool_log_err(result, "Pool handle not created");
    //     return result;
    // }


    /* Perform the sanity check on pool handle passed */
    result = cy_buffer_pool_sanity_check(node->pool_handle);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Basic sanity check failed");
        return result;
    }

    pool_handle = (cy_internal_buffer_pool_t*) node->pool_handle;

    /* Acquire lock */
    result = cy_rtos_get_mutex(&pool_handle->lock, CY_RTOS_NEVER_TIMEOUT);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Acquiring mutex failed");
        result = CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
        return result;
    }

    list_node = pool_handle->head;

    node->next = list_node;
    pool_handle->head = node;

    /* Decrement count for buffer allocated */
    pool_handle->num_buf_allocted = pool_handle->num_buf_allocted - 1;

    if (pool_handle->callback != NULL) {
        pool_handle->callback(pool_handle, pool_handle->user_args);
    }

    /* Release lock */
    result = cy_rtos_set_mutex(&pool_handle->lock);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Unlocking mutex failed");
        result = CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
        return result;
    }

    return result;
}

/*******************************************************************************
 * Function Name: cy_buffer_pool_delete
 ********************************************************************************
 * Summary:
 *   Delete the created buffer pool
 *
 * Parameters:
 *   pool_handle (in, out)   : Buffer pool handle to be deleted.
 *******************************************************************************/
cy_rslt_t cy_buffer_pool_delete(cy_buffer_pool_handle *pool_handle) {
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_internal_buffer_pool_t *handle =
            (cy_internal_buffer_pool_t*) *pool_handle;

    /* Perform the sanity check on pool handle passed */
    result = cy_buffer_pool_sanity_check(handle);
    if (result != CY_RSLT_SUCCESS) {
        cy_buffer_pool_log_err(result, "Basic sanity check failed");
        return result;
    }

    /* Acquire lock */
    result = cy_rtos_get_mutex(&handle->lock, CY_RTOS_NEVER_TIMEOUT);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Acquiring mutex failed");
        result = CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
        return result;
    }

    free(handle->data_buffer);

    result = cy_rtos_deinit_mutex(&handle->lock);
    if (result != CY_RSLT_SUCCESS) {
        cy_buffer_pool_log_err(result, "Failed to de-initialize the mutex");
        result = CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
        return result;
    }

    free(handle);

    *pool_handle = NULL;

    cy_buffer_pool_log_info("cy_buffer_pool_delete success");

    return result;
}

/*******************************************************************************
 * Function Name: cy_buffer_pool_get_stats
 ********************************************************************************
 * Summary:
 *   Get the statistics of buffer pool (Buffer created, allocated etc)
 *
 * Parameters:
 *   pool_handle (in)        : Buffer pool handle
 *   buffer_pool_stats (out) : Buffer pool statistics to be returned
 *******************************************************************************/
cy_rslt_t cy_buffer_pool_get_stats(cy_buffer_pool_handle pool_handle,
        cy_buffer_pool_stats_t *buffer_pool_stats) {
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_internal_buffer_pool_t *handle = (cy_internal_buffer_pool_t*) pool_handle;

    /* Perform the sanity check on pool handle passed */
    result = cy_buffer_pool_sanity_check(handle);
    if (result != CY_RSLT_SUCCESS) {
        cy_buffer_pool_log_err(result, "Basic sanity check failed");
        return result;
    }

    /* Acquire lock */
    result = cy_rtos_get_mutex(&handle->lock, CY_RTOS_NEVER_TIMEOUT);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Acquiring mutex failed");
        result = CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
        return result;
    }

    buffer_pool_stats->num_buf_created = handle->total_num_buf_created;
    buffer_pool_stats->num_buf_in_use = handle->num_buf_allocted;
    buffer_pool_stats->peak_num_buf_used = handle->buf_high_mark;

    /* Release lock */
    result = cy_rtos_set_mutex(&handle->lock);
    if (CY_RSLT_SUCCESS != result) {
        cy_buffer_pool_log_err(result, "Unlocking mutex failed");
        result = CY_RSLT_BUFFER_POOL_GENERIC_ERROR;
        return result;
    }

//  cy_buffer_pool_log_dbg("Buffer pool manager statistics.... ");
//  cy_buffer_pool_log_dbg("Total no of buffers created : [%d], Buffer allocated currently : [%d], Highest number of buffers ever allocated : [%d]",
//          handle->total_num_buf_created, handle->num_buf_allocted,
//          handle->buf_high_mark);
//
//  cy_buffer_pool_log_info("cy_buffer_pool_get_stats success");

    return result;
}

/*******************************************************************************
 * Function Name: cy_buffer_pool_register_callback
 ********************************************************************************
 * Summary:
 *   When there are no buffers available to the buffer pool and cy_buffer_pool_get API
 *   returns CY_RSLT_BUFFER_POOL_BUFFER_NOT_AVAILABLE. Application can register the
 *   callback to know when the buffer is available.
 *
 *   Note : Only one callback can be registered to the pool. If multiple callbacks are
 *   registered, then the last one will override the previous one.
 *
 * Parameters:
 *   pool_handle (in) : Buffer pool handle
 *   callback (in)    : Callbacks to be registered
 *   user_args (in)   : User arguments
 *******************************************************************************/
cy_rslt_t cy_buffer_pool_register_callback(cy_buffer_pool_handle pool_handle,
        cy_buffer_pool_notification_callback_t callback, void *user_args) {
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_internal_buffer_pool_t *handle = (cy_internal_buffer_pool_t*) pool_handle;

    /* Perform the sanity check on pool handle passed */
    result = cy_buffer_pool_sanity_check(handle);
    if (result != CY_RSLT_SUCCESS) {
        cy_buffer_pool_log_err(result, "Basic sanity check failed");
        return result;
    }

    handle->callback = callback;
    handle->user_args = user_args;

    cy_buffer_pool_log_info("cy_buffer_pool_register_callback success");

    return result;
}

/*******************************************************************************
 * Function Name: cy_buffer_pool_sanity_check
 ********************************************************************************
 * Summary:
 *   Check if the buffer pool handle passed has magic header and footer.
 *
 * Parameters:
 *   handle (in) : Buffer pool handle
 *******************************************************************************/
static cy_rslt_t cy_buffer_pool_sanity_check(cy_internal_buffer_pool_t *handle) {
    if (NULL == handle) {
        return CY_RSLT_BUFFER_POOL_BAD_ARG;
    }

    if (BUFFER_POOL_MAGIC_HEADER != handle->magic_header
            && BUFFER_POOL_MAGIC_FOOTER != handle->magic_footer) {
        return CY_RSLT_BUFFER_POOL_BAD_ARG;
    }

    return CY_RSLT_SUCCESS;
}
